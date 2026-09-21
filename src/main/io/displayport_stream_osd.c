/*
 * This file is part of INAV.
 *
 * INAV is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * INAV is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with INAV.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#if defined(USE_SIM_STREAM) && defined(USE_OSD)

#include "common/bitarray.h"
#include "common/utils.h"

#include "drivers/display.h"
#include "drivers/display_font_metadata.h"
#include "drivers/osd_symbols.h"
#include "drivers/time.h"

#include "io/displayport_msp_osd.h"
#include "io/displayport_stream_osd.h"
#include "io/sim_stream.h"

/* Wire constants shared with the MSP DisplayPort grid modes; third-party OSD firmware hardcodes
 * these ordinals, so the simulator gets the same numbers rather than a private set. */
#define STREAM_OSD_MODE_SD_3016     0
#define STREAM_OSD_MODE_HD_5018     1
#define STREAM_OSD_MODE_HD_6022     3
#define STREAM_OSD_MODE_HD_5320     4

// the shadow is sized for the largest grid, so one buffer serves every video system
#define STREAM_OSD_COLS             60
#define STREAM_OSD_ROWS             22
#define STREAM_OSD_SCREENSIZE       (STREAM_OSD_COLS * STREAM_OSD_ROWS)

#define STREAM_OSD_DRAW_INTERVAL_MS 50
// a full redraw would otherwise burst >1 kB into the port and delay the PID loop's MOTOR frames
#define STREAM_OSD_MAX_FRAMES       4

#define STREAM_OSD_RUN_HEADER_LEN   4
#define STREAM_OSD_SCREEN_LEN       4

#define STREAM_OSD_RUN_PRESENT      (1 << 0)
#define STREAM_OSD_SCREEN_CLEARED   (1 << 0)
#define STREAM_OSD_SCREEN_REDRAW    (1 << 1)

static displayPort_t streamOsdDisplayPort;

static uint8_t screen[STREAM_OSD_SCREENSIZE];
static uint8_t attrs[STREAM_OSD_SCREENSIZE];
static BITARRAY_DECLARE(dirty, STREAM_OSD_SCREENSIZE);

static uint8_t screenRows = 16;
static uint8_t screenCols = 30;
static uint8_t screenMode = STREAM_OSD_MODE_SD_3016;
static videoSystem_e currentVideoSystem = VIDEO_SYSTEM_AUTO;

static timeMs_t lastDrawMs = 0;
static uint32_t seenEnableEpoch = 0;
static uint32_t seenRedrawEpoch = 0;
static bool gridChanged = false;

static void streamOsdBlank(void)
{
    memset(screen, SYM_BLANK, sizeof(screen));
    memset(attrs, 0, sizeof(attrs));
    BITARRAY_CLR_ALL(dirty);
}

static void streamOsdMarkAllDirty(void)
{
    for (uint8_t row = 0; row < screenRows; row++) {
        for (uint8_t col = 0; col < screenCols; col++) {
            bitArraySet(dirty, (unsigned)(row * STREAM_OSD_COLS + col));
        }
    }
}

static bool streamOsdSendScreen(uint8_t flags)
{
    uint8_t payload[STREAM_OSD_SCREEN_LEN];

    payload[0] = screenCols;
    payload[1] = screenRows;
    payload[2] = screenMode;
    payload[3] = flags;

    return simStreamOsdSendScreen(payload, STREAM_OSD_SCREEN_LEN);
}

// a dropped frame must not lose the cells it carried, so the run headers are read back
static void streamOsdRedirty(const uint8_t *payload, uint8_t len)
{
    uint8_t i = 1;

    while ((uint8_t)(i + STREAM_OSD_RUN_HEADER_LEN) <= len) {
        const unsigned pos = (unsigned)payload[i] * STREAM_OSD_COLS + payload[i + 1];
        const uint8_t count = payload[i + 3];
        for (uint8_t n = 0; n < count; n++) {
            bitArraySet(dirty, pos + n);
        }
        i += STREAM_OSD_RUN_HEADER_LEN + count;
    }
}

static void streamOsdSendDirty(void)
{
    uint8_t payload[SIM_STREAM_OSD_RUN_PAYLOAD_MAX];
    uint8_t len = 1;
    uint8_t frames = 0;
    bool pending = false;

    int next = BITARRAY_FIND_FIRST_SET(dirty, 0);
    while (next >= 0) {
        int pos = next;
        const uint8_t row = (uint8_t)(pos / STREAM_OSD_COLS);
        const uint8_t col = (uint8_t)(pos % STREAM_OSD_COLS);
        const uint8_t attr = attrs[pos];
        const int endOfLine = row * STREAM_OSD_COLS + screenCols;

        // a string that ran past the edge of the active grid is never shown, so it is not sent
        if ((row >= screenRows) || (col >= screenCols)) {
            bitArrayClr(dirty, (unsigned)pos);
            next = BITARRAY_FIND_FIRST_SET(dirty, (unsigned)pos + 1);
            continue;
        }

        // a run costs its header plus at least one character
        if ((len + STREAM_OSD_RUN_HEADER_LEN + 1) > SIM_STREAM_OSD_RUN_PAYLOAD_MAX) {
            const bool last = (frames + 1) >= STREAM_OSD_MAX_FRAMES;

            payload[0] = last ? STREAM_OSD_RUN_PRESENT : 0;
            if (!simStreamOsdSendRun(payload, len)) {
                streamOsdRedirty(payload, len);
                return;
            }

            // what is left stays dirty and goes out in the next cycles
            if (last) {
                return;
            }

            frames++;
            len = 1;
            pending = false;
        }

        const uint8_t header = len;
        uint8_t count = 0;
        len += STREAM_OSD_RUN_HEADER_LEN;
        do {
            bitArrayClr(dirty, (unsigned)pos);
            payload[len++] = screen[pos++];
            count++;
        } while ((pos < endOfLine) && (len < SIM_STREAM_OSD_RUN_PAYLOAD_MAX)
                 && bitArrayGet(dirty, (unsigned)pos) && (attrs[pos] == attr));

        payload[header] = row;
        payload[header + 1] = col;
        payload[header + 2] = attr;
        payload[header + 3] = count;
        pending = true;

        next = BITARRAY_FIND_FIRST_SET(dirty, (unsigned)pos);
    }

    if (!pending) {
        return;
    }

    payload[0] = STREAM_OSD_RUN_PRESENT;
    if (!simStreamOsdSendRun(payload, len)) {
        streamOsdRedirty(payload, len);
    }
}

// true while the simulator wants the picture; also turns the control-flag edges into a full frame
static bool streamOsdWanted(void)
{
    if (!simStreamOsdWanted()) {
        return false;
    }

    const uint32_t enableEpoch = simStreamOsdEnableEpoch();
    const uint32_t redrawEpoch = simStreamOsdRedrawEpoch();

    if ((enableEpoch == seenEnableEpoch) && (redrawEpoch == seenRedrawEpoch) && !gridChanged) {
        return true;
    }

    if (!streamOsdSendScreen(STREAM_OSD_SCREEN_REDRAW)) {
        return true;
    }

    seenEnableEpoch = enableEpoch;
    seenRedrawEpoch = redrawEpoch;
    gridChanged = false;
    streamOsdMarkAllDirty();
    return true;
}

static int clearScreen(displayPort_t *displayPort)
{
    UNUSED(displayPort);

    streamOsdBlank();
    if (simStreamOsdWanted()) {
        streamOsdSendScreen(STREAM_OSD_SCREEN_CLEARED);
    }
    return 0;
}

static int setChar(const unsigned pos, const uint16_t c, textAttributes_t attr)
{
    if (pos >= STREAM_OSD_SCREENSIZE) {
        return 0;
    }

    const uint8_t ch = (uint8_t)(c & 0xFF);
    const uint8_t page = (uint8_t)((c >> 8) & DISPLAYPORT_MSP_ATTR_FONTPAGE_MASK);
    const uint8_t next = (uint8_t)(page | (TEXT_ATTRIBUTES_HAVE_BLINK(attr) ? DISPLAYPORT_MSP_ATTR_BLINK_MASK : 0));

    if ((screen[pos] != ch) || (attrs[pos] != next)) {
        screen[pos] = ch;
        attrs[pos] = next;
        bitArraySet(dirty, pos);
    }
    return 0;
}

static int writeChar(displayPort_t *displayPort, uint8_t col, uint8_t row, uint16_t c, textAttributes_t attr)
{
    UNUSED(displayPort);

    return setChar((unsigned)row * STREAM_OSD_COLS + col, c, attr);
}

static int writeString(displayPort_t *displayPort, uint8_t col, uint8_t row, const char *string, textAttributes_t attr)
{
    UNUSED(displayPort);

    unsigned pos = (unsigned)row * STREAM_OSD_COLS + col;
    while (*string) {
        setChar(pos++, (uint8_t)*string++, attr);
    }
    return 0;
}

static bool readChar(displayPort_t *displayPort, uint8_t col, uint8_t row, uint16_t *c, textAttributes_t *attr)
{
    UNUSED(displayPort);

    const unsigned pos = (unsigned)row * STREAM_OSD_COLS + col;
    if (pos >= STREAM_OSD_SCREENSIZE) {
        return false;
    }

    *c = screen[pos] | ((uint16_t)(attrs[pos] & DISPLAYPORT_MSP_ATTR_FONTPAGE_MASK) << 8);
    if (attr) {
        *attr = TEXT_ATTRIBUTES_NONE;
    }
    return true;
}

static int drawScreen(displayPort_t *displayPort)
{
    UNUSED(displayPort);

    if (!streamOsdWanted()) {
        return 0;
    }

    const timeMs_t nowMs = millis();
    if ((nowMs - lastDrawMs) < STREAM_OSD_DRAW_INTERVAL_MS) {
        return 0;
    }
    lastDrawMs = nowMs;

    streamOsdSendDirty();
    return 0;
}

static void resync(displayPort_t *displayPort)
{
    displayPort->rows = screenRows;
    displayPort->cols = screenCols;
}

static int screenSize(const displayPort_t *displayPort)
{
    return displayPort->rows * displayPort->cols;
}

static uint32_t txBytesFree(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return simStreamOsdTxBytesFree();
}

static bool getFontMetadata(displayFontMetadata_t *metadata, const displayPort_t *displayPort)
{
    UNUSED(displayPort);

    metadata->charCount = 1024;
    metadata->version = 3;
    return true;
}

static textAttributes_t supportedTextAttributes(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return TEXT_ATTRIBUTES_NONE;
}

static bool isTransferInProgress(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return false;
}

// always ready: the shadow keeps tracking the OSD so a later enable can send a whole picture
static bool isReady(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return true;
}

static int heartbeat(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return 0;
}

static int grab(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return 0;
}

static int release(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return 0;
}

static const displayPortVTable_t streamOsdVTable = {
    .grab = grab,
    .release = release,
    .clearScreen = clearScreen,
    .drawScreen = drawScreen,
    .screenSize = screenSize,
    .writeString = writeString,
    .writeChar = writeChar,
    .readChar = readChar,
    .isTransferInProgress = isTransferInProgress,
    .heartbeat = heartbeat,
    .resync = resync,
    .txBytesFree = txBytesFree,
    .supportedTextAttributes = supportedTextAttributes,
    .getFontMetadata = getFontMetadata,
    .isReady = isReady,
};

static void streamOsdSetVideoSystem(videoSystem_e videoSystem)
{
    if (videoSystem == currentVideoSystem) {
        return;
    }

    switch (videoSystem) {
    case VIDEO_SYSTEM_NTSC:
        screenMode = STREAM_OSD_MODE_SD_3016;
        screenRows = 13;
        screenCols = 30;
        break;
    case VIDEO_SYSTEM_HDZERO:
        screenMode = STREAM_OSD_MODE_HD_5018;
        screenRows = 18;
        screenCols = 50;
        break;
    case VIDEO_SYSTEM_DJIWTF:
        screenMode = STREAM_OSD_MODE_HD_6022;
        screenRows = 22;
        screenCols = 60;
        break;
    case VIDEO_SYSTEM_DJICOMPAT_HD:
    case VIDEO_SYSTEM_AVATAR:
    case VIDEO_SYSTEM_DJI_NATIVE:
        screenMode = STREAM_OSD_MODE_HD_5320;
        screenRows = 20;
        screenCols = 53;
        break;
    default:
        screenMode = STREAM_OSD_MODE_SD_3016;
        screenRows = 16;
        screenCols = 30;
        break;
    }

    currentVideoSystem = videoSystem;
    gridChanged = true;
}

displayPort_t *streamOsdDisplayPortInit(videoSystem_e videoSystem)
{
    static bool initialised = false;

    if (initialised) {
        streamOsdSetVideoSystem(videoSystem);
        resync(&streamOsdDisplayPort);
        return &streamOsdDisplayPort;
    }

    // AUTO is the default, so force the first call through the resolution switch
    currentVideoSystem = (videoSystem_e)-1;
    streamOsdSetVideoSystem(videoSystem);
    streamOsdBlank();

    displayInit(&streamOsdDisplayPort, &streamOsdVTable);
    resync(&streamOsdDisplayPort);
    streamOsdDisplayPort.displayPortType = "Simulator stream OSD";
    initialised = true;

    return &streamOsdDisplayPort;
}

#endif
