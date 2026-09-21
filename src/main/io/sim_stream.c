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

#ifdef USE_SIM_STREAM

#include "common/axis.h"
#include "common/crc.h"
#include "common/maths.h"
#include "common/time.h"
#include "common/utils.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/serial.h"
#include "drivers/time.h"

#include "fc/config.h"
#include "fc/fc_core.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/servos.h"

#include "io/serial.h"
#include "io/sim_stream.h"

#include "msp/msp_serial.h"

#include "navigation/navigation.h"
#include "navigation/navigation_private.h"

#include "scheduler/scheduler.h"

#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"

#ifdef USE_MAG
#include "drivers/compass/compass.h"
#include "sensors/compass.h"
#endif

#ifdef USE_BARO
#include "drivers/barometer/barometer.h"
#include "sensors/barometer.h"
#endif

#ifdef USE_PITOT
#include "drivers/pitotmeter/pitotmeter.h"
#include "sensors/pitotmeter.h"
#endif

#if defined(USE_RANGEFINDER) && defined(USE_RANGEFINDER_FAKE)
#include "drivers/rangefinder/rangefinder_virtual.h"
#include "io/rangefinder.h"
#include "sensors/rangefinder.h"
#endif

#ifdef USE_RX_SIM
#include "rx/rx.h"
#include "rx/sim.h"
#endif

#define SIM_STREAM_SYNC1                0xA5
#define SIM_STREAM_SYNC2                0x5A
#define SIM_STREAM_HEADER_SIZE          5
#define SIM_STREAM_FRAME_OVERHEAD       6

#define SIM_STREAM_FRAME_IMU            0x01
#define SIM_STREAM_FRAME_MAG            0x02
#define SIM_STREAM_FRAME_BARO           0x03
#define SIM_STREAM_FRAME_GPS            0x04
#define SIM_STREAM_FRAME_RANGE          0x05
#define SIM_STREAM_FRAME_POWER          0x06
#define SIM_STREAM_FRAME_RC             0x07
#define SIM_STREAM_FRAME_PITOT          0x08
#define SIM_STREAM_FRAME_CONTROL        0x10
#define SIM_STREAM_FRAME_MOTOR          0x81
#define SIM_STREAM_FRAME_SERVO          0x82
#define SIM_STREAM_FRAME_STATUS         0x83
#define SIM_STREAM_FRAME_NAV            0x84
#define SIM_STREAM_FRAME_STATS          0x85
#define SIM_STREAM_FRAME_ARMING         0x88

#define SIM_STREAM_CONTROL_VERSION      1
#define SIM_STREAM_CONTROL_LEN          8
#define SIM_STREAM_CONTROL_ENABLE       (1 << 0)
#define SIM_STREAM_CONTROL_IMU          (1 << 1)
#define SIM_STREAM_CONTROL_MAG          (1 << 2)
#define SIM_STREAM_CONTROL_BARO         (1 << 3)
#define SIM_STREAM_CONTROL_GPS          (1 << 4)
#define SIM_STREAM_CONTROL_RANGE        (1 << 5)
#define SIM_STREAM_CONTROL_POWER        (1 << 6)
#define SIM_STREAM_CONTROL_RC           (1 << 7)
#define SIM_STREAM_CONTROL_NAV          (1 << 8)
#define SIM_STREAM_CONTROL_PITOT        (1 << 9)
#define SIM_STREAM_CONTROL_ARMING       (1 << 12)
#define SIM_STREAM_DEFAULT_TIMEOUT_MS   100

// an attached port belongs to MSP; it is given back by rebooting once the simulator is done with it
#define SIM_STREAM_ATTACH_REBOOT_MS     5000

#define SIM_STREAM_STATUS_LEN           23
#define SIM_STREAM_STATUS_PERIOD_US     100000
#define SIM_STREAM_NAV_LEN              19
#define SIM_STREAM_STATS_LEN            60
#define SIM_STREAM_STATS_PERIOD_US      1000000
#define SIM_STREAM_ARMING_LEN           10

#define SIM_STREAM_MOTOR_RATE_HZ        500
#define SIM_STREAM_GYRO_LSB_PER_DPS     16.4f
#define SIM_STREAM_ACC_1G               2048
#define SIM_STREAM_MAG_LSB_NT           10

// 64 rather than the 37 bytes a full servo frame needs, because STATS is 60
#define SIM_STREAM_TX_PAYLOAD_MAX       64
#define SIM_STREAM_TX_FRAME_MAX         (SIM_STREAM_FRAME_OVERHEAD + SIM_STREAM_TX_PAYLOAD_MAX)

// The parser runs in the UART interrupt (boards) or in the TCP receive thread (SITL) while the
// PID loop reads what it produced, so everything shared between them is published under a seqlock.
#define SIM_STREAM_BARRIER()            __asm__ volatile("" ::: "memory")

typedef struct {
    uint16_t flags;
    uint16_t imuRateHz;
    uint16_t timeoutMs;
    uint8_t session;            // bumped on every enable, so the return path can restart its sequences
    bool active;
} simStreamControl_t;

typedef enum {
    SIM_PARSE_SYNC1 = 0,
    SIM_PARSE_SYNC2,
    SIM_PARSE_TYPE,
    SIM_PARSE_SEQ,
    SIM_PARSE_LEN,
    SIM_PARSE_PAYLOAD,
    SIM_PARSE_CRC
} simStreamParseState_e;

/* Every slot carries the frame counter its data belongs to, so one seqlock copy tells a reader
 * both what the value is and whether it is new. */
typedef struct {
    uint32_t count;
    int16_t gyro[XYZ_AXIS_COUNT];
    int16_t acc[XYZ_AXIS_COUNT];
} simStreamImuSlot_t;

typedef struct {
    uint32_t count;
    int16_t mag[XYZ_AXIS_COUNT];
} simStreamMagSlot_t;

typedef struct {
    uint32_t count;
    int32_t pressurePa;
    int32_t temperatureCC;
} simStreamBaroSlot_t;

typedef struct {
    uint32_t count;
    float pressurePa;
    float temperatureK;
} simStreamPitotSlot_t;

typedef struct {
    uint32_t count;
    int32_t distanceCm;
} simStreamRangeSlot_t;

typedef struct {
    uint32_t count;
    uint16_t milliVolts;
    uint16_t centiAmps;
} simStreamPowerSlot_t;

typedef struct {
    uint32_t count;
    uint8_t channelCount;
    uint8_t rssi;
    uint16_t channel[SIM_STREAM_RC_MAX_CHANNELS];
} simStreamRcSlot_t;

typedef struct {
    uint32_t count;
    simStreamGpsFrame_t fix;
} simStreamGpsSlot_t;

typedef struct {
    uint32_t sensorMask;
    bool accCalibrated;
    bool compassCalibrated;

    sensorGyroReadFuncPtr gyroReadFn;
    float gyroScale;
    float gyroZero[XYZ_AXIS_COUNT];
    sensor_align_e gyroAlign;

    sensorAccReadFuncPtr accReadFn;
    uint16_t accOneG;
    sensor_align_e accAlign;

#ifdef USE_MAG
    magDev_t magDev;
#endif
#ifdef USE_BARO
    baroDev_t baroDev;
#endif
#ifdef USE_PITOT
    pitotDev_t pitotDev;
#endif
#if defined(USE_RANGEFINDER) && defined(USE_RANGEFINDER_FAKE)
    rangefinderDev_t rangefinderDev;
#endif
} simStreamSaved_t;

static serialPort_t *simPort = NULL;
static int8_t simPortIdentifier = SERIAL_PORT_NONE;
static uint32_t simPortBaud = 0;
static bool portAttached = false;
static uint32_t attachBaudRequest = 0;
static bool rebootArmed = false;
static timeUs_t rebootDeadlineUs = 0;

static simStreamStats_t stats;
static simStreamControl_t control;
static volatile uint32_t controlSeq = 0;

/* A session that INAV ends by itself leaves the simulator's CONTROL(enable) latched, so the
 * repeated enable would no longer look like an edge. The PID loop bumps this epoch on every
 * stop and the parser takes the next enable as a fresh handshake; each side writes only its own. */
static volatile uint32_t sessionStopEpoch = 0;
static uint32_t controlStopEpoch = 0;

static uint32_t imuLastFrameUs = 0;         // 32 bit so the PID loop reads it in one instruction
static bool imuGapValid = false;
static volatile uint32_t imuGapSinceStatusUs = 0;

static volatile uint32_t imuSlotGuard = 0;
static simStreamImuSlot_t imuSlot;
static volatile uint32_t magSlotGuard = 0;
static simStreamMagSlot_t magSlot;
static volatile uint32_t baroSlotGuard = 0;
static simStreamBaroSlot_t baroSlot;
static volatile uint32_t pitotSlotGuard = 0;
static simStreamPitotSlot_t pitotSlot;
static volatile uint32_t rangeSlotGuard = 0;
static simStreamRangeSlot_t rangeSlot;
static volatile uint32_t powerSlotGuard = 0;
static simStreamPowerSlot_t powerSlot;
static volatile uint32_t rcSlotGuard = 0;
static simStreamRcSlot_t rcSlot;
static volatile uint32_t gpsSlotGuard = 0;
static simStreamGpsSlot_t gpsSlot;

/* Reader-side marks. A session starts by latching the counters the parser has reached, never by
 * writing the slots: the parser owns them and runs in another context (UART interrupt, or the TCP
 * receive thread on SITL), so a memset from the PID loop would be a plain data race. */
static uint32_t gyroSlotSeen = 0;
static uint32_t accSlotSeen = 0;
static uint32_t rangeSlotSeen = 0;
static uint32_t rcSlotSeen = 0;
static uint32_t gpsSlotSeen = 0;
static uint32_t magSlotBase = 0;
static uint32_t baroSlotBase = 0;
static uint32_t pitotSlotBase = 0;
static uint32_t powerSlotBase = 0;

static struct {
    simStreamParseState_e state;
    uint8_t type;
    uint8_t seq;
    uint8_t len;
    uint8_t fill;
    uint8_t crc;
    uint8_t payload[SIM_STREAM_MAX_PAYLOAD];
} parser;

static simStreamSaved_t saved;
static simStreamSessionState_e sessionState = SIM_STREAM_SESSION_IDLE;
static uint16_t sessionFlags = 0;

static uint8_t txSeq[SIM_STREAM_TX_TYPE_COUNT];
static uint16_t motorDivider = 0;
static uint16_t motorDividerMax = 1;
static uint16_t servoDivider = 0;
static uint16_t servoDividerMax = 1;
static timeUs_t lastStatusUs = 0;
static timeUs_t lastStatsUs = 0;
static uint8_t txSession = 0;
static uint32_t imuSeenAtUs = 0;
static bool timeoutActive = false;
static bool timeoutTripped = false;

static int simStreamTypeIndex(uint8_t type)
{
    if (type >= SIM_STREAM_FRAME_IMU && type <= SIM_STREAM_FRAME_PITOT) {
        return type - SIM_STREAM_FRAME_IMU;
    }
    if (type == SIM_STREAM_FRAME_CONTROL) {
        return SIM_STREAM_RX_CONTROL;
    }
    return -1;
}

static uint16_t simStreamPayloadU16(uint8_t offset)
{
    return (uint16_t)parser.payload[offset] | ((uint16_t)parser.payload[offset + 1] << 8);
}

static uint32_t simStreamPayloadU32(uint8_t offset)
{
    return (uint32_t)simStreamPayloadU16(offset) | ((uint32_t)simStreamPayloadU16(offset + 2) << 16);
}

static void simStreamSlotWriteBegin(volatile uint32_t *guard)
{
    (*guard)++;
    SIM_STREAM_BARRIER();
}

static void simStreamSlotWriteEnd(volatile uint32_t *guard)
{
    SIM_STREAM_BARRIER();
    (*guard)++;
}

static bool simStreamSlotRead(volatile uint32_t *guard, const void *src, void *dst, size_t size)
{
    for (int retry = 0; retry < 4; retry++) {
        const uint32_t before = *guard;
        if (before & 1) {
            continue;
        }
        SIM_STREAM_BARRIER();
        memcpy(dst, src, size);
        SIM_STREAM_BARRIER();
        if (*guard == before) {
            return true;
        }
    }
    return false;
}

static void simStreamResetStats(void)
{
    memset(&stats, 0, sizeof(stats));
    stats.imuGapMinUs = UINT32_MAX;
    imuGapValid = false;
    imuGapSinceStatusUs = 0;
}

void simStreamResetCounters(void)
{
    simStreamResetStats();
}

static void simStreamPublishControl(const simStreamControl_t *next)
{
    controlSeq++;
    SIM_STREAM_BARRIER();
    control = *next;
    SIM_STREAM_BARRIER();
    controlSeq++;
}

static bool simStreamReadControl(simStreamControl_t *out)
{
    for (int retry = 0; retry < 4; retry++) {
        const uint32_t before = controlSeq;
        if (before & 1) {
            continue;
        }
        SIM_STREAM_BARRIER();
        *out = control;
        SIM_STREAM_BARRIER();
        if (controlSeq == before) {
            return true;
        }
    }
    return false;
}

static void simStreamHandleControl(uint32_t nowUs)
{
    if (parser.len < SIM_STREAM_CONTROL_LEN || parser.payload[0] != SIM_STREAM_CONTROL_VERSION) {
        return;
    }

    simStreamControl_t next;
    next.flags = simStreamPayloadU16(1);
    next.imuRateHz = simStreamPayloadU16(3);
    next.timeoutMs = simStreamPayloadU16(6);
    if (next.timeoutMs == 0) {
        next.timeoutMs = SIM_STREAM_DEFAULT_TIMEOUT_MS;
    }
    next.active = (next.flags & SIM_STREAM_CONTROL_ENABLE) != 0;
    next.session = control.session;

    const uint32_t stopEpoch = sessionStopEpoch;
    if (next.active && (!control.active || (stopEpoch != controlStopEpoch))) {
        controlStopEpoch = stopEpoch;
        next.session++;
        simStreamResetStats();
        // the enabling frame itself is part of the run the counters describe
        stats.rx[SIM_STREAM_RX_CONTROL].received = 1;
        stats.rx[SIM_STREAM_RX_CONTROL].lastSeq = parser.seq;
        stats.rx[SIM_STREAM_RX_CONTROL].seqValid = true;
        imuLastFrameUs = nowUs;
    }

    simStreamPublishControl(&next);
}

static void simStreamImuTiming(uint32_t nowUs)
{
    if (imuGapValid) {
        const uint32_t gapUs = nowUs - imuLastFrameUs;

        if (gapUs < stats.imuGapMinUs) {
            stats.imuGapMinUs = gapUs;
        }
        if (gapUs > stats.imuGapMaxUs) {
            stats.imuGapMaxUs = gapUs;
        }
        if (gapUs > imuGapSinceStatusUs) {
            imuGapSinceStatusUs = gapUs;
        }
        if (gapUs > SIM_STREAM_LATE_GAP_US) {
            stats.imuLateFrames++;
        }

        uint32_t bin = gapUs / SIM_STREAM_GAP_BIN_US;
        if (bin >= SIM_STREAM_GAP_BINS) {
            bin = SIM_STREAM_GAP_BINS - 1;
        }
        stats.imuGapHist[bin]++;
    }

    imuLastFrameUs = nowUs;
    imuGapValid = true;
}

static void simStreamStoreImu(void)
{
    simStreamSlotWriteBegin(&imuSlotGuard);
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        imuSlot.gyro[axis] = (int16_t)simStreamPayloadU16(4 + 2 * axis);
        imuSlot.acc[axis] = (int16_t)simStreamPayloadU16(10 + 2 * axis);
    }
    imuSlot.count++;
    simStreamSlotWriteEnd(&imuSlotGuard);
}

static void simStreamStoreSlowFrame(void)
{
    switch (parser.type) {
    case SIM_STREAM_FRAME_MAG:
        simStreamSlotWriteBegin(&magSlotGuard);
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            magSlot.mag[axis] = (int16_t)simStreamPayloadU16(2 * axis);
        }
        magSlot.count++;
        simStreamSlotWriteEnd(&magSlotGuard);
        break;

    case SIM_STREAM_FRAME_BARO:
        simStreamSlotWriteBegin(&baroSlotGuard);
        baroSlot.pressurePa = (int32_t)simStreamPayloadU32(0);
        baroSlot.temperatureCC = (int16_t)simStreamPayloadU16(4);
        baroSlot.count++;
        simStreamSlotWriteEnd(&baroSlotGuard);
        break;

    case SIM_STREAM_FRAME_PITOT:
        simStreamSlotWriteBegin(&pitotSlotGuard);
        pitotSlot.pressurePa = (float)(int32_t)simStreamPayloadU32(0) * 0.1f;
        pitotSlot.temperatureK = (float)(int16_t)simStreamPayloadU16(4) * 0.01f + 273.15f;
        pitotSlot.count++;
        simStreamSlotWriteEnd(&pitotSlotGuard);
        break;

    case SIM_STREAM_FRAME_RANGE:
        simStreamSlotWriteBegin(&rangeSlotGuard);
        {
            const int32_t distanceMm = (int32_t)simStreamPayloadU32(0);
            // the rangefinder stack works in centimetres, the wire carries millimetres
            rangeSlot.distanceCm = (distanceMm < 0) ? -1 : (distanceMm / 10);
        }
        rangeSlot.count++;
        simStreamSlotWriteEnd(&rangeSlotGuard);
        break;

    case SIM_STREAM_FRAME_POWER:
        simStreamSlotWriteBegin(&powerSlotGuard);
        powerSlot.milliVolts = simStreamPayloadU16(0);
        powerSlot.centiAmps = simStreamPayloadU16(2);
        powerSlot.count++;
        simStreamSlotWriteEnd(&powerSlotGuard);
        break;

    case SIM_STREAM_FRAME_RC: {
        const uint8_t count = MIN(parser.payload[0], (uint8_t)SIM_STREAM_RC_MAX_CHANNELS);
        if (parser.len < (uint8_t)(2 + 2 * parser.payload[0])) {
            break;
        }
        simStreamSlotWriteBegin(&rcSlotGuard);
        for (uint8_t i = 0; i < count; i++) {
            rcSlot.channel[i] = simStreamPayloadU16(1 + 2 * i);
        }
        rcSlot.channelCount = count;
        rcSlot.rssi = parser.payload[1 + 2 * parser.payload[0]];
        rcSlot.count++;
        simStreamSlotWriteEnd(&rcSlotGuard);
        break;
    }

    case SIM_STREAM_FRAME_GPS:
        simStreamSlotWriteBegin(&gpsSlotGuard);
        gpsSlot.fix.fixType = parser.payload[0];
        gpsSlot.fix.numSat = parser.payload[1];
        gpsSlot.fix.lat = (int32_t)simStreamPayloadU32(2);
        gpsSlot.fix.lon = (int32_t)simStreamPayloadU32(6);
        gpsSlot.fix.alt = (int32_t)simStreamPayloadU32(10);
        for (int axis = 0; axis < 3; axis++) {
            gpsSlot.fix.velNED[axis] = (int16_t)simStreamPayloadU16(14 + 2 * axis);
        }
        gpsSlot.fix.groundSpeed = simStreamPayloadU16(20);
        gpsSlot.fix.groundCourse = simStreamPayloadU16(22);
        gpsSlot.fix.hdop = simStreamPayloadU16(24);
        gpsSlot.fix.eph = simStreamPayloadU16(26);
        gpsSlot.fix.epv = simStreamPayloadU16(28);
        gpsSlot.fix.year = simStreamPayloadU16(30);
        gpsSlot.fix.month = parser.payload[32];
        gpsSlot.fix.day = parser.payload[33];
        gpsSlot.fix.hours = parser.payload[34];
        gpsSlot.fix.minutes = parser.payload[35];
        gpsSlot.fix.seconds = parser.payload[36];
        gpsSlot.fix.millis = simStreamPayloadU16(37);
        gpsSlot.count++;
        simStreamSlotWriteEnd(&gpsSlotGuard);
        break;

    default:
        break;
    }
}

static void simStreamFrameComplete(bool crcOk, uint32_t nowUs)
{
    const int index = simStreamTypeIndex(parser.type);

    if (index < 0) {
        stats.unknownFrames++;
        return;
    }

    simStreamTypeStats_t *typeStats = &stats.rx[index];

    if (!crcOk) {
        // the frame is not delivered, so its sequence number shows up as a gap at the next good one
        typeStats->crcErrors++;
        return;
    }

    if (typeStats->seqValid) {
        const uint8_t delta = (uint8_t)(parser.seq - typeStats->lastSeq);
        if (delta != 1) {
            typeStats->gapEvents++;
            typeStats->lost += (uint8_t)(delta - 1);
        }
    }
    typeStats->lastSeq = parser.seq;
    typeStats->seqValid = true;
    typeStats->received++;

    if (parser.type == SIM_STREAM_FRAME_IMU) {
        simStreamStoreImu();
        simStreamImuTiming(nowUs);
    } else if (parser.type == SIM_STREAM_FRAME_CONTROL) {
        simStreamHandleControl(nowUs);
    } else {
        simStreamStoreSlowFrame();
    }
}

static void simStreamParseByte(uint8_t c)
{
    switch (parser.state) {
    case SIM_PARSE_SYNC1:
        if (c == SIM_STREAM_SYNC1) {
            parser.state = SIM_PARSE_SYNC2;
        } else {
            stats.resyncBytes++;
        }
        break;

    case SIM_PARSE_SYNC2:
        if (c == SIM_STREAM_SYNC2) {
            parser.state = SIM_PARSE_TYPE;
        } else if (c == SIM_STREAM_SYNC1) {
            stats.resyncBytes++;
        } else {
            stats.resyncBytes += 2;
            parser.state = SIM_PARSE_SYNC1;
        }
        break;

    case SIM_PARSE_TYPE:
        parser.type = c;
        parser.crc = crc8_dvb_s2(0, c);
        parser.state = SIM_PARSE_SEQ;
        break;

    case SIM_PARSE_SEQ:
        parser.seq = c;
        parser.crc = crc8_dvb_s2(parser.crc, c);
        parser.state = SIM_PARSE_LEN;
        break;

    case SIM_PARSE_LEN:
        if (c > SIM_STREAM_MAX_PAYLOAD) {
            stats.resyncBytes += SIM_STREAM_HEADER_SIZE;
            parser.state = SIM_PARSE_SYNC1;
            break;
        }
        parser.len = c;
        parser.crc = crc8_dvb_s2(parser.crc, c);
        parser.fill = 0;
        parser.state = (c == 0) ? SIM_PARSE_CRC : SIM_PARSE_PAYLOAD;
        break;

    case SIM_PARSE_PAYLOAD:
        parser.payload[parser.fill++] = c;
        parser.crc = crc8_dvb_s2(parser.crc, c);
        if (parser.fill >= parser.len) {
            parser.state = SIM_PARSE_CRC;
        }
        break;

    case SIM_PARSE_CRC:
        simStreamFrameComplete(c == parser.crc, (uint32_t)micros());
        parser.state = SIM_PARSE_SYNC1;
        break;
    }
}

static void simStreamReceiveCallback(uint16_t c, void *rxCallbackData)
{
    UNUSED(rxCallbackData);
    simStreamParseByte((uint8_t)c);
}

// kept out of line: the caller is the gyro task, which lives in the F7 instruction TCM
NOINLINE void simStreamPoll(void)
{
    if (!simPort) {
        return;
    }

    // the USB VCP driver ignores the receive callback and only fills its ring, so it is drained here
    uint32_t waiting = serialRxBytesWaiting(simPort);
    while (waiting--) {
        simStreamParseByte(serialRead(simPort));
    }
}

/* ---- stream sensor devices ------------------------------------------------------------ */

static bool simStreamGyroRead(gyroDev_t *gyroDev)
{
    simStreamImuSlot_t sample;

    if (!simStreamSlotRead(&imuSlotGuard, &imuSlot, &sample, sizeof(sample)) || (sample.count == gyroSlotSeen)) {
        // no new frame: gyroUpdate() keeps the previous sample rather than duplicating this one
        return false;
    }

    gyroSlotSeen = sample.count;
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        gyroDev->gyroADCRaw[axis] = sample.gyro[axis];
    }
    return true;
}

static bool simStreamAccRead(accDev_t *accDev)
{
    simStreamImuSlot_t sample;

    if (!simStreamSlotRead(&imuSlotGuard, &imuSlot, &sample, sizeof(sample)) || (sample.count == accSlotSeen)) {
        return false;
    }

    accSlotSeen = sample.count;
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        accDev->ADCRaw[axis] = sample.acc[axis];
    }
    return true;
}

#ifdef USE_MAG
static bool simStreamMagRead(magDev_t *magDev)
{
    simStreamMagSlot_t sample;

    if (!simStreamSlotRead(&magSlotGuard, &magSlot, &sample, sizeof(sample)) || (sample.count == magSlotBase)) {
        return false;
    }

    // hold the last field between frames: a false here would zero mag.magADC
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        magDev->magADCRaw[axis] = sample.mag[axis];
    }
    return true;
}
#endif

#ifdef USE_BARO
static bool simStreamBaroNoop(baroDev_t *baroDev)
{
    UNUSED(baroDev);
    return true;
}

static bool simStreamBaroCalculate(baroDev_t *baroDev, int32_t *pressure, int32_t *temperature)
{
    simStreamBaroSlot_t sample;

    UNUSED(baroDev);
    const bool fresh = simStreamSlotRead(&baroSlotGuard, &baroSlot, &sample, sizeof(sample))
                       && (sample.count != baroSlotBase);

    if (pressure) {
        *pressure = fresh ? sample.pressurePa : 101325;
    }
    if (temperature) {
        *temperature = fresh ? sample.temperatureCC : 1500;
    }
    return true;
}
#endif

#ifdef USE_PITOT
static bool simStreamPitotNoop(pitotDev_t *pitotDev)
{
    UNUSED(pitotDev);
    return true;
}

static void simStreamPitotCalculate(pitotDev_t *pitotDev, float *pressure, float *temperature)
{
    simStreamPitotSlot_t sample;

    UNUSED(pitotDev);
    const bool fresh = simStreamSlotRead(&pitotSlotGuard, &pitotSlot, &sample, sizeof(sample))
                       && (sample.count != pitotSlotBase);

    if (pressure) {
        *pressure = fresh ? sample.pressurePa : 0.0f;
    }
    if (temperature) {
        *temperature = fresh ? sample.temperatureK : 288.15f;
    }
}
#endif

/* ---- session handling ----------------------------------------------------------------- */

static uint32_t simStreamSlotCount(volatile uint32_t *guard, const void *slot)
{
    uint32_t count;

    // the frame counter is the first member of every slot, so one word is the whole read
    if (!simStreamSlotRead(guard, slot, &count, sizeof(count))) {
        return 0;
    }
    return count;
}

static void simStreamResetSlots(void)
{
    const uint32_t imuCount = simStreamSlotCount(&imuSlotGuard, &imuSlot);

    gyroSlotSeen = imuCount;
    accSlotSeen = imuCount;
    rangeSlotSeen = simStreamSlotCount(&rangeSlotGuard, &rangeSlot);
    rcSlotSeen = simStreamSlotCount(&rcSlotGuard, &rcSlot);
    gpsSlotSeen = simStreamSlotCount(&gpsSlotGuard, &gpsSlot);
    magSlotBase = simStreamSlotCount(&magSlotGuard, &magSlot);
    baroSlotBase = simStreamSlotCount(&baroSlotGuard, &baroSlot);
    pitotSlotBase = simStreamSlotCount(&pitotSlotGuard, &pitotSlot);
    powerSlotBase = simStreamSlotCount(&powerSlotGuard, &powerSlot);
}

static uint16_t simStreamReturnDivider(uint16_t rateHz)
{
    const uint32_t looptimeUs = MAX(getLooptime(), (uint32_t)1);
    const uint32_t loopHz = 1000000 / looptimeUs;

    if (rateHz == 0) {
        return 1;
    }

    const uint32_t divider = (loopHz + rateHz / 2) / rateHz;
    return (divider < 1) ? 1 : (uint16_t)divider;
}

static void simStreamSetSensorPresence(uint32_t sensorBit, cfTaskId_e taskId, bool present)
{
    if (present) {
        sensorsSet(sensorBit);
    } else {
        sensorsClear(sensorBit);
    }
    setTaskEnabled(taskId, present);
}

static bool simStreamSessionStart(const simStreamControl_t *ctl)
{
    // the IMU is the heartbeat of a session; without it there is nothing to fly on
    if (!(ctl->flags & SIM_STREAM_CONTROL_IMU) || !gyro.initialized) {
        return false;
    }

    gyroDev_t *gyroDev = gyroGetPrimaryDevice();

    saved.sensorMask = sensorsMask();
    saved.accCalibrated = STATE(ACCELEROMETER_CALIBRATED) != 0;
    saved.compassCalibrated = STATE(COMPASS_CALIBRATED) != 0;

    saved.gyroReadFn = gyroDev->readFn;
    saved.gyroScale = gyroDev->scale;
    saved.gyroAlign = gyroDev->gyroAlign;
    memcpy(saved.gyroZero, gyroDev->gyroZero, sizeof(saved.gyroZero));

    saved.accReadFn = acc.dev.readFn;
    saved.accOneG = acc.dev.acc_1G;
    saved.accAlign = acc.dev.accAlign;

    simStreamResetSlots();

    gyroDev->readFn = simStreamGyroRead;
    gyroDev->scale = 1.0f / SIM_STREAM_GYRO_LSB_PER_DPS;
    gyroDev->gyroAlign = CW0_DEG;

    acc.dev.readFn = simStreamAccRead;
    acc.dev.acc_1G = SIM_STREAM_ACC_1G;
    acc.dev.accAlign = CW0_DEG;

    sessionFlags = ctl->flags;
    ENABLE_ARMING_FLAG(SIMULATOR_MODE_STREAM);
    imuResetForNewSession(micros());

    // the board's stored accelerometer and compass calibration belongs to the real sensors
    ENABLE_STATE(ACCELEROMETER_CALIBRATED);
    ENABLE_STATE(COMPASS_CALIBRATED);

    gyroStartCalibration();

#ifdef USE_MAG
    saved.magDev = mag.dev;
    {
        const bool present = (sessionFlags & SIM_STREAM_CONTROL_MAG) && (compassConfig()->mag_hardware != MAG_NONE);
        if (present) {
            mag.dev.read = simStreamMagRead;
            mag.dev.magAlign.useExternal = false;
            mag.dev.magAlign.onBoard = CW0_DEG;
        }
        simStreamSetSensorPresence(SENSOR_MAG, TASK_COMPASS, present);
    }
#endif

#ifdef USE_BARO
    saved.baroDev = baro.dev;
    {
        const bool present = (sessionFlags & SIM_STREAM_CONTROL_BARO) && (barometerConfig()->baro_hardware != BARO_NONE);
        if (present) {
            baro.dev.ut_delay = 0;
            baro.dev.up_delay = 1000;
            baro.dev.start_ut = simStreamBaroNoop;
            baro.dev.get_ut = simStreamBaroNoop;
            baro.dev.start_up = simStreamBaroNoop;
            baro.dev.get_up = simStreamBaroNoop;
            baro.dev.calculate = simStreamBaroCalculate;
            baroStartCalibration();
        }
        simStreamSetSensorPresence(SENSOR_BARO, TASK_BARO, present);
    }
#endif

#ifdef USE_PITOT
    saved.pitotDev = pitot.dev;
    {
        const bool present = (sessionFlags & SIM_STREAM_CONTROL_PITOT)
                             && (pitotmeterConfig()->pitot_hardware != PITOT_NONE)
                             && (detectedSensors[SENSOR_INDEX_PITOT] != PITOT_VIRTUAL);
        if (present) {
            pitot.dev.delay = 10000;
            pitot.dev.calibThreshold = 0.00001f;
            pitot.dev.start = simStreamPitotNoop;
            pitot.dev.get = simStreamPitotNoop;
            pitot.dev.calculate = simStreamPitotCalculate;
            pitotStartCalibration();
        }
        simStreamSetSensorPresence(SENSOR_PITOT, TASK_PITOT, present);
    }
#endif

#if defined(USE_RANGEFINDER) && defined(USE_RANGEFINDER_FAKE)
    saved.rangefinderDev = rangefinder.dev;
    {
        const bool present = (sessionFlags & SIM_STREAM_CONTROL_RANGE)
                             && (rangefinderConfig()->rangefinder_hardware != RANGEFINDER_NONE);
        if (present) {
            virtualRangefinderDetect(&rangefinder.dev, &rangefinderFakeVtable);
            rangefinder.dev.init(&rangefinder.dev);
        }
        simStreamSetSensorPresence(SENSOR_RANGEFINDER, TASK_RANGEFINDER, present);
    }
#endif

    if (!(sessionFlags & SIM_STREAM_CONTROL_GPS)) {
        sensorsClear(SENSOR_GPS);
        DISABLE_STATE(GPS_FIX);
    }

    motorDividerMax = simStreamReturnDivider(SIM_STREAM_MOTOR_RATE_HZ);
    servoDividerMax = simStreamReturnDivider(servoConfig()->servoPwmRate);
    motorDivider = 0;
    servoDivider = 0;
    memset(txSeq, 0, sizeof(txSeq));
    timeoutActive = false;
    timeoutTripped = false;

    rebootArmed = false;
    sessionState = SIM_STREAM_SESSION_CALIBRATING;
    return true;
}

static void simStreamSessionStop(void)
{
    if (ARMING_FLAG(ARMED)) {
        disarm(DISARM_SIM_STREAM);
    }

    gyroDev_t *gyroDev = gyroGetPrimaryDevice();

    gyroDev->readFn = saved.gyroReadFn;
    gyroDev->scale = saved.gyroScale;
    gyroDev->gyroAlign = saved.gyroAlign;
    memcpy(gyroDev->gyroZero, saved.gyroZero, sizeof(saved.gyroZero));

    acc.dev.readFn = saved.accReadFn;
    acc.dev.acc_1G = saved.accOneG;
    acc.dev.accAlign = saved.accAlign;

#ifdef USE_MAG
    mag.dev = saved.magDev;
    simStreamSetSensorPresence(SENSOR_MAG, TASK_COMPASS, (saved.sensorMask & SENSOR_MAG) != 0);
#endif
#ifdef USE_BARO
    baro.dev = saved.baroDev;
    simStreamSetSensorPresence(SENSOR_BARO, TASK_BARO, (saved.sensorMask & SENSOR_BARO) != 0);
#endif
#ifdef USE_PITOT
    pitot.dev = saved.pitotDev;
    simStreamSetSensorPresence(SENSOR_PITOT, TASK_PITOT, (saved.sensorMask & SENSOR_PITOT) != 0);
#endif
#if defined(USE_RANGEFINDER) && defined(USE_RANGEFINDER_FAKE)
    rangefinder.dev = saved.rangefinderDev;
    simStreamSetSensorPresence(SENSOR_RANGEFINDER, TASK_RANGEFINDER, (saved.sensorMask & SENSOR_RANGEFINDER) != 0);
#endif

    if (saved.sensorMask & SENSOR_GPS) {
        sensorsSet(SENSOR_GPS);
    } else {
        sensorsClear(SENSOR_GPS);
    }

    DISABLE_ARMING_FLAG(SIMULATOR_MODE_STREAM);

    if (saved.accCalibrated) {
        ENABLE_STATE(ACCELEROMETER_CALIBRATED);
    } else {
        DISABLE_STATE(ACCELEROMETER_CALIBRATED);
    }
    if (saved.compassCalibrated) {
        ENABLE_STATE(COMPASS_CALIBRATED);
    } else {
        DISABLE_STATE(COMPASS_CALIBRATED);
    }

    sessionFlags = 0;
    sessionState = SIM_STREAM_SESSION_IDLE;
    sessionStopEpoch++;

    if (portAttached) {
        rebootArmed = true;
        rebootDeadlineUs = micros() + MS2US((timeUs_t)SIM_STREAM_ATTACH_REBOOT_MS);
    }
}

static void simStreamFeedSlowSensors(void)
{
#if defined(USE_RANGEFINDER) && defined(USE_RANGEFINDER_FAKE)
    if (sessionFlags & SIM_STREAM_CONTROL_RANGE) {
        simStreamRangeSlot_t range;
        if (simStreamSlotRead(&rangeSlotGuard, &rangeSlot, &range, sizeof(range)) && (range.count != rangeSlotSeen)) {
            rangeSlotSeen = range.count;
            fakeRangefindersSetData(range.distanceCm);
        }
    }
#endif

#ifdef USE_RX_SIM
    if ((sessionFlags & SIM_STREAM_CONTROL_RC) && (rxConfig()->receiverType == RX_TYPE_SIM)) {
        simStreamRcSlot_t rc;
        if (simStreamSlotRead(&rcSlotGuard, &rcSlot, &rc, sizeof(rc)) && (rc.count != rcSlotSeen)) {
            rcSlotSeen = rc.count;
            rxSimSetChannelValue(rc.channel, rc.channelCount);
            rxSimSetRssi((uint16_t)(((uint32_t)rc.rssi * RSSI_MAX_VALUE) / 255));
        }
    }
#endif
}

bool simStreamGpsPoll(simStreamGpsFrame_t *fix)
{
    simStreamGpsSlot_t slot;

    if (!(sessionFlags & SIM_STREAM_CONTROL_GPS)) {
        return false;
    }

    if (!simStreamSlotRead(&gpsSlotGuard, &gpsSlot, &slot, sizeof(slot)) || (slot.count == gpsSlotSeen)) {
        return false;
    }

    gpsSlotSeen = slot.count;
    *fix = slot.fix;
    return true;
}

bool simStreamGetVoltage(uint16_t *milliVolts)
{
    simStreamPowerSlot_t power;

    if (!(sessionFlags & SIM_STREAM_CONTROL_POWER)) {
        return false;
    }

    if (!simStreamSlotRead(&powerSlotGuard, &powerSlot, &power, sizeof(power)) || (power.count == powerSlotBase)) {
        return false;
    }

    *milliVolts = power.milliVolts;
    return true;
}

bool simStreamGetAmperage(uint16_t *centiAmps)
{
    simStreamPowerSlot_t power;

    if (!(sessionFlags & SIM_STREAM_CONTROL_POWER)) {
        return false;
    }

    if (!simStreamSlotRead(&powerSlotGuard, &powerSlot, &power, sizeof(power)) || (power.count == powerSlotBase)) {
        return false;
    }

    *centiAmps = power.centiAmps;
    return true;
}

/* ---- return path ----------------------------------------------------------------------- */

static bool simStreamWriteFrame(uint8_t type, simStreamTxType_e txType, const uint8_t *payload, uint8_t len)
{
    uint8_t frame[SIM_STREAM_TX_FRAME_MAX];

    frame[0] = SIM_STREAM_SYNC1;
    frame[1] = SIM_STREAM_SYNC2;
    frame[2] = type;
    frame[3] = txSeq[txType];
    frame[4] = len;
    memcpy(&frame[SIM_STREAM_HEADER_SIZE], payload, len);
    frame[SIM_STREAM_HEADER_SIZE + len] = crc8_dvb_s2_update(0, &frame[2], len + 3);

    const uint8_t frameSize = len + SIM_STREAM_FRAME_OVERHEAD;
    if (serialTxBytesFree(simPort) < frameSize) {
        stats.txDropped++;
        return false;
    }

    serialWriteBuf(simPort, frame, frameSize);
    txSeq[txType]++;
    stats.txFrames[txType]++;
    return true;
}

static void simStreamPutU16(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value & 0xFF);
    dst[1] = (uint8_t)((value >> 8) & 0xFF);
}

static void simStreamPutU32(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)(value & 0xFF);
    dst[1] = (uint8_t)((value >> 8) & 0xFF);
    dst[2] = (uint8_t)((value >> 16) & 0xFF);
    dst[3] = (uint8_t)((value >> 24) & 0xFF);
}

static void simStreamWriteMotors(void)
{
    uint8_t payload[SIM_STREAM_TX_PAYLOAD_MAX];

    const uint8_t motorCount = MIN(getMotorCount(), MAX_SUPPORTED_MOTORS);
    payload[0] = motorCount;
    for (uint8_t i = 0; i < motorCount; i++) {
        simStreamPutU16(&payload[1 + 2 * i], (uint16_t)motor[i]);
    }
    simStreamWriteFrame(SIM_STREAM_FRAME_MOTOR, SIM_STREAM_TX_MOTOR, payload, 1 + 2 * motorCount);
}

static void simStreamWriteServos(void)
{
    uint8_t payload[SIM_STREAM_TX_PAYLOAD_MAX];

    // by servo index, so the simulator maps by position even when the lowest used servo is not servo 1
    const uint8_t servoCount = (uint8_t)MIN(getServoMaxIndex() + 1, MAX_SUPPORTED_SERVOS);
    payload[0] = servoCount;
    for (uint8_t i = 0; i < servoCount; i++) {
        simStreamPutU16(&payload[1 + 2 * i], (uint16_t)servo[i]);
    }
    simStreamWriteFrame(SIM_STREAM_FRAME_SERVO, SIM_STREAM_TX_SERVO, payload, 1 + 2 * servoCount);
}

static void simStreamWriteStatus(void)
{
    uint8_t payload[SIM_STREAM_STATUS_LEN];

    const bool active = sessionState != SIM_STREAM_SESSION_IDLE;
    uint16_t flags = 0;

    if (active) {
        flags |= (1 << 0);
        if (sessionState == SIM_STREAM_SESSION_CALIBRATING) {
            flags |= (1 << 1);
        } else {
            flags |= (1 << 2);
        }
        flags |= (1 << 7);      // the pads never see a streamed session's outputs
    }
    if (ARMING_FLAG(ARMED)) {
        flags |= (1 << 3);
    }
    if (STATE(ACCELEROMETER_CALIBRATED)) {
        flags |= (1 << 4);
    }
    if (STATE(COMPASS_CALIBRATED)) {
        flags |= (1 << 5);
    }
    if (timeoutTripped) {
        flags |= (1 << 6);
        timeoutTripped = false;
    }

    const timeDelta_t pidDeltaUs = getTaskDeltaTime(TASK_PID);
    const uint16_t loopHz = (pidDeltaUs > 0) ? (uint16_t)MIN(1000000 / pidDeltaUs, (int32_t)UINT16_MAX) : 0;
    const simStreamTypeStats_t *imu = &stats.rx[SIM_STREAM_RX_IMU];

    simStreamPutU16(&payload[0], flags);
    simStreamPutU16(&payload[2], (uint16_t)MIN(getLooptime(), (uint32_t)UINT16_MAX));
    simStreamPutU16(&payload[4], loopHz);
    payload[6] = (uint8_t)MIN(averageSystemLoadPercent, (uint16_t)UINT8_MAX);
    simStreamPutU16(&payload[7], (uint16_t)attitude.values.roll);
    simStreamPutU16(&payload[9], (uint16_t)attitude.values.pitch);
    simStreamPutU16(&payload[11], (uint16_t)attitude.values.yaw);
    simStreamPutU32(&payload[13], imu->received);
    simStreamPutU16(&payload[17], (uint16_t)MIN(imu->lost, (uint32_t)UINT16_MAX));
    simStreamPutU16(&payload[19], (uint16_t)MIN(imu->crcErrors, (uint32_t)UINT16_MAX));
    const uint32_t maxGapUs = imuGapSinceStatusUs;
    imuGapSinceStatusUs = 0;
    simStreamPutU16(&payload[21], (uint16_t)MIN(maxGapUs, (uint32_t)UINT16_MAX));

    simStreamWriteFrame(SIM_STREAM_FRAME_STATUS, SIM_STREAM_TX_STATUS, payload, SIM_STREAM_STATUS_LEN);
}

static void simStreamWriteNav(void)
{
    uint8_t payload[SIM_STREAM_NAV_LEN];

    // INAV estimates in a local NEU frame; the wire carries NED
    const float posNorth = getEstimatedActualPosition(X);
    const float posEast = getEstimatedActualPosition(Y);
    const float velNorth = getEstimatedActualVelocity(X);
    const float velEast = getEstimatedActualVelocity(Y);
    const float velUp = getEstimatedActualVelocity(Z);
    const bool homeSet = STATE(GPS_FIX_HOME) != 0;

    simStreamPutU32(&payload[0], (uint32_t)lrintf(getEstimatedActualPosition(Z)));
    simStreamPutU16(&payload[4], (uint16_t)(int16_t)constrainf(velNorth, -32768.0f, 32767.0f));
    simStreamPutU16(&payload[6], (uint16_t)(int16_t)constrainf(velEast, -32768.0f, 32767.0f));
    simStreamPutU16(&payload[8], (uint16_t)(int16_t)constrainf(-velUp, -32768.0f, 32767.0f));
    simStreamPutU32(&payload[10], (uint32_t)lrintf(homeSet ? (posNorth - posControl.rthState.homePosition.pos.x) : 0.0f));
    simStreamPutU32(&payload[14], (uint32_t)lrintf(homeSet ? (posEast - posControl.rthState.homePosition.pos.y) : 0.0f));
    payload[18] = homeSet ? 1 : 0;

    simStreamWriteFrame(SIM_STREAM_FRAME_NAV, SIM_STREAM_TX_NAV, payload, SIM_STREAM_NAV_LEN);
}

static void simStreamWriteArming(void)
{
    uint8_t payload[SIM_STREAM_ARMING_LEN];

    // the raw words, so the simulator decodes them with INAV's own flag tables
    simStreamPutU32(&payload[0], armingFlags);
    payload[4] = (uint8_t)getDisarmReason();
    simStreamPutU32(&payload[5], flightModeFlags);
    payload[9] = (uint8_t)getFlightModeForTelemetry();

    simStreamWriteFrame(SIM_STREAM_FRAME_ARMING, SIM_STREAM_TX_ARMING, payload, SIM_STREAM_ARMING_LEN);
}

static uint16_t simStreamSatU16(uint32_t value)
{
    return (uint16_t)MIN(value, (uint32_t)UINT16_MAX);
}

static void simStreamWriteStats(void)
{
    uint8_t payload[SIM_STREAM_STATS_LEN];

    const simStreamTypeStats_t *imu = &stats.rx[SIM_STREAM_RX_IMU];
    const uint32_t minGapUs = (stats.imuGapMinUs == UINT32_MAX) ? 0 : stats.imuGapMinUs;

    simStreamPutU32(&payload[0], imu->received);
    simStreamPutU32(&payload[4], imu->lost);
    simStreamPutU32(&payload[8], imu->gapEvents);
    simStreamPutU32(&payload[12], imu->crcErrors);
    simStreamPutU32(&payload[16], stats.rx[SIM_STREAM_RX_CONTROL].received);
    simStreamPutU16(&payload[20], simStreamSatU16(minGapUs));
    simStreamPutU16(&payload[22], simStreamSatU16(simStreamGapPercentileUs(50)));
    simStreamPutU16(&payload[24], simStreamSatU16(simStreamGapPercentileUs(95)));
    simStreamPutU16(&payload[26], simStreamSatU16(stats.imuGapMaxUs));
    simStreamPutU32(&payload[28], stats.imuLateFrames);
    simStreamPutU32(&payload[32], stats.timeouts);
    simStreamPutU32(&payload[36], stats.resyncBytes);
    simStreamPutU32(&payload[40], stats.unknownFrames);
    simStreamPutU32(&payload[44], stats.txDropped);
    simStreamPutU32(&payload[48], stats.txFrames[SIM_STREAM_TX_MOTOR]);
    simStreamPutU32(&payload[52], stats.txFrames[SIM_STREAM_TX_SERVO]);
    simStreamPutU32(&payload[56], stats.txFrames[SIM_STREAM_TX_STATUS]);

    simStreamWriteFrame(SIM_STREAM_FRAME_STATS, SIM_STREAM_TX_STATS, payload, SIM_STREAM_STATS_LEN);
}

static bool simStreamTimedOut(const simStreamControl_t *ctl, timeUs_t nowUs)
{
    const uint32_t lastImuUs = imuLastFrameUs;
    if (lastImuUs != imuSeenAtUs) {
        imuSeenAtUs = lastImuUs;
        timeoutActive = false;
    }

    // signed: the stamp is taken in interrupt context and can postdate nowUs
    const int32_t ageUs = (int32_t)((uint32_t)nowUs - lastImuUs);

    if (!timeoutActive && ageUs > (int32_t)MS2US((uint32_t)ctl->timeoutMs)) {
        timeoutActive = true;
        timeoutTripped = true;
        stats.timeouts++;
        return true;
    }

    return false;
}

void simStreamOnPidLoop(void)
{
    if (!simPort) {
        return;
    }

    simStreamControl_t ctl;
    if (!simStreamReadControl(&ctl)) {
        return;
    }

    const timeUs_t nowUs = micros();

    if (ctl.session != txSession) {
        // a handshake is a session boundary: an enable that cannot be honoured is refused whole
        txSession = ctl.session;
        lastStatusUs = nowUs;
        lastStatsUs = nowUs;
        if (sessionState != SIM_STREAM_SESSION_IDLE) {
            simStreamSessionStop();
        }
        if (ctl.active) {
            simStreamSessionStart(&ctl);
        }
    }

    // checked after the handshake so a late enable still wins the race against the deadline
    if (rebootArmed && !ARMING_FLAG(ARMED) && (cmpTimeUs(nowUs, rebootDeadlineUs) >= 0)) {
        rebootArmed = false;
        fcReboot(false);
    }

    if (!ctl.active) {
        if (sessionState != SIM_STREAM_SESSION_IDLE) {
            serialBeginWrite(simPort);
            simStreamWriteStats();
            serialEndWrite(simPort);
            simStreamSessionStop();
        }
        return;
    }

    if (sessionState == SIM_STREAM_SESSION_CALIBRATING && gyroIsCalibrationComplete()) {
        sessionState = SIM_STREAM_SESSION_RUNNING;
    }

    if ((sessionState != SIM_STREAM_SESSION_IDLE) && simStreamTimedOut(&ctl, nowUs)) {
        serialBeginWrite(simPort);
        simStreamWriteStats();
        serialEndWrite(simPort);
        simStreamSessionStop();
    }

    serialBeginWrite(simPort);

    if (sessionState != SIM_STREAM_SESSION_IDLE) {
        simStreamFeedSlowSensors();

        if (++motorDivider >= motorDividerMax) {
            motorDivider = 0;
            simStreamWriteMotors();
        }

        if (++servoDivider >= servoDividerMax) {
            servoDivider = 0;
            simStreamWriteServos();
        }
    }

    if (cmpTimeUs(nowUs, lastStatusUs) >= SIM_STREAM_STATUS_PERIOD_US) {
        lastStatusUs = nowUs;
        simStreamWriteStatus();
        if (ctl.flags & SIM_STREAM_CONTROL_NAV) {
            simStreamWriteNav();
        }
        if (ctl.flags & SIM_STREAM_CONTROL_ARMING) {
            simStreamWriteArming();
        }
    }

    if ((sessionState != SIM_STREAM_SESSION_IDLE) && (cmpTimeUs(nowUs, lastStatsUs) >= SIM_STREAM_STATS_PERIOD_US)) {
        lastStatsUs = nowUs;
        simStreamWriteStats();
    }

    serialEndWrite(simPort);
}

void simStreamInit(void)
{
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_SIM_STREAM);
    if (!portConfig) {
        return;
    }

    simStreamResetStats();

    const uint32_t baud = baudRates[portConfig->peripheral_baudrateIndex];
    simPort = openSerialPort(portConfig->identifier, FUNCTION_SIM_STREAM, simStreamReceiveCallback, NULL,
                             baud, MODE_RXTX, SERIAL_NOT_INVERTED);
    if (!simPort) {
        return;
    }

    simPortIdentifier = portConfig->identifier;
    simPortBaud = baud;
}

uint8_t simStreamAttachRequest(uint32_t baud)
{
    if (ARMING_FLAG(ARMED)) {
        return SIM_STREAM_ATTACH_ARMED;
    }

    if (simPort) {
        return SIM_STREAM_ATTACH_PORT_IN_USE;
    }

    if ((baud != 0) && (lookupBaudRateIndex(baud) == BAUD_AUTO)) {
        return SIM_STREAM_ATTACH_BAD_BAUD;
    }

    attachBaudRequest = baud;
    return SIM_STREAM_ATTACH_OK;
}

void simStreamAttachPostProcess(serialPort_t *port)
{
    if (!port || simPort) {
        return;
    }

    waitForSerialPortToFinishTransmitting(port);

    const serialPortIdentifier_e identifier = port->identifier;
    const uint32_t baud = (attachBaudRequest == 0) ? port->baudRate : attachBaudRequest;

    mspSerialReleasePortIfAllocated(port);

#if defined(SITL_BUILD)
    /* tcpOpen() would spawn a second receive thread on the still-listening socket, so the port is
     * re-pointed in place instead of being re-opened. */
    serialPortUsage_t *usage = findSerialPortUsageByIdentifier(identifier);
    if (!usage) {
        mspSerialAllocatePorts();
        return;
    }

    port->rxCallback = simStreamReceiveCallback;
    port->rxCallbackData = NULL;
    usage->function = FUNCTION_SIM_STREAM;
    usage->serialPort = port;
    simPort = port;
#else
    simPort = openSerialPort(identifier, FUNCTION_SIM_STREAM, simStreamReceiveCallback, NULL,
                             baud, MODE_RXTX, SERIAL_NOT_INVERTED);
    if (!simPort) {
        mspSerialAllocatePorts();
        return;
    }
#endif

    simStreamResetStats();
    simPortIdentifier = identifier;
    simPortBaud = baud;
    portAttached = true;
    // an attach that never enables must not block the MSP port for good either
    rebootArmed = true;
    rebootDeadlineUs = micros() + MS2US((timeUs_t)SIM_STREAM_ATTACH_REBOOT_MS);
}

const simStreamStats_t *simStreamGetStats(void)
{
    return &stats;
}

void simStreamGetStatus(simStreamStatus_t *status)
{
    simStreamControl_t ctl;

    if (!simStreamReadControl(&ctl)) {
        memset(&ctl, 0, sizeof(ctl));
    }

    status->portIdentifier = simPort ? simPortIdentifier : SERIAL_PORT_NONE;
    status->baud = simPortBaud;
    status->active = sessionState != SIM_STREAM_SESSION_IDLE;
    status->sessionState = (uint8_t)sessionState;
    status->controlFlags = ctl.flags;
    status->imuRateHz = ctl.imuRateHz;
    status->timeoutMs = ctl.timeoutMs;
    status->attached = portAttached;
    status->rebootPending = rebootArmed;

    const timeDelta_t remainingUs = cmpTimeUs(rebootDeadlineUs, micros());
    status->rebootInMs = (rebootArmed && (remainingUs > 0)) ? (uint32_t)(remainingUs / 1000) : 0;
}

// returns the upper edge of the histogram bin the requested percentile falls into
uint32_t simStreamGapPercentileUs(uint8_t percent)
{
    uint32_t total = 0;
    for (int i = 0; i < SIM_STREAM_GAP_BINS; i++) {
        total += stats.imuGapHist[i];
    }
    if (total == 0) {
        return 0;
    }

    const uint32_t target = (total * percent + 99) / 100;
    uint32_t cumulative = 0;
    for (int i = 0; i < SIM_STREAM_GAP_BINS; i++) {
        cumulative += stats.imuGapHist[i];
        if (cumulative >= target) {
            return (uint32_t)(i + 1) * SIM_STREAM_GAP_BIN_US;
        }
    }

    return SIM_STREAM_GAP_BINS * SIM_STREAM_GAP_BIN_US;
}

#endif
