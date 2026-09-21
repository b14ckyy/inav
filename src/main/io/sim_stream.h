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

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef USE_SIM_STREAM

/*
 * High-rate simulator stream: while a session is active the module owns INAV's sensor
 * devices, so calibration, alignment, filters, AHRS and navigation run on the stream
 * exactly as they run on real hardware. Motor and servo outputs go to the simulator.
 *
 * The pilot assigns the port on the CLI; the stream cannot share a port with any other
 * function, and its baud rate is the peripheral one (the last of the four baud columns):
 *
 *     serial 3 1073741824 115200 115200 0 921600
 *     serial 3 +30 115200 115200 0 921600
 *
 * Both forms work; the second only adds bit 30 to whatever the port already had.
 */

struct serialPort_s;

#define SIM_STREAM_MAX_PAYLOAD          64
#define SIM_STREAM_OSD_RUN_PAYLOAD_MAX  128
#define SIM_STREAM_GAP_BINS             64
#define SIM_STREAM_GAP_BIN_US           50
#define SIM_STREAM_LATE_GAP_US          1500
#define SIM_STREAM_RC_MAX_CHANNELS      16

// MSP2_INAV_SIM_STREAM_ATTACH reply codes
#define SIM_STREAM_ATTACH_OK            0
#define SIM_STREAM_ATTACH_ARMED         1
#define SIM_STREAM_ATTACH_PORT_IN_USE   2
#define SIM_STREAM_ATTACH_BAD_BAUD      3

typedef enum {
    SIM_STREAM_RX_IMU = 0,
    SIM_STREAM_RX_MAG,
    SIM_STREAM_RX_BARO,
    SIM_STREAM_RX_GPS,
    SIM_STREAM_RX_RANGE,
    SIM_STREAM_RX_POWER,
    SIM_STREAM_RX_RC,
    SIM_STREAM_RX_PITOT,
    SIM_STREAM_RX_CONTROL,
    SIM_STREAM_RX_TYPE_COUNT
} simStreamRxType_e;

typedef enum {
    SIM_STREAM_TX_MOTOR = 0,
    SIM_STREAM_TX_SERVO,
    SIM_STREAM_TX_STATUS,
    SIM_STREAM_TX_STATS,
    SIM_STREAM_TX_NAV,
    SIM_STREAM_TX_ARMING,
    SIM_STREAM_TX_OSD_RUN,
    SIM_STREAM_TX_OSD_SCREEN,
    SIM_STREAM_TX_TYPE_COUNT
} simStreamTxType_e;

typedef enum {
    SIM_STREAM_SESSION_IDLE = 0,
    SIM_STREAM_SESSION_CALIBRATING,
    SIM_STREAM_SESSION_RUNNING
} simStreamSessionState_e;

typedef struct {
    uint32_t received;
    uint32_t lost;
    uint32_t gapEvents;
    uint32_t crcErrors;
    uint8_t lastSeq;
    bool seqValid;
} simStreamTypeStats_t;

typedef struct {
    simStreamTypeStats_t rx[SIM_STREAM_RX_TYPE_COUNT];
    uint32_t resyncBytes;
    uint32_t unknownFrames;         // type outside the known set, bad CRC included: it cannot be attributed
    uint32_t txDropped;
    uint32_t txFrames[SIM_STREAM_TX_TYPE_COUNT];
    uint32_t timeouts;
    uint32_t imuGapMinUs;
    uint32_t imuGapMaxUs;
    uint32_t imuLateFrames;
    uint32_t imuGapHist[SIM_STREAM_GAP_BINS];
} simStreamStats_t;

typedef struct {
    int8_t portIdentifier;          // SERIAL_PORT_NONE when the feature is built in but unassigned
    uint32_t baud;
    bool active;
    uint8_t sessionState;
    uint16_t controlFlags;
    uint16_t imuRateHz;
    uint16_t timeoutMs;
    bool attached;                  // port taken over at runtime instead of assigned in the serial config
    bool rebootPending;
    uint32_t rebootInMs;
} simStreamStatus_t;

// one GPS fix as the wire carries it, in INAV's own units
typedef struct {
    uint8_t fixType;
    uint8_t numSat;
    int32_t lat;                    // 1e-7 degrees
    int32_t lon;                    // 1e-7 degrees
    int32_t alt;                    // cm MSL
    int16_t velNED[3];              // cm/s
    uint16_t groundSpeed;           // cm/s
    uint16_t groundCourse;          // 0.1 degrees
    uint16_t hdop;                  // * 100
    uint16_t eph;                   // cm
    uint16_t epv;                   // cm
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
    uint16_t millis;
} simStreamGpsFrame_t;

void simStreamInit(void);
void simStreamPoll(void);
void simStreamOnPidLoop(void);

/* Runtime attach: the MSP handler asks first, then hands the port over from the MSP post-process
 * hook so the reply still leaves through the MSP framing. */
uint8_t simStreamAttachRequest(uint32_t baud);
void simStreamAttachPostProcess(struct serialPort_s *port);

// consumed by the GPS task: true when a fix arrived that has not been handed over yet
bool simStreamGpsPoll(simStreamGpsFrame_t *fix);
bool simStreamGetVoltage(uint16_t *milliVolts);
bool simStreamGetAmperage(uint16_t *centiAmps);

#ifdef USE_OSD
/* The stream OSD driver asks whether the simulator wants the picture and watches the two epochs
 * for the control-flag edges that ask for a full frame. */
bool simStreamOsdWanted(void);
uint32_t simStreamOsdEnableEpoch(void);
uint32_t simStreamOsdRedrawEpoch(void);
uint32_t simStreamOsdTxBytesFree(void);
bool simStreamOsdSendRun(const uint8_t *payload, uint8_t len);
bool simStreamOsdSendScreen(const uint8_t *payload, uint8_t len);
#endif

void simStreamResetCounters(void);
const simStreamStats_t *simStreamGetStats(void);
void simStreamGetStatus(simStreamStatus_t *status);
uint32_t simStreamGapPercentileUs(uint8_t percent);

#endif
