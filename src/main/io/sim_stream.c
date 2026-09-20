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

#include "common/crc.h"
#include "common/maths.h"
#include "common/time.h"
#include "common/utils.h"

#include "drivers/serial.h"
#include "drivers/time.h"

#include "fc/config.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/servos.h"

#include "io/serial.h"
#include "io/sim_stream.h"

#include "scheduler/scheduler.h"

#include "sensors/gyro.h"

#define SIM_STREAM_SYNC1                0xA5
#define SIM_STREAM_SYNC2                0x5A
#define SIM_STREAM_HEADER_SIZE          5
#define SIM_STREAM_FRAME_OVERHEAD       6

#define SIM_STREAM_FRAME_IMU            0x01
#define SIM_STREAM_FRAME_RC             0x07
#define SIM_STREAM_FRAME_CONTROL        0x10
#define SIM_STREAM_FRAME_MOTOR          0x81
#define SIM_STREAM_FRAME_SERVO          0x82
#define SIM_STREAM_FRAME_STATUS         0x83
#define SIM_STREAM_FRAME_STATS          0x85

#define SIM_STREAM_CONTROL_VERSION      1
#define SIM_STREAM_CONTROL_LEN          8
#define SIM_STREAM_CONTROL_ENABLE       (1 << 0)
#define SIM_STREAM_CONTROL_IMU          (1 << 1)
#define SIM_STREAM_DEFAULT_TIMEOUT_MS   100

#define SIM_STREAM_STATUS_LEN           23
#define SIM_STREAM_STATUS_PERIOD_US     100000
#define SIM_STREAM_STATS_LEN            60
#define SIM_STREAM_STATS_PERIOD_US      1000000

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
    uint8_t returnDivisor;
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

static serialPort_t *simPort = NULL;
static int8_t simPortIdentifier = SERIAL_PORT_NONE;
static uint32_t simPortBaud = 0;

static simStreamStats_t stats;
static simStreamControl_t control;
static volatile uint32_t controlSeq = 0;

static uint32_t imuLastFrameUs = 0;         // 32 bit so the PID loop reads it in one instruction
static bool imuGapValid = false;
static volatile uint32_t imuGapSinceStatusUs = 0;

static struct {
    simStreamParseState_e state;
    uint8_t type;
    uint8_t seq;
    uint8_t len;
    uint8_t fill;
    uint8_t crc;
    uint8_t payload[SIM_STREAM_MAX_PAYLOAD];
} parser;

static uint8_t txSeq[SIM_STREAM_TX_TYPE_COUNT];
static uint8_t returnDivider = 0;
static timeUs_t lastStatusUs = 0;
static timeUs_t lastStatsUs = 0;
static uint8_t txSession = 0;
static uint32_t imuSeenAtUs = 0;
static bool timeoutActive = false;
static bool timeoutTripped = false;
static bool returnPathRunning = false;

static int simStreamTypeIndex(uint8_t type)
{
    if (type >= SIM_STREAM_FRAME_IMU && type <= SIM_STREAM_FRAME_RC) {
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
    next.returnDivisor = constrain(parser.payload[5], 1, 8);
    next.timeoutMs = simStreamPayloadU16(6);
    if (next.timeoutMs == 0) {
        next.timeoutMs = SIM_STREAM_DEFAULT_TIMEOUT_MS;
    }
    next.active = (next.flags & SIM_STREAM_CONTROL_ENABLE) != 0;
    next.session = control.session;

    if (next.active && !control.active) {
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
        simStreamImuTiming(nowUs);
    } else if (parser.type == SIM_STREAM_FRAME_CONTROL) {
        simStreamHandleControl(nowUs);
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

static void simStreamWriteOutputs(void)
{
    uint8_t payload[SIM_STREAM_TX_PAYLOAD_MAX];

    const uint8_t motorCount = MIN(getMotorCount(), MAX_SUPPORTED_MOTORS);
    payload[0] = motorCount;
    for (uint8_t i = 0; i < motorCount; i++) {
        payload[1 + 2 * i] = (uint8_t)(motor[i] & 0xFF);
        payload[2 + 2 * i] = (uint8_t)((motor[i] >> 8) & 0xFF);
    }
    simStreamWriteFrame(SIM_STREAM_FRAME_MOTOR, SIM_STREAM_TX_MOTOR, payload, 1 + 2 * motorCount);

    const uint8_t servoCount = MIN((uint8_t)getServoCount(), (uint8_t)MAX_SUPPORTED_SERVOS);
    payload[0] = servoCount;
    for (uint8_t i = 0; i < servoCount; i++) {
        payload[1 + 2 * i] = (uint8_t)(servo[i] & 0xFF);
        payload[2 + 2 * i] = (uint8_t)((servo[i] >> 8) & 0xFF);
    }
    simStreamWriteFrame(SIM_STREAM_FRAME_SERVO, SIM_STREAM_TX_SERVO, payload, 1 + 2 * servoCount);
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

static void simStreamWriteStatus(void)
{
    uint8_t payload[SIM_STREAM_STATUS_LEN];

    const bool gyroCalibrated = gyroIsCalibrationComplete();
    uint16_t flags = SIM_STREAM_CONTROL_ENABLE;
    if (!gyroCalibrated) {
        flags |= (1 << 1);
    } else {
        flags |= (1 << 2);
    }
    if (ARMING_FLAG(ARMED)) {
        flags |= (1 << 3);
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

static void simStreamCheckTimeout(const simStreamControl_t *ctl, timeUs_t nowUs)
{
    if (!(ctl->flags & SIM_STREAM_CONTROL_IMU)) {
        return;
    }

    const uint32_t lastImuUs = imuLastFrameUs;
    if (lastImuUs != imuSeenAtUs) {
        imuSeenAtUs = lastImuUs;
        timeoutActive = false;
    }

    // signed: the stamp is taken in interrupt context and can postdate nowUs
    const int32_t ageUs = (int32_t)((uint32_t)nowUs - lastImuUs);

    // transport test only: a dead stream is latched and reported, it does not disarm
    if (!timeoutActive && ageUs > (int32_t)MS2US((uint32_t)ctl->timeoutMs)) {
        timeoutActive = true;
        timeoutTripped = true;
        stats.timeouts++;
    }
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

    if (!ctl.active) {
        if (returnPathRunning) {
            // the simulator gets the closing table on the wire, so a run needs no CLI paste
            returnPathRunning = false;
            serialBeginWrite(simPort);
            simStreamWriteStats();
            serialEndWrite(simPort);
        }
        return;
    }

    const timeUs_t nowUs = micros();

    if (!returnPathRunning || ctl.session != txSession) {
        // a handshake is a session boundary: the return sequences start over with it
        returnPathRunning = true;
        txSession = ctl.session;
        memset(txSeq, 0, sizeof(txSeq));
        returnDivider = 0;
        lastStatusUs = nowUs;
        lastStatsUs = nowUs;
        timeoutActive = false;
        timeoutTripped = false;
    }

    simStreamCheckTimeout(&ctl, nowUs);

    serialBeginWrite(simPort);

    if (++returnDivider >= ctl.returnDivisor) {
        returnDivider = 0;
        simStreamWriteOutputs();
    }

    if (cmpTimeUs(nowUs, lastStatusUs) >= SIM_STREAM_STATUS_PERIOD_US) {
        lastStatusUs = nowUs;
        simStreamWriteStatus();
    }

    if (cmpTimeUs(nowUs, lastStatsUs) >= SIM_STREAM_STATS_PERIOD_US) {
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
    status->active = ctl.active;
    status->controlFlags = ctl.flags;
    status->imuRateHz = ctl.imuRateHz;
    status->timeoutMs = ctl.timeoutMs;
    status->returnDivisor = ctl.returnDivisor;
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
