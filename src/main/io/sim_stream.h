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
 * High-rate simulator stream, transport test stage: the frames are parsed, counted and
 * answered, but nothing is injected into the sensor stack and nothing touches arming.
 *
 * The pilot assigns the port on the CLI; the stream cannot share a port with any other
 * function, and its baud rate is the peripheral one (the last of the four baud columns):
 *
 *     serial 3 1073741824 115200 115200 0 921600
 *     serial 3 +30 115200 115200 0 921600
 *
 * Both forms work; the second only adds bit 30 to whatever the port already had.
 */

#define SIM_STREAM_MAX_PAYLOAD          64
#define SIM_STREAM_GAP_BINS             64
#define SIM_STREAM_GAP_BIN_US           50
#define SIM_STREAM_LATE_GAP_US          1500

typedef enum {
    SIM_STREAM_RX_IMU = 0,
    SIM_STREAM_RX_MAG,
    SIM_STREAM_RX_BARO,
    SIM_STREAM_RX_GPS,
    SIM_STREAM_RX_RANGE,
    SIM_STREAM_RX_POWER,
    SIM_STREAM_RX_RC,
    SIM_STREAM_RX_CONTROL,
    SIM_STREAM_RX_TYPE_COUNT
} simStreamRxType_e;

typedef enum {
    SIM_STREAM_TX_MOTOR = 0,
    SIM_STREAM_TX_SERVO,
    SIM_STREAM_TX_STATUS,
    SIM_STREAM_TX_STATS,
    SIM_STREAM_TX_TYPE_COUNT
} simStreamTxType_e;

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
    uint16_t controlFlags;
    uint16_t imuRateHz;
    uint16_t timeoutMs;
    uint8_t returnDivisor;
} simStreamStatus_t;

void simStreamInit(void);
void simStreamPoll(void);
void simStreamOnPidLoop(void);

void simStreamResetCounters(void);
const simStreamStats_t *simStreamGetStats(void);
void simStreamGetStatus(simStreamStatus_t *status);
uint32_t simStreamGapPercentileUs(uint8_t percent);

#endif
