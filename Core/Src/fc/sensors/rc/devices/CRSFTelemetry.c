/**
 * ******************************************************************************
 * @file    crsf_telemetry.c
 * @brief   Unified Crossfire (CRSF) Telemetry Serialization Module
 * Target Architecture: STM32H7 Core (Little-Endian, Cache-Enabled)
 * ******************************************************************************
 */

#include <stdint.h>
#include <string.h>
#include "stm32h7xx.h"
#include "../../../logger/Logger.h"
#include "../../../io/uart/UART.h"
#include "../../../dsp/CircularQueue.h"
#include "../../../memory/Memory.h"
#include "CRSF.h"

/* CRSF Protocol Constants */
#define CRSF_SYNC_BYTE                  0xC8
#define CRSF_MAX_FRAME_SIZE             64

/* Standard CRSF Frame Type IDs (Supported natively by ExpressLRS/Crossfire) */
#define CRSF_FRAMETYPE_GPS              0x02
#define CRSF_FRAMETYPE_BATTERY_SENSOR   0x08
#define CRSF_FRAMETYPE_BARO_ALTITUDE    0x09
#define CRSF_FRAMETYPE_ATTITUDE         0x1E
#define CRSF_FRAMETYPE_FLIGHT_MODE      0x21

__ATTR_RAM_D2 uint8_t crsfTxBuffer[CRSF_MAX_FRAME_SIZE];

/* Extern links to your hardware-level UART DMA driver functions & CRC tool */
extern void uart4WriteDMA(uint8_t *buffer, uint16_t length);
extern uint8_t crsf_calculate_crc(const uint8_t *payload, uint8_t length);


/* ================================================================= *
 * CRSF TELEMETRY STRUCTURES                     *
 * ================================================================= */

/**
 * @brief Battery Diagnostics Data Container (8 Bytes)
 */
typedef struct __attribute__((packed)) {
    uint16_t voltage;       // Big-Endian | Scale: 0.1V steps (Decivolts)
    uint16_t current;       // Big-Endian | Scale: 0.1A steps (Deciamperes)
    uint8_t  capacity[3];   // Big-Endian | Non-standard 24-bit block (mAh)
    uint8_t  remaining_pct; // Raw Byte   | Range: 0 to 100%
} crsf_payload_battery_t;

/**
 * @brief GPS Space Vehicle Coordinate & Track Matrix (15 Bytes)
 */
typedef struct __attribute__((packed)) {
    int32_t  latitude;      // Big-Endian | Scale: Degrees * 10,000,000 (Signed)
    int32_t  longitude;     // Big-Endian | Scale: Degrees * 10,000,000 (Signed)
    uint16_t ground_speed;  // Big-Endian | Scale: 0.1 km/h steps
    uint16_t heading;       // Big-Endian | Scale: 0.01 Degree steps (0 to 35999)
    uint16_t altitude;      // Big-Endian | Scale: Meters with a +1000m offset buffer
    uint8_t  satellites;    // Raw Byte   | Total locked GNSS satellites
} crsf_payload_gps_t;

/**
 * @brief Attitude Dynamics Orientation Payload (6 Bytes)
 */
typedef struct __attribute__((packed)) {
    int16_t pitch;          // Big-Endian | Scale: Radians * 10,000 (Signed)
    int16_t roll;           // Big-Endian | Scale: Radians * 10,000 (Signed)
    int16_t yaw;            // Big-Endian | Scale: Radians * 10,000 (Signed)
} crsf_payload_attitude_t;

/**
 * @brief Barometric Tracking & Variometer Profile (4 Bytes)
 */
typedef struct __attribute__((packed)) {
    uint16_t altitude;      // Big-Endian | Scale: Decimeters with a +10000dm offset
    int16_t  vertical_speed;// Big-Endian | Scale: Clime/Dive rate in cm/s (Signed)
} crsf_payload_baro_t;


/* ================================================================= *
 * COMMON CORE TRANSMITTER                       *
 * ================================================================= */

/**
 * @brief Generic core routine to frame, calculate checksum, cache-clean, and dispatch any CRSF frame.
 * @param frameType      Target CRSF macro identification token (e.g., 0x02, 0x08)
 * @param payload        Source pointer pointing to the serialized Big-Endian telemetry structure
 * @param payloadLength  Memory payload footprint matching the target frame specification
 */
void crsfSendTelemetry(uint8_t frameType, const uint8_t *payload, uint8_t payloadLength) {
    // CRSF Length specification field = Type identifier byte + payload data length + CRC checksum byte
    uint8_t lengthField = payloadLength + 2;

    // 1. Structural Header Packing
    crsfTxBuffer[0] = CRSF_SYNC_BYTE;
    crsfTxBuffer[1] = lengthField;
    crsfTxBuffer[2] = frameType;

    // 2. Transmit Window Memory Extraction
    for (uint8_t i = 0; i < payloadLength; i++) {
        crsfTxBuffer[3 + i] = payload[i];
    }

    // 3. CRC Evaluation Block (Covers Type byte through the end of the Payload payload)
    uint8_t crc = crsf_calculate_crc(&crsfTxBuffer[2], lengthField - 1);
    crsfTxBuffer[3 + payloadLength] = crc;

    // Total footprint across the serial wire interface
    uint16_t totalPacketSize = lengthField + 2;

    // 4. STM32H7 L1 Data-Cache Maintenance Guard
    // Forces the processed frame variables out of CPU cache down to physical RAM cells where DMA can access it
    SCB_CleanDCache_by_Addr((uint32_t*)crsfTxBuffer, totalPacketSize);

    // 5. Transfer Execution Link
    uart4WriteDMA(crsfTxBuffer, totalPacketSize);
}


/* ================================================================= *
 * INDIVIDUAL SENSOR WRAPPERS                    *
 * ================================================================= */

/**
 * @brief Processes and streams real-time battery electrical tracking
 * @param voltage_dV Decivolts (Volts * 10, e.g., 14.8V = 148)
 * @param current_dA Deciamperes (Amps * 10, e.g., 8.2A = 82)
 */
void crsfSendBattery(uint16_t voltage_dV, uint16_t current_dA, uint32_t capacity_mAh, uint8_t remaining_pct) {
    crsf_payload_battery_t frame;

    // Convert native Little-Endian scalars into wire Big-Endian blocks via compiler hardware built-ins
    frame.voltage = (uint16_t)__REV16(voltage_dV);
    frame.current = (uint16_t)__REV16(current_dA);

    // Custom packing syntax for the 24-bit block allocation
    frame.capacity[0] = (capacity_mAh >> 16) & 0xFF;
    frame.capacity[1] = (capacity_mAh >> 8)  & 0xFF;
    frame.capacity[2] = capacity_mAh         & 0xFF;

    frame.remaining_pct = remaining_pct;

    crsfSendTelemetry(CRSF_FRAMETYPE_BATTERY_SENSOR, (uint8_t*)&frame, sizeof(frame));
}

/**
 * @brief Processes and streams navigation coordinates from GPS module
 * @param heading_deg Scaled parameter (Degrees * 100, Range: 0 to 35999)
 * @param altitude_m Raw true altitude reading in meters (Supports negative values)
 */
void crsfSendGNSS(int32_t lat_scaled, int32_t lon_scaled, uint16_t speed_scaled, uint16_t heading_scaled, float altitude_m, uint8_t satellites) {
    crsf_payload_gps_t frame;

    frame.latitude     = (int32_t)__REV((uint32_t)lat_scaled);
    frame.longitude    = (int32_t)__REV((uint32_t)lon_scaled);
    frame.ground_speed = (uint16_t)__REV16(speed_scaled);
    frame.heading      = (uint16_t)__REV16(heading_scaled);
    frame.altitude     = (uint16_t)__REV16(altitude_m);
    frame.satellites   = satellites;

    crsfSendTelemetry(CRSF_FRAMETYPE_GPS, (uint8_t*)&frame, sizeof(frame));
}

/**
 * @brief Streams IMU-derived spatial position matrix outputs (From EKF or Mahony filter)
 * @param pitch_scaled, roll_scaled, yaw_scaled (Radians * 10,000)
 */
void crsfSendAttitude(int16_t pitchScaled, int16_t rollScaled, int16_t yawScaled) {
    crsf_payload_attitude_t frame;
    frame.pitch = (int16_t)__REV16((uint16_t)pitchScaled);
    frame.roll  = (int16_t)__REV16((uint16_t)rollScaled);
    frame.yaw   = (int16_t)__REV16((uint16_t)yawScaled);
    crsfSendTelemetry(CRSF_FRAMETYPE_ATTITUDE, (uint8_t*)&frame, sizeof(frame));
}

/**
 * @brief Streams calculated Barometer tracking and Vario climb metrics
 * @param altitude_m True sensor altitude in meters
 * @param vertical_speed_cms Clime or dive rate speed scaled to cm/s
 */
void crsfSendAltitude(float altScaled, int16_t vVelScaled) {
    crsf_payload_baro_t frame;
    frame.altitude       = (uint16_t)__REV16((uint16_t)altScaled);
    frame.vertical_speed = (int16_t)__REV16((uint16_t)vVelScaled);
    crsfSendTelemetry(CRSF_FRAMETYPE_BARO_ALTITUDE, (uint8_t*)&frame, sizeof(frame));
}

/**
 * @brief Transmits flight context character arrays to the handset OSD architecture
 * @param modeStr Pointer to standard null-terminated flight mode string (e.g., "ANGLE")
 */
void crsfSendFlightMode(const char *modeStr) {
    uint8_t payloadBuffer[32];
    uint8_t len = 0;
    // Copy loop to extract character array safely within framing margins
    while (modeStr[len] != '\0' && len < 31) {
        payloadBuffer[len] = (uint8_t)modeStr[len];
        len++;
    }
    payloadBuffer[len] = '\0'; // Enforce the required final trailing null character
    len++;
    crsfSendTelemetry(CRSF_FRAMETYPE_FLIGHT_MODE, payloadBuffer, len);
}
