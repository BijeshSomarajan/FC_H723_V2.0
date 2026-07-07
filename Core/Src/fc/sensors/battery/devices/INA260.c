#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "../../../io/i2c/I2C.h"
#include "../../../timers/DelayTimer.h"
#include "../../../logger/Logger.h"
#include "../../../util/CommonUtil.h"
#include "../../../util/MathUtil.h"

#include "INA260.h"
INA260Data ina260Data;
volatile INA260State ina260State = INA_STATE_IDLE; // Guarded state variable

void configureINA260(void) {
    uint16_t config = INA260_AVG_128_SAMPLES | INA260_MODE_CONTINUOUS;
    // Reset
    ina260Data.buffer[0] = INA260_REG_CONFIG;
    ina260Data.buffer[1] = (uint8_t) (INA260_REG_CONFIG_RESET >> 8);   // MSB
    ina260Data.buffer[2] = (uint8_t) (INA260_REG_CONFIG_RESET & 0xFF); // LSB
    i2c1Write(INA260_ADDRESS, ina260Data.buffer, 3);
    delayMs(10);

    // Set config
    ina260Data.buffer[0] = INA260_REG_CONFIG;
    ina260Data.buffer[1] = (uint8_t) (config >> 8);
    ina260Data.buffer[2] = (uint8_t) (config & 0xFF);
    i2c1Write(INA260_ADDRESS, ina260Data.buffer, 3);
}

uint8_t initINA260(void) {
    uint8_t status = 0;
    status = initI2C1();
    if (!status) {
        logString("[INA260] I2C Init Failed\n");
        return 0;
    } else {
        logString("[INA260] I2C Init Success\n");
    }
    i2c1_ScanBus();
    status = i2c1CheckAddress(INA260_ADDRESS);
    if (!status) {
        logString("[INA260] I2C Check Failed\n");
        return 0;
    } else {
        logString("[INA260] I2C Check Success\n");
    }
    configureINA260();
    logString("[INA260] I2C Configuration Success\n");
    return 1;
}

// 4. Callback when asynchronous read completes
void __deviceINA260DataReadCallback(uint8_t *buf, uint16_t len) {
    uint16_t rawVoltage = (buf[0] << 8) | buf[1];
    ina260Data.voltage = (float) rawVoltage * 0.00125f;

    ina260State = INA_STATE_IDLE; // Cycle complete! Ready for next request
}

uint8_t ina260ReadDataAsync() {
    return i2c1ReadAsync(INA260_ADDRESS, 2, __deviceINA260DataReadCallback);
}

// 2. Callback when asynchronous register write completes
void __deviceINA260DataRequestCallback(uint8_t *buf, uint16_t len) {
    ina260State = INA_STATE_REQ_COMPLETE; // Request is finished, ready to read
}

uint8_t ina260RequestDataAsync() {
    ina260Data.buffer[0] = INA260_REG_BUS_VOLTAGE;
    return i2c1WriteAsync(INA260_ADDRESS, ina260Data.buffer, 1, __deviceINA260DataRequestCallback);
}

// 1. Non-blocking polling handler called in your main loop
uint8_t readINA260() {
    if (ina260State == INA_STATE_IDLE) {
        ina260State = INA_STATE_BUSY_REQUESTING; // Lock state instantly
        ina260RequestDataAsync();
    }
    else if (ina260State == INA_STATE_REQ_COMPLETE) {
        ina260State = INA_STATE_BUSY_READING;    // Lock state instantly
        ina260ReadDataAsync();
    }
     return 1;
}
