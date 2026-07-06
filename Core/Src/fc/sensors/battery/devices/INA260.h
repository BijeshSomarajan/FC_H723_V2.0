#ifndef SRC_FC_SENSORS_BATTERY_DEVICES_INA260_H_
#define SRC_FC_SENSORS_BATTERY_DEVICES_INA260_H_


#define INA260_ADDRESS             0x40  // Default I2C address
#define INA260_REG_CONFIG              0x00
#define INA260_REG_CURRENT             0x01
#define INA260_REG_BUS_VOLTAGE         0x02
#define INA260_REG_POWER               0x03
#define INA260_REG_MASK_ENABLE         0x06
#define INA260_REG_ALERT_LIMIT         0x07
#define INA260_REG_CONFIG_RESET        ((uint16_t)0x8000)
#define INA260_MODE_CONTINUOUS         ((uint16_t)0x0007)
#define INA260_AVG_128_SAMPLES         ((uint16_t)0x0400)

typedef struct {
	float voltage;   // Volts
	float current;   // Amperes
	uint8_t buffer[8]; // Buffer for I2C read/write operations
} INA260Data;

typedef enum {
    INA_STATE_IDLE,
    INA_STATE_BUSY_REQUESTING,
    INA_STATE_REQ_COMPLETE,
    INA_STATE_BUSY_READING
} INA260State;

extern INA260Data ina260Data;

uint8_t initINA260(void);
uint8_t readINA260(void);

#endif /* SRC_FC_SENSORS_BATTERY_DEVICES_INA260_H_ */
