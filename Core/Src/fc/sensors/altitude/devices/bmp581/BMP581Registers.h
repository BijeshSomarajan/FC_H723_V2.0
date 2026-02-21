/*
 * bmp581Registers.h
 *
 * Created on: Oct 23, 2025
 * Author: bijes
 */

#ifndef SRC_FC_SENSORS_ALTITUDE_BMP581_BMP581REGISTERS_H_
#define SRC_FC_SENSORS_ALTITUDE_BMP581_BMP581REGISTERS_H_

#define BMP581_REG_CHIP_ID              0x01
#define BMP581_REG_REV_ID               0x02
#define BMP581_REG_STATUS               0x28



#define BMP581_REG_TEMP_DATA_XLSB       0x1D
#define BMP581_REG_TEMP_DATA_LSB        0x1E
#define BMP581_REG_TEMP_DATA_MSB        0x1F

#define BMP581_REG_PRESS_DATA_XLSB      0x20
#define BMP581_REG_PRESS_DATA_LSB       0x21
#define BMP581_REG_PRESS_DATA_MSB       0x22

#define BMP581_REG_INT_STATUS           0x27

#define BMP581_REG_FIFO_CONFIG_1        0x16
#define BMP581_REG_FIFO_FILL_LEVEL      0x17
#define BMP581_REG_FIFO_CONFIG_2        0x18
#define BMP581_REG_FIFO_DATA            0x29

#define BMP581_REG_DSP_CONFIG           0x30
#define BMP581_REG_DSP_IIR              0x31
#define BMP581_REG_OSR_CONFIG           0x36
#define BMP581_REG_ODR_CONFIG           0x37

#define BMP581_REG_CMD                  0x7E

#define BMP581_REG_NVM_ADDR             0x2B
#define BMP581_REG_NVM_DATA_LSB         0x2C
#define BMP581_REG_NVM_DATA_MSB         0x2D

#define BMP581_CHIP_ID_VALUE            0x50
#define BMP581_SOFT_RESET_CMD           0xB6

#define BMP581_POWERMODE_SLEEP          0x00
#define BMP581_POWERMODE_FORCED         0x01
#define BMP581_POWERMODE_NORMAL         0x03

#define BMP581_CMD_NOP                  0x00
#define BMP581_CMD_RESET                0xB6
#define BMP581_CMD_FIFO_FLUSH           0xB0
#define BMP581_CMD_INT_ACK              0xA0

#define BMP581_STATUS_DRDY_DATA         0x01
#define BMP581_STATUS_NVM_READY         0x02

#define BMP581_ERR_FATAL                0x01
#define BMP581_ERR_CMD                  0x02
#define BMP581_ERR_CONF                 0x04

#define BMP581_EVENT_POR_READY          0x10
#define BMP581_EVENT_DRDY               0x01
#define BMP581_EVENT_FIFO_FULL          0x02


#endif /* SRC_FC_SENSORS_ALTITUDE_BMP581_BMP581REGISTERS_H_ */
