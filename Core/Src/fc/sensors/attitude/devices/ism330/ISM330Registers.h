#include "../AttitudeDevice.h"

#if DEVICE_AG_SELECTED == DEVICE_ISM330

/**
 * @brief ISM330DHCX device I2C address
 * The I2C address is 0x6B (1010101b) for the default setting (SDO/SA0 = 0).
 * If SDO/SA0 is high, the address is 0x6A (1010100b).
 */
#define ISM330DHCX_I2C_ADDRESS  0x6B
/**
 * @brief Register addresses
 */
#define ISM330DHCX_FUNC_CFG_ACCESS      0x01
#define ISM330DHCX_WHO_AM_I             0x0F
#define ISM330DHCX_CTRL1_XL             0x10
#define ISM330DHCX_CTRL2_G              0x11
#define ISM330DHCX_CTRL3_C              0x12
#define ISM330DHCX_CTRL4_C              0x13
#define ISM330DHCX_CTRL5_C              0x14
#define ISM330DHCX_CTRL6_C              0x15
#define ISM330DHCX_CTRL7_G              0x16
#define ISM330DHCX_CTRL8_XL             0x17
#define ISM330DHCX_CTRL9_XL             0x18
#define ISM330DHCX_CTRL10_C             0x19
#define ISM330DHCX_STATUS_REG           0x1E

#define ISM330DHCX_OUT_TEMP_L           0x20
#define ISM330DHCX_OUT_TEMP_H           0x21

#define ISM330DHCX_OUTX_L_G             0x22
#define ISM330DHCX_OUTX_H_G             0x23
#define ISM330DHCX_OUTY_L_G             0x24
#define ISM330DHCX_OUTY_H_G             0x25
#define ISM330DHCX_OUTZ_L_G             0x26
#define ISM330DHCX_OUTZ_H_G             0x27

#define ISM330DHCX_OUTX_L_A             0x28
#define ISM330DHCX_OUTX_H_A             0x29
#define ISM330DHCX_OUTY_L_A             0x2A
#define ISM330DHCX_OUTY_H_A             0x2B
#define ISM330DHCX_OUTZ_L_A             0x2C
#define ISM330DHCX_OUTZ_H_A             0x2D

/**
 * @brief Register bit masks and values
 */

// WHO_AM_I register (0x0F)
#define ISM330DHCX_WHO_AM_I_VAL         0x6B

// CTRL1_XL register (0x10) - Accelerometer settings
// ODR_XL (Output Data Rate for Accelerometer)
#define ISM330DHCX_XL_ODR_OFF           (0x00 << 4)
#define ISM330DHCX_XL_ODR_12Hz5         (0x01 << 4)
#define ISM330DHCX_XL_ODR_26Hz          (0x02 << 4)
#define ISM330DHCX_XL_ODR_52Hz          (0x03 << 4)
#define ISM330DHCX_XL_ODR_104Hz         (0x04 << 4)
#define ISM330DHCX_XL_ODR_208Hz         (0x05 << 4)
#define ISM330DHCX_XL_ODR_416Hz         (0x06 << 4)
#define ISM330DHCX_XL_ODR_833Hz         (0x07 << 4)
#define ISM330DHCX_XL_ODR_1666Hz        (0x08 << 4)
#define ISM330DHCX_XL_ODR_3332Hz        (0x09 << 4)
#define ISM330DHCX_XL_ODR_6667Hz        (0x0A << 4)

// FS_XL (Full-Scale for Accelerometer)
#define ISM330DHCX_XL_FS_2G             (0x00 << 2)
#define ISM330DHCX_XL_FS_16G            (0x01 << 2)
#define ISM330DHCX_XL_FS_4G             (0x02 << 2)
#define ISM330DHCX_XL_FS_8G             (0x03 << 2)

// CTRL2_G register (0x11) - Gyroscope settings
// ODR_G (Output Data Rate for Gyroscope)
#define ISM330DHCX_G_ODR_OFF            (0x00 << 4)
#define ISM330DHCX_G_ODR_12Hz5          (0x01 << 4)
#define ISM330DHCX_G_ODR_26Hz           (0x02 << 4)
#define ISM330DHCX_G_ODR_52Hz           (0x03 << 4)
#define ISM330DHCX_G_ODR_104Hz          (0x04 << 4)
#define ISM330DHCX_G_ODR_208Hz          (0x05 << 4)
#define ISM330DHCX_G_ODR_416Hz          (0x06 << 4)
#define ISM330DHCX_G_ODR_833Hz          (0x07 << 4)
#define ISM330DHCX_G_ODR_1666Hz         (0x08 << 4)
#define ISM330DHCX_G_ODR_3332Hz         (0x09 << 4)
#define ISM330DHCX_G_ODR_6667Hz         (0x0A << 4)

// FS_G (Full-Scale for Gyroscope)
#define ISM330DHCX_G_FS_125DPS          (0x01 << 1)
#define ISM330DHCX_G_FS_250DPS          (0x00 << 2)
#define ISM330DHCX_G_FS_500DPS          (0x01 << 2)
#define ISM330DHCX_G_FS_1000DPS         (0x02 << 2)
#define ISM330DHCX_G_FS_2000DPS         (0x03 << 2)
#define ISM330DHCX_G_FS_4000DPS         (0x04 << 2)

// CTRL3_C register (0x12) - General control
#define ISM330DHCX_SW_RESET             (0x01 << 7)
#define ISM330DHCX_BDU_EN               (0x01 << 6) // Block Data Update
#define ISM330DHCX_IF_INC               (0x01 << 2) // Auto increment registers for multi-byte read/write

#endif // ISM330DHCX_H

