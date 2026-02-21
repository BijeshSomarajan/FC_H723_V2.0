#ifndef FC_FCDEVICES_RM3100_RM3100REGISTERS_H_
#define FC_FCDEVICES_RM3100_RM3100REGISTERS_H_

#if DEVICE_MAG_SELECTED == DEVICE_RM3100

#define RM3100_WHO_AM_I_REG 0x36 // Hexadecimal address for the Revid internal register

#define RM3100_POLL_REG 0x00   // Hexadecimal address for the Poll internal register
#define RM3100_CMM_REG 0x01	   // Hexadecimal address for the Continuous Measurement Mode internal register
#define RM3100_STATUS_REG 0x34 // Hexadecimal address for the Status internal register
#define RM3100_CCX1_REG 0x04   // Hexadecimal address for the Cycle Count X1 internal register
#define RM3100_CCX0_REG 0x05   // Hexadecimal address for the Cycle Count X0 internal register

#define RM3100_MEASUREMENT_REG 0xA4 // Hexadecimal address for the Cycle Count X0 internal register

#define RM3100_WHO_AM_I_REG_VALUE 0x22 // Hexadecimal address for the Revid internal register

#define RM3100_READ_ASYNC 1

#endif

#endif
