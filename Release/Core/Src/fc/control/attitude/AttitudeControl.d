Core/Src/fc/control/attitude/AttitudeControl.o: \
 ../Core/Src/fc/control/attitude/AttitudeControl.c \
 ../Core/Src/fc/control/attitude/AttitudeControl.h \
 ../Core/Src/fc/control/attitude/../../calibration/Calibration.h \
 ../Core/Src/fc/control/attitude/../../calibration/../storage/flash/Flash.h \
 ../Drivers/CMSIS/Device/ST/STM32H7xx/Include/stm32h7xx.h \
 ../Drivers/CMSIS/Device/ST/STM32H7xx/Include/stm32h723xx.h \
 ../Drivers/CMSIS/Include/core_cm7.h \
 ../Drivers/CMSIS/Include/cmsis_version.h \
 ../Drivers/CMSIS/Include/cmsis_compiler.h \
 ../Drivers/CMSIS/Include/cmsis_gcc.h \
 ../Drivers/CMSIS/Include/mpu_armv7.h \
 ../Drivers/CMSIS/Device/ST/STM32H7xx/Include/system_stm32h7xx.h \
 ../Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal.h \
 ../Core/Inc/stm32h7xx_hal_conf.h \
 ../Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_rcc.h \
 ../Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_def.h \
 ../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h \
 ../Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_rcc_ex.h \
 ../Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_gpio.h \
 ../Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_gpio_ex.h \
 ../Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_dma.h \
 ../Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_dma_ex.h \
 ../Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_mdma.h \
 ../Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_exti.h \
 ../Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_cortex.h \
 ../Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_flash.h \
 ../Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_flash_ex.h \
 ../Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_hsem.h \
 ../Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_i2c.h \
 ../Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_i2c_ex.h \
 ../Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_pwr.h \
 ../Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_pwr_ex.h \
 ../Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_tim.h \
 ../Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_tim_ex.h \
 ../Core/Src/fc/control/attitude/../../memory/Memory.h \
 ../Core/Src/fc/control/attitude/../../sensors/attitude/AttitudeSensor.h \
 ../Core/Src/fc/control/attitude/../../sensors/attitude/devices/AttitudeDevice.h \
 ../Core/Src/fc/control/attitude/../../sensors/attitude/devices/../../../memory/Memory.h \
 ../Core/Src/fc/control/attitude/../../status/FCStatus.h \
 ../Core/Src/fc/control/attitude/../ControlData.h \
 ../Core/Src/fc/control/attitude/../Pid.h \
 ../Core/Src/fc/control/attitude/../../dsp/LowPassFilter.h \
 ../Core/Src/fc/control/attitude/../../dsp/../util/MathUtil.h \
 ../Core/Src/fc/control/attitude/../../dsp/../util/../memory/Memory.h
../Core/Src/fc/control/attitude/AttitudeControl.h:
../Core/Src/fc/control/attitude/../../calibration/Calibration.h:
../Core/Src/fc/control/attitude/../../calibration/../storage/flash/Flash.h:
../Drivers/CMSIS/Device/ST/STM32H7xx/Include/stm32h7xx.h:
../Drivers/CMSIS/Device/ST/STM32H7xx/Include/stm32h723xx.h:
../Drivers/CMSIS/Include/core_cm7.h:
../Drivers/CMSIS/Include/cmsis_version.h:
../Drivers/CMSIS/Include/cmsis_compiler.h:
../Drivers/CMSIS/Include/cmsis_gcc.h:
../Drivers/CMSIS/Include/mpu_armv7.h:
../Drivers/CMSIS/Device/ST/STM32H7xx/Include/system_stm32h7xx.h:
../Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal.h:
../Core/Inc/stm32h7xx_hal_conf.h:
../Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_rcc.h:
../Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_def.h:
../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h:
../Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_rcc_ex.h:
../Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_gpio.h:
../Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_gpio_ex.h:
../Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_dma.h:
../Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_dma_ex.h:
../Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_mdma.h:
../Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_exti.h:
../Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_cortex.h:
../Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_flash.h:
../Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_flash_ex.h:
../Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_hsem.h:
../Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_i2c.h:
../Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_i2c_ex.h:
../Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_pwr.h:
../Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_pwr_ex.h:
../Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_tim.h:
../Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_tim_ex.h:
../Core/Src/fc/control/attitude/../../memory/Memory.h:
../Core/Src/fc/control/attitude/../../sensors/attitude/AttitudeSensor.h:
../Core/Src/fc/control/attitude/../../sensors/attitude/devices/AttitudeDevice.h:
../Core/Src/fc/control/attitude/../../sensors/attitude/devices/../../../memory/Memory.h:
../Core/Src/fc/control/attitude/../../status/FCStatus.h:
../Core/Src/fc/control/attitude/../ControlData.h:
../Core/Src/fc/control/attitude/../Pid.h:
../Core/Src/fc/control/attitude/../../dsp/LowPassFilter.h:
../Core/Src/fc/control/attitude/../../dsp/../util/MathUtil.h:
../Core/Src/fc/control/attitude/../../dsp/../util/../memory/Memory.h:
