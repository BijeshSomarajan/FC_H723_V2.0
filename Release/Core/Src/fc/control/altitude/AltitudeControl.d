Core/Src/fc/control/altitude/AltitudeControl.o: \
 ../Core/Src/fc/control/altitude/AltitudeControl.c \
 ../Core/Src/fc/control/altitude/AltitudeControl.h \
 ../Core/Src/fc/control/altitude/../../calibration/Calibration.h \
 ../Core/Src/fc/control/altitude/../../calibration/../storage/flash/Flash.h \
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
 ../Core/Src/fc/control/altitude/../../memory/Memory.h \
 ../Core/Src/fc/control/altitude/../../managers/position/PositionManager.h \
 ../Core/Src/fc/control/altitude/../ControlData.h \
 ../Core/Src/fc/control/altitude/../Pid.h \
 ../Core/Src/fc/control/altitude/../../dsp/LowPassFilter.h \
 ../Core/Src/fc/control/altitude/../../dsp/../util/MathUtil.h \
 ../Core/Src/fc/control/altitude/../../dsp/../util/../memory/Memory.h
../Core/Src/fc/control/altitude/AltitudeControl.h:
../Core/Src/fc/control/altitude/../../calibration/Calibration.h:
../Core/Src/fc/control/altitude/../../calibration/../storage/flash/Flash.h:
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
../Core/Src/fc/control/altitude/../../memory/Memory.h:
../Core/Src/fc/control/altitude/../../managers/position/PositionManager.h:
../Core/Src/fc/control/altitude/../ControlData.h:
../Core/Src/fc/control/altitude/../Pid.h:
../Core/Src/fc/control/altitude/../../dsp/LowPassFilter.h:
../Core/Src/fc/control/altitude/../../dsp/../util/MathUtil.h:
../Core/Src/fc/control/altitude/../../dsp/../util/../memory/Memory.h:
