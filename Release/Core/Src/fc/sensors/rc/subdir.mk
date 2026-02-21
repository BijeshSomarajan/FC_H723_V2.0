################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/fc/sensors/rc/RCSensor.c 

OBJS += \
./Core/Src/fc/sensors/rc/RCSensor.o 

C_DEPS += \
./Core/Src/fc/sensors/rc/RCSensor.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/fc/sensors/rc/%.o Core/Src/fc/sensors/rc/%.su Core/Src/fc/sensors/rc/%.cyclo: ../Core/Src/fc/sensors/rc/%.c Core/Src/fc/sensors/rc/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H723xx -DUSE_FULL_LL_DRIVER -DSTM32_THREAD_SAFE_STRATEGY=2 -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Core/ThreadSafe -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-fc-2f-sensors-2f-rc

clean-Core-2f-Src-2f-fc-2f-sensors-2f-rc:
	-$(RM) ./Core/Src/fc/sensors/rc/RCSensor.cyclo ./Core/Src/fc/sensors/rc/RCSensor.d ./Core/Src/fc/sensors/rc/RCSensor.o ./Core/Src/fc/sensors/rc/RCSensor.su

.PHONY: clean-Core-2f-Src-2f-fc-2f-sensors-2f-rc

