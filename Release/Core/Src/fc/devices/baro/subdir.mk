################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/fc/devices/baro/baroCommon.c 

OBJS += \
./Core/Src/fc/devices/baro/baroCommon.o 

C_DEPS += \
./Core/Src/fc/devices/baro/baroCommon.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/fc/devices/baro/%.o Core/Src/fc/devices/baro/%.su Core/Src/fc/devices/baro/%.cyclo: ../Core/Src/fc/devices/baro/%.c Core/Src/fc/devices/baro/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H723xx -DUSE_FULL_LL_DRIVER -DSTM32_THREAD_SAFE_STRATEGY=2 -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Core/ThreadSafe -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-fc-2f-devices-2f-baro

clean-Core-2f-Src-2f-fc-2f-devices-2f-baro:
	-$(RM) ./Core/Src/fc/devices/baro/baroCommon.cyclo ./Core/Src/fc/devices/baro/baroCommon.d ./Core/Src/fc/devices/baro/baroCommon.o ./Core/Src/fc/devices/baro/baroCommon.su

.PHONY: clean-Core-2f-Src-2f-fc-2f-devices-2f-baro

