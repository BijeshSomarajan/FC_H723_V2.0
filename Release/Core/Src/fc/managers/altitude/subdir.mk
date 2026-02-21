################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/fc/managers/altitude/AltitudeManager.c 

OBJS += \
./Core/Src/fc/managers/altitude/AltitudeManager.o 

C_DEPS += \
./Core/Src/fc/managers/altitude/AltitudeManager.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/fc/managers/altitude/%.o Core/Src/fc/managers/altitude/%.su Core/Src/fc/managers/altitude/%.cyclo: ../Core/Src/fc/managers/altitude/%.c Core/Src/fc/managers/altitude/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H723xx -DUSE_FULL_LL_DRIVER -DSTM32_THREAD_SAFE_STRATEGY=2 -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Core/ThreadSafe -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-fc-2f-managers-2f-altitude

clean-Core-2f-Src-2f-fc-2f-managers-2f-altitude:
	-$(RM) ./Core/Src/fc/managers/altitude/AltitudeManager.cyclo ./Core/Src/fc/managers/altitude/AltitudeManager.d ./Core/Src/fc/managers/altitude/AltitudeManager.o ./Core/Src/fc/managers/altitude/AltitudeManager.su

.PHONY: clean-Core-2f-Src-2f-fc-2f-managers-2f-altitude

