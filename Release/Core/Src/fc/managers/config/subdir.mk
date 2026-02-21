################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/fc/managers/config/ConfigHelper.c \
../Core/Src/fc/managers/config/ConfigManager.c 

OBJS += \
./Core/Src/fc/managers/config/ConfigHelper.o \
./Core/Src/fc/managers/config/ConfigManager.o 

C_DEPS += \
./Core/Src/fc/managers/config/ConfigHelper.d \
./Core/Src/fc/managers/config/ConfigManager.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/fc/managers/config/%.o Core/Src/fc/managers/config/%.su Core/Src/fc/managers/config/%.cyclo: ../Core/Src/fc/managers/config/%.c Core/Src/fc/managers/config/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H723xx -DUSE_FULL_LL_DRIVER -DSTM32_THREAD_SAFE_STRATEGY=2 -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Core/ThreadSafe -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-fc-2f-managers-2f-config

clean-Core-2f-Src-2f-fc-2f-managers-2f-config:
	-$(RM) ./Core/Src/fc/managers/config/ConfigHelper.cyclo ./Core/Src/fc/managers/config/ConfigHelper.d ./Core/Src/fc/managers/config/ConfigHelper.o ./Core/Src/fc/managers/config/ConfigHelper.su ./Core/Src/fc/managers/config/ConfigManager.cyclo ./Core/Src/fc/managers/config/ConfigManager.d ./Core/Src/fc/managers/config/ConfigManager.o ./Core/Src/fc/managers/config/ConfigManager.su

.PHONY: clean-Core-2f-Src-2f-fc-2f-managers-2f-config

