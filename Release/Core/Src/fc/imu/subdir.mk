################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/fc/imu/IMU.c \
../Core/Src/fc/imu/MadgwickFilter.c \
../Core/Src/fc/imu/MahonyFilter.c 

OBJS += \
./Core/Src/fc/imu/IMU.o \
./Core/Src/fc/imu/MadgwickFilter.o \
./Core/Src/fc/imu/MahonyFilter.o 

C_DEPS += \
./Core/Src/fc/imu/IMU.d \
./Core/Src/fc/imu/MadgwickFilter.d \
./Core/Src/fc/imu/MahonyFilter.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/fc/imu/%.o Core/Src/fc/imu/%.su Core/Src/fc/imu/%.cyclo: ../Core/Src/fc/imu/%.c Core/Src/fc/imu/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H723xx -DUSE_FULL_LL_DRIVER -DSTM32_THREAD_SAFE_STRATEGY=2 -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Core/ThreadSafe -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-fc-2f-imu

clean-Core-2f-Src-2f-fc-2f-imu:
	-$(RM) ./Core/Src/fc/imu/IMU.cyclo ./Core/Src/fc/imu/IMU.d ./Core/Src/fc/imu/IMU.o ./Core/Src/fc/imu/IMU.su ./Core/Src/fc/imu/MadgwickFilter.cyclo ./Core/Src/fc/imu/MadgwickFilter.d ./Core/Src/fc/imu/MadgwickFilter.o ./Core/Src/fc/imu/MadgwickFilter.su ./Core/Src/fc/imu/MahonyFilter.cyclo ./Core/Src/fc/imu/MahonyFilter.d ./Core/Src/fc/imu/MahonyFilter.o ./Core/Src/fc/imu/MahonyFilter.su

.PHONY: clean-Core-2f-Src-2f-fc-2f-imu

