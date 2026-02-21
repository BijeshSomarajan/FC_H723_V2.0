################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/fc/timers/DWTTimer.c \
../Core/Src/fc/timers/DelayTimer.c \
../Core/Src/fc/timers/DeltaTimer.c \
../Core/Src/fc/timers/GPTimer24.c \
../Core/Src/fc/timers/GPTimer3.c \
../Core/Src/fc/timers/GPTimer4.c \
../Core/Src/fc/timers/GPTimer6.c \
../Core/Src/fc/timers/GPTimer7.c \
../Core/Src/fc/timers/Scheduler.c 

OBJS += \
./Core/Src/fc/timers/DWTTimer.o \
./Core/Src/fc/timers/DelayTimer.o \
./Core/Src/fc/timers/DeltaTimer.o \
./Core/Src/fc/timers/GPTimer24.o \
./Core/Src/fc/timers/GPTimer3.o \
./Core/Src/fc/timers/GPTimer4.o \
./Core/Src/fc/timers/GPTimer6.o \
./Core/Src/fc/timers/GPTimer7.o \
./Core/Src/fc/timers/Scheduler.o 

C_DEPS += \
./Core/Src/fc/timers/DWTTimer.d \
./Core/Src/fc/timers/DelayTimer.d \
./Core/Src/fc/timers/DeltaTimer.d \
./Core/Src/fc/timers/GPTimer24.d \
./Core/Src/fc/timers/GPTimer3.d \
./Core/Src/fc/timers/GPTimer4.d \
./Core/Src/fc/timers/GPTimer6.d \
./Core/Src/fc/timers/GPTimer7.d \
./Core/Src/fc/timers/Scheduler.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/fc/timers/%.o Core/Src/fc/timers/%.su Core/Src/fc/timers/%.cyclo: ../Core/Src/fc/timers/%.c Core/Src/fc/timers/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H723xx -DUSE_FULL_LL_DRIVER -DSTM32_THREAD_SAFE_STRATEGY=2 -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Core/ThreadSafe -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-fc-2f-timers

clean-Core-2f-Src-2f-fc-2f-timers:
	-$(RM) ./Core/Src/fc/timers/DWTTimer.cyclo ./Core/Src/fc/timers/DWTTimer.d ./Core/Src/fc/timers/DWTTimer.o ./Core/Src/fc/timers/DWTTimer.su ./Core/Src/fc/timers/DelayTimer.cyclo ./Core/Src/fc/timers/DelayTimer.d ./Core/Src/fc/timers/DelayTimer.o ./Core/Src/fc/timers/DelayTimer.su ./Core/Src/fc/timers/DeltaTimer.cyclo ./Core/Src/fc/timers/DeltaTimer.d ./Core/Src/fc/timers/DeltaTimer.o ./Core/Src/fc/timers/DeltaTimer.su ./Core/Src/fc/timers/GPTimer24.cyclo ./Core/Src/fc/timers/GPTimer24.d ./Core/Src/fc/timers/GPTimer24.o ./Core/Src/fc/timers/GPTimer24.su ./Core/Src/fc/timers/GPTimer3.cyclo ./Core/Src/fc/timers/GPTimer3.d ./Core/Src/fc/timers/GPTimer3.o ./Core/Src/fc/timers/GPTimer3.su ./Core/Src/fc/timers/GPTimer4.cyclo ./Core/Src/fc/timers/GPTimer4.d ./Core/Src/fc/timers/GPTimer4.o ./Core/Src/fc/timers/GPTimer4.su ./Core/Src/fc/timers/GPTimer6.cyclo ./Core/Src/fc/timers/GPTimer6.d ./Core/Src/fc/timers/GPTimer6.o ./Core/Src/fc/timers/GPTimer6.su ./Core/Src/fc/timers/GPTimer7.cyclo ./Core/Src/fc/timers/GPTimer7.d ./Core/Src/fc/timers/GPTimer7.o ./Core/Src/fc/timers/GPTimer7.su ./Core/Src/fc/timers/Scheduler.cyclo ./Core/Src/fc/timers/Scheduler.d ./Core/Src/fc/timers/Scheduler.o ./Core/Src/fc/timers/Scheduler.su

.PHONY: clean-Core-2f-Src-2f-fc-2f-timers

