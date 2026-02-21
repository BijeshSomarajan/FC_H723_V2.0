################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/fc/dsp/BiQuadFilter.c \
../Core/Src/fc/dsp/CircularQueue.c \
../Core/Src/fc/dsp/CircularQueue2D.c \
../Core/Src/fc/dsp/FFT.c \
../Core/Src/fc/dsp/HighPassFilter.c \
../Core/Src/fc/dsp/LeakyIntegrationFilter.c \
../Core/Src/fc/dsp/LowPassFilter.c \
../Core/Src/fc/dsp/TimeDecayFilter.c 

OBJS += \
./Core/Src/fc/dsp/BiQuadFilter.o \
./Core/Src/fc/dsp/CircularQueue.o \
./Core/Src/fc/dsp/CircularQueue2D.o \
./Core/Src/fc/dsp/FFT.o \
./Core/Src/fc/dsp/HighPassFilter.o \
./Core/Src/fc/dsp/LeakyIntegrationFilter.o \
./Core/Src/fc/dsp/LowPassFilter.o \
./Core/Src/fc/dsp/TimeDecayFilter.o 

C_DEPS += \
./Core/Src/fc/dsp/BiQuadFilter.d \
./Core/Src/fc/dsp/CircularQueue.d \
./Core/Src/fc/dsp/CircularQueue2D.d \
./Core/Src/fc/dsp/FFT.d \
./Core/Src/fc/dsp/HighPassFilter.d \
./Core/Src/fc/dsp/LeakyIntegrationFilter.d \
./Core/Src/fc/dsp/LowPassFilter.d \
./Core/Src/fc/dsp/TimeDecayFilter.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/fc/dsp/%.o Core/Src/fc/dsp/%.su Core/Src/fc/dsp/%.cyclo: ../Core/Src/fc/dsp/%.c Core/Src/fc/dsp/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H723xx -DUSE_FULL_LL_DRIVER -DSTM32_THREAD_SAFE_STRATEGY=2 -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Core/ThreadSafe -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-fc-2f-dsp

clean-Core-2f-Src-2f-fc-2f-dsp:
	-$(RM) ./Core/Src/fc/dsp/BiQuadFilter.cyclo ./Core/Src/fc/dsp/BiQuadFilter.d ./Core/Src/fc/dsp/BiQuadFilter.o ./Core/Src/fc/dsp/BiQuadFilter.su ./Core/Src/fc/dsp/CircularQueue.cyclo ./Core/Src/fc/dsp/CircularQueue.d ./Core/Src/fc/dsp/CircularQueue.o ./Core/Src/fc/dsp/CircularQueue.su ./Core/Src/fc/dsp/CircularQueue2D.cyclo ./Core/Src/fc/dsp/CircularQueue2D.d ./Core/Src/fc/dsp/CircularQueue2D.o ./Core/Src/fc/dsp/CircularQueue2D.su ./Core/Src/fc/dsp/FFT.cyclo ./Core/Src/fc/dsp/FFT.d ./Core/Src/fc/dsp/FFT.o ./Core/Src/fc/dsp/FFT.su ./Core/Src/fc/dsp/HighPassFilter.cyclo ./Core/Src/fc/dsp/HighPassFilter.d ./Core/Src/fc/dsp/HighPassFilter.o ./Core/Src/fc/dsp/HighPassFilter.su ./Core/Src/fc/dsp/LeakyIntegrationFilter.cyclo ./Core/Src/fc/dsp/LeakyIntegrationFilter.d ./Core/Src/fc/dsp/LeakyIntegrationFilter.o ./Core/Src/fc/dsp/LeakyIntegrationFilter.su ./Core/Src/fc/dsp/LowPassFilter.cyclo ./Core/Src/fc/dsp/LowPassFilter.d ./Core/Src/fc/dsp/LowPassFilter.o ./Core/Src/fc/dsp/LowPassFilter.su ./Core/Src/fc/dsp/TimeDecayFilter.cyclo ./Core/Src/fc/dsp/TimeDecayFilter.d ./Core/Src/fc/dsp/TimeDecayFilter.o ./Core/Src/fc/dsp/TimeDecayFilter.su

.PHONY: clean-Core-2f-Src-2f-fc-2f-dsp

