################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/fc/io/spi/SPI2.c \
../Core/Src/fc/io/spi/SPI4.c \
../Core/Src/fc/io/spi/SPI6.c 

OBJS += \
./Core/Src/fc/io/spi/SPI2.o \
./Core/Src/fc/io/spi/SPI4.o \
./Core/Src/fc/io/spi/SPI6.o 

C_DEPS += \
./Core/Src/fc/io/spi/SPI2.d \
./Core/Src/fc/io/spi/SPI4.d \
./Core/Src/fc/io/spi/SPI6.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/fc/io/spi/%.o Core/Src/fc/io/spi/%.su Core/Src/fc/io/spi/%.cyclo: ../Core/Src/fc/io/spi/%.c Core/Src/fc/io/spi/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H723xx -DUSE_FULL_LL_DRIVER -DSTM32_THREAD_SAFE_STRATEGY=2 -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Core/ThreadSafe -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-fc-2f-io-2f-spi

clean-Core-2f-Src-2f-fc-2f-io-2f-spi:
	-$(RM) ./Core/Src/fc/io/spi/SPI2.cyclo ./Core/Src/fc/io/spi/SPI2.d ./Core/Src/fc/io/spi/SPI2.o ./Core/Src/fc/io/spi/SPI2.su ./Core/Src/fc/io/spi/SPI4.cyclo ./Core/Src/fc/io/spi/SPI4.d ./Core/Src/fc/io/spi/SPI4.o ./Core/Src/fc/io/spi/SPI4.su ./Core/Src/fc/io/spi/SPI6.cyclo ./Core/Src/fc/io/spi/SPI6.d ./Core/Src/fc/io/spi/SPI6.o ./Core/Src/fc/io/spi/SPI6.su

.PHONY: clean-Core-2f-Src-2f-fc-2f-io-2f-spi

