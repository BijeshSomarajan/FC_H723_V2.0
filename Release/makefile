################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32H7xx_HAL_Driver/Src/subdir.mk
-include Core/ThreadSafe/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/fc/util/subdir.mk
-include Core/Src/fc/timers/subdir.mk
-include Core/Src/fc/storage/flash/subdir.mk
-include Core/Src/fc/status/subdir.mk
-include Core/Src/fc/sensors/rc/devices/subdir.mk
-include Core/Src/fc/sensors/rc/subdir.mk
-include Core/Src/fc/sensors/attitude/noisefilter/subdir.mk
-include Core/Src/fc/sensors/attitude/devices/rm3100/subdir.mk
-include Core/Src/fc/sensors/attitude/devices/ism330/subdir.mk
-include Core/Src/fc/sensors/attitude/devices/subdir.mk
-include Core/Src/fc/sensors/attitude/subdir.mk
-include Core/Src/fc/sensors/altitude/devices/bmp581/subdir.mk
-include Core/Src/fc/sensors/altitude/devices/bmp390/subdir.mk
-include Core/Src/fc/sensors/altitude/devices/subdir.mk
-include Core/Src/fc/sensors/altitude/subdir.mk
-include Core/Src/fc/memory/subdir.mk
-include Core/Src/fc/managers/rc/subdir.mk
-include Core/Src/fc/managers/position/estimator/subdir.mk
-include Core/Src/fc/managers/position/subdir.mk
-include Core/Src/fc/managers/output/subdir.mk
-include Core/Src/fc/managers/indicator/subdir.mk
-include Core/Src/fc/managers/debug/subdir.mk
-include Core/Src/fc/managers/config/subdir.mk
-include Core/Src/fc/managers/attitude/subdir.mk
-include Core/Src/fc/managers/altitude/subdir.mk
-include Core/Src/fc/managers/subdir.mk
-include Core/Src/fc/logger/subdir.mk
-include Core/Src/fc/io/uart/subdir.mk
-include Core/Src/fc/io/spi/subdir.mk
-include Core/Src/fc/io/pwm/subdir.mk
-include Core/Src/fc/io/indicator/led/subdir.mk
-include Core/Src/fc/io/indicator/subdir.mk
-include Core/Src/fc/imu/subdir.mk
-include Core/Src/fc/dsp/subdir.mk
-include Core/Src/fc/control/attitude/subdir.mk
-include Core/Src/fc/control/altitude/subdir.mk
-include Core/Src/fc/control/subdir.mk
-include Core/Src/fc/calibration/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := FC_H723_V2.0
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
FC_H723_V2.0.elf \

MAP_FILES += \
FC_H723_V2.0.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
FC_H723_V2.0.list \


# All Target
all: main-build

# Main-build Target
main-build: FC_H723_V2.0.elf secondary-outputs

# Tool invocations
FC_H723_V2.0.elf FC_H723_V2.0.map: $(OBJS) $(USER_OBJS) C:\Bijesh\STM_H7_WS_2.0\FC_H723_V2.0\STM32H723VGTX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "FC_H723_V2.0.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m7 -T"C:\Bijesh\STM_H7_WS_2.0\FC_H723_V2.0\STM32H723VGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="FC_H723_V2.0.map" -Wl,--gc-sections -static -u _printf_float --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -u _printf_float -u _scanf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

FC_H723_V2.0.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "FC_H723_V2.0.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) FC_H723_V2.0.elf FC_H723_V2.0.list FC_H723_V2.0.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
