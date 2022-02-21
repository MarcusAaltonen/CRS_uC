################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32L0xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
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


BUILD_ARTIFACT_NAME := CRS RC Car uC UFQ 32-pin
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
CRS\ RC\ Car\ uC\ UFQ\ 32-pin.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
CRS\ RC\ Car\ uC\ UFQ\ 32-pin.list \

OBJCOPY_BIN += \
CRS\ RC\ Car\ uC\ UFQ\ 32-pin.bin \


# All Target
all: main-build

# Main-build Target
main-build: CRS\ RC\ Car\ uC\ UFQ\ 32-pin.elf secondary-outputs

# Tool invocations
CRS\ RC\ Car\ uC\ UFQ\ 32-pin.elf: $(OBJS) $(USER_OBJS) C:\Users\Marcus\Documents\GitHub\CRS_uC\CRS\ RC\ Car\ uC\ UFQ\ 32-pin\STM32L011K4UX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "CRS RC Car uC UFQ 32-pin.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m0plus -T"C:\Users\Marcus\Documents\GitHub\CRS_uC\CRS RC Car uC UFQ 32-pin\STM32L011K4UX_FLASH.ld" --specs=nosys.specs -Wl,-Map="CRS RC Car uC UFQ 32-pin.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

CRS\ RC\ Car\ uC\ UFQ\ 32-pin.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "CRS RC Car uC UFQ 32-pin.list"
	@echo 'Finished building: $@'
	@echo ' '

CRS\ RC\ Car\ uC\ UFQ\ 32-pin.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "CRS RC Car uC UFQ 32-pin.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) CRS\ RC\ Car\ uC\ UFQ\ 32-pin.bin CRS\ RC\ Car\ uC\ UFQ\ 32-pin.elf CRS\ RC\ Car\ uC\ UFQ\ 32-pin.list CRS\\ RC\\ Car\\ uC\\ UFQ\\ 32-pin.elf default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
