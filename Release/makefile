################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include utility/subdir.mk
-include pcint/subdir.mk
-include Arduino/avr-libc/subdir.mk
-include Arduino/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
LSS += \
DesimaETHER.lss \

FLASH_IMAGE += \
DesimaETHER.hex \

EEPROM_IMAGE += \
DesimaETHER.eep \

SIZEDUMMY += \
sizedummy \


# All Target
all: DesimaETHER.elf secondary-outputs

# Tool invocations
DesimaETHER.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C++ Linker'
	avr-g++ -s -Os -o"DesimaETHER.elf" $(OBJS) $(USER_OBJS) $(LIBS) -lm -Wl,-Map,DesimaETHER.map,--cref -mrelax -Wl,--gc-sections -Wl,--gc-sections -mmcu=atmega644p
	@echo 'Finished building target: $@'
	@echo ' '

DesimaETHER.lss: DesimaETHER.elf
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S DesimaETHER.elf  >"DesimaETHER.lss"
	@echo 'Finished building: $@'
	@echo ' '

DesimaETHER.hex: DesimaETHER.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -R .fuse -R .lock -R .signature -O ihex DesimaETHER.elf  "DesimaETHER.hex"
	@echo 'Finished building: $@'
	@echo ' '

DesimaETHER.eep: DesimaETHER.elf
	@echo 'Create eeprom image (ihex format)'
	-avr-objcopy -j .eeprom --no-change-warnings --change-section-lma .eeprom=0 -O ihex DesimaETHER.elf  "DesimaETHER.eep"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: DesimaETHER.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=atmega644p DesimaETHER.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(C_UPPER_DEPS)$(S_DEPS)$(PDE_DEPS)$(LSS)$(C_DEPS)$(FLASH_IMAGE)$(CC_DEPS)$(ELFS)$(C++_DEPS)$(OBJS)$(CXX_DEPS)$(ASM_DEPS)$(EEPROM_IMAGE)$(INO_DEPS)$(SIZEDUMMY)$(S_UPPER_DEPS)$(CPP_DEPS) DesimaETHER.elf
	-@echo ' '

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(EEPROM_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
