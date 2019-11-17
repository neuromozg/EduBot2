################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
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

# Add inputs and outputs from these tool invocations to the build variables 
LSS += \
EduBot2AVR.lss \

FLASH_IMAGE += \
EduBot2AVR.hex \

EEPROM_IMAGE += \
EduBot2AVR.eep \

SIZEDUMMY += \
sizedummy \


# All Target
all: EduBot2AVR.elf secondary-outputs

# Tool invocations
EduBot2AVR.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C Linker'
	avr-gcc -Wl,-Map,EduBot2AVR.map -mmcu=atmega328p -o "EduBot2AVR.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

EduBot2AVR.lss: EduBot2AVR.elf
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S EduBot2AVR.elf  >"EduBot2AVR.lss"
	@echo 'Finished building: $@'
	@echo ' '

EduBot2AVR.hex: EduBot2AVR.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -R .fuse -R .lock -R .signature -O ihex EduBot2AVR.elf  "EduBot2AVR.hex"
	@echo 'Finished building: $@'
	@echo ' '

EduBot2AVR.eep: EduBot2AVR.elf
	@echo 'Create eeprom image (ihex format)'
	-avr-objcopy -j .eeprom --no-change-warnings --change-section-lma .eeprom=0 -O ihex EduBot2AVR.elf  "EduBot2AVR.eep"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: EduBot2AVR.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=atmega328p EduBot2AVR.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(FLASH_IMAGE)$(ELFS)$(OBJS)$(ASM_DEPS)$(EEPROM_IMAGE)$(S_DEPS)$(SIZEDUMMY)$(S_UPPER_DEPS)$(LSS)$(C_DEPS) EduBot2AVR.elf
	-@echo ' '

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(EEPROM_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
