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
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
LSS += \
avrtest.lss \

FLASH_IMAGE += \
avrtest.hex \

SIZEDUMMY += \
sizedummy \


# All Target
all: avrtest.elf secondary-outputs

# Tool invocations
avrtest.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C++ Linker'
	avr-g++ -Wl,-Map,avrtest.map,--cref -mmcu=attiny25 -o"avrtest.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

avrtest.lss: avrtest.elf
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S avrtest.elf  >"avrtest.lss"
	@echo 'Finished building: $@'
	@echo ' '

avrtest.hex: avrtest.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -O ihex avrtest.elf  "avrtest.hex"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: avrtest.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=attiny25 avrtest.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(ASM_DEPS)$(ELFS)$(S_UPPER_DEPS)$(C++_DEPS)$(LSS)$(S_DEPS)$(CC_DEPS)$(CPP_DEPS)$(CXX_DEPS)$(OBJS)$(FLASH_IMAGE)$(C_DEPS)$(C_UPPER_DEPS)$(SIZEDUMMY) avrtest.elf
	-@echo ' '

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
