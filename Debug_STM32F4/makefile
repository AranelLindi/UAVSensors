################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include .metadata/.plugins/org.eclipse.cdt.make.core/subdir.mk
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
SECONDARY_FLASH += \
stm32f4.hex \

SECONDARY_LIST += \
stm32f4.lst \

SECONDARY_SIZE += \
stm32f4.siz \


# All Target
all: stm32f4.elf secondary-outputs

# Tool invocations
stm32f4.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: ARM Sourcery Linux GCC C++ Linker'
	/opt/gcc-arm-none-eabi-4_7-2013q3/bin/arm-none-eabi-gcc -T"/home/rodos/Discovery_WorkSpace_Updated/rodos/src/bare-metal/stm32f4/scripts/stm32_flash.ld" -nostartfiles -nodefaultlibs -nostdlib -Xlinker --gc-sections -L/home/info8/3DfloatSat/rodos/Debug_STM32F4 -L"/home/rodos/Discovery_WorkSpace_Updated/rodos/Debug_STM32F4" -L"/home/rodos/Discovery_WorkSpace_Updated/support_libs/Debug_STM32F4" -Wl,-Map,"stm32f4.map" -fno-unwind-tables -fno-asynchronous-unwind-tables -mcpu=cortex-m4 -mthumb -mfloat-abi=softfp -mfpu=fpv4-sp-d16 -g3 -gdwarf-2 -o "stm32f4.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

stm32f4.hex: stm32f4.elf
	@echo 'Invoking: ARM Sourcery Linux GNU Create Flash Image'
	arm-none-eabi-objcopy -O ihex "stm32f4.elf"  "stm32f4.hex"
	@echo 'Finished building: $@'
	@echo ' '

stm32f4.lst: stm32f4.elf
	@echo 'Invoking: ARM Sourcery Linux GNU Create Listing'
	arm-none-eabi-objdump -h -S "stm32f4.elf" > "stm32f4.lst"
	@echo 'Finished building: $@'
	@echo ' '

stm32f4.siz: stm32f4.elf
	@echo 'Invoking: ARM Sourcery Linux GNU Print Size'
	arm-none-eabi-size  --format=berkeley "stm32f4.elf"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(C_UPPER_DEPS)$(SECONDARY_LIST)$(SECONDARY_SIZE)$(C_DEPS)$(CC_DEPS)$(C++_DEPS)$(EXECUTABLES)$(OBJS)$(CXX_DEPS)$(SECONDARY_FLASH)$(ASM_DEPS)$(S_UPPER_DEPS)$(CPP_DEPS) stm32f4.elf
	-@echo ' '

secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_LIST) $(SECONDARY_SIZE)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
