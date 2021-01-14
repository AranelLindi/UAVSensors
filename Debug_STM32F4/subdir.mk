################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../TaskSPI.cpp 

OBJS += \
./TaskSPI.o 

CPP_DEPS += \
./TaskSPI.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Sourcery Linux GCC C++ Compiler'
	/opt/gcc-arm-none-eabi-4_7-2013q3/bin/arm-none-eabi-g++ -DSTM32F40_41xxx -DUSE_STM32_DISCOVERY -I"/home/rodos/Discovery_WorkSpace_Updated/rodos/src/bare-metal/stm32f4/STM32F4xx_StdPeriph_Driver/inc" -I"/home/rodos/Discovery_WorkSpace_Updated/rodos/src/bare-metal/stm32f4/CMSIS/Device/ST/STM32F4xx/Include" -I"/home/rodos/Discovery_WorkSpace_Updated/rodos/src/bare-metal/stm32f4/CMSIS/Include" -I"/home/rodos/Discovery_WorkSpace_Updated/rodos/src/bare-metal/stm32f4/hal" -I"/home/rodos/Discovery_WorkSpace_Updated/rodos/src/bare-metal/stm32f4" -I"/home/rodos/Discovery_WorkSpace_Updated/rodos/src/bare-metal-generic" -I"/home/rodos/Discovery_WorkSpace_Updated/rodos/src/independent/gateway" -I"/home/rodos/Discovery_WorkSpace_Updated/rodos/src/independent" -I"/home/rodos/Discovery_WorkSpace_Updated/rodos/api" -I"/home/rodos/Discovery_WorkSpace_Updated/rodos/default_usr_configs" -I"/home/rodos/Discovery_WorkSpace_Updated/rodos/api/hal" -I"/home/rodos/Discovery_WorkSpace_Updated/support_libs" -O0 -ffunction-sections -fdata-sections -Wall -Wa,-adhlns="$@.lst" -fno-exceptions -fno-rtti -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m4 -mthumb -mfloat-abi=softfp -mfpu=fpv4-sp-d16 -g3 -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


