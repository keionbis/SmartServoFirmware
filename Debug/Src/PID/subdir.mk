################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Src/PID/PID.cpp 

OBJS += \
./Src/PID/PID.o 

CPP_DEPS += \
./Src/PID/PID.d 


# Each subdirectory must supply rules for building sources it contributes
Src/PID/%.o: ../Src/PID/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: MCU G++ Compiler'
	@echo $(PWD)
	arm-none-eabi-g++ -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32L432xx -I"/home/kbisland/idek2/Inc" -I"/home/kbisland/idek2/Drivers/STM32L4xx_HAL_Driver/Inc" -I"/home/kbisland/idek2/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy" -I"/home/kbisland/idek2/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"/home/kbisland/idek2/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fno-exceptions -fno-rtti -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


