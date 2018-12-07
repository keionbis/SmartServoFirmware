################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Src/MA702/MA702.cpp 

OBJS += \
./Src/MA702/MA702.o 

CPP_DEPS += \
./Src/MA702/MA702.d 


# Each subdirectory must supply rules for building sources it contributes
Src/MA702/%.o: ../Src/MA702/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: MCU G++ Compiler'
	@echo $(PWD)
	arm-none-eabi-g++ -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -I"/home/kbisland/idek2/Inc" -I"/home/kbisland/idek2/Drivers/STM32L4xx_HAL_Driver/Inc" -I"/home/kbisland/idek2/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy" -I"/home/kbisland/idek2/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"/home/kbisland/idek2/Drivers/CMSIS/Include"  -Og -Wall -fmessage-length=0 -ffunction-sections -c -fno-exceptions -fno-rtti __weak="__attribute__((weak))" __packed="__attribute__((__packed__))" USE_HAL_DRIVER STM32L432xx -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


