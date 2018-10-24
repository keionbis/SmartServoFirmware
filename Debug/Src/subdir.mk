################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/stm32l4xx_hal_msp.c \
../Src/system_stm32l4xx.c 

CPP_SRCS += \
../Src/eeprom.cpp \
../Src/main.cpp \
../Src/stm32l4xx_it.cpp 

OBJS += \
./Src/eeprom.o \
./Src/main.o \
./Src/stm32l4xx_hal_msp.o \
./Src/stm32l4xx_it.o \
./Src/system_stm32l4xx.o 

C_DEPS += \
./Src/stm32l4xx_hal_msp.d \
./Src/system_stm32l4xx.d 

CPP_DEPS += \
./Src/eeprom.d \
./Src/main.d \
./Src/stm32l4xx_it.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: MCU G++ Compiler'
	@echo $(PWD)
	arm-none-eabi-g++ -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32L432xx -I"/home/kbisland/idek2/Inc" -I"/home/kbisland/idek2/Drivers/STM32L4xx_HAL_Driver/Inc" -I"/home/kbisland/idek2/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy" -I"/home/kbisland/idek2/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"/home/kbisland/idek2/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fno-exceptions -fno-rtti -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32L432xx -I"/home/kbisland/idek2/Inc" -I"/home/kbisland/idek2/Drivers/STM32L4xx_HAL_Driver/Inc" -I"/home/kbisland/idek2/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy" -I"/home/kbisland/idek2/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"/home/kbisland/idek2/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


