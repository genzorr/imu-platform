################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/tasks/sensors.c 

OBJS += \
./src/tasks/sensors.o 

C_DEPS += \
./src/tasks/sensors.d 


# Each subdirectory must supply rules for building sources it contributes
src/tasks/%.o: ../src/tasks/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM GNU C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=soft -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DUSE_FULL_ASSERT -DTRACE -DOS_USE_TRACE_SEMIHOSTING_DEBUG -DSTM32F407xx -DUSE_HAL_DRIVER -DHSE_VALUE=8000000 -I"../include" -I"../system/include" -I"../system/include/cmsis" -I"../system/include/stm32f4-hal" -I"/home/michael/stm/stm32f4-imu/src" -I"/home/michael/stm/stm32f4-imu/src/drivers" -I"/home/michael/stm/stm32f4-imu/src/library" -I"/home/michael/stm/stm32f4-imu/src/tasks" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


