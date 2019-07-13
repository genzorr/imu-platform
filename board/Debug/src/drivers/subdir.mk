################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/drivers/MPU9255.c \
../src/drivers/nRF24L01.c 

OBJS += \
./src/drivers/MPU9255.o \
./src/drivers/nRF24L01.o 

C_DEPS += \
./src/drivers/MPU9255.d \
./src/drivers/nRF24L01.d 


# Each subdirectory must supply rules for building sources it contributes
src/drivers/%.o: ../src/drivers/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM GNU C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=soft -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DUSE_FULL_ASSERT -DTRACE -DOS_USE_TRACE_SEMIHOSTING_DEBUG -DSTM32F407xx -DUSE_HAL_DRIVER -DHSE_VALUE=8000000 -I"../include" -I"../system/include" -I"../system/include/cmsis" -I../system/include/board -I"/home/michael/stm/stm32f4-imu/board/system/src/stm32f4-hal" -I"/home/michael/stm/stm32f4-imu/board/system/include/stm32f4-hal" -I"/home/michael/stm/stm32f4-imu/board/src" -I"/home/michael/stm/stm32f4-imu/board/src/drivers" -I"/home/michael/stm/stm32f4-imu/board/src/library" -I"/home/michael/stm/stm32f4-imu/board/src/tasks" -I"/home/michael/stm/stm32f4-imu/board/src/tasks" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


