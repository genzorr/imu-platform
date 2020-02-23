################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/library/MadgwickAHRS.c \
../src/library/quaternion.c \
../src/library/vector.c \
../src/library/xprintf.c 

OBJS += \
./src/library/MadgwickAHRS.o \
./src/library/quaternion.o \
./src/library/vector.o \
./src/library/xprintf.o 

C_DEPS += \
./src/library/MadgwickAHRS.d \
./src/library/quaternion.d \
./src/library/vector.d \
./src/library/xprintf.d 


# Each subdirectory must supply rules for building sources it contributes
src/library/%.o: ../src/library/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM GNU C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -march=armv7e-m -mthumb -mlittle-endian -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DUSE_HAL_DRIVER -DUSE_FULL_ASSERT -DOS_USE_SEMIHOSTING -DTRACE -DOS_USE_TRACE_SEMIHOSTING_DEBUG -DSTM32F407xx -DHSE_VALUE=8000000 -I"../include" -I"../system/include" -I"../system/include/cmsis" -I"/home/michael/stm/stm32f4-imu/board/system/include/stm32f4-hal" -I"/home/michael/stm/stm32f4-imu/board/src" -I"/home/michael/stm/stm32f4-imu/board/src/drivers" -I"/home/michael/stm/stm32f4-imu/board/src/library" -I"/home/michael/stm/stm32f4-imu/board/src/tasks" -I"/home/michael/stm/stm32f4-imu/board/message_definitions" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


