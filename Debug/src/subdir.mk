################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/WifiTask.c \
../src/_write.c \
../src/fifo.c \
../src/main.c 

OBJS += \
./src/WifiTask.o \
./src/_write.o \
./src/fifo.o \
./src/main.o 

C_DEPS += \
./src/WifiTask.d \
./src/_write.d \
./src/fifo.d \
./src/main.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wunused -Wuninitialized -Wall -Wextra -Wmissing-declarations -Wconversion -Wpointer-arith -Wpadded -Wshadow -Wlogical-op -Waggregate-return -Wfloat-equal  -g3 -DDEBUG -DTRACE -DOS_USE_TRACE_ITM -I"../include" -I"../system/include" -I"../system/include/cmsis" -I"../system/include/LM3S8962" -I"/home/anis/Documents/Projects/workspace/SmartSprinkler/FreeRTOS/Source/include" -I"/home/anis/Documents/Projects/workspace/SmartSprinkler/FreeRTOS/Source/portable/GCC/ARM_CM3" -std=gnu11 -Wmissing-prototypes -Wstrict-prototypes -Wbad-function-cast -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


