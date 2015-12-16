################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../system/src/cmsis/system_LM3S.c \
../system/src/cmsis/vectors_LM3S8962.c 

OBJS += \
./system/src/cmsis/system_LM3S.o \
./system/src/cmsis/vectors_LM3S8962.o 

C_DEPS += \
./system/src/cmsis/system_LM3S.d \
./system/src/cmsis/vectors_LM3S8962.d 


# Each subdirectory must supply rules for building sources it contributes
system/src/cmsis/%.o: ../system/src/cmsis/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wunused -Wuninitialized -Wall -Wextra -Wmissing-declarations -Wconversion -Wpointer-arith -Wpadded -Wshadow -Wlogical-op -Waggregate-return -Wfloat-equal  -g3 -DDEBUG -DTRACE -DOS_USE_TRACE_ITM -I/home/anis/Documents/Projects/workspace/SmartSprinkler/include -I/home/anis/Documents/Projects/workspace/SmartSprinkler/system/include -I/home/anis/Documents/Projects/workspace/SmartSprinkler/system/include/cmsis -I/home/anis/Documents/Projects/workspace/SmartSprinkler/system/include/LM3S8962 -I/home/anis/Documents/Projects/workspace/SmartSprinkler/FreeRTOS -I/home/anis/Documents/Projects/workspace/SmartSprinkler/FreeRTOS/Source/include -I/home/anis/Documents/Projects/workspace/SmartSprinkler/FreeRTOS/Source/portable/GCC/ARM_CM3 -std=gnu11 -Wmissing-prototypes -Wstrict-prototypes -Wbad-function-cast -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


