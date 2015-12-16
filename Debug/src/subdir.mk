################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/Timer.cpp \
../src/main.cpp 

C_SRCS += \
../src/_write.c 

OBJS += \
./src/Timer.o \
./src/_write.o \
./src/main.o 

C_DEPS += \
./src/_write.d 

CPP_DEPS += \
./src/Timer.d \
./src/main.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C++ Compiler'
	arm-none-eabi-g++ -mcpu=cortex-m3 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wunused -Wuninitialized -Wall -Wextra -Wmissing-declarations -Wconversion -Wpointer-arith -Wpadded -Wshadow -Wlogical-op -Waggregate-return -Wfloat-equal  -g3 -DDEBUG -DTRACE -DOS_USE_TRACE_ITM -I/home/anis/Documents/Projects/workspace/SmartSprinkler/include -I/home/anis/Documents/Projects/workspace/SmartSprinkler/system/include -I/home/anis/Documents/Projects/workspace/SmartSprinkler/system/include/cmsis -I/home/anis/Documents/Projects/workspace/SmartSprinkler/system/include/LM3S8962 -I/home/anis/Documents/Projects/workspace/SmartSprinkler/FreeRTOS -I/home/anis/Documents/Projects/workspace/SmartSprinkler/FreeRTOS/Source/include -I/home/anis/Documents/Projects/workspace/SmartSprinkler/FreeRTOS/Source/portable/GCC/ARM_CM3 -std=gnu++11 -fabi-version=0 -fno-exceptions -fno-rtti -fno-use-cxa-atexit -fno-threadsafe-statics -Wabi -Wctor-dtor-privacy -Wnoexcept -Wnon-virtual-dtor -Wstrict-null-sentinel -Wsign-promo -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wunused -Wuninitialized -Wall -Wextra -Wmissing-declarations -Wconversion -Wpointer-arith -Wpadded -Wshadow -Wlogical-op -Waggregate-return -Wfloat-equal  -g3 -DDEBUG -DTRACE -DOS_USE_TRACE_ITM -I/home/anis/Documents/Projects/workspace/SmartSprinkler/include -I/home/anis/Documents/Projects/workspace/SmartSprinkler/system/include -I/home/anis/Documents/Projects/workspace/SmartSprinkler/system/include/cmsis -I/home/anis/Documents/Projects/workspace/SmartSprinkler/system/include/LM3S8962 -I/home/anis/Documents/Projects/workspace/SmartSprinkler/FreeRTOS -I/home/anis/Documents/Projects/workspace/SmartSprinkler/FreeRTOS/Source/include -I/home/anis/Documents/Projects/workspace/SmartSprinkler/FreeRTOS/Source/portable/GCC/ARM_CM3 -std=gnu11 -Wmissing-prototypes -Wstrict-prototypes -Wbad-function-cast -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


