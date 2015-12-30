################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../system/src/Drivers/ESP8266.cpp \
../system/src/Drivers/Usart.cpp 

OBJS += \
./system/src/Drivers/ESP8266.o \
./system/src/Drivers/Usart.o 

CPP_DEPS += \
./system/src/Drivers/ESP8266.d \
./system/src/Drivers/Usart.d 


# Each subdirectory must supply rules for building sources it contributes
system/src/Drivers/%.o: ../system/src/Drivers/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C++ Compiler'
	arm-none-eabi-g++ -mcpu=cortex-m3 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wunused -Wuninitialized -Wall -Wextra -Wmissing-declarations -Wconversion -Wpointer-arith -Wpadded -Wshadow -Wlogical-op -Waggregate-return -Wfloat-equal  -g3 -DDEBUG -DTRACE -DOS_USE_TRACE_ITM -I/home/anis/Documents/Projects/workspace/SmartSprinkler/include -I/home/anis/Documents/Projects/workspace/SmartSprinkler/system/include -I/home/anis/Documents/Projects/workspace/SmartSprinkler/system/include/cmsis -std=gnu++11 -fabi-version=0 -fno-exceptions -fno-rtti -fno-use-cxa-atexit -fno-threadsafe-statics -Wabi -Wctor-dtor-privacy -Wnoexcept -Wnon-virtual-dtor -Wstrict-null-sentinel -Wsign-promo -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


