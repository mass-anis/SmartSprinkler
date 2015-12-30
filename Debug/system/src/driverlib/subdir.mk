################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../system/src/driverlib/adc.c \
../system/src/driverlib/can.c \
../system/src/driverlib/comp.c \
../system/src/driverlib/cpu.c \
../system/src/driverlib/eeprom.c \
../system/src/driverlib/epi.c \
../system/src/driverlib/ethernet.c \
../system/src/driverlib/fan.c \
../system/src/driverlib/flash.c \
../system/src/driverlib/fpu.c \
../system/src/driverlib/gpio.c \
../system/src/driverlib/hibernate.c \
../system/src/driverlib/i2c.c \
../system/src/driverlib/i2s.c \
../system/src/driverlib/interrupt.c \
../system/src/driverlib/lpc.c \
../system/src/driverlib/mpu.c \
../system/src/driverlib/peci.c \
../system/src/driverlib/pwm.c \
../system/src/driverlib/qei.c \
../system/src/driverlib/ssi.c \
../system/src/driverlib/sysctl.c \
../system/src/driverlib/sysexc.c \
../system/src/driverlib/systick.c \
../system/src/driverlib/timer.c \
../system/src/driverlib/uart.c \
../system/src/driverlib/udma.c \
../system/src/driverlib/usb.c \
../system/src/driverlib/watchdog.c 

OBJS += \
./system/src/driverlib/adc.o \
./system/src/driverlib/can.o \
./system/src/driverlib/comp.o \
./system/src/driverlib/cpu.o \
./system/src/driverlib/eeprom.o \
./system/src/driverlib/epi.o \
./system/src/driverlib/ethernet.o \
./system/src/driverlib/fan.o \
./system/src/driverlib/flash.o \
./system/src/driverlib/fpu.o \
./system/src/driverlib/gpio.o \
./system/src/driverlib/hibernate.o \
./system/src/driverlib/i2c.o \
./system/src/driverlib/i2s.o \
./system/src/driverlib/interrupt.o \
./system/src/driverlib/lpc.o \
./system/src/driverlib/mpu.o \
./system/src/driverlib/peci.o \
./system/src/driverlib/pwm.o \
./system/src/driverlib/qei.o \
./system/src/driverlib/ssi.o \
./system/src/driverlib/sysctl.o \
./system/src/driverlib/sysexc.o \
./system/src/driverlib/systick.o \
./system/src/driverlib/timer.o \
./system/src/driverlib/uart.o \
./system/src/driverlib/udma.o \
./system/src/driverlib/usb.o \
./system/src/driverlib/watchdog.o 

C_DEPS += \
./system/src/driverlib/adc.d \
./system/src/driverlib/can.d \
./system/src/driverlib/comp.d \
./system/src/driverlib/cpu.d \
./system/src/driverlib/eeprom.d \
./system/src/driverlib/epi.d \
./system/src/driverlib/ethernet.d \
./system/src/driverlib/fan.d \
./system/src/driverlib/flash.d \
./system/src/driverlib/fpu.d \
./system/src/driverlib/gpio.d \
./system/src/driverlib/hibernate.d \
./system/src/driverlib/i2c.d \
./system/src/driverlib/i2s.d \
./system/src/driverlib/interrupt.d \
./system/src/driverlib/lpc.d \
./system/src/driverlib/mpu.d \
./system/src/driverlib/peci.d \
./system/src/driverlib/pwm.d \
./system/src/driverlib/qei.d \
./system/src/driverlib/ssi.d \
./system/src/driverlib/sysctl.d \
./system/src/driverlib/sysexc.d \
./system/src/driverlib/systick.d \
./system/src/driverlib/timer.d \
./system/src/driverlib/uart.d \
./system/src/driverlib/udma.d \
./system/src/driverlib/usb.d \
./system/src/driverlib/watchdog.d 


# Each subdirectory must supply rules for building sources it contributes
system/src/driverlib/%.o: ../system/src/driverlib/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wunused -Wuninitialized -Wall -Wextra -Wmissing-declarations -Wconversion -Wpointer-arith -Wpadded -Wshadow -Wlogical-op -Waggregate-return -Wfloat-equal  -g3 -DDEBUG -DTRACE -DOS_USE_TRACE_ITM -I"../include" -I"../system/include" -I"../system/include/cmsis" -I"../system/include/LM3S8962" -I"/home/anis/Documents/Projects/workspace/SmartSprinkler/FreeRTOS/Source/include" -I"/home/anis/Documents/Projects/workspace/SmartSprinkler/FreeRTOS/Source/portable/GCC/ARM_CM3" -std=gnu11 -Wmissing-prototypes -Wstrict-prototypes -Wbad-function-cast -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


