################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../system/src/Drivers/adc.c \
../system/src/Drivers/can.c \
../system/src/Drivers/comp.c \
../system/src/Drivers/cpu.c \
../system/src/Drivers/eeprom.c \
../system/src/Drivers/epi.c \
../system/src/Drivers/ethernet.c \
../system/src/Drivers/fan.c \
../system/src/Drivers/flash.c \
../system/src/Drivers/fpu.c \
../system/src/Drivers/gpio.c \
../system/src/Drivers/hibernate.c \
../system/src/Drivers/i2c.c \
../system/src/Drivers/i2s.c \
../system/src/Drivers/interrupt.c \
../system/src/Drivers/lpc.c \
../system/src/Drivers/mpu.c \
../system/src/Drivers/peci.c \
../system/src/Drivers/pwm.c \
../system/src/Drivers/qei.c \
../system/src/Drivers/ssi.c \
../system/src/Drivers/sysctl.c \
../system/src/Drivers/sysexc.c \
../system/src/Drivers/systick.c \
../system/src/Drivers/timer.c \
../system/src/Drivers/uart.c \
../system/src/Drivers/udma.c \
../system/src/Drivers/usb.c \
../system/src/Drivers/watchdog.c 

OBJS += \
./system/src/Drivers/adc.o \
./system/src/Drivers/can.o \
./system/src/Drivers/comp.o \
./system/src/Drivers/cpu.o \
./system/src/Drivers/eeprom.o \
./system/src/Drivers/epi.o \
./system/src/Drivers/ethernet.o \
./system/src/Drivers/fan.o \
./system/src/Drivers/flash.o \
./system/src/Drivers/fpu.o \
./system/src/Drivers/gpio.o \
./system/src/Drivers/hibernate.o \
./system/src/Drivers/i2c.o \
./system/src/Drivers/i2s.o \
./system/src/Drivers/interrupt.o \
./system/src/Drivers/lpc.o \
./system/src/Drivers/mpu.o \
./system/src/Drivers/peci.o \
./system/src/Drivers/pwm.o \
./system/src/Drivers/qei.o \
./system/src/Drivers/ssi.o \
./system/src/Drivers/sysctl.o \
./system/src/Drivers/sysexc.o \
./system/src/Drivers/systick.o \
./system/src/Drivers/timer.o \
./system/src/Drivers/uart.o \
./system/src/Drivers/udma.o \
./system/src/Drivers/usb.o \
./system/src/Drivers/watchdog.o 

C_DEPS += \
./system/src/Drivers/adc.d \
./system/src/Drivers/can.d \
./system/src/Drivers/comp.d \
./system/src/Drivers/cpu.d \
./system/src/Drivers/eeprom.d \
./system/src/Drivers/epi.d \
./system/src/Drivers/ethernet.d \
./system/src/Drivers/fan.d \
./system/src/Drivers/flash.d \
./system/src/Drivers/fpu.d \
./system/src/Drivers/gpio.d \
./system/src/Drivers/hibernate.d \
./system/src/Drivers/i2c.d \
./system/src/Drivers/i2s.d \
./system/src/Drivers/interrupt.d \
./system/src/Drivers/lpc.d \
./system/src/Drivers/mpu.d \
./system/src/Drivers/peci.d \
./system/src/Drivers/pwm.d \
./system/src/Drivers/qei.d \
./system/src/Drivers/ssi.d \
./system/src/Drivers/sysctl.d \
./system/src/Drivers/sysexc.d \
./system/src/Drivers/systick.d \
./system/src/Drivers/timer.d \
./system/src/Drivers/uart.d \
./system/src/Drivers/udma.d \
./system/src/Drivers/usb.d \
./system/src/Drivers/watchdog.d 


# Each subdirectory must supply rules for building sources it contributes
system/src/Drivers/%.o: ../system/src/Drivers/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wunused -Wuninitialized -Wall -Wextra -Wmissing-declarations -Wconversion -Wpointer-arith -Wpadded -Wshadow -Wlogical-op -Waggregate-return -Wfloat-equal  -g3 -DDEBUG -DTRACE -DOS_USE_TRACE_ITM -I"../include" -I"/home/anis/Documents/Projects/workspace/blinky/system/include/Drivers" -I"../system/include" -I"../system/include/cmsis" -I"../system/include/LM3S8962" -std=gnu11 -Wmissing-prototypes -Wstrict-prototypes -Wbad-function-cast -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


