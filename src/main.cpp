//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

#include "diag/Trace.h"
#include "Timer.h"
#include "Drivers/Usart.h"

#include "driverlib/inc/hw_ints.h"
#include <driverlib/debug.h>
#include <driverlib/inc/hw_types.h>
#include <driverlib/sysctl.h>
#include <driverlib/gpio.h>
#include <driverlib/adc.h>
//#include "driverlib/uart.h"
#include "driverlib/interrupt.h"


// ----------------------------------------------------------------------------
//
// Print a greeting message on the trace device and enter a loop
// to count seconds.
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the ITM output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//
// ----------------------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"
#pragma GCC diagnostic ignored "-Wsign-conversion"

extern void __error__(char *pcFilename, unsigned long ulLine)
{
	trace_printf("error: in %s line %ul", pcFilename, ulLine);
}

char Buf[100];
int idx=0;
Usart u0(UART0);
//*****************************************************************************
//
// Send a string to the UART.
//
//*****************************************************************************
void UARTSend(const char *pucBuffer, unsigned long ulCount)
{
	//
	// Loop while there are more characters to send.
	//
	while (ulCount--)
	{
		//
		// Write the next character to the UART.
		//
		//UARTCharPutNonBlocking(UART0_BASE, *pucBuffer++);
		u0.CharPutNonBlocking(*pucBuffer++);
	}
}
void UART0_Handler1(void);

int main(int argc, char* argv[])
{
	// Send a greeting to the trace device (skipped on Release).
	trace_puts("Hello ARM World!");

	// At this stage the system clock should have already been configured
	// at high speed.
	trace_printf("System clock: %u Hz\n", SystemCoreClock);

	//
	// Set pins F0 output, SW controlled.
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOOutput(GPIOF_BASE, GPIO_PIN_0);

	//
	// Enable the peripherals used by this example.
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	//
	// Enable processor interrupts.
	//
	IntMasterEnable();

	//
	// Set GPIO A0 and A1 as UART pins.
	//
	GPIOPinTypeUART(GPIOA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	//
	// Configure the UART for 115,200, 8-N-1 operation.
	//
	//UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
	//		(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

	u0.ConfigSetExpClk(SysCtlClockGet(), 115200,
				(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
	//
	// Enable the UART interrupt.
	//
	IntRegister(INT_UART0,UART0_Handler1);
	IntEnable(INT_UART0);
	//UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
	u0.IntEnable(UART_INT_RX | UART_INT_RT);
	//Timer timer;
	//timer.start();

	int seconds = 0;

	// Infinite loop
	while (1)
	{
		for(int i=0;i<0xFFFFFF;i++);
		//timer.sleep(Timer::FREQUENCY_HZ);
		++seconds;
		GPIOPinWrite(GPIOF_BASE, GPIO_PIN_0, seconds & 0x01);
		idx=0;
		UARTSend("AT+GMR\r\n", 8);
		// Count seconds on the trace device.
		trace_printf("Second %d\n", seconds);
	}
	// Infinite loop, never return.
}

#pragma GCC diagnostic pop

//*****************************************************************************
//
// The UART interrupt handler.
//
//*****************************************************************************
//extern "C"
void UART0_Handler1(void)
{
	unsigned long ulStatus;

	//
	// Get the interrrupt status.
	//
	//ulStatus = UARTIntStatus(UART0_BASE, true);
	ulStatus = u0.IntStatus(true);

	//
	// Clear the asserted interrupts.
	//
	//UARTIntClear(UART0_BASE, ulStatus);
	u0.IntClear(ulStatus);
	//
	// Loop while there are characters in the receive FIFO.
	//
	//while (UARTCharsAvail(UART0_BASE))
	while (u0.CharsAvail())
	{
		//
		// Read the next character from the UART and write it back to the UART.
		//
		//UARTCharPutNonBlocking(UART0_BASE, UARTCharGetNonBlocking(UART0_BASE));
		//printf("%x", UARTCharGetNonBlocking(UART0_BASE));
		//char c = UARTCharGetNonBlocking(UART0_BASE);
		char c = u0.CharGetNonBlocking();
		Buf[idx++]=c;
		trace_write(&c, 1);
		trace_write(".", 1);
	}
	trace_puts("- ------\n");
}

// ----------------------------------------------------------------------------
