/****************************************************************************************

 Author: Mohamed Anis Messaoud
 Email: medanis.messaoud@gmail.com

 Copyright (c) 2015 Mohamed Anis Messaoud
 *****************************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <diag/Trace.h>
#include <LM3S8962.h>
#include <driverlib/inc/hw_ints.h>
#include <driverlib/debug.h>
#include <driverlib/inc/hw_types.h>
#include <driverlib/sysctl.h>
#include <driverlib/gpio.h>
#include <driverlib/adc.h>
#include <driverlib/uart.h>
#include <driverlib/interrupt.h>
#include "Drivers/ESP8266.h"
#include "delay.h"

static void HardwareInit(void);
void UARTSend(const char *pucBuffer, unsigned long ulCount);

esp8266_t wifi(UART0_BASE);

int main(int argc, char* argv[])
{

	// Send a greeting to the trace device (skipped on Release).
	trace_puts("Hello ARM World!");

	HardwareInit();

	// At this stage the system clock should have already been configured
	// at high speed.
	trace_printf("System clock: %u Hz\n", SystemCoreClock);


	wifi.init();
	wifi.Connect("salon 2.4G","RTt567fgh");

	int seconds = 0;
	// Infinite loop
	while (1)
	{
		//timer.sleep(Timer::FREQUENCY_HZ);
		++seconds;
		GPIOPinWrite(GPIOF_BASE, GPIO_PIN_0, seconds & 0x01);
		//wifi.SendATCommand("AT\r\n");
		DelayMs(2000);
		wifi.GetIPAddress();
		// Count seconds on the trace device.
		trace_printf("\nSecond %d\n", seconds);

	}
	// Infinite loop, never return.
}

static void HardwareInit(void)
{
	//
	// Configure Led Pin
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOOutput(GPIOF_BASE, GPIO_PIN_0);

	//
	// COnfigure UART0
	//
	//enable Peripheral
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	//Configure PIN A0 and A1 in UART mode
	GPIOPinTypeUART(GPIOA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	// Configure the UART for 115,200, 8-N-1 operation.
	UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
			(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
	//enable the transmit and receive FIFOs
	UARTFIFOEnable(UART0_BASE);
	UARTFIFOLevelSet(UART0_BASE, UART_FIFO_TX1_8, UART_FIFO_RX6_8);
	// Enable the UART interrupt.
	IntEnable(INT_UART0);
	UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT | UART_INT_TX);
	IntMasterEnable();

}

//*****************************************************************************
//
// The UART interrupt handler.
//
//*****************************************************************************

extern "C" void UART0_Handler(void)
{
	wifi.UartISR();
}

extern void __error__(char *pcFilename, unsigned long ulLine)
{
	trace_printf("error: in %s line %ul", pcFilename, ulLine);
}

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
		UARTCharPutNonBlocking(UART0_BASE, *pucBuffer++);
	}
}

