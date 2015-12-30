
#include <stdio.h>
#include <stdlib.h>
#include <LM3S8962.h>
#include <driverlib/inc/hw_ints.h>
#include <driverlib/debug.h>
#include <driverlib/inc/hw_types.h>
#include <driverlib/sysctl.h>
#include <driverlib/gpio.h>
#include <driverlib/adc.h>
#include <driverlib/uart.h>
#include <driverlib/interrupt.h>
#include "diag/Trace.h"
/* Scheduler includes. */
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"


extern void __error__(char *pcFilename, unsigned long ulLine)
{
	trace_printf("error: in %s line %ul", pcFilename, ulLine);
}


static void HardwareInit(void);
void vTask1( void * pvParameters );
void vTask2( void * pvParameters );

int main (void)
{

  // Send a greeting to the trace device (skipped on Release).
  trace_puts("Hello ARM World!");

  // At this stage the system clock should have already been configured
  // at high speed.
  trace_printf("System clock: %u Hz\n", SystemCoreClock);




  xTaskCreate( vTask1, "Task1", 240, NULL, 1,NULL );
  xTaskCreate( vTask2, "Task2", 240, NULL, 1,NULL );
  /* Start the scheduler so the tasks start executing. */
  vTaskStartScheduler();
  // Infinite loop
  while (1)
    {
    }
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
	//IntEnable(INT_UART0);
	//UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT | UART_INT_TX);
	//IntMasterEnable();

}

void vTask1( void * pvParameters )
{
	int led=0;
	HardwareInit();

	while(1)
	{
		// Count seconds on the trace device.
		trace_printf ("Task 1\n");
		vTaskDelay(1000/portTICK_RATE_MS);
		GPIOPinWrite(GPIOF_BASE, GPIO_PIN_0, led++ & 0x01);
	}
}

void vTask2( void * pvParameters )
{
	while(1)
	{
		// Count seconds on the trace device.
		trace_printf ("Task 2\n");
		vTaskDelay(1000/portTICK_RATE_MS);
	}
}

