#include <stdio.h>
#include <stdlib.h>
#include <LM3S8962.h>
#include <driverlib/inc/hw_ints.h>
#include <driverlib/debug.h>
#include <driverlib/inc/hw_types.h>
#include <driverlib/sysctl.h>
#include <driverlib/gpio.h>
#include <driverlib/uart.h>
#include <driverlib/interrupt.h>
#include "diag/Trace.h"
/* Scheduler includes. */
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "WifiTask.h"

extern void __error__(char *pcFilename, unsigned long ulLine)
{
#ifdef DEBUG
	trace_printf("error: in %s line %ul", pcFilename, ulLine);
	asm volatile ("bkpt 0");
	while(1);
#else
	//reset the device
	SysCtlReset();
#endif
}

static void HardwareInit(void);

int main(void)
{
	// Send a greeting to the trace device (skipped on Release).
	trace_puts("Hello ARM World!");

	// At this stage the system clock should have already been configured
	// at high speed.
	trace_printf("System clock: %u Hz\n", SystemCoreClock);

	HardwareInit();

	xTaskCreate(vWifiTask, "Wifi", 240, NULL, 1, NULL);

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
}

