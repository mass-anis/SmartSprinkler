#include <LM3S8962.h>
#include <driverlib/inc/hw_ints.h>
#include <driverlib/debug.h>
#include <driverlib/inc/hw_types.h>
#include <driverlib/sysctl.h>
#include <driverlib/interrupt.h>
#include <driverlib/gpio.h>
#include <driverlib/uart.h>
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "fifo.h"
#include "esp8266.h"
#include  "WifiTask.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

const char Page[]=
"<!DOCTYPE html><html>\
<body>\
<form action=\"CONFIG\" method=\"post\">\
<fieldset>\
<legend>Wifi Configuration:</legend>\
SSID:<br>\
<input type=\"text\" name=\"ssid\" value=\"\">\
<br>\
Password:<br>\
<input type=\"text\" name=\"pwd\" value=\"\">\
<br><br>\
<input type=\"submit\" value=\"Submit\">\
</form>\
</fieldset>\
</body>\
</html>\r\n\0";

typedef enum
{
	StateReady, StateSendingCmd, StateWaitingForResponse,
} ESPStates;

typedef enum
{
	AppServer,
	AppClient,
} AppModes;

uint8_t TxBuf[TX_BUF_SIZE];
uint8_t RxBuf[RX_BUF_SIZE];
fifo_t TxFifo =
{ 0, 0, 0, TX_BUF_SIZE, TxBuf };
fifo_t RxFifo =
{ 0, 0, 0, RX_BUF_SIZE, RxBuf };

//ESPStates State;

//volatile uint32_t RxTimeOut = 0;

void UART0_Handler(void);
static void SendATCommand(const char* com);
static void WifiRestart(void);
static uint32_t WifiSetMode(WifiModes mode);
static uint32_t WifiDisableEcho(void);
static uint32_t CheckResponseOk(void);
static uint32_t WifiGetSavedAP(char* ssid);
static uint32_t WifiSetMultipleConn(uint32_t mux);
static uint32_t WifiStartServer(uint32_t port);
static uint32_t WifiSendBuffer(const char* page);
static uint32_t WifiCloseChannel(uint32_t chan);
static uint32_t WifiHandleHTML(void);
static uint32_t WifiConnect(const char* ssid, const char* pwd);


xSemaphoreHandle SemUartRxTimeOut;
xSemaphoreHandle SemUartTxDone;

void vWifiTask(void * pvParameters)
{
	int led = 0;
	AppModes mode;
	char ssid[20] =
	{ 0 };
	// create the binary semaphore for the uart RX timeout
	vSemaphoreCreateBinary(SemUartRxTimeOut);
	ASSERT(SemUartRxTimeOut!=NULL); //check semaphore created successfully

	// create the binary semaphore for the uart TX done
	vSemaphoreCreateBinary(SemUartTxDone);
	ASSERT(SemUartTxDone!=NULL); //check semaphore created successfully

//---------- Configure UART0 -----------------
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
//	State = StateReady;

	WifiRestart(); // restart wifi module
	// echo must be disabled otherwise the timeout
	// must be increased for the commands like connect
	// because of the initial echo
	WifiDisableEcho();
	WifiSetMode(ModeSta); // set station mode
	SendATCommand("AT+CWAUTOCONN?\r\n");
	CheckResponseOk();
	mode =AppClient;// init to client mode

	//if(WifiGetSavedAP(ssid)!=0)//check if we have a saved ssid
	{
		// no ssid found start a web server to configure the network
		mode=AppServer;
		WifiSetMode(ModeBoth); // set station mode
		WifiSetMultipleConn(1);// disable multiple connections
		WifiStartServer(80); //start http server on port 80
	}

	while (1)
	{
		switch(mode)
		{
			case AppServer:
				// block waiting for the uart timeout interrupt
				xSemaphoreTake(SemUartRxTimeOut, portMAX_DELAY);
				// wait for data transmission to finish
				vTaskDelay(1000 / portTICK_RATE_MS);
				//Data Received  parse it
				WifiHandleHTML();
				break;
			case AppClient:
				while(1)
				{
					// block waiting for the uart timeout interrupt
					xSemaphoreTake(SemUartRxTimeOut, portMAX_DELAY);
					// wait for data transmission to finish
					vTaskDelay(1000 / portTICK_RATE_MS);
					//Data Received  parse it
					WifiHandleHTML();
//					GPIOPinWrite(GPIOF_BASE, GPIO_PIN_0, led++ & 0x01);
//					vTaskDelay(1000 / portTICK_RATE_MS);
				}
				break;
		}
	}
}

static uint32_t WifiHandleHTML(void)
{
	uint32_t chan,len;
	char tmp[5],Method[10],URI[25],ssid[20],pwd[20];
	char * pch = NULL;
	//data format: +IPD,<chan>,<len>:<method> <URI> HTTP/1.1
	pch = strstr((char*) RxFifo.buffer, "+IPD");

	if (pch != NULL)
	{
		//get the channel number
		int idx=5;
		int l=0;
		while(pch[idx]!=',')
		{
			tmp[l++]=pch[idx++];
		}
		tmp[l]=0;
		chan=atoi(tmp);

		//get the data length
		l=0;
		idx++;
		while(pch[idx]!=':')
		{
			tmp[l++]=pch[idx++];
		}
		tmp[l]=0;
		len=atoi(tmp);

		//get the method
		l=0;
		idx++;
		while(pch[idx]!=' ')
		{
			Method[l++]=pch[idx++];
		}
		Method[l]=0;
		//get the URI
		l=0;
		idx++;
		while(pch[idx]!=' ')
		{
			URI[l++]=pch[idx++];
		}
		URI[l]=0;

		if(strcmp(Method,"GET")==0)
		{
			fifo_clear(&RxFifo);
			if(strcmp(URI,"/")==0)
			{
				WifiSendBuffer(Page);
				vTaskDelay(100 / portTICK_RATE_MS);
			}
			//close the channel
//			WifiCloseChannel(chan);
			vTaskDelay(100 / portTICK_RATE_MS);
		}
		else if(strcmp(Method,"POST")==0)
		{
			if(strcmp(URI,"/CONFIG")==0)
			{
				pch = strstr((char*) RxFifo.buffer, "ssid=");
				//get the ssid
				l=0;
				idx=5;
				while(pch[idx]!='&')
				{
					if(pch[idx]=='+')
					{
						ssid[l++]=' ';
						idx++;
					}
					else
						ssid[l++]=pch[idx++];

				}
				ssid[l]=0;

				//get the pwd
				l=0;
				idx+=5;
				while(pch[idx]!=0)
				{
					pwd[l++]=pch[idx++];
				}
				pwd[l]=0;
				fifo_clear(&RxFifo);
				//close the channel
				vTaskDelay(100 / portTICK_RATE_MS);
//				WifiCloseChannel(chan);
				vTaskDelay(500 / portTICK_RATE_MS);
				WifiConnect(ssid,pwd);
			}

		}
	}
	else
	{
		fifo_clear(&RxFifo);
	}

}
static uint32_t WifiCloseChannel(uint32_t chan)
{
	char cmd[25];
	sprintf(cmd, "AT+CIPCLOSE=%d\r\n\0", chan);
	SendATCommand(cmd);

	return CheckResponseOk();
}

static uint32_t WifiSendBuffer(const char* page)
{
	char cmd[25];
	char * pch = NULL;
	sprintf(cmd, "AT+CIPSEND=0,%d\r\n", strlen(page));
	SendATCommand(cmd);
	// block waiting for the uart timeout interrupt
	xSemaphoreTake(SemUartRxTimeOut, 100 / portTICK_RATE_MS);
	//check the rxbuffer for > character
	pch = strstr((char*) RxFifo.buffer, ">");
	if (pch != NULL) //> found send the page
	{
		SendATCommand(page);
	}
	return CheckResponseOk();
}

static uint32_t WifiStartServer(uint32_t port)
{
	char cmd[50];
	char * pch = NULL;
	sprintf(cmd, "AT+CIPSERVER=1,%d\r\n", port);
	SendATCommand(cmd);

	return CheckResponseOk();
}
static uint32_t WifiConnect(const char* ssid, const char* pwd)
{
	char cmd[50];
	char * pch = NULL;
	sprintf(cmd, "AT+CWJAP=\"%s\",\"%s\"\r\n", ssid, pwd);
	SendATCommand(cmd);

	return CheckResponseOk();
}

/**
 * read the ssid of the saved AP
 *
 * @param ssid
 * @return 0 if successful
 */
static uint32_t WifiGetSavedAP(char* ssid)
{
	SendATCommand("AT+CWJAP?\r\n");
	char *pch = NULL;
	pch = strstr((char*) RxFifo.buffer, "JAP:\"");
	if (pch != NULL) //ssid found try reading it
	{
		int idx = 0;
		char c = 0;
		while (c != '\"')
		{
			c = fifo_pop(&RxFifo);
			if(fifo_empty(&RxFifo))
				return 2;
		}
		c = fifo_pop(&RxFifo);
		while (c != '\"')
		{
			if(fifo_empty(&RxFifo))
				return 2;
			ssid[idx] = c;
			idx++;
			c = fifo_pop(&RxFifo);
		}
		fifo_clear(&RxFifo);
		return 0;
	}
	else
	{
		fifo_clear(&RxFifo);
		ssid[0] = 0;
		return 1;
	}
}

uint32_t WifiSetMultipleConn(uint32_t mux)
{
	char cmd[14] = "AT+CIPMUX=1\r\n";
	if(mux == 0)
		cmd[10]='0';

	SendATCommand(cmd);
	return CheckResponseOk();
}

static uint32_t WifiDisableEcho(void)
{
	char cmd[14] = "ATE0\r\n";
	SendATCommand(cmd);
	return CheckResponseOk();
}

static uint32_t WifiSetMode(WifiModes mode)
{
	char cmd[14] = "AT+CWMODE=1\r\n";
	switch (mode)
	{
	case ModeSta:
		break;
	case ModeAP:
		cmd[10] = '2';
		break;
	case ModeBoth:
		cmd[10] = '3';
		break;
	default:
		return 1;
	}
	SendATCommand(cmd);
	return CheckResponseOk();
}

static void WifiRestart(void)
{
	//send command
	SendATCommand("AT+RST\r\n");
	vTaskDelay(4000);
	fifo_clear(&RxFifo);
}

static uint32_t CheckResponseOk(void)
{
	char * pch = NULL;
	pch = strstr((char*) RxFifo.buffer, "OK");
	fifo_clear(&RxFifo);
	if (pch == NULL)
		return 1;
	else
		return 0;
}

static void SendATCommand(const char* com)
{
	//start by filling the UART fifo
	//once it is full put the data into the txbuffer
	int idx = 0;
	int len = strlen(com);

//	State = StateSendingCmd;
	while (UARTSpaceAvail(UART0_BASE) && (idx < len))
	{
		UARTCharPutNonBlocking(UART0_BASE, com[idx++]);
	}

	if (idx < len)
	{
		//check that we have enough space in the txbuffer
		if ((len - idx) < (fifo_size(&TxFifo) - fifo_count(&TxFifo)))
		{
			//write the rest of he string to the txbuffer
			// this will be sent later by the interrupt
			for (; idx < len; idx++)
			{
				fifo_push(&TxFifo, com[idx]);
			}
		}
	}
	else
	{
//		State = StateWaitingForResponse;
	}

	//make sure the semaphore is not already available
	xSemaphoreTake(SemUartRxTimeOut, 0);

//	// block waiting for the uart timeout interrupt
//	xSemaphoreTake(SemUartRxTimeOut, 3000 / portTICK_RATE_MS);
//	RxTimeOut = 0;
//	while (RxTimeOut < 10)
//	{
//		vTaskDelay(100 / portTICK_RATE_MS);
//		RxTimeOut++;
//	}
	portBASE_TYPE TimeOut=pdTRUE;
	while(TimeOut==pdTRUE)
	{
		// block waiting for the uart timeout interrupt
		TimeOut=xSemaphoreTake(SemUartRxTimeOut, 2000 / portTICK_RATE_MS);
	}
}

void UART0_Handler(void)
{
	uint32_t ulStatus;
	static portBASE_TYPE xHPTaskWoken;
	//
	// Get the interrrupt status.
	//
	ulStatus = UARTIntStatus(UART0_BASE, true);
	//
	// Clear the asserted interrupts.
	//
	UARTIntClear(UART0_BASE, ulStatus);

	//TX interrupt
	if (ulStatus & UART_INT_TX)
	{
		while (UARTSpaceAvail(UART0_BASE) && (!fifo_empty(&TxFifo)))
		{
			UARTCharPutNonBlocking(UART0_BASE, fifo_pop(&TxFifo));
		}

		if (fifo_empty(&TxFifo))
		{
//			State = StateWaitingForResponse;
			xHPTaskWoken = pdFALSE;
			xSemaphoreGiveFromISR(SemUartTxDone,&xHPTaskWoken);
//			// if the semaphore unblocked a task with a higher priority than the
//			// currently running one force a context switch
//			if(xHPTaskWoken == pdTRUE)
//			{
//				vPortYield();
//			}
		}
	}

	// RX interrupt
	if ((ulStatus & (UART_INT_RX | UART_INT_RT)) != 0)
	{
//		RxTimeOut = 1;
		//
		// Loop while there are characters in the receive FIFO.
		//
		uint32_t ulSpace = fifo_available(&RxFifo);
		while (ulSpace && UARTCharsAvail(UART0_BASE))
		{
			uint32_t lChar;
			uint8_t ucChar;

			//
			// Read a character from the UART FIFO into the ring buffer if no
			// errors are reported.
			//
			lChar = UARTCharGetNonBlocking(UART0_BASE);

			//
			// If the character did not contain any error notifications,
			// copy it to the output buffer.
			//
			if (!(lChar & ~0xFF))
			{
				ucChar = (unsigned char) (lChar & 0xFF);
//				if (State == StateWaitingForResponse)
				if (fifo_push(&RxFifo, ucChar))
				{
#ifdef DEBUG
					//push failed because fifo is full
					trace_printf("Error : RX Fifo Full");
					__error__(__FILE__, __LINE__);
#endif
				}
#ifdef DEBUG
				trace_write(&ucChar, 1);
#endif
				//
				// Decrement the number of bytes the buffer can accept.
				//
				ulSpace--;
			}
#ifdef DEBUG
			else	 //error
			{
				trace_printf("UART rx error %d\n", lChar);
			}
#endif
		}
	}

	if ((ulStatus & UART_INT_RT) != 0)
	{
//		State = StateReady;
		xHPTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(SemUartRxTimeOut,&xHPTaskWoken);
//		// if the semaphore unblocked a task with a higher priority than the
//		// currently running one force a context switch
//		if(xHPTaskWoken == pdTRUE)
//		{
//			vPortYield();
//		}
	}
}
