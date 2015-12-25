/*
 * esp8266_t.cpp
 *
 *
 *  Created on: Dec 19, 2015
 *      Author: Mohamed Anis Messaoud
 *		 Email: medanis.messaoud@gmail.com
 *
 *
 * Copyright (c) 2015 Mohamed Anis Messaoud
 */

#include "Drivers/ESP8266.h"


using namespace std;

esp8266_t::esp8266_t(uint32_t usart_base) :
		UsartBase(usart_base),State(StateReady),SDKVer(0),ATVer(0)
{

}

esp8266_t::~esp8266_t()
{
	// TODO Auto-generated destructor stub
}

void esp8266_t::init()
{
	char *pch;
	RestartModule();
	DisableEcho();
	SetWifiMode(ModeSta);
	/*SendATCommand("AT+GMR\r\n");
	//wait for answer
	while(State!=StateReady);
	pch = std::strstr (RxBuf.buffer(),"OK");
	if(pch != NULL)
	{

	}*/
	RxBuf.clear();
}

bool esp8266_t::ListWifiNetworks()
{
	char * pch = NULL;
	//send command
	SendATCommand("AT+CWLAP\r\n");
	while(State!=StateReady);
	pch = std::strstr (RxBuf.buffer(),"OK");
	RxBuf.clear();
	if(pch == NULL)
		return false;
	else
		return true;
}


bool esp8266_t::AutoConnect(bool Auto)
{
	char * pch = NULL;
	char cmd[]="AT+CWAUTOCONN=1\r\n";

	if(Auto==false)
		cmd[14]=0;
	//send command
	SendATCommand(cmd);
	while(State!=StateReady);
	pch = std::strstr (RxBuf.buffer(),"OK");
	RxBuf.clear();
	if(pch == NULL)
		return false;
	else
		return true;
}

bool esp8266_t::GetIPAddress()
{
	char * pch = NULL;
	//send command
	SendATCommand("AT+CIFSR\r\n");
	while(State!=StateReady);
	pch = std::strstr (RxBuf.buffer(),"OK");
	RxBuf.clear();
	if(pch == NULL)
		return false;
	else
		return true;
}

bool esp8266_t::Disconnect()
{
	char * pch = NULL;
	//send command
	SendATCommand("AT+CWQAP\r\n");
	while(State!=StateReady);
	pch = std::strstr (RxBuf.buffer(),"OK");
	RxBuf.clear();
	if(pch == NULL)
		return false;
	else
		return true;
}

bool esp8266_t::Connect(const char* ssid, const char* pwd)
{
	char cmd[50];
	char * pch = NULL;
	sprintf(cmd, "AT+CWJAP=\"%s\",\"%s\"\r\n",ssid,pwd);
	SendATCommand(cmd);
	//wait for answer
	while(State!=StateReady);
	pch = std::strstr (RxBuf.buffer(),"OK");
	RxBuf.clear();
	if(pch == NULL)
		return false;
	else
		return true;
}

bool esp8266_t:: SetWifiMode(WifiModes mode)
{
	char cmd[7]="ATE1\r\n";
	char * pch = NULL;
	switch(mode)
	{
		case ModeSta:
			break;
		case ModeAP:
			cmd[3]='2';
			break;
		case ModeBoth:
			cmd[3]='3';
			break;
		default:
			return false;
	}
	SendATCommand(cmd);
	//wait for answer
	while(State!=StateReady);
	pch = std::strstr (RxBuf.buffer(),"OK");
	RxBuf.clear();
	if(pch == NULL)
		return false;
	else
		return true;
}

bool esp8266_t::RestartModule()
{
	char * pch=NULL;
	//send command
	SendATCommand("AT+RST\r\n");
	DelayMs(2000);
	//wait until OK is received
	while(RxBuf.count()<10);
	pch = std::strstr (RxBuf.buffer(),"OK");
	RxBuf.clear();
	if(pch == NULL)
		return false;
	else
		return true;
}
void esp8266_t::EnableEcho()
{
	SendATCommand("ATE1\r\n");
	//wait for answer
	while(State!=StateReady);
	RxBuf.clear();
}

void esp8266_t::DisableEcho()
{
	SendATCommand("ATE0\r\n");
	//wait for answer
	while(State!=StateReady);
	RxBuf.clear();
}

bool esp8266_t::TestAT(void)
{
	char * pch;
	//send command
	SendATCommand("AT\r\n");
	while(State!=StateReady);
	pch = std::strstr (RxBuf.buffer(),"OK");
	RxBuf.clear();
	if(pch == NULL)
		return false;
	else
		return true;
}


void esp8266_t::SendATCommand(const char* com)
{
	//start by filling the UART fifo
	//once it is full put the data into the txbuffer
	int idx = 0;
	int len = strlen(com);

	//wait until the device is ready
	while(State!=StateReady);

	State = StateSendingCmd;
	while (UARTSpaceAvail(UsartBase) && (idx < len))
	{
		UARTCharPutNonBlocking(UsartBase, com[idx++]);
	}

	if(idx < len)
	{
		//check that we have enough space in the txbuffer
		if ((len - idx) < (TxBuf.size() - TxBuf.count()))
		{
			//write the rest of he string to the txbuffer
			// this will be sent later by the interrupt
			for (; idx < len; idx++)
			{
				TxBuf.push(com[idx]);
			}
		}
	}
	else
	{
		State = StateWaitingForResponse;
	}
}

void esp8266_t::UartISR()
{
	unsigned long ulStatus;

	//
	// Get the interrrupt status.
	//
	ulStatus = UARTIntStatus(UsartBase, true);
	//
	// Clear the asserted interrupts.
	//
	UARTIntClear(UsartBase, ulStatus);

	//TX interrupt
	if (ulStatus & UART_INT_TX)
	{
		while (UARTSpaceAvail(UsartBase) && (!TxBuf.empty()))
		{
			UARTCharPutNonBlocking(UsartBase, TxBuf.pop());
		}

		if(TxBuf.empty())
		{State = StateWaitingForResponse;}
	}

	// RX interrupt
	if ((ulStatus & (UART_INT_RX | UART_INT_RT)) != 0)
	{
		//
		// Loop while there are characters in the receive FIFO.
		//
		uint32_t ulSpace = RxBuf.available();
		while (ulSpace && UARTCharsAvail(UsartBase))
		{
			long lChar;
			char ucChar;

			//
			// Read a character from the UART FIFO into the ring buffer if no
			// errors are reported.
			//
			lChar = UARTCharGetNonBlocking(UsartBase);

			//
			// If the character did not contain any error notifications,
			// copy it to the output buffer.
			//
			if (!(lChar & ~0xFF))
			{
				ucChar = (unsigned char) (lChar & 0xFF);
				if(State == StateWaitingForResponse)
					RxBuf.push(ucChar);
				trace_write(&ucChar, 1);
				//
				// Decrement the number of bytes the buffer can accept.
				//
				ulSpace--;
			}
			else//error
			{
				trace_printf("UART rx error %d\n", lChar);
			}
		}
	}

	//RX timeout interrupt
	if ((ulStatus & UART_INT_RT)&& (State == StateWaitingForResponse))
	{
		// a full AT command has been received
		State=StateReady;
	}

	//trace_puts("- ------\n");
}


