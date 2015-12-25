/*
 * ESP8266.h
 *
 *
 *  Created on: Dec 19, 2015
 *      Author: Mohamed Anis Messaoud
 *		 Email: medanis.messaoud@gmail.com
 *
 *
 * Copyright (c) 2015 Mohamed Anis Messaoud
 *
 */

#ifndef ESP8266_H_
#define ESP8266_H_

#include <cstdint>
#include <cstring>
#include <cstdio>
#include <queue>
#include <diag/Trace.h>
#include <driverlib/inc/hw_ints.h>
#include <driverlib/inc/hw_types.h>
#include <driverlib/uart.h>
#include <driverlib/sysctl.h>
#include "Drivers/fifo.h"
#include "../../src/delay.h"


typedef enum {
	StateReady,
	StateSendingCmd,
	StateWaitingForResponse,
}ESPStates;

typedef enum {
	ModeSta=1,
	ModeAP=2,
	ModeBoth=3,
}WifiModes;

class esp8266_t
{
private:
	uint32_t UsartBase;
	fifo_t<char,1024> RxBuf;
	fifo_t<char,256> TxBuf;
	volatile ESPStates State;
	uint32_t SDKVer;
	uint32_t ATVer;
	void SendATCommand(const char* com);
public:
	const uint32_t &SDKVersion=SDKVer;
	const uint32_t &ATVersion=ATVer;

	esp8266_t(const uint32_t usart_base);
	~esp8266_t();
	void init();

	bool TestAT(void);
	void EnableEcho();
	void DisableEcho();
	bool RestartModule(void);
	bool Connect(const char* ssid, const char* pwd);
	bool ListWifiNetworks();
	bool Disconnect();
	bool AutoConnect(bool Auto);
	bool GetIPAddress();
	bool SetWifiMode(WifiModes mode);
	void UartISR();
};

#endif /* ESP8266_H_ */
