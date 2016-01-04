/*
 * esp8266.h
 *
 *
 *  Created on: Dec 30, 2015
 *      Author: Mohamed Anis Messaoud
 *		 Email: medanis.messaoud@gmail.com
 *
 *
 * Copyright (c) 2015 Mohamed Anis Messaoud
 */

#ifndef ESP8266_H_
#define ESP8266_H_

typedef enum {
	ModeSta=1,
	ModeAP=2,
	ModeBoth=3,
}WifiModes;

typedef struct{
	uint32_t len;
	char* cmd;
} atcmd_t;

//const CMD_RESET ={8,"AT+RST\r\n"};

#endif /* ESP8266_H_ */
