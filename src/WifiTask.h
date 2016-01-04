/*
 * WifiTask.h
 *
 *
 *  Created on: Dec 30, 2015
 *      Author: Mohamed Anis Messaoud
 *		 Email: medanis.messaoud@gmail.com
 *
 *
 * Copyright (c) 2015 Mohamed Anis Messaoud
 */

#ifndef WIFITASK_H_
#define WIFITASK_H_


#define TX_BUF_SIZE (512)	///< size of the UART TX buffer
#define RX_BUF_SIZE (1024)	///< size of the UART RX buffer

void vWifiTask( void * pvParameters );

#endif /* WIFITASK_H_ */
