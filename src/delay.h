/*
 * delay.h
 *
 *
 *  Created on: Dec 25, 2015
 *      Author: Mohamed Anis Messaoud
 *		 Email: medanis.messaoud@gmail.com
 *
 *
 * Copyright (c) 2015 Mohamed Anis Messaoud
 */

#ifndef DELAY_H_
#define DELAY_H_

#if defined(__cplusplus)
extern "C"
{
#endif

#include <LM3S8962.h>

void __attribute__((naked)) Delay10Cycle(unsigned long ulCount);

#define DelayUs(delay) 	Delay10Cycle(((SystemCoreClock/10000)*delay)/1000)
#define DelayMs(delay) 	Delay10Cycle(((SystemCoreClock/10000)*delay))

#ifdef __cplusplus
}
#endif

#endif /* DELAY_H_ */
