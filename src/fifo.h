/*
 * fifo.h
 *
 *
 *  Created on: Dec 30, 2015
 *      Author: Mohamed Anis Messaoud
 *		 Email: medanis.messaoud@gmail.com
 *
 *
 * Copyright (c) 2015 Mohamed Anis Messaoud
 */

#ifndef FIFO_H_
#define FIFO_H_

typedef struct
{
	uint32_t wpos;	///< write position
	uint32_t rpos;	///< read position
	uint32_t nelem;	///< number of elements in the buffer
	uint32_t size;	///< size of the buffer
	uint8_t* buffer;  ///< array holding the data
}fifo_t;

uint32_t fifo_size(fifo_t* fifo);
uint32_t fifo_count(fifo_t* fifo);
uint32_t fifo_available(fifo_t* fifo);
uint32_t fifo_empty(fifo_t* fifo);
uint32_t fifo_full(fifo_t* fifo);
void fifo_clear(fifo_t* fifo);
void fifo_reset(fifo_t* fifo);
void fifo_init(fifo_t* fifo);
void fifo_clear(fifo_t* fifo);
uint32_t fifo_push(fifo_t* fifo,uint8_t c);
uint8_t fifo_pop(fifo_t* fifo);

#endif /* FIFO_H_ */
