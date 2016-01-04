/*
 * fifo.c
 *
 *
 *  Created on: Dec 30, 2015
 *      Author: Mohamed Anis Messaoud
 *		 Email: medanis.messaoud@gmail.com
 *
 *
 * Copyright (c) 2015 Mohamed Anis Messaoud
 */


/*
 * fifot.h
 *
 *
 *  Created on: Dec 22, 2015
 *      Author: Mohamed Anis Messaoud
 *		 Email: medanis.messaoud@gmail.com
 *
 *
 * Copyright (c) 2015 Mohamed Anis Messaoud
 */

/**
 * \class fifo_t
 * \brief a fifo buffer class using a fixed size array
 *
 * this class creates a fixed size array that and implements methods to use it
 * as fifo. internally the buffer is managed like a circular buffer
 *
 */
#include <stdio.h>
#include <stdint.h>
#include "fifo.h"

uint32_t fifo_size(fifo_t* fifo){return fifo->size;}
uint32_t fifo_count(fifo_t* fifo){return fifo->nelem;}
uint32_t fifo_available(fifo_t* fifo){return fifo->size-fifo->nelem;}
uint32_t fifo_empty(fifo_t* fifo){return (fifo->nelem==0);}
uint32_t fifo_full(fifo_t* fifo){return fifo->nelem==fifo->size;}
void fifo_clear(fifo_t* fifo);
void fifo_reset(fifo_t* fifo){fifo->wpos=fifo->rpos=fifo->nelem=0;}

void fifo_init(fifo_t* fifo)
{
	fifo->wpos = 0;
	fifo->rpos = 0;
	fifo->nelem = 0;
}

void fifo_clear(fifo_t* fifo)
{
	uint32_t i;
	for(i=0;i<fifo->nelem;i++)
	{
		fifo->buffer[fifo->rpos] = 0;
		fifo->rpos = (fifo->rpos+1) % fifo->size; //calculate the next read position
	}
	fifo_reset(fifo);
}

uint32_t fifo_push(fifo_t* fifo,uint8_t c)
{
	uint32_t ret = 1;
	if(!fifo_full(fifo)) // the buffer is not full
	{
		fifo->buffer[fifo->wpos]=c; //add the element
		fifo->nelem++; //increment the number of elements
		fifo->wpos = (fifo->wpos+1) % fifo->size; //calculate the next write position
		ret = 0;
	}
	return ret;
}

uint8_t fifo_pop(fifo_t* fifo)
{
	if(!fifo_empty(fifo))
	{
		uint8_t elem;
		elem = fifo->buffer[fifo->rpos];
		fifo->buffer[fifo->rpos] = 0;
		fifo->nelem--; //decrement the number of elements
		fifo->rpos = (fifo->rpos+1) % fifo->size; //calculate the next read position
		return elem;
	}
	else
	{
		return 0;
	}
}
