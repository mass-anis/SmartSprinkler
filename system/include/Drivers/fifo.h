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

#ifndef SRC_DRIVERS_FIFOT_H_
#define SRC_DRIVERS_FIFOT_H_

/**
 * \class fifo_t
 * \brief a fifo buffer class using a fixed size array
 *
 * this class creates a fixed size array that and implements methods to use it
 * as fifo. internally the buffer is managed like a circular buffer
 *
 */
template<class T, int Size>
class fifo_t
{
private:
	T Buffer[Size]; ///< array holding the data
	uint32_t wpos;	///< write position
	uint32_t rpos;	///< read position
	uint32_t nelem;	///< number of elements in the buffer
public:
	fifo_t():wpos(0),rpos(0),nelem(0){};
	~fifo_t(){}
	uint32_t size(){return Size;}
	uint32_t count(){return nelem;}
	uint32_t available(){return Size-nelem;}
	bool push(T c);
	T pop();
	bool empty(){return (nelem==0);}
	bool full(){return nelem==Size;}
	void clear();
	void reset(){wpos=rpos=nelem=0;}
	T* buffer(){return Buffer;} ///< return a pointer to the buffer
};

template<class T, int Size>
inline void fifo_t<T,Size>::clear()
{
	for(int i=0;i<nelem;i++)
	{
		Buffer[rpos] = 0;
		rpos = (rpos+1) % Size; //calculate the next read position
	}
	reset();
}

template<class T, int Size>
inline bool fifo_t<T,Size>::push(T c)
{
	bool ret = false;
	if(!full()) // the buffer is not full
	{
		Buffer[wpos]=c; //add the element
		nelem++; //increment the number of elements
		wpos = (wpos+1) % Size; //calculate the next write position
		ret = true;
	}
	return ret;
}

template<class T, int Size>
inline T fifo_t<T,Size>::pop()
{
	if(!empty())
	{
		T elem;
		elem = Buffer[rpos];
		Buffer[rpos] = 0;
		nelem--; //decrement the number of elements
		rpos = (rpos+1) % Size; //calculate the next read position
		return elem;
	}
	else
	{
		return NULL;
	}
}

#endif /* SRC_DRIVERS_FIFOT_H_ */
