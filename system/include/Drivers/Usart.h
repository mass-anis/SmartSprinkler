/**
 * \file Usart.h
 *
 *  Created on: Dec 16, 2015
 *      Author: Messaoud Mohamed Anis
 *  	 email: medanis.messaoud@gmail.com
 * 	this class is a c++ wrapper for the Stellaris Peripheral Driver Library
 */

//*****************************************************************************
//
// uart.c - Driver for the UART.
//
// Copyright (c) 2005-2012 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
//
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions
//   are met:
//
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the
//   distribution.
//
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// This is part of revision 9453 of the Stellaris Peripheral Driver Library.
//
//*****************************************************************************
#ifndef INCLUDE_DRIVERS_USART_H_
#define INCLUDE_DRIVERS_USART_H_

#include <LM3S8962.h>
#include <stdint.h>
#include <assert.h>
#include "driverlib/inc/hw_sysctl.h"
#include "driverlib/inc/hw_uart.h"
#include "driverlib/inc/hw_types.h"
#include "driverlib/interrupt.h"
//*****************************************************************************
//
// Values that can be passed to UARTIntEnable, UARTIntDisable, and UARTIntClear
// as the ulIntFlags parameter, and returned from UARTIntStatus.
//
//*****************************************************************************
#define UART_INT_9BIT           0x1000      // 9-bit address match interrupt
#define UART_INT_OE             0x400       // Overrun Error Interrupt Mask
#define UART_INT_BE             0x200       // Break Error Interrupt Mask
#define UART_INT_PE             0x100       // Parity Error Interrupt Mask
#define UART_INT_FE             0x080       // Framing Error Interrupt Mask
#define UART_INT_RT             0x040       // Receive Timeout Interrupt Mask
#define UART_INT_TX             0x020       // Transmit Interrupt Mask
#define UART_INT_RX             0x010       // Receive Interrupt Mask
#define UART_INT_DSR            0x008       // DSR Modem Interrupt Mask
#define UART_INT_DCD            0x004       // DCD Modem Interrupt Mask
#define UART_INT_CTS            0x002       // CTS Modem Interrupt Mask
#define UART_INT_RI             0x001       // RI Modem Interrupt Mask

//*****************************************************************************
//
// Values that can be passed to UARTConfigSetExpClk as the ulConfig parameter
// and returned by UARTConfigGetExpClk in the pulConfig parameter.
// Additionally, the UART_CONFIG_PAR_* subset can be passed to
// UARTParityModeSet as the ulParity parameter, and are returned by
// UARTParityModeGet.
//
//*****************************************************************************
#define UART_CONFIG_WLEN_MASK   0x00000060  // Mask for extracting word length
#define UART_CONFIG_WLEN_8      0x00000060  // 8 bit data
#define UART_CONFIG_WLEN_7      0x00000040  // 7 bit data
#define UART_CONFIG_WLEN_6      0x00000020  // 6 bit data
#define UART_CONFIG_WLEN_5      0x00000000  // 5 bit data
#define UART_CONFIG_STOP_MASK   0x00000008  // Mask for extracting stop bits
#define UART_CONFIG_STOP_ONE    0x00000000  // One stop bit
#define UART_CONFIG_STOP_TWO    0x00000008  // Two stop bits
#define UART_CONFIG_PAR_MASK    0x00000086  // Mask for extracting parity
#define UART_CONFIG_PAR_NONE    0x00000000  // No parity
#define UART_CONFIG_PAR_EVEN    0x00000006  // Even parity
#define UART_CONFIG_PAR_ODD     0x00000002  // Odd parity
#define UART_CONFIG_PAR_ONE     0x00000082  // Parity bit is one
#define UART_CONFIG_PAR_ZERO    0x00000086  // Parity bit is zero

//*****************************************************************************
//
// Values that can be passed to UARTFIFOLevelSet as the ulTxLevel parameter and
// returned by UARTFIFOLevelGet in the pulTxLevel.
//
//*****************************************************************************
#define UART_FIFO_TX1_8         0x00000000  // Transmit interrupt at 1/8 Full
#define UART_FIFO_TX2_8         0x00000001  // Transmit interrupt at 1/4 Full
#define UART_FIFO_TX4_8         0x00000002  // Transmit interrupt at 1/2 Full
#define UART_FIFO_TX6_8         0x00000003  // Transmit interrupt at 3/4 Full
#define UART_FIFO_TX7_8         0x00000004  // Transmit interrupt at 7/8 Full

//*****************************************************************************
//
// Values that can be passed to UARTFIFOLevelSet as the ulRxLevel parameter and
// returned by UARTFIFOLevelGet in the pulRxLevel.
//
//*****************************************************************************
#define UART_FIFO_RX1_8         0x00000000  // Receive interrupt at 1/8 Full
#define UART_FIFO_RX2_8         0x00000008  // Receive interrupt at 1/4 Full
#define UART_FIFO_RX4_8         0x00000010  // Receive interrupt at 1/2 Full
#define UART_FIFO_RX6_8         0x00000018  // Receive interrupt at 3/4 Full
#define UART_FIFO_RX7_8         0x00000020  // Receive interrupt at 7/8 Full

//*****************************************************************************
//
// Values that can be passed to UARTDMAEnable() and UARTDMADisable().
//
//*****************************************************************************
#define UART_DMA_ERR_RXSTOP     0x00000004  // Stop DMA receive if UART error
#define UART_DMA_TX             0x00000002  // Enable DMA for transmit
#define UART_DMA_RX             0x00000001  // Enable DMA for receive

//*****************************************************************************
//
// Values returned from UARTRxErrorGet().
//
//*****************************************************************************
#define UART_RXERROR_OVERRUN    0x00000008
#define UART_RXERROR_BREAK      0x00000004
#define UART_RXERROR_PARITY     0x00000002
#define UART_RXERROR_FRAMING    0x00000001

//*****************************************************************************
//
// Values that can be passed to UARTHandshakeOutputsSet() or returned from
// UARTHandshakeOutputGet().
//
//*****************************************************************************
#define UART_OUTPUT_RTS         0x00000800
#define UART_OUTPUT_DTR         0x00000400

//*****************************************************************************
//
// Values that can be returned from UARTHandshakeInputsGet().
//
//*****************************************************************************
#define UART_INPUT_RI           0x00000100
#define UART_INPUT_DCD          0x00000004
#define UART_INPUT_DSR          0x00000002
#define UART_INPUT_CTS          0x00000001

//*****************************************************************************
//
// Values that can be passed to UARTFlowControl() or returned from
// UARTFlowControlGet().
//
//*****************************************************************************
#define UART_FLOWCONTROL_TX     0x00008000
#define UART_FLOWCONTROL_RX     0x00004000
#define UART_FLOWCONTROL_NONE   0x00000000

//*****************************************************************************
//
// Values that can be passed to UARTTxIntModeSet() or returned from
// UARTTxIntModeGet().
//
//*****************************************************************************
#define UART_TXINT_MODE_FIFO    0x00000000
#define UART_TXINT_MODE_EOT     0x00000010

//*****************************************************************************
//
// Values that can be passed to UARTClockSourceSet() or returned from
// UARTClockSourceGet().
//
//*****************************************************************************
#define UART_CLOCK_SYSTEM       0x00000000
#define UART_CLOCK_PIOSC        0x00000005

/*
 * \class Usart
 * \brief a c++ wrapper for the Stellaris Peripheral Driver uart Library
 */
class usart_t
{
	UART_Type *Regs;
public:
	usart_t(UART_Type * regs);
	~usart_t();
	//*****************************************************************************
	//
	//! Sets the type of parity.
	//!
	//! \param ulBase is the base address of the UART port.
	//! \param ulParity specifies the type of parity to use.
	//!
	//! This function configures the type of parity to use for transmitting and
	//! expect when receiving.  The \e ulParity parameter must be one of
	//! \b UART_CONFIG_PAR_NONE, \b UART_CONFIG_PAR_EVEN, \b UART_CONFIG_PAR_ODD,
	//! \b UART_CONFIG_PAR_ONE, or \b UART_CONFIG_PAR_ZERO.  The last two
	//! parameters allow direct control of the parity bit; it is always either one
	//! or zero based on the mode.
	//!
	//! \return None.
	//
	//*****************************************************************************
	void ParityModeSet(uint32_t ulParity);
	//*****************************************************************************
	//
	//! Gets the type of parity currently being used.
	//!
	//! \param ulBase is the base address of the UART port.
	//!
	//! This function gets the type of parity used for transmitting data and
	//! expected when receiving data.
	//!
	//! \return Returns the current parity settings, specified as one of
	//! \b UART_CONFIG_PAR_NONE, \b UART_CONFIG_PAR_EVEN, \b UART_CONFIG_PAR_ODD,
	//! \b UART_CONFIG_PAR_ONE, or \b UART_CONFIG_PAR_ZERO.
	//
	//*****************************************************************************
	uint32_t ParityModeGet();
	void FIFOLevelSet(uint32_t ulTxLevel, uint32_t ulRxLevel);
	void FIFOLevelGet(uint32_t *pulTxLevel, uint32_t *pulRxLevel);
	void ConfigSetExpClk(uint32_t ulClk, uint32_t ulBaud, uint32_t ulConfig);
	void ConfigGetExpClk(uint32_t ulClk, uint32_t *pulBaud,
			uint32_t *pulConfig);
	void Enable();
	void Disable();
	void FIFOEnable();
	void FIFODisable();
	void EnableSIR(bool bLowPower);
	void DisableSIR();
	bool CharsAvail();
	bool SpaceAvail();
	long CharGetNonBlocking();
	long CharGet();
	bool CharPutNonBlocking(uint8_t ucData);
	void CharPut(uint8_t ucData);
	void BreakCtl(bool bBreakState);
	bool Busy();
	void IsrRegister(void (*pfnHandler)(void));
	void IsrUnregister();
	void IntEnable(uint32_t ulIntFlags);
	void IntDisable(uint32_t ulIntFlags);
	uint32_t IntStatus(bool bMasked);
	void IntClear(uint32_t ulIntFlags);
	void DMAEnable(uint32_t ulDMAFlags);
	void DMADisable(uint32_t ulDMAFlags);
	uint32_t RxErrorGet();
	void RxErrorClear();
	void SmartCardEnable();
	void SmartCardDisable();
	void ModemControlSet(uint32_t ulControl);
	void ModemControlClear(uint32_t ulControl);
	uint32_t ModemControlGet();
	uint32_t ModemStatusGet();
	void FlowControlSet(uint32_t ulMode);
	uint32_t FlowControlGet();
	void TxIntModeSet(uint32_t ulMode);
	uint32_t TxIntModeGet();
	void ClockSourceSet(uint32_t ulSource);
	uint32_t ClockSourceGet();
	void Enable9Bit();
	void Disable9Bit();
	void AddrSet9Bit(uint8_t ucAddr, uint8_t ucMask);
	void AddrSend9Bit(uint8_t ucAddr);
};

#endif /* INCLUDE_DRIVERS_USART_H_ */
