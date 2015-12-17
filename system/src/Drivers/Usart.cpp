/*
 * Usart.cpp
 *
 *  Created on: Dec 16, 2015
 *      Author: anis
 */

#include <Drivers/Usart.h>

//*****************************************************************************
//
// A mapping of UART base address to interupt number.
//
//*****************************************************************************
static const uint32_t g_ppulUARTIntMap[][2] =
{
{ (uint32_t) UART0, UART0_IRQn },
{ (uint32_t) UART1, UART1_IRQn } };

//*****************************************************************************
//
//! \internal
//! Gets the UART interrupt number.
//!
//! \param ulBase is the base address of the UART port.
//!
//! Given a UART base address, this function returns the corresponding
//! interrupt number.
//!
//! \return Returns a UART interrupt number, or -1 if \e ulBase is invalid.
//
//*****************************************************************************
static long IntNumberGet(unsigned long ulBase)
{
	unsigned long ulIdx;

	//
	// Loop through the table that maps UART base addresses to interrupt
	// numbers.
	//
	for (ulIdx = 0;
			ulIdx < (sizeof(g_ppulUARTIntMap) / sizeof(g_ppulUARTIntMap[0]));
			ulIdx++)
	{
		//
		// See if this base address matches.
		//
		if (g_ppulUARTIntMap[ulIdx][0] == ulBase)
		{
			//
			// Return the corresponding interrupt number.
			//
			return (g_ppulUARTIntMap[ulIdx][1]);
		}
	}

	//
	// The base address could not be found, so return an error.
	//
	return (-1);
}
//*****************************************************************************
//
// The system clock divider defining the maximum baud rate supported by the
// UART.
//
//*****************************************************************************
#define UART_CLK_DIVIDER        ((CLASS_IS_SANDSTORM ||                       \
                                  (CLASS_IS_FURY && REVISION_IS_A2) ||        \
                                  (CLASS_IS_DUSTDEVIL && REVISION_IS_A0)) ?   \
                                 16 : 8)

Usart::Usart(UART0_Type * reg) :
		Regs(reg)
{
}

Usart::~Usart()
{

}

void Usart::ParityModeSet(uint32_t ulParity)
{
	//
	// Check the arguments.
	//
	assert(
			( ulParity == UART_CONFIG_PAR_NONE) || (ulParity == UART_CONFIG_PAR_EVEN) || (ulParity == UART_CONFIG_PAR_ODD) || (ulParity == UART_CONFIG_PAR_ONE) || (ulParity == UART_CONFIG_PAR_ZERO));

	//
	// Set the parity mode.
	//
	Regs->LCRH = (Regs->LCRH & ~(UART_LCRH_SPS | UART_LCRH_EPS | UART_LCRH_PEN))
			| ulParity;
}

uint32_t Usart::ParityModeGet()
{
	//
	// Return the current parity setting.
	//
	return (Regs->LCRH & (UART_LCRH_SPS | UART_LCRH_EPS | UART_LCRH_PEN));
}

void Usart::FIFOLevelSet(uint32_t ulTxLevel, uint32_t ulRxLevel)
{
	//
	// Check the arguments.
	//

	assert(
			(ulTxLevel == UART_FIFO_TX1_8) || (ulTxLevel == UART_FIFO_TX2_8) || (ulTxLevel == UART_FIFO_TX4_8) || (ulTxLevel == UART_FIFO_TX6_8) || (ulTxLevel == UART_FIFO_TX7_8));
	assert(
			(ulRxLevel == UART_FIFO_RX1_8) || (ulRxLevel == UART_FIFO_RX2_8) || (ulRxLevel == UART_FIFO_RX4_8) || (ulRxLevel == UART_FIFO_RX6_8) || (ulRxLevel == UART_FIFO_RX7_8));

	//
	// Set the FIFO interrupt levels.
	//
	Regs->IFLS = ulTxLevel | ulRxLevel;
}

void Usart::FIFOLevelGet(uint32_t * pulTxLevel, uint32_t * pulRxLevel)
{
	uint32_t ulTemp;

	//
	// Read the FIFO level register.
	//
	ulTemp = Regs->IFLS;

	//
	// Extract the transmit and receive FIFO levels.
	//
	*pulTxLevel = ulTemp & UART_IFLS_TX_M;
	*pulRxLevel = ulTemp & UART_IFLS_RX_M;
}

void Usart::ConfigSetExpClk(uint32_t ulUARTClk, uint32_t ulBaud,
		uint32_t ulConfig)
{
	uint32_t ulDiv;

	//
	// Check the arguments.
	//

	assert(ulBaud != 0);
	assert(ulUARTClk >= (ulBaud * UART_CLK_DIVIDER));

	//
	// Stop the UART.
	//
	Disable();

	//
	// Is the required baud rate greater than the maximum rate supported
	// without the use of high speed mode?
	//
	if ((ulBaud * 16) > ulUARTClk)
	{
		//
		// Enable high speed mode.
		//
		Regs->CTL |= UART_CTL_HSE;

		//
		// Half the supplied baud rate to compensate for enabling high speed
		// mode.  This allows the following code to be common to both cases.
		//
		ulBaud /= 2;
	}
	else
	{
		//
		// Disable high speed mode.
		//
		Regs->CTL &= ~(UART_CTL_HSE);
	}

	//
	// Compute the fractional baud rate divider.
	//
	ulDiv = (((ulUARTClk * 8) / ulBaud) + 1) / 2;

	//
	// Set the baud rate.
	//
	Regs->IBRD = ulDiv / 64;
	Regs->FBRD = ulDiv % 64;

	//
	// Set parity, data length, and number of stop bits.
	//
	Regs->LCRH = ulConfig;

	//
	// Clear the flags register.
	//
	Regs->FR = 0;

	//
	// Start the UART.
	//
	Enable();
}

void Usart::ConfigGetExpClk(uint32_t ulUARTClk, uint32_t * pulBaud,
		uint32_t * pulConfig)
{
	uint32_t ulInt, ulFrac;

	//
	// Compute the baud rate.
	//
	ulInt = Regs->IBRD;
	ulFrac = Regs->FBRD;
	*pulBaud = (ulUARTClk * 4) / ((64 * ulInt) + ulFrac);

	//
	// See if high speed mode enabled.
	//
	if (Regs->CTL & UART_CTL_HSE)
	{
		//
		// High speed mode is enabled so the actual baud rate is actually
		// double what was just calculated.
		//
		*pulBaud *= 2;
	}

	//
	// Get the parity, data length, and number of stop bits.
	//
	*pulConfig = (Regs->LCRH
			& (UART_LCRH_SPS | UART_LCRH_WLEN_M | UART_LCRH_STP2 |
			UART_LCRH_EPS | UART_LCRH_PEN));
}

void Usart::Enable()
{
//
// Enable the FIFO.
//
	Regs->LCRH |= UART_LCRH_FEN;

//
// Enable RX, TX, and the UART.
//
	Regs->CTL |= (UART_CTL_UARTEN | UART_CTL_TXE |
	UART_CTL_RXE);
}

void Usart::Disable()
{
//
// Wait for end of TX.
//
	while (Regs->FR & UART_FR_BUSY)
	{
	}

//
// Disable the FIFO.
//
	Regs->LCRH &= ~(UART_LCRH_FEN);

//
// Disable the UART.
//
	Regs->CTL &= ~(UART_CTL_UARTEN | UART_CTL_TXE |
	UART_CTL_RXE);
}

void Usart::FIFOEnable()
{
//
// Enable the FIFO.
//
	Regs->LCRH |= UART_LCRH_FEN;
}

void Usart::FIFODisable()
{
//
// Disable the FIFO.
//
	Regs->LCRH &= ~(UART_LCRH_FEN);
}

void Usart::EnableSIR(bool bLowPower)
{
//
// Enable SIR and SIRLP (if appropriate).
//
	if (bLowPower)
	{
		Regs->CTL |= (UART_CTL_SIREN | UART_CTL_SIRLP);
	}
	else
	{
		Regs->CTL |= (UART_CTL_SIREN);
	}
}

void Usart::DisableSIR()
{
//
// Disable SIR and SIRLP (if appropriate).
//
	Regs->CTL &= ~(UART_CTL_SIREN | UART_CTL_SIRLP);
}

bool Usart::CharsAvail()
{
//
// Return the availability of characters.
//
	return ((Regs->FR & UART_FR_RXFE) ? false : true);
}

bool Usart::SpaceAvail()
{

//
// Return the availability of space.
//
	return ((Regs->FR & UART_FR_TXFF) ? false : true);
}

long Usart::CharGetNonBlocking()
{
//
// See if there are any characters in the receive FIFO.
//
	if (!(Regs->FR & UART_FR_RXFE))
	{
		//
		// Read and return the next character.
		//
		return (Regs->DR);
	}
	else
	{
		//
		// There are no characters, so return a failure.
		//
		return (-1);
	}
}

long Usart::CharGet()
{

//
// Wait until a char is available.
//
	while (Regs->FR & UART_FR_RXFE)
	{
	}

//
// Now get the char.
//
	return (Regs->DR);
}

bool Usart::CharPutNonBlocking(uint8_t ucData)
{

//
// See if there is space in the transmit FIFO.
//
	if (!(Regs->FR & UART_FR_TXFF))
	{
		//
		// Write this character to the transmit FIFO.
		//
		Regs->DR = ucData;

		//
		// Success.
		//
		return (true);
	}
	else
	{
		//
		// There is no space in the transmit FIFO, so return a failure.
		//
		return (false);
	}
}

void Usart::CharPut(uint8_t ucData)
{

//
// Wait until space is available.
//
	while (Regs->FR & UART_FR_TXFF)
	{
	}

//
// Send the char.
//
	Regs->DR = ucData;
}

void Usart::BreakCtl(bool bBreakState)
{

//
// Set the break condition as requested.
//
	Regs->LCRH = (
			bBreakState ?
					(Regs->LCRH | UART_LCRH_BRK) :
					(Regs->LCRH & ~(UART_LCRH_BRK)));
}

bool Usart::Busy()
{

//
// Determine if the UART is busy.
//
	return ((Regs->FR & UART_FR_BUSY) ? true : false);
}

void Usart::IsrRegister(void (*pfnHandler)(void))
{
	uint32_t ulInt;

//
// Determine the interrupt number based on the UART port.
//
	ulInt = IntNumberGet((uint32_t) Regs);

//
// Register the interrupt handler.
//
	IntRegister(ulInt, pfnHandler);

//
// Enable the UART interrupt.
//
	IntEnable(ulInt);
}

void Usart::IsrUnregister()
{
	uint32_t ulInt;

//
// Determine the interrupt number based on the UART port.
//
	ulInt = IntNumberGet((uint32_t) Regs);

//
// Disable the interrupt.
//
	IntDisable(ulInt);

//
// Unregister the interrupt handler.
//
	IntUnregister(ulInt);
}

void Usart::IntEnable(uint32_t ulIntFlags)
{
//
// Check the arguments.
//

//
// Enable the specified interrupts.
//
	Regs->IM |= ulIntFlags;
}

void Usart::IntDisable(uint32_t ulIntFlags)
{

//
// Check the arguments.
//

//
// Disable the specified interrupts.
//
	Regs->IM &= ~(ulIntFlags);
}

uint32_t Usart::IntStatus(bool bMasked)
{
	//
	// Check the arguments.
	//

	//
	// Return either the interrupt status or the raw interrupt status as
	// requested.
	//
	if (bMasked)
	{
		return (Regs->MIS);
	}
	else
	{
		return (Regs->RIS);
	}
}

void Usart::IntClear(uint32_t ulIntFlags)
{
	//
	// Check the arguments.
	//

	//
	// Clear the requested interrupt sources.
	//
	Regs->ICR = ulIntFlags;
}

uint32_t Usart::RxErrorGet()
{
	//
	// Check the arguments.
	//

	//
	// Return the current value of the receive status register.
	//
	return (Regs->RSR & 0x0000000F);
}

void Usart::RxErrorClear()
{
//
// Any write to the Error Clear Register clears all bits which are
// currently set.
//
	Regs->RSR = 0;
}

void Usart::SmartCardEnable()
{
	uint32_t ulVal;

	//
	// Check the arguments.
	//
	assert(!CLASS_IS_SANDSTORM && !CLASS_IS_FURY && !CLASS_IS_DUSTDEVIL);

	//
	// Set 8-bit word length, even parity, 2 stop bits (note that although the
	// STP2 bit is ignored when in smartcard mode, this code lets the caller
	// read back the actual setting in use).
	//
	ulVal = Regs->LCRH;
	ulVal &= ~(UART_LCRH_SPS | UART_LCRH_EPS | UART_LCRH_PEN |
	UART_LCRH_WLEN_M);
	ulVal |= UART_LCRH_WLEN_8 | UART_LCRH_PEN | UART_LCRH_EPS | UART_LCRH_STP2;
	Regs->LCRH = ulVal;

	//
	// Enable SMART mode.
	//
	Regs->CTL |= UART_CTL_SMART;
}

void Usart::SmartCardDisable()
{
	//
	// Check the arguments.
	//
	assert(!CLASS_IS_SANDSTORM && !CLASS_IS_FURY && !CLASS_IS_DUSTDEVIL);

	//
	// Disable the SMART bit.
	//
	Regs->CTL &= ~UART_CTL_SMART;
}

void Usart::ModemControlSet(uint32_t ulControl)
{
	uint32_t ulTemp;

	//
	// Check the arguments.
	//
	assert(!CLASS_IS_SANDSTORM && !CLASS_IS_FURY && !CLASS_IS_DUSTDEVIL);
	assert((uint32_t)Regs == UART1_BASE);
	assert((ulControl & ~(UART_OUTPUT_RTS | UART_OUTPUT_DTR)) == 0);

	//
	// Set the appropriate modem control output bits.
	//
	ulTemp = Regs->CTL;
	ulTemp |= (ulControl & (UART_OUTPUT_RTS | UART_OUTPUT_DTR));
	Regs->CTL = ulTemp;
}

void Usart::ModemControlClear(uint32_t ulControl)
{
	uint32_t ulTemp;

	//
	// Check the arguments.
	//
	assert(!CLASS_IS_SANDSTORM && !CLASS_IS_FURY && !CLASS_IS_DUSTDEVIL);
	assert((uint32_t)Regs == UART1_BASE);
	assert((ulControl & ~(UART_OUTPUT_RTS | UART_OUTPUT_DTR)) == 0);

	//
	// Set the appropriate modem control output bits.
	//
	ulTemp = Regs->CTL;
	ulTemp &= ~(ulControl & (UART_OUTPUT_RTS | UART_OUTPUT_DTR));
	Regs->CTL = ulTemp;
}

uint32_t Usart::ModemControlGet()
{
//
// Check the arguments.
//
	assert(!CLASS_IS_SANDSTORM && !CLASS_IS_FURY && !CLASS_IS_DUSTDEVIL);
	assert((uint32_t)Regs == UART1_BASE);

	return (Regs->CTL & (UART_OUTPUT_RTS | UART_OUTPUT_DTR));
}

uint32_t Usart::ModemStatusGet()
{
	//
	// Check the arguments.
	//
	assert(!CLASS_IS_SANDSTORM && !CLASS_IS_FURY && !CLASS_IS_DUSTDEVIL);
	assert((uint32_t)Regs == UART1_BASE);

	return (Regs->FR & (UART_INPUT_RI | UART_INPUT_DCD |
	UART_INPUT_CTS | UART_INPUT_DSR));
}

void Usart::FlowControlSet(uint32_t ulMode)
{
	//
	// Check the arguments.
	//
	assert(!CLASS_IS_SANDSTORM && !CLASS_IS_FURY && !CLASS_IS_DUSTDEVIL);

	assert((ulMode & ~(UART_FLOWCONTROL_TX | UART_FLOWCONTROL_RX)) == 0);

	//
	// Set the flow control mode as requested.
	//
	Regs->CTL = ((Regs->CTL & ~(UART_FLOWCONTROL_TX | UART_FLOWCONTROL_RX))
			| ulMode);
}

uint32_t Usart::FlowControlGet()
{
//
// Check the arguments.
//
	assert(!CLASS_IS_SANDSTORM && !CLASS_IS_FURY && !CLASS_IS_DUSTDEVIL);

	return (Regs->CTL & (UART_FLOWCONTROL_TX | UART_FLOWCONTROL_RX));
}

void Usart::TxIntModeSet(uint32_t ulMode)
{
	//
	// Check the arguments.
	//

	assert((ulMode == UART_TXINT_MODE_EOT) || (ulMode == UART_TXINT_MODE_FIFO));

	//
	// Set or clear the EOT bit of the UART control register as appropriate.
	//
	Regs->CTL = ((Regs->CTL & ~(UART_TXINT_MODE_EOT | UART_TXINT_MODE_FIFO))
			| ulMode);
}

uint32_t Usart::TxIntModeGet()
{
	//
	// Return the current transmit interrupt mode.
	//
	return (Regs->CTL & (UART_TXINT_MODE_EOT |
	UART_TXINT_MODE_FIFO));
}

