//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu
//

// ----------------------------------------------------------------------------

#include "cortexm/ExceptionHandlers.h"

// ----------------------------------------------------------------------------

void __attribute__((weak))
Default_Handler(void);

// Forward declaration of the specific IRQ handlers. These are aliased
// to the Default_Handler, which is a 'forever' loop. When the application
// defines a handler (with the same name), this will automatically take
// precedence over these weak definitions
//
// TODO: Rename this and add the actual routines here.

void __attribute__ ((weak, alias ("Default_Handler")))
GPIOA_Handler(void);               // GPIO Port A
void __attribute__ ((weak, alias ("Default_Handler")))
GPIOB_Handler(void);               // GPIO Port B
void __attribute__ ((weak, alias ("Default_Handler")))
GPIOC_Handler(void);               // GPIO Port C
void __attribute__ ((weak, alias ("Default_Handler")))
GPIOD_Handler(void);               // GPIO Port D
void __attribute__ ((weak, alias ("Default_Handler")))
GPIOE_Handler(void);               // GPIO Port E
void __attribute__ ((weak, alias ("Default_Handler")))
UART0_Handler(void);               // UART0 Rx and Tx
void __attribute__ ((weak, alias ("Default_Handler")))
UART1_Handler(void);               // UART1 Rx and Tx
void __attribute__ ((weak, alias ("Default_Handler")))
SSI0_Handler(void);                // SSI0 Rx and Tx
void __attribute__ ((weak, alias ("Default_Handler")))
I2C0_Handler(void);                // I2C0 Master and Slave
void __attribute__ ((weak, alias ("Default_Handler")))
PMW0_FAULT_Handler(void);          // PWM Fault
void __attribute__ ((weak, alias ("Default_Handler")))
PWM0_0_Handler(void);              // PWM Generator 0
void __attribute__ ((weak, alias ("Default_Handler")))
PWM0_1_Handler(void);              // PWM Generator 1
void __attribute__ ((weak, alias ("Default_Handler")))
PWM0_2_Handler(void);              // PWM Generator 2
void __attribute__ ((weak, alias ("Default_Handler")))
QEI0_Handler(void);                // Quadrature Encoder 0
void __attribute__ ((weak, alias ("Default_Handler")))
ADC0SS0_Handler(void);             // ADC Sequence 0
void __attribute__ ((weak, alias ("Default_Handler")))
ADC0SS1_Handler(void);             // ADC Sequence 1
void __attribute__ ((weak, alias ("Default_Handler")))
ADC0SS2_Handler(void);             // ADC Sequence 2
void __attribute__ ((weak, alias ("Default_Handler")))
ADC0SS3_Handler(void);             // ADC Sequence 3
void __attribute__ ((weak, alias ("Default_Handler")))
WDT0_Handler(void);                // Watchdog timer
void __attribute__ ((weak, alias ("Default_Handler")))
TIMER0A_Handler(void);             // Timer 0 subtimer A
void __attribute__ ((weak, alias ("Default_Handler")))
TIMER0B_Handler(void);             // Timer 0 subtimer B
void __attribute__ ((weak, alias ("Default_Handler")))
TIMER1A_Handler(void);             // Timer 1 subtimer A
void __attribute__ ((weak, alias ("Default_Handler")))
TIMER1B_Handler(void);             // Timer 1 subtimer B
void __attribute__ ((weak, alias ("Default_Handler")))
TIMER2A_Handler(void);             // Timer 2 subtimer A
void __attribute__ ((weak, alias ("Default_Handler")))
TIMER2B_Handler(void);             // Timer 2 subtimer B
void __attribute__ ((weak, alias ("Default_Handler")))
COMP0_Handler(void);               // Analog Comparator 0
void __attribute__ ((weak, alias ("Default_Handler")))
COMP1_Handler(void);               // Analog Comparator 1
void __attribute__ ((weak, alias ("Default_Handler")))
COMP2_Handler(void);               // Analog Comparator 2
void __attribute__ ((weak, alias ("Default_Handler")))
SYSCTL_Handler(void);              // System Control (PLL(void); OSC(void); BO)
void __attribute__ ((weak, alias ("Default_Handler")))
FLASH_Handler(void);               // FLASH Control
void __attribute__ ((weak, alias ("Default_Handler")))
GPIOF_Handler(void);               // GPIO Port F
void __attribute__ ((weak, alias ("Default_Handler")))
GPIOG_Handler(void);               // GPIO Port G
void __attribute__ ((weak, alias ("Default_Handler")))
GPIOH_Handler(void);               // GPIO Port H
void __attribute__ ((weak, alias ("Default_Handler")))
UART2_Handler(void);               // UART2 Rx and Tx
void __attribute__ ((weak, alias ("Default_Handler")))
SSI1_Handler(void);                // SSI1 Rx and Tx
void __attribute__ ((weak, alias ("Default_Handler")))
TIMER3A_Handler(void);             // Timer 3 subtimer A
void __attribute__ ((weak, alias ("Default_Handler")))
TIMER3B_Handler(void);             // Timer 3 subtimer B
void __attribute__ ((weak, alias ("Default_Handler")))
I2C1_Handler(void);                // I2C1 Master and Slave
void __attribute__ ((weak, alias ("Default_Handler")))
QEI1_Handler(void);                // Quadrature Encoder 1
void __attribute__ ((weak, alias ("Default_Handler")))
CAN0_Handler(void);                // CAN0
void __attribute__ ((weak, alias ("Default_Handler")))
CAN1_Handler(void);                // CAN1
void __attribute__ ((weak, alias ("Default_Handler")))
CAN2_Handler(void);                // CAN2
void __attribute__ ((weak, alias ("Default_Handler")))
ETH0_Handler(void);                // Ethernet
void __attribute__ ((weak, alias ("Default_Handler")))
HIB_Handler(void);                 // Hibernate
void __attribute__ ((weak, alias ("Default_Handler")))
USB0_Handler(void);                // USB0
void __attribute__ ((weak, alias ("Default_Handler")))
PWM0_3_Handler(void);              // PWM Generator 3
void __attribute__ ((weak, alias ("Default_Handler")))
UDMA_Handler(void);                // uDMA Software Transfer
void __attribute__ ((weak, alias ("Default_Handler")))
UDMAERR_Handler(void);             // uDMA Error
void __attribute__ ((weak, alias ("Default_Handler")))
ADC1SS0_Handler(void);             // ADC1 Sequence 0
void __attribute__ ((weak, alias ("Default_Handler")))
ADC1SS1_Handler(void);             // ADC1 Sequence 1
void __attribute__ ((weak, alias ("Default_Handler")))
ADC1SS2_Handler(void);             // ADC1 Sequence 2
void __attribute__ ((weak, alias ("Default_Handler")))
ADC1SS3_Handler(void);             // ADC1 Sequence 3
void __attribute__ ((weak, alias ("Default_Handler")))
I2S0_Handler(void);                // I2S0
void __attribute__ ((weak, alias ("Default_Handler")))
EPI0_Handler(void);                // External Bus Interface 0
void __attribute__ ((weak, alias ("Default_Handler")))
GPIOJ_Handler(void);               // GPIO Port J

// ----------------------------------------------------------------------------

extern unsigned int _estack;

typedef void
(* const pHandler)(void);

// ----------------------------------------------------------------------------

// The vector table.
// This relies on the linker script to place at correct location in memory.

__attribute__ ((section(".isr_vector"),used))
pHandler __isr_vectors[] =
  { //
    (pHandler) &_estack,                          // The initial stack pointer
        Reset_Handler,                            // The reset handler

        NMI_Handler,                              // The NMI handler
        HardFault_Handler,                        // The hard fault handler

#if defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__)
        MemManage_Handler,                        // The MPU fault handler
        BusFault_Handler,// The bus fault handler
        UsageFault_Handler,// The usage fault handler
#else
        0, 0, 0,				  // Reserved
#endif
        0,                                        // Reserved
        0,                                        // Reserved
        0,                                        // Reserved
        0,                                        // Reserved
        SVC_Handler,                              // SVCall handler
#if defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__)
        DebugMon_Handler,                         // Debug monitor handler
#else
        0,					  // Reserved
#endif
        0,                                        // Reserved
        PendSV_Handler,                           // The PendSV handler
        SysTick_Handler,                          // The SysTick handler

        // ----------------------------------------------------------------------
        // LM3S8962 vectors
		GPIOA_Handler,               // GPIO Port A
		GPIOB_Handler,               // GPIO Port B
		GPIOC_Handler,               // GPIO Port C
		GPIOD_Handler,               // GPIO Port D
		GPIOE_Handler,               // GPIO Port E
		UART0_Handler,               // UART0 Rx and Tx
		UART1_Handler,               // UART1 Rx and Tx
		SSI0_Handler,                // SSI0 Rx and Tx
		I2C0_Handler,                // I2C0 Master and Slave
		PMW0_FAULT_Handler,          // PWM Fault
		PWM0_0_Handler,              // PWM Generator 0
		PWM0_1_Handler,              // PWM Generator 1
		PWM0_2_Handler,              // PWM Generator 2
		QEI0_Handler,                // Quadrature Encoder 0
		ADC0SS0_Handler,             // ADC Sequence 0
		ADC0SS1_Handler,             // ADC Sequence 1
		ADC0SS2_Handler,             // ADC Sequence 2
		ADC0SS3_Handler,             // ADC Sequence 3
		WDT0_Handler,                // Watchdog timer
		TIMER0A_Handler,             // Timer 0 subtimer A
		TIMER0B_Handler,             // Timer 0 subtimer B
		TIMER1A_Handler,             // Timer 1 subtimer A
		TIMER1B_Handler,             // Timer 1 subtimer B
		TIMER2A_Handler,             // Timer 2 subtimer A
		TIMER2B_Handler,             // Timer 2 subtimer B
		COMP0_Handler,               // Analog Comparator 0
		COMP1_Handler,               // Analog Comparator 1
		COMP2_Handler,               // Analog Comparator 2
		SYSCTL_Handler,              // System Control (PLL, OSC, BO)
		FLASH_Handler,               // FLASH Control
		GPIOF_Handler,               // GPIO Port F
		GPIOG_Handler,               // GPIO Port G
		GPIOH_Handler,               // GPIO Port H
		UART2_Handler,               // UART2 Rx and Tx
		SSI1_Handler,                // SSI1 Rx and Tx
		TIMER3A_Handler,             // Timer 3 subtimer A
		TIMER3B_Handler,             // Timer 3 subtimer B
		I2C1_Handler,                // I2C1 Master and Slave
		QEI1_Handler,                // Quadrature Encoder 1
		CAN0_Handler,                // CAN0
		CAN1_Handler,                // CAN1
		CAN2_Handler,                // CAN2
		ETH0_Handler,                // Ethernet
		HIB_Handler,                 // Hibernate
		USB0_Handler,                // USB0
		PWM0_3_Handler,              // PWM Generator 3
		UDMA_Handler,                // uDMA Software Transfer
		UDMAERR_Handler,             // uDMA Error
		ADC1SS0_Handler,             // ADC1 Sequence 0
		ADC1SS1_Handler,             // ADC1 Sequence 1
		ADC1SS2_Handler,             // ADC1 Sequence 2
		ADC1SS3_Handler,             // ADC1 Sequence 3
		I2S0_Handler,                // I2S0
		EPI0_Handler,                // External Bus Interface 0
		GPIOJ_Handler,               // GPIO Port J
    // TODO: rename and add more vectors here
    };

// ----------------------------------------------------------------------------

// Processor ends up here if an unexpected interrupt occurs or a specific
// handler is not present in the application code.

void __attribute__ ((section(".after_vectors")))
Default_Handler(void)
{
#if defined(DEBUG)
  __DEBUG_BKPT();
#endif
  while (1)
    {
    }
}

// ----------------------------------------------------------------------------
