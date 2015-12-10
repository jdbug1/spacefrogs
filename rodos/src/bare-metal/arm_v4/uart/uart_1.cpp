// WARNING: This file is generated automatically, do not edit!
//          Modify 'uart.cpp.in' instead and then run 'python generate.py'

#include "uart_1.h"
#include "queue.h"

#include "../lpc24xx_registers.h"
#include "../hw_uart_settings.h"

// ----------------------------------------------------------------------------
// Prototype for IRQ-Installation
extern void
installInterrupt(unsigned long source,
		unsigned long priority, void (* isrFunction)());

// ----------------------------------------------------------------------------
// InterruptID's for UART-Interrupts
#define UART_INT_RLS 	3		// Receive Line Status (is set on errors, see UxLSR[4:1])
#define UART_INT_RDA 	2		// Receive Data Available
#define UART_INT_CTI 	6		// Character Time-out indicator
#define UART_INT_THRE 	1		// THRE interrupt

// Bit Masks for Line Status Register
#define UART_TX_BUFFER_FREE 	0x20
#define UART_RX_HOLDS_CHAR 		0x01

// Bit Masks for Interrupt Information Register
#define UART_INT_PENDING_BM		0x01

// Bit definitions
#define	UART_FIFO_ENABLE				(1 << 0)
#define	UART_RX_FIFO_RESET				(1 << 1)
#define	UART_TX_FIFO_RESET				(1 << 2)

#define	UART_DLAB						(1 << 7)

#define	UART_ENABLE_RBR_INTERRUPT		(1 << 0)
#define	UART_ENABLE_THRE_INTERRUPT		(1 << 1)

// ----------------------------------------------------------------------------
namespace
{
	static xpcc::atomic::Queue<char, 128> receiveQueue;
}

// ----------------------------------------------------------------------------
// UART1 interrupt handler
static void
uart1_interrupt(void)
{
	uint32_t interruptInformation = U1IIR;
	if ((interruptInformation & UART_INT_PENDING_BM) == 0)
	{
		// At least one interrupt is pending => get causing interrupt
		uint32_t interruptCause = (interruptInformation & 0x0E) >> 1;
		
		switch (interruptCause)
		{
			// Receive Line Status, thrown on any kind of error (receive overrun (OE),
			// parity error (PE), framing error (FE) or break interrupt (BI)).
			// Error condition is in UxLSR[4:1].
			// The interrupt is cleared upon an UxLSR read.
			case UART_INT_RLS:
				(void) U1LSR;
				// TODO Error handling or reporting
				break;
			
			// The THRE is activated when the UARTn THR FIFO is empty
			case UART_INT_THRE:
				// TODO reactivate: UxTxEvent.propagate();
				break;
			
            // The RDA is activated when the UARTn Rx FIFO reaches the
			// trigger level defined in UnFCR[7:6] and is reset when the
			// UARTn Rx FIFO depth falls below the trigger level. When the
			// RDA interrupt goes active, the CPU can read a block of
			// data defined by the trigger level.
			case UART_INT_RDA:
				while (U1LSR & UART_RX_HOLDS_CHAR) {
					char c = U1RBR;
					if (!receiveQueue.push(c)) {
						// Error: Queue overflow
						// TODO Error handling or reporting
					}
				}
				break;
			
			// The CTI is set when the UARTnRx FIFO contains at least one
			// character and no UARTn Rx FIFO activity has occurred in 3.5
			// to 4.5 character times.
			// Any UARTn Rx FIFO activity (read or write of UARTn RSR) will
			// clear the interrupt. This interrupt is intended to flush the
			// UARTn RBR after a message has been received that is not a
			// multiple of the trigger level size
			case UART_INT_CTI:
				while (U1LSR & UART_RX_HOLDS_CHAR) {
					char c = U1RBR;
					if (!receiveQueue.push(c)) {
						// Error: Queue overflow
						// TODO Error handling or reporting
					}
				}
				break;
		}
	}
}

// ----------------------------------------------------------------------------
void
Uart1::initialize(uint8_t level)
{
	// Turn power on
	PCONP_p->peripheral.UART1 = 1;
	
	// Enable peripheral clock source
	PCLKSEL0_p->peripheral.UART1 = U1PCLK;
	
	// Baudrate
	// Set DLAB = 1 => access to DLL registers
	U1LCR = UART_DLAB | U1MODE;
	
	U1DLL = U1DLLVAL;
	U1DLM = U1DLMVAL;
	U1FDR = U1DIVMUL; // divaddval & mulval (Frac=0.22 s.b.)
	
	// Enable & clear FIFOs, set RX Trigger level
	U1FCR  = UART_FIFO_ENABLE | UART_RX_FIFO_RESET | UART_TX_FIFO_RESET |
			(level & 0x3) << 6;
	
	// Pins
	PINSEL4  |= (2 << 0);	// Set P2[0] to UART1 TXD
	PINSEL4  |= (2 << 2);	// Set P2[1] to UART1 RXD
	PINMODE4 |= (2 << 0);	// Set P2[0] to Neither pull-up or pull-down
	PINMODE4 |= (2 << 2); 	// Set P2[1] to Neither pull-up or pull-down
	
	// Interrupts
	// clear DLAB bit => access to UxIER and UxRBR
	U1LCR &= ~UART_DLAB;
	
	installInterrupt(iUART1, 0, uart1_interrupt);
	
	// Enable RBR (Receive Data Available) interrupt
	U1IER = UART_ENABLE_RBR_INTERRUPT;
}

// ----------------------------------------------------------------------------
bool
Uart1::write(char c, bool blocking)
{
	while (!(U1LSR & UART_TX_BUFFER_FREE))
	{
		if (!blocking) {
			// return immediately if a non blocking mode is requested
			return false;
		}
		
		// else wait until there is space in the send buffer (16-byte)
	}
	
	U1THR = c;
	return true;
}

// ----------------------------------------------------------------------------
bool
Uart1::isCharacterAvailable()
{
	return !receiveQueue.isEmpty();
}

bool
Uart1::read(char &c, bool blocking)
{
	do
	{
		if (!receiveQueue.isEmpty())
		{
			c = receiveQueue.get();
			receiveQueue.pop();
			return true;
		}
	} while (blocking);
	
	return false;
}