//writer : bonus adityas (bonusonic@gmail.com)
//4 march 2018 (revised in 19 may 2020)


#ifndef __STM8S_UART1_H
#define __STM8S_UART1_H


// UART1 REGISTERS

#define UART1_BASE_ADDRESS 0x5230

#define UART1_SR *(volatile unsigned char*)(UART1_BASE_ADDRESS + 0x00)		// UART1 Status Register
#define UART1_DR *(volatile unsigned char*)(UART1_BASE_ADDRESS + 0x01)		// UART1 Data Register
#define UART1_BRR1 *(volatile unsigned char*)(UART1_BASE_ADDRESS + 0x02)	// UART1 Baud Rate Register 1
#define UART1_BRR2 *(volatile unsigned char*)(UART1_BASE_ADDRESS + 0x03)	// UART1 Baud Rate Register 2
#define UART1_CR1 *(volatile unsigned char*)(UART1_BASE_ADDRESS + 0x04)		// UART1 Control Register 1
#define UART1_CR2 *(volatile unsigned char*)(UART1_BASE_ADDRESS + 0x05)		// UART1 Control Register 2
#define UART1_CR3 *(volatile unsigned char*)(UART1_BASE_ADDRESS + 0x06)		// UART1 Control Register 3
#define UART1_CR4 *(volatile unsigned char*)(UART1_BASE_ADDRESS + 0x07)		// UART1 Control Register 4
#define UART1_CR5 *(volatile unsigned char*)(UART1_BASE_ADDRESS + 0x08)		// UART1 Control Register 5
#define UART1_GTR *(volatile unsigned char*)(UART1_BASE_ADDRESS + 0x09)		// UART1 Guard Time Register
#define UART1_PSCR *(volatile unsigned char*)(UART1_BASE_ADDRESS + 0x0A)	// UART1 Prescaler Register


//UART1_SR bits
#define UART1_SR_PE	0	// Parity Error
#define UART1_SR_FE	1	// Framing Error
#define UART1_SR_NF	2	// Noise Flag
#define UART1_SR_OR	3	// Overrun Error
#define UART1_SR_IDLE	4	// IDLE Line Detected
#define UART1_SR_RXNE	5	// Read Data Register not Empty
#define UART1_SR_TC	6	// Transmission Complete
#define UART1_SR_TXE	7	// Transmit Data Register Empty

//UART1_BRR1 bits
//bit 4-11

//UART1_BRR2 bits
//bit 0-3 (LSB)
//bit 12-15 (MSB)

//UART1_CR1 bits
#define UART1_CR1_PIEN	0	// Parity Interrupt Enable
#define UART1_CR1_PS	1	// Parity Selection
#define UART1_CR1_PCEN	2	// Parity Control Enable
#define UART1_CR1_WAKE	3	// Wakeup Method
#define UART1_CR1_M	4	// Word Length
#define UART1_CR1_UARTD	5	// UART Disable
#define UART1_CR1_T8	6	// Transmit Data Bit 8
#define UART1_CR1_R8	7	// Receive Data Bit 8

//UART1_CR2 bits
#define UART1_CR2_SBK	0	// Send Break
#define UART1_CR2_RWU	1	// Receiver Wakeup
#define UART1_CR2_REN	2	// Receiver Enable
#define UART1_CR2_TEN	3	// Transmitter Enable
#define UART1_CR2_ILIEN	4	// IDLE Line Interrupt Enable
#define UART1_CR2_RIEN	5	// Receiver Interrupt Enable
#define UART1_CR2_TCIEN	6	// Transmission Complete Interrupt Enable
#define UART1_CR2_TIEN	7	// Transmitter Interrupt Enable

//UART1_CR3 bits
#define UART1_CR3_LBCL	0	// Last Bit Clock Pulse
#define UART1_CR3_CPHA	1	// Clock Phase
#define UART1_CR3_CPOL	2	// Clock Polarity
#define UART1_CR3_CKEN	3	// Clock Enable
#define UART1_CR3_STOP	4	// STOP bits
#define UART1_CR3_LINEN	6	// LIN Mode Enable

//STOP bits
#define UART_1_Stop_Bit		0	// 1 Stop bit
#define UART_2_Stop_Bit		2	// 2 Stop bits
#define UART_1_5_Stop_Bit	3	// 1.5 Stop bits

//UART1_CR4 bits
#define UART1_CR4_ADD		0	// Address of the UART node
#define UART1_CR4_LBDF		4	// LIN Break Detection Flag
#define UART1_CR4_LBDL		5	// LIN Break Detection Length
#define UART1_CR4_LBDIEN	6	// LIN Break Detection Interrupt Enable

//UART1_CR5 bits
#define UART1_CR5_IREN	1	// IrDA Mode Enable
#define UART1_CR5_IRLP	2	// IrDA Low Power
#define UART1_CR5_HDSEL	3	// Half-Duplex Selection
#define UART1_CR5_NACK	4	// Smartcard NACK Enable
#define UART1_CR5_SCEN	5	// Smartcard Mode Enable

//UART1_GTR bits
//Guard time value in terms of number of baud clocks (for Smartcard Mode)

//UART1_PSCR bits
//IrDA Low Power Mode -> clock divider (bit 0-7)
//Smartcard Mode -> clock divider multiplied by 2 (bit 0-5)


#endif
