//writer : bonus adityas (bonusonic@gmail.com)
//20 may 2020


#ifndef __STM8S_UART4_H
#define __STM8S_UART4_H


// UART1 REGISTERS

#define UART4_BASE_ADDRESS 0x524A

#define UART4_SR *(volatile unsigned char*)(UART1_BASE_ADDRESS + 0x00)		// UART4 Status Register
#define UART4_DR *(volatile unsigned char*)(UART1_BASE_ADDRESS + 0x01)		// UART4 Data Register
#define UART4_BRR1 *(volatile unsigned char*)(UART1_BASE_ADDRESS + 0x02)	// UART4 Baud Rate Register 1
#define UART4_BRR2 *(volatile unsigned char*)(UART1_BASE_ADDRESS + 0x03)	// UART4 Baud Rate Register 2
#define UART4_CR1 *(volatile unsigned char*)(UART1_BASE_ADDRESS + 0x04)		// UART4 Control Register 1
#define UART4_CR2 *(volatile unsigned char*)(UART1_BASE_ADDRESS + 0x05)		// UART4 Control Register 2
#define UART4_CR3 *(volatile unsigned char*)(UART1_BASE_ADDRESS + 0x06)		// UART4 Control Register 3
#define UART4_CR4 *(volatile unsigned char*)(UART1_BASE_ADDRESS + 0x07)		// UART4 Control Register 4
#define UART4_CR5 *(volatile unsigned char*)(UART1_BASE_ADDRESS + 0x08)		// UART4 Control Register 5
#define UART4_CR6 *(volatile unsigned char*)(UART1_BASE_ADDRESS + 0x09)		// UART4 Control Register 6
#define UART4_GTR *(volatile unsigned char*)(UART1_BASE_ADDRESS + 0x0A)		// UART4 Guard Time Register
#define UART4_PSCR *(volatile unsigned char*)(UART1_BASE_ADDRESS + 0x0B)	// UART4 Prescaler Register


//UART4_SR bits
#define UART4_SR_PE	0	// Parity Error
#define UART4_SR_FE	1	// Framing Error
#define UART4_SR_NF	2	// Noise Flag
#define UART4_SR_OR	3	// Overrun Error
#define UART4_SR_IDLE	4	// IDLE Line Detected
#define UART4_SR_RXNE	5	// Read Data Register not Empty
#define UART4_SR_TC	6	// Transmission Complete
#define UART4_SR_TXE	7	// Transmit Data Register Empty

//UART4_BRR1 bits
//bit 4-11

//UART4_BRR2 bits
//bit 0-3 (LSB)
//bit 12-15 (MSB)

//UART4_CR1 bits
#define UART4_CR1_PIEN	0	// Parity Interrupt Enable
#define UART4_CR1_PS	1	// Parity Selection
#define UART4_CR1_PCEN	2	// Parity Control Enable
#define UART4_CR1_WAKE	3	// Wakeup Method
#define UART4_CR1_M	4	// Word Length
#define UART4_CR1_UARTD	5	// UART Disable
#define UART4_CR1_T8	6	// Transmit Data Bit 8
#define UART4_CR1_R8	7	// Receive Data Bit 8

//UART4_CR2 bits
#define UART4_CR2_SBK	0	// Send Break
#define UART4_CR2_RWU	1	// Receiver Wakeup
#define UART4_CR2_REN	2	// Receiver Enable
#define UART4_CR2_TEN	3	// Transmitter Enable
#define UART4_CR2_ILIEN	4	// IDLE Line Interrupt Enable
#define UART4_CR2_RIEN	5	// Receiver Interrupt Enable
#define UART4_CR2_TCIEN	6	// Transmission Complete Interrupt Enable
#define UART4_CR2_TIEN	7	// Transmitter Interrupt Enable

//UART4_CR3 bits
#define UART4_CR3_LBCL	0	// Last Bit Clock Pulse
#define UART4_CR3_CPHA	1	// Clock Phase
#define UART4_CR3_CPOL	2	// Clock Polarity
#define UART4_CR3_CKEN	3	// Clock Enable
#define UART4_CR3_STOP	4	// STOP bits
#define UART4_CR3_LINEN	6	// LIN Mode Enable

//STOP bits
#define UART_1_Stop_Bit		0	// 1 Stop bit
#define UART_2_Stop_Bit		2	// 2 Stop bits
#define UART_1_5_Stop_Bit	3	// 1.5 Stop bits

//UART4_CR4 bits
#define UART4_CR4_ADD		0	// Address of the UART node
#define UART4_CR4_LBDF		4	// LIN Break Detection Flag
#define UART4_CR4_LBDL		5	// LIN Break Detection Length
#define UART4_CR4_LBDIEN	6	// LIN Break Detection Interrupt Enable

//UART4_CR5 bits
#define UART4_CR5_IREN	1	// IrDA Mode Enable
#define UART4_CR5_IRLP	2	// IrDA Low Power
#define UART4_CR5_HDSEL	3	// Half-Duplex Selection
#define UART4_CR5_NACK	4	// Smartcard NACK Enable
#define UART4_CR5_SCEN	5	// Smartcard Mode Enable

//UART4_CR6 bits
#define UART4_CR6_LSF		0	// LIN Sync Field
#define UART4_CR6_LHDF		1	// LIN Header Detection Flag
#define UART4_CR6_LHDIEN	2	// LIN Header Detection Interrupt Enable
#define UART4_CR6_LASE		4	// LIN Automatic Resynchronisation Enable
#define UART4_CR6_LSLV		5	// LIN Slave Enable
#define UART4_CR6_LDUM		7	// LIN Divider Update Method

//UART4_GTR bits
//Guard time value in terms of number of baud clocks (for Smartcard Mode)

//UART4_PSCR bits
//IrDA Low Power Mode -> clock divider (bit 0-7)
//Smartcard Mode -> clock divider multiplied by 2 (bit 0-5)


#endif
