//writer : bonus adityas (bonusonic@gmail.com)
//4 march 2018 (revised in 15 may 2020)


#ifndef __STM8S_SPI_H
#define __STM8S_SPI_H


// SPI REGISTERS

#define SPI_BASE_ADDRESS 0x5200

#define SPI_CR1 *(volatile unsigned char*)(SPI_BASE_ADDRESS + 0x00)	// SPI Control Register 1
#define SPI_CR2 *(volatile unsigned char*)(SPI_BASE_ADDRESS + 0x01)	// SPI Control Register 2
#define SPI_ICR *(volatile unsigned char*)(SPI_BASE_ADDRESS + 0x02)	// SPI Interrupt Control Register
#define SPI_SR *(volatile unsigned char*)(SPI_BASE_ADDRESS + 0x03)	// SPI Status Register
#define SPI_DR *(volatile unsigned char*)(SPI_BASE_ADDRESS + 0x04)	// SPI Data Register
#define SPI_CRCPR *(volatile unsigned char*)(SPI_BASE_ADDRESS + 0x05)	// SPI CRC Polynomial Register
#define SPI_RXCRCR *(volatile unsigned char*)(SPI_BASE_ADDRESS + 0x06)	// SPI RX CRC Register
#define SPI_TXCRCR *(volatile unsigned char*)(SPI_BASE_ADDRESS + 0x07)	// SPI TX CRC Register


//SPI_CR1 bits
#define SPI_CR1_CPHA	  0	// Clock Phase
#define SPI_CR1_CPOL	  1	// Clock Polarity
#define SPI_CR1_MSTR	  2	// Master Selection
#define SPI_CR1_BR	  3	// Baud Rate Control bits
#define SPI_CR1_SPE	  6	// SPI Enable
#define SPI_CR1_LSBFIRST  7	// Frame Format

//SPI Baud Rate -> for BR
#define SPI_BR_fMASTER_2    0	// Baud Rate = fMASTER/2
#define SPI_BR_fMASTER_4    1	// Baud Rate = fMASTER/4
#define SPI_BR_fMASTER_8    2	// Baud Rate = fMASTER/8
#define SPI_BR_fMASTER_16   3	// Baud Rate = fMASTER/16
#define SPI_BR_fMASTER_32   4	// Baud Rate = fMASTER/32
#define SPI_BR_fMASTER_64   5	// Baud Rate = fMASTER/64
#define SPI_BR_fMASTER_128  6	// Baud Rate = fMASTER/128
#define SPI_BR_fMASTER_256  7	// Baud Rate = fMASTER/256

//SPI_CR2 bits
#define SPI_CR2_SSI	0	// Internal Slave Select
#define SPI_CR2_SSM	1	// Software Slave Management
#define SPI_CR2_RXONLY	2	// Receive Only
#define SPI_CR2_CRCNEXT	4	// Transmit CRC Next
#define SPI_CR2_CRCEN	5	// Hardware CRC Calculation Enable
#define SPI_CR2_BDOE	6	// Input/Output Enable in Bidirectional Mode
#define SPI_CR2_BDM	7	// Bidirectional Data Mode Enable

//SPI_ICR bits
#define SPI_ICR_WKIE	4	// Wakeup Interrupt Enable
#define SPI_ICR_ERRIE	5	// Error Interrupt Enable
#define SPI_ICR_RXIE	6	// RX Buffer Empty Interrupt Enable
#define SPI_ICR_TXIE	7	// TX Buffer Empty Interrupt Enable

//SPI_SR bits
#define SPI_SR_RXNE	0	// Receive Buffer Not Empty
#define SPI_SR_TXE	1	// Transmit Buffer Empty
#define SPI_SR_WKUP	3	// Wakeup Flag
#define SPI_SR_CRCERR	4	// CRC Error Flag
#define SPI_SR_MODF	5	// Mode Fault
#define SPI_SR_OVR	6	// Overrun Flag
#define SPI_SR_BSY	7	// Busy Flag


#endif
