//writer : bonus adityas (bonusonic@gmail.com)
//4 march 2018 (revised in 18 may 2020)


#ifndef __STM8S_I2C_H
#define __STM8S_I2C_H


// I2C REGISTERS

#define I2C_BASE_ADDRESS 0x5210

#define I2C_CR1 *(volatile unsigned char*)(I2C_BASE_ADDRESS + 0x00)	// I2C Control Register 1
#define I2C_CR2 *(volatile unsigned char*)(I2C_BASE_ADDRESS + 0x01)	// I2C Control Register 2
#define I2C_FREQR *(volatile unsigned char*)(I2C_BASE_ADDRESS + 0x02)	// I2C Frequency Register
#define I2C_OARL *(volatile unsigned char*)(I2C_BASE_ADDRESS + 0x03)	// I2C Address Register LSB
#define I2C_OARH *(volatile unsigned char*)(I2C_BASE_ADDRESS + 0x04)	// I2C Address Register MSB
#define I2C_DR *(volatile unsigned char*)(I2C_BASE_ADDRESS + 0x06)	// I2C Data Register
#define I2C_SR1 *(volatile unsigned char*)(I2C_BASE_ADDRESS + 0x07)	// I2C Status Register 1
#define I2C_SR2 *(volatile unsigned char*)(I2C_BASE_ADDRESS + 0x08)	// I2C Status Register 2
#define I2C_SR3 *(volatile unsigned char*)(I2C_BASE_ADDRESS + 0x09)	// I2C Status Register 3
#define I2C_ITR *(volatile unsigned char*)(I2C_BASE_ADDRESS + 0x0A)	// I2C Interrupt Register
#define I2C_CCRL *(volatile unsigned char*)(I2C_BASE_ADDRESS + 0x0B)	// I2C Clock Control Register Low
#define I2C_CCRH *(volatile unsigned char*)(I2C_BASE_ADDRESS + 0x0C)	// I2C Clock Control Register High
#define I2C_TRISER *(volatile unsigned char*)(I2C_BASE_ADDRESS + 0x0D)	// I2C TRISE Register
#define I2C_PECR *(volatile unsigned char*)(I2C_BASE_ADDRESS + 0x0E)	// I2C Packet Error Checking Register


//I2C_CR1 bits
#define I2C_CR1_PE	   0	// Peripheral Enable
#define I2C_CR1_ENGC	   6	// General Call Enable
#define I2C_CR1_NOSTRETCH  7	// Clock Stretching Disable

//I2C_CR2 bits
#define I2C_CR2_START	0	// Start Generation
#define I2C_CR2_STOP	1	// Stop Generation
#define I2C_CR2_ACK	2	// Acknowledge Enable
#define I2C_CR2_POS	3	// Acknowledge Position
#define I2C_CR2_SWRST	7	// Software Reset

//I2C Peripheral Clock Frequency -> for I2C_FREQR
// 1-24 MHz -> input 1-24
// 1 MHz for Standard Mode
// 4 MHz for Fast Mode

//I2C_OARH bits
#define I2C_OARH_ADD8		1	// 10-bit Addressing bit 8
#define I2C_OARH_ADD9		2	// 10-bit Addressing bit 9
#define I2C_OARH_ADDCONF	6	// Address Mode Configuration
#define I2C_OARH_ADDMODE	7	// Addressing Mode (Slave Mode)

//I2C_SR1 bits
#define I2C_SR1_SB	0	// Start Bit
#define I2C_SR1_ADDR	1	// Address Sent (Master Mode) / Address Matched (Slave Mode)
#define I2C_SR1_BTF	2	// Byte Transfer Finished
#define I2C_SR1_ADD10	3	// 10-bit Header Sent (Master Mode)
#define I2C_SR1_STOPF	4	// Stop Detection (Slave Mode)
#define I2C_SR1_RXNE	6	// Data Register not Empty (Receivers)
#define I2C_SR1_TXE	7	// Data Register Empty (Transmitters)

//I2C_SR2 bits
#define I2C_SR2_BERR	0	// Bus Error
#define I2C_SR2_ARLO	1	// Arbitration Lost (Master Mode)
#define I2C_SR2_AF	2	// Acknowledge Failure
#define I2C_SR2_OVR	3	// Overrun/Underrun
#define I2C_SR2_WUFH	5	// Wakeup from Halt

//I2C_SR3 bits
#define I2C_SR3_MSL	0	// Master/Slave
#define I2C_SR3_BUSY	1	// Bus Busy
#define I2C_SR3_TRA	2	// Transmitter/Receiver
#define I2C_SR3_GENCALL	4	// General Call Header (Slave Mode)
#define I2C_SR3_DUALF	7	// Dual Flag (Slave Mode)

//I2C_ITR bits
#define I2C_ITR_ITERREN	0	// Error Interrupt Enable
#define I2C_ITR_ITEVTEN	1	// Event Interrupt Enable
#define I2C_ITR_ITBUFEN	2	// Buffer Interrupt Enable

//I2C_CCRH bits
#define I2C_CCRH_CCR8	0	// CCR bit 8
#define I2C_CCRH_CCR9	1	// CCR bit 9
#define I2C_CCRH_CCR10	2	// CCR bit 10
#define I2C_CCRH_CCR11	3	// CCR bit 11
#define I2C_CCRH_DUTY	6	// Fast Mode Duty Cycle
#define I2C_CCRH_FS	7	// I2C Master Mode Selection

//I2C Data Direction -> Combine with Slave Address (Master Mode)
#define I2C_WRITE 0xFE	// I2C_DR = (I2C_ADDR<<1)&I2C_WRITE
#define I2C_READ  0x01	// I2C_DR = (I2C_ADDR<<1)|I2C_READ


#endif
