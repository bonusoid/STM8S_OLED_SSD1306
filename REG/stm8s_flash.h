//writer : bonus adityas (bonusonic@gmail.com)
//23 april 2020 (revised in 17 may 2020)


#ifndef __STM8S_FLASH_H
#define __STM8S_FLASH_H


// FLASH REGISTERS

#define FLASH_BASE_ADDRESS 0x505A

#define FLASH_CR1	*(volatile unsigned char*)(FLASH_BASE_ADDRESS + 0x00)	// Flash Control Register 1
#define FLASH_CR2	*(volatile unsigned char*)(FLASH_BASE_ADDRESS + 0x01)	// Flash Control Register 2
#define FLASH_NCR2	*(volatile unsigned char*)(FLASH_BASE_ADDRESS + 0x02)	// Flash Complementary Control Register 2
#define FLASH_FPR	*(volatile unsigned char*)(FLASH_BASE_ADDRESS + 0x03)	// Flash Protection Register
#define FLASH_NFPR	*(volatile unsigned char*)(FLASH_BASE_ADDRESS + 0x04)   // Flash Complementary Protection Register
#define FLASH_IAPSR	*(volatile unsigned char*)(FLASH_BASE_ADDRESS + 0x05)	// Flash Status Register
#define FLASH_PUKR	*(volatile unsigned char*)(FLASH_BASE_ADDRESS + 0x08)	// Flash Program Memory Unprotecting Key Register
#define FLASH_DUKR	*(volatile unsigned char*)(FLASH_BASE_ADDRESS + 0x0A)	// Data EEPROM Unprotection Key Register

//FLASH_CR1 bits
#define FLASH_CR1_FIX	0	// Fixed Byte Programming Time
#define FLASH_CR1_IE	1	// Flash Interrupt Enable
#define FLASH_CR1_AHALT	2	// Power-Down in Active-Halt Mode
#define FLASH_CR1_HALT	3	// Power-Down in Halt Mode

//FLASH_CR2 bits
#define FLASH_CR2_PRG	0	// Standard Block Programming
#define FLASH_CR2_FPRG	4	// Fast Block Programming
#define FLASH_CR2_ERASE	5	// Block Erasing
#define FLASH_CR2_WPRG	6	// Word Programming
#define FLASH_CR2_OPT	7	// Write Option Bytes

//FLASH_NCR2 bits
#define FLASH_NCR2_NPRG		0	// Block Programming
#define FLASH_NCR2_NFPRG	4	// Fast Block Programming
#define FLASH_NCR2_NERASE	5	// Block Erasing
#define FLASH_NCR2_NWPRG	6	// Word Programming
#define FLASH_NCR2_NOPT		7	// Write Option Bytes

//FLASH_IAPSR bits
#define FLASH_IAPSR_WR_PG_DIS 	0	// Write Attempted to Protected Page Flag
#define FLASH_IAPSR_PUL	  	1	// Flash Program Memory Unlocked Flag
#define FLASH_IAPSR_EOP 	2	// End of Programming (Write or Erase Operation) Flag
#define FLASH_IAPSR_DUL       	3	// Data EEPROM area Unlocked Flag
#define FLASH_IAPSR_HVOFF     	6	// End of High Voltage Flag

#endif
