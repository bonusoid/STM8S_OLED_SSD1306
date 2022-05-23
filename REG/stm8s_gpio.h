//writer : bonus adityas (bonusonic@gmail.com)
//18 august 2016 (revised in 17 may 2020)


#ifndef __STM8S_GPIO_H
#define __STM8S_GPIO_H


// GPIO REGISTERS

#define GPIO_BASE_ADDRESS 0x5000

#define PA_ODR *(volatile unsigned char*)(GPIO_BASE_ADDRESS + 0x00)	// Port A Output Data Register
#define PA_IDR *(volatile unsigned char*)(GPIO_BASE_ADDRESS + 0x01)	// Port A Input Data Register
#define PA_DDR *(volatile unsigned char*)(GPIO_BASE_ADDRESS + 0x02)	// Port A Data Direction Register
#define PA_CR1 *(volatile unsigned char*)(GPIO_BASE_ADDRESS + 0x03)	// Port A Control Register 1
#define PA_CR2 *(volatile unsigned char*)(GPIO_BASE_ADDRESS + 0x04)	// Port A Control Register 2

#define PB_ODR *(volatile unsigned char*)(GPIO_BASE_ADDRESS + 0x05)	// Port B Output Data Register
#define PB_IDR *(volatile unsigned char*)(GPIO_BASE_ADDRESS + 0x06)	// Port B Input Data Register
#define PB_DDR *(volatile unsigned char*)(GPIO_BASE_ADDRESS + 0x07)	// Port B Data Direction Register
#define PB_CR1 *(volatile unsigned char*)(GPIO_BASE_ADDRESS + 0x08)	// Port B Control Register 1
#define PB_CR2 *(volatile unsigned char*)(GPIO_BASE_ADDRESS + 0x09)	// Port B Control Register 2

#define PC_ODR *(volatile unsigned char*)(GPIO_BASE_ADDRESS + 0x0A)	// Port C Output Data Register
#define PC_IDR *(volatile unsigned char*)(GPIO_BASE_ADDRESS + 0x0B)	// Port C Input Data Register	
#define PC_DDR *(volatile unsigned char*)(GPIO_BASE_ADDRESS + 0x0C)	// Port C Data Direction Register
#define PC_CR1 *(volatile unsigned char*)(GPIO_BASE_ADDRESS + 0x0D)	// Port C Control Register 1
#define PC_CR2 *(volatile unsigned char*)(GPIO_BASE_ADDRESS + 0x0E)	// Port C Control Register 2

#define PD_ODR *(volatile unsigned char*)(GPIO_BASE_ADDRESS + 0x0F)	// Port D Output Data Register
#define PD_IDR *(volatile unsigned char*)(GPIO_BASE_ADDRESS + 0x10)	// Port D Input Data Register
#define PD_DDR *(volatile unsigned char*)(GPIO_BASE_ADDRESS + 0x11)	// Port D Data Direction Register
#define PD_CR1 *(volatile unsigned char*)(GPIO_BASE_ADDRESS + 0x12)	// Port D Control Register 1
#define PD_CR2 *(volatile unsigned char*)(GPIO_BASE_ADDRESS + 0x13)	// Port D Control Register 2

#define PE_ODR *(volatile unsigned char*)(GPIO_BASE_ADDRESS + 0x14)	// Port E Output Data Register
#define PE_IDR *(volatile unsigned char*)(GPIO_BASE_ADDRESS + 0x15)	// Port E Input Data Register
#define PE_DDR *(volatile unsigned char*)(GPIO_BASE_ADDRESS + 0x16)	// Port E Data Direction Register
#define PE_CR1 *(volatile unsigned char*)(GPIO_BASE_ADDRESS + 0x17)	// Port E Control Register 1
#define PE_CR2 *(volatile unsigned char*)(GPIO_BASE_ADDRESS + 0x18)	// Port E Control Register 2

#define PF_ODR *(volatile unsigned char*)(GPIO_BASE_ADDRESS + 0x19)	// Port F Output Data Register
#define PF_IDR *(volatile unsigned char*)(GPIO_BASE_ADDRESS + 0x1A)	// Port F Input Data Register
#define PF_DDR *(volatile unsigned char*)(GPIO_BASE_ADDRESS + 0x1B)	// Port F Data Direction Register
#define PF_CR1 *(volatile unsigned char*)(GPIO_BASE_ADDRESS + 0x1C)	// Port F Control Register 1
#define PF_CR2 *(volatile unsigned char*)(GPIO_BASE_ADDRESS + 0x1D)	// Port F Control Register 2

//PINs
#define P0	0
#define P1	1
#define P2	2
#define P3	3
#define P4	4
#define P5	5
#define P6	6
#define P7	7

//DATA DIRECTION MODE -> for Px_DDR_n
#define INPUT	0
#define OUTPUT	1

//IO STATE
#define LOW	0
#define HIGH	1

//IO MASK
#define P0_MASKL	0xFE
#define P1_MASKL	0xFD
#define P2_MASKL	0xFB
#define P3_MASKL	0xF7
#define P4_MASKL	0xEF
#define P5_MASKL	0xDF
#define P6_MASKL	0xBF
#define P7_MASKL	0x7F
#define P0_MASKH	0x01
#define P1_MASKH	0x02
#define P2_MASKH	0x04
#define P3_MASKH	0x08
#define P4_MASKH	0x10
#define P5_MASKH	0x20
#define P6_MASKH	0x40
#define P7_MASKH	0x80

//INPUT MODE
#define floating 	0  // for Px_CR1_n
#define pullup 		1  // for Px_CR1_n
#define exti_disabled	0  // for Px_CR2_n
#define exti_enabled	1  // for Px_CR2_n

//OUTPUT MODE
#define opendrain 	0  // for Px_CR1_n
#define pushpull	1  // for Px_CR1_n
#define speed_2MHz	0  // for Px_CR2_n
#define speed_10MHz	1  // for Px_CR2_n


#endif

