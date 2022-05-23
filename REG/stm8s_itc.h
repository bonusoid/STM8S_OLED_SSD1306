//writer : bonus adityas (bonusonic@gmail.com)
//07 september 2016 (revised in 17 may 2020)


#ifndef __STM8S_ITC_H
#define __STM8S_ITC_H


#define enable_interrupts()	__asm rim __endasm

// ITC REGISTERS

#define EXTI_BASE_ADDRESS 0x50A0
#define SPR_BASE_ADDRESS 0x7F70

#define EXTI_CR1 *(volatile unsigned char*)(EXTI_BASE_ADDRESS + 0x00)	// External Interrupt Control Register 1
#define EXTI_CR2 *(volatile unsigned char*)(EXTI_BASE_ADDRESS + 0x01)	// External Interrupt Control Register 2

#define ITC_SPR1 *(volatile unsigned char*)(SPR_BASE_ADDRESS + 0x00)	// Software Priority Register 1
#define ITC_SPR2 *(volatile unsigned char*)(SPR_BASE_ADDRESS + 0x01)	// Software Priority Register 2
#define ITC_SPR3 *(volatile unsigned char*)(SPR_BASE_ADDRESS + 0x02)	// Software Priority Register 3	
#define ITC_SPR4 *(volatile unsigned char*)(SPR_BASE_ADDRESS + 0x03)	// Software Priority Register 4
#define ITC_SPR5 *(volatile unsigned char*)(SPR_BASE_ADDRESS + 0x04)	// Software Priority Register 5
#define ITC_SPR6 *(volatile unsigned char*)(SPR_BASE_ADDRESS + 0x05)	// Software Priority Register 6	
#define ITC_SPR7 *(volatile unsigned char*)(SPR_BASE_ADDRESS + 0x06)	// Software Priority Register 7
#define ITC_SPR8 *(volatile unsigned char*)(SPR_BASE_ADDRESS + 0x07)	// Software Priority Register 8

//EXTI_CR1 bits
#define EXTI_CR1_PAIS 	0	// Port A External Interrupt Sensitivity bits
#define EXTI_CR1_PBIS 	2	// Port B External Interrupt Sensitivity bits
#define EXTI_CR1_PCIS 	4	// Port C External Interrupt Sensitivity bits
#define EXTI_CR1_PDIS 	6	// Port D External Interrupt Sensitivity bits

//EXTI_CR2 bits
#define EXTI_CR2_PEIS 	0	// Port E External Interrupt Sensitivity bits
#define EXTI_CR2_TLIS	2	// Top Level Interrupt Sensitivity

//EXTI Sensitivity -> for PxIS
#define low_level	0
#define rising_edge	1
#define falling_edge	2
#define both_edge	3

//EXTI TL Sensitivity -> for TLIS
#define TL_falling_edge	0
#define TL_rising_edge	1

//ITC_SPR1 bits
#define ITC_SPR1_VECT0 	0 	// Vector 0 Software Priority bits	
#define ITC_SPR1_VECT1	2 	// Vector 1 Software Priority bits
#define ITC_SPR1_VECT2	4 	// Vector 2 Software Priority bits
#define ITC_SPR1_VECT3	6 	// Vector 3 Software Priority bits	

//ITC_SPR2 bits
#define ITC_SPR2_VECT4	0 	// Vector 4 Software Priority bits
#define ITC_SPR2_VECT5	2 	// Vector 5 Software Priority bits
#define ITC_SPR2_VECT6	4 	// Vector 6 Software Priority bits
#define ITC_SPR2_VECT7	6 	// Vector 7 Software Priority bits

//ITC_SPR3 bits
#define ITC_SPR3_VECT8	0 	// Vector 8 Software Priority bits
#define ITC_SPR3_VECT9	2 	// Vector 9 Software Priority bits
#define ITC_SPR3_VECT10	4 	// Vector 10 Software Priority bits
#define ITC_SPR3_VECT11	6 	// Vector 11 Software Priority bits

//ITC_SPR4 bits
#define ITC_SPR4_VECT12	0 	// Vector 12 Software Priority bits
#define ITC_SPR4_VECT13	2 	// Vector 13 Software Priority bits
#define ITC_SPR4_VECT14	4 	// Vector 14 Software Priority bits
#define ITC_SPR4_VECT15	6 	// Vector 15 Software Priority bits

//ITC_SPR5 bits
#define ITC_SPR5_VECT16	0 	// Vector 16 Software Priority bits
#define ITC_SPR5_VECT17	2 	// Vector 17 Software Priority bits
#define ITC_SPR5_VECT18	4 	// Vector 18 Software Priority bits
#define ITC_SPR5_VECT19	6 	// Vector 19 Software Priority bits

//ITC_SPR6 bits
#define ITC_SPR6_VECT20	0 	// Vector 20 Software Priority bits
#define ITC_SPR6_VECT21	2 	// Vector 21 Software Priority bits
#define ITC_SPR6_VECT22	4 	// Vector 22 Software Priority bits
#define ITC_SPR6_VECT23	6 	// Vector 23 Software Priority bits

//ITC_SPR7 bits
#define ITC_SPR7_VECT24	0 	// Vector 24 Software Priority bits
#define ITC_SPR7_VECT25	2 	// Vector 25 Software Priority bits
#define ITC_SPR7_VECT26	4 	// Vector 26 Software Priority bits
#define ITC_SPR7_VECT27	6 	// Vector 27 Software Priority bits

//ITC_SPR8 bits
#define ITC_SPR8_VECT28	0 	// Vector 28 Software Priority bits

//ITC Level -> for VECTx
#define level_1		1
#define level_2		0
#define level_3		3


#endif
