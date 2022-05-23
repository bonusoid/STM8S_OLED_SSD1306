//writer : bonus adityas (bonusonic@gmail.com)
//21 august 2016 (revised in 21 may 2020)


#ifndef __STM8S_TIM4_H
#define __STM8S_TIM4_H


// TIMER 4 REGISTERS
//	8-bit Basic Timer

#define TIM4_BASE_ADDRESS 0x5340
#define TIM4_PRODUCT_OFFSET 0x02	// Product Dependant

#define TIM4_CR1	*(volatile unsigned char*)(TIM4_BASE_ADDRESS + 0x00)				// TIM4 Control Register 1
#define TIM4_IER	*(volatile unsigned char*)(TIM4_BASE_ADDRESS + 0x01 + TIM4_PRODUCT_OFFSET)	// TIM4 Interrupt Enable Register
#define TIM4_SR1	*(volatile unsigned char*)(TIM4_BASE_ADDRESS + 0x02 + TIM4_PRODUCT_OFFSET)	// TIM4 Status Register 1
#define TIM4_EGR	*(volatile unsigned char*)(TIM4_BASE_ADDRESS + 0x03 + TIM4_PRODUCT_OFFSET)	// TIM4 Event Generation Register
#define TIM4_CNTR	*(volatile unsigned char*)(TIM4_BASE_ADDRESS + 0x04 + TIM4_PRODUCT_OFFSET)	// TIM4 Counter Register
#define TIM4_PSCR	*(volatile unsigned char*)(TIM4_BASE_ADDRESS + 0x05 + TIM4_PRODUCT_OFFSET)	// TIM4 Prescaler Register
#define TIM4_ARR	*(volatile unsigned char*)(TIM4_BASE_ADDRESS + 0x06 + TIM4_PRODUCT_OFFSET)	// TIM4 Auto-Reload Register

//TIM4_CR1 bits
#define TIM4_CR1_CEN	0	// Counter Enable
#define TIM4_CR1_UDIS	1	// Update Disable
#define TIM4_CR1_URS	2	// Update Request Source
#define TIM4_CR1_OPM	3	// One-Pulse Mode
#define TIM4_CR1_ARPE	7	// Auto-Reload Preload Enable

//TIM4_IER bits
#define TIM4_IER_UIE	0	// Update Interrupt Enable

//TIM4_SR1 bits
#define TIM4_SR1_UIF	0	// Update Interrupt Flag

//TIM4_EGR bits
#define TIM4_EGR_UG	0	// Update Generation

//TIM4_PSCR bits
#define TIM4_PSCR_PSC	0	// Prescaler Value -> PSC[2:0]

//Prescaler Value -> for PSC
//fCK_CNT = fCK_PSC/(2^PSC)


#endif
