//writer : bonus adityas (bonusonic@gmail.com)
//21 august 2016 (revised in 21 may 2020)


#ifndef __STM8S_TIM6_H
#define __STM8S_TIM6_H


// TIMER 6 REGISTERS
//	8-bit Basic Timer with Trigger/Clock Controller

#define TIM6_BASE_ADDRESS 0x5340

#define TIM6_CR1	*(volatile unsigned char*)(TIM6_BASE_ADDRESS + 0x00)	// TIM6 Control Register 1
#define TIM6_CR2	*(volatile unsigned char*)(TIM6_BASE_ADDRESS + 0x01)	// TIM6 Control Register 2
#define TIM6_SMCR	*(volatile unsigned char*)(TIM6_BASE_ADDRESS + 0x02)	// TIM6 Slave Mode Control Register
#define TIM6_IER	*(volatile unsigned char*)(TIM6_BASE_ADDRESS + 0x03)	// TIM6 Interrupt Enable Register
#define TIM6_SR1	*(volatile unsigned char*)(TIM6_BASE_ADDRESS + 0x04)	// TIM6 Status Register 1
#define TIM6_EGR	*(volatile unsigned char*)(TIM6_BASE_ADDRESS + 0x05)	// TIM6 Event Generation Register
#define TIM6_CNTR	*(volatile unsigned char*)(TIM6_BASE_ADDRESS + 0x06)	// TIM6 Counter Register
#define TIM6_PSCR	*(volatile unsigned char*)(TIM6_BASE_ADDRESS + 0x07)	// TIM6 Prescaler Register
#define TIM6_ARR	*(volatile unsigned char*)(TIM6_BASE_ADDRESS + 0x08)	// TIM6 Auto-Reload Register

//TIM6_CR1 bits
#define TIM6_CR1_CEN	0	// Counter Enable
#define TIM6_CR1_UDIS	1	// Update Disable
#define TIM6_CR1_URS	2	// Update Request Source
#define TIM6_CR1_OPM	3	// One-Pulse Mode
#define TIM6_CR1_ARPE	7	// Auto-Reload Preload Enable

//TIM6_CR2 bits
#define TIM6_CR2_MMS	4	// Master Mode Selection

//Master Mode Selection -> for MMS
#define TIM6_MMS_reset	0	// TIM6_EGR_UG as TRGO
#define TIM6_MMS_enable	1	// Counter Enable Signal as TRGO
#define TIM6_MMS_update	2	// Update Event as TRGO

//TIM6_SMCR bits
#define TIM6_SMCR_SMS	0	// Clock/Trigger/Slave Mode Selection
#define TIM6_SMCR_TS	4	// Trigger Selection
#define TIM6_SMCR_MSM	7	// Master/Slave Mode

//Clock/Trigger/Slave Mode Selection -> for SMS
#define TIM6_SMS_disabled		0	// Prescaler is clocked directly by internal clock
#define TIM6_SMS_trigger_reset_mode	4	// Rising edge of TRGI reinitializes the counter
#define TIM6_SMS_trigger_gated_mode	5	// Counter clock is enabled when TRGI is high
#define TIM6_SMS_trigger_standard_mode	6	// Counter starts at a rising edge of the TRGI
#define TIM6_SMS_ext_clk_mode1		7	// Rising edges of TRGI clock the signal

//Trigger Selection -> for TS
#define TIM6_TS_ITR1_TIM1_TRGO	1	// ITR1 connected to TIM1 TRGO
#define TIM6_TS_ITR3_TIM5_TRGO	3	// ITR3 connected to TIM5 TRGO

//TIM6_IER bits
#define TIM6_IER_UIE	0	// Update Interrupt Enable
#define TIM6_IER_TIE	6	// Trigger Interrupt Enable

//TIM6_SR1 bits
#define TIM6_SR1_UIF	0	// Update Interrupt Flag
#define TIM6_SR1_TIF	6	// Trigger Interrupt Flag

//TIM6_EGR bits
#define TIM6_EGR_UG	0	// Update Generation
#define TIM6_EGR_TG	6	// Trigger Generation

//TIM6_PSCR bits
#define TIM6_PSCR_PSC	0	// Prescaler Value -> PSC[2:0]

//Prescaler Value -> for PSC
//fCK_CNT = fCK_PSC/(2^PSC)


#endif
