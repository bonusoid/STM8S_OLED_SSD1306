//writer : bonus adityas (bonusonic@gmail.com)
//22 august 2016 (revised in 25 may 2020)


#ifndef __STM8S_TIM5_H
#define __STM8S_TIM5_H


// TIMER 5 REGISTERS
//	16-bit General Purpose Timer

#define TIM5_BASE_ADDRESS 0x5300

#define TIM5_CR1	*(volatile unsigned char*)(TIM5_BASE_ADDRESS + 0x00)	// TIM5 Control Register 1
#define TIM5_CR2	*(volatile unsigned char*)(TIM5_BASE_ADDRESS + 0x01)	// TIM5 Control Register 2
#define TIM5_SMCR	*(volatile unsigned char*)(TIM5_BASE_ADDRESS + 0x02)	// TIM5 Slave Mode Control Register
#define TIM5_IER	*(volatile unsigned char*)(TIM5_BASE_ADDRESS + 0x03)	// TIM5 Interrupt Enable Register
#define TIM5_SR1	*(volatile unsigned char*)(TIM5_BASE_ADDRESS + 0x04)	// TIM5 Status Register 1
#define TIM5_SR2	*(volatile unsigned char*)(TIM5_BASE_ADDRESS + 0x05)	// TIM5 Status Register 2
#define TIM5_EGR	*(volatile unsigned char*)(TIM5_BASE_ADDRESS + 0x06)	// TIM5 Event Generation Register
#define TIM5_CCMR1	*(volatile unsigned char*)(TIM5_BASE_ADDRESS + 0x07)	// TIM5 Capture/Compare Mode Register 1
#define TIM5_CCMR2	*(volatile unsigned char*)(TIM5_BASE_ADDRESS + 0x08)	// TIM5 Capture/Compare Mode Register 2
#define TIM5_CCMR3	*(volatile unsigned char*)(TIM5_BASE_ADDRESS + 0x09)	// TIM5 Capture/Compare Mode Register 3
#define TIM5_CCER1	*(volatile unsigned char*)(TIM5_BASE_ADDRESS + 0x0A)	// TIM5 Capture/Compare Enable Register 1
#define TIM5_CCER2	*(volatile unsigned char*)(TIM5_BASE_ADDRESS + 0x0B)	// TIM5 Capture/Compare Enable Register 2
#define TIM5_CNTRH	*(volatile unsigned char*)(TIM5_BASE_ADDRESS + 0x0C)	// TIM5 Counter Register High
#define TIM5_CNTRL	*(volatile unsigned char*)(TIM5_BASE_ADDRESS + 0x0D)	// TIM5 Counter Register Low
#define TIM5_PSCR	*(volatile unsigned char*)(TIM5_BASE_ADDRESS + 0x0E)	// TIM5 Prescaler Register
#define TIM5_ARRH	*(volatile unsigned char*)(TIM5_BASE_ADDRESS + 0x0F)	// TIM5 Auto-Reload Register High
#define TIM5_ARRL	*(volatile unsigned char*)(TIM5_BASE_ADDRESS + 0x10)	// TIM5 Auto-Reload Register Low
#define TIM5_CCR1H	*(volatile unsigned char*)(TIM5_BASE_ADDRESS + 0x11)	// TIM5 Capture/Compare Register 1 High
#define TIM5_CCR1L	*(volatile unsigned char*)(TIM5_BASE_ADDRESS + 0x12)	// TIM5 Capture/Compare Register 1 Low
#define TIM5_CCR2H	*(volatile unsigned char*)(TIM5_BASE_ADDRESS + 0x13)	// TIM5 Capture/Compare Register 2 High
#define TIM5_CCR2L	*(volatile unsigned char*)(TIM5_BASE_ADDRESS + 0x14)	// TIM5 Capture/Compare Register 2 Low
#define TIM5_CCR3H	*(volatile unsigned char*)(TIM5_BASE_ADDRESS + 0x15)	// TIM5 Capture/Compare Register 3 High
#define TIM5_CCR3L	*(volatile unsigned char*)(TIM5_BASE_ADDRESS + 0x16)	// TIM5 Capture/Compare Register 3 Low

//TIM5_CR1 bits
#define TIM5_CR1_CEN	0	// Counter Enable
#define TIM5_CR1_UDIS	1	// Update Disable
#define TIM5_CR1_URS	2	// Update Request Source
#define TIM5_CR1_OPM	3	// One-Pulse Mode
#define TIM5_CR1_ARPE	7	// Auto-Reload Preload Register

//TIM5_CR2 bits
#define TIM5_CR2_CCPC	0	// Capture/Compare Preloaded Control
#define TIM5_CR2_COMS	2	// Capture/Compare Control Update Selection
#define TIM5_CR2_MMS 	4	// Master Mode Selection

//Master Mode Selection -> for MMS
#define TIM5_MMS_reset	0	// TIM5_EGR_UG as TRGO
#define TIM5_MMS_enable	1	// Counter Enable Signal as TRGO
#define TIM5_MMS_update	2	// Update Event as TRGO

//TIM5_SMCR bits
#define TIM5_SMCR_SMS 	0	// Clock/Trigger/Slave Mode Selection
#define TIM5_SMCR_TS 	4	// Trigger Selection
#define TIM5_SMCR_MSM	7	// Master/Slave Mode

//Clock/Trigger/Slave Mode Selection -> for SMS
#define TIM5_SMS_disabled		0	// Prescaler is clocked directly by internal clock
#define TIM5_SMS_trigger_reset_mode	4	// Rising edge of TRGI reinitializes the counter
#define TIM5_SMS_trigger_gated_mode	5	// Counter clock is enabled when TRGI is high
#define TIM5_SMS_trigger_standard_mode	6	// Counter starts at a rising edge of the TRGI
#define TIM5_SMS_ext_clk_mode1		7	// Rising edges of TRGI clock the signal

//Trigger Selection -> for TS
#define TIM5_TS_ITR0_TIM6_TRGO	1	// ITR0 connected to TIM6 TRGO
#define TIM5_TS_ITR1_TIM6_TRGO	3	// ITR1 connected to TIM6 TRGO

//TIM5_IER bits
#define TIM5_IER_UIE	0	// Update Interrupt Enable
#define TIM5_IER_CC1IE	1	// Capture/Compare 1 Interrupt Enable
#define TIM5_IER_CC2IE	2	// Capture/Compare 2 Interrupt Enable
#define TIM5_IER_CC3IE	3	// Capture/Compare 3 Interrupt Enable
#define TIM5_IER_TIE	6	// Trigger Interrupt Enable

//TIM5_SR1 bits
#define TIM5_SR1_UIF	0	// Update Interrupt Flag
#define TIM5_SR1_CC1IF	1	// Capture/Compare 1 Interrupt Flag
#define TIM5_SR1_CC2IF	2	// Capture/Compare 2 Interrupt Flag
#define TIM5_SR1_CC3IF	3	// Capture/Compare 3 Interrupt Flag
#define TIM5_SR1_TIF	6	// Trigger Interrupt Flag

//TIM5_SR2 bits
#define TIM5_SR2_CC1OF	1	// Capture/Compare 1 Overcapture Flag 
#define TIM5_SR2_CC2OF	2	// Capture/Compare 2 Overcapture Flag
#define TIM5_SR2_CC3OF	3	// Capture/Compare 3 Overcapture Flag

//TIM5_EGR bits
#define TIM5_EGR_UG	0	// Update Generation
#define TIM5_EGR_CC1G	1	// Capture/Compare 1 Generation
#define TIM5_EGR_CC2G	2	// Capture/Compare 2 Generation
#define TIM5_EGR_CC3G	3	// Capture/Compare 3 Generation
#define TIM5_EGR_TG	6	// Trigger Generation

//TIM5_CCMR1 bits
#define TIM5_CCMR1_CC1S		0	// Capture/Compare 1 Selection
//Output Mode
#define TIM5_CCMR1_OC1PE	3	// Output Compare 1 Preload Enable
#define TIM5_CCMR1_OC1M 	4	// Output Compare 1 Mode
//Input Mode
#define TIM5_CCMR1_IC1PSC 	2	// Input Capture 1 Prescaler
#define TIM5_CCMR1_IC1F 	4	// Input Capture 1 Filter

//TIM5_CCMR2 bits
#define TIM5_CCMR2_CC1S		0	// Capture/Compare 1 Selection
//Output Mode
#define TIM5_CCMR2_OC1PE	3	// Output Compare 1 Preload Enable
#define TIM5_CCMR2_OC1M 	4	// Output Compare 1 Mode
//Input Mode
#define TIM5_CCMR2_IC1PSC 	2	// Input Capture 1 Prescaler
#define TIM5_CCMR2_IC1F 	4	// Input Capture 1 Filter

//TIM5_CCMR3 bits
#define TIM5_CCMR3_CC1S		0	// Capture/Compare 1 Selection
//Output Mode
#define TIM5_CCMR3_OC1PE	3	// Output Compare 1 Preload Enable
#define TIM5_CCMR3_OC1M 	4	// Output Compare 1 Mode
//Input Mode
#define TIM5_CCMR3_IC1PSC 	2	// Input Capture 1 Prescaler
#define TIM5_CCMR3_IC1F 	4	// Input Capture 1 Filter

//Capture/Compare Selection -> for CC1S
#define TIM5_CC1_output		0	// CC1 Channel Configured as Output
#define TIM5_CC1_input_TI1FP1	1	// CC1 Channel Configured as Input, IC1 is mapped on TI1FP1
#define TIM5_CC1_input_TI2FP1	2	// CC1 Channel Configured as Input, IC1 is mapped on TI2FP1
#define TIM5_CC1_input_TRC	3	// CC1 Channel Configured as Input, IC1 is mapped on TRC
//Capture/Compare Selection -> for CC2S
#define TIM5_CC2_output		0	// CC2 Channel Configured as Output
#define TIM5_CC2_input_TI2FP2	1	// CC2 Channel Configured as Input, IC2 is mapped on TI2FP2
#define TIM5_CC2_input_TI1FP2	2	// CC2 Channel Configured as Input, IC2 is mapped on TI1FP2
#define TIM5_CC2_input_TRC	3	// CC2 Channel Configured as Input, IC2 is mapped on TRC
//Capture/Compare Selection -> for CC3S
#define TIM5_CC3_output		0	// CC3 Channel Configured as Output
#define TIM5_CC3_input_TI3FP3	1	// CC3 Channel Configured as Input, IC3 is mapped on TI3FP3

//Output Compare Mode -> for OCxM
#define TIM5_OCxREF_frozen		0	// The comparison has no effect on the outputs
#define TIM5_OCxREF_active		1	// OCxREF is forced high when TIM1_CNT matches TIM1_CCRx
#define TIM5_OCxREF_inactive		2	// OCxREF is forced low when TIM1_CNT matches TIM1_CCRx
#define TIM5_OCxREF_toggle		3	// OCxREF toggles when TIM1_CNT matches TIM1_CCRx
#define TIM5_OCxREF_force_inactive	4	// OCxREF is forced low
#define TIM5_OCxREF_force_active	5	// OCxREF is forced high
#define TIM5_OCxREF_PWM_mode1		6	// PWM Mode 1
#define TIM5_OCxREF_PWM_mode2		7	// PWM Mode 2

//Input Capture Filter -> for ICxPSC
#define TIM5_ICxPSC_noprescaler	0	// Capture is made each time an edge is detected
#define TIM5_ICxPSC_2events	1	// Capture is made once every 2 events
#define TIM5_ICxPSC_4events	2	// Capture is made once every 4 events
#define TIM5_ICxPSC_8events	3	// Capture is made once every 8 events

//Input Capture Filter -> for ICxF
#define TIM5_fSAMPLING_nofilter		0	// No Filter
#define TIM5_fSAMPLING_fMASTER_N2 	1	// fSAMPLING = fMASTER, N=2
#define TIM5_fSAMPLING_fMASTER_N4 	2	// fSAMPLING = fMASTER, N=4
#define TIM5_fSAMPLING_fMASTER_N8 	3	// fSAMPLING = fMASTER, N=8
#define TIM5_fSAMPLING_fMASTER_2_N6 	4	// fSAMPLING = fMASTER/2, N=6
#define TIM5_fSAMPLING_fMASTER_2_N8 	5	// fSAMPLING = fMASTER/2, N=8
#define TIM5_fSAMPLING_fMASTER_4_N6 	6	// fSAMPLING = fMASTER/4, N=6
#define TIM5_fSAMPLING_fMASTER_4_N8 	7	// fSAMPLING = fMASTER/4, N=8
#define TIM5_fSAMPLING_fMASTER_8_N6 	8	// fSAMPLING = fMASTER/8, N=6
#define TIM5_fSAMPLING_fMASTER_8_N8 	9	// fSAMPLING = fMASTER/8, N=8
#define TIM5_fSAMPLING_fMASTER_16_N5 	10	// fSAMPLING = fMASTER/16, N=5
#define TIM5_fSAMPLING_fMASTER_16_N6 	11	// fSAMPLING = fMASTER/16, N=6
#define TIM5_fSAMPLING_fMASTER_16_N8 	12	// fSAMPLING = fMASTER/16, N=8
#define TIM5_fSAMPLING_fMASTER_32_N5 	13	// fSAMPLING = fMASTER/16, N=5
#define TIM5_fSAMPLING_fMASTER_32_N6 	14	// fSAMPLING = fMASTER/16, N=6
#define TIM5_fSAMPLING_fMASTER_32_N8 	15	// fSAMPLING = fMASTER/16, N=8

//TIM5_CCER1 bits
#define TIM5_CCER1_CC1E	0	// Capture/Compare 1 Output Enable 
#define TIM5_CCER1_CC1P	1	// Capture/Compare 1 Output Polarity
#define TIM5_CCER1_CC2E	4	// Capture/Compare 2 Output Enable
#define TIM5_CCER1_CC2P	5	// Capture/Compare 2 Output Polarity

//TIM5_CCER2 bits
#define TIM5_CCER2_CC3E	0	// Capture/Compare 3 Output Enable
#define TIM5_CCER2_CC3P	1	// Capture/Compare 3 Output Polarity

//TIM5_PSCR bits
#define TIM5_PSCR_PSC 	0	// Prescaler Value -> PSC[3:0]

//Prescaler Value -> for PSC
//fCK_CNT = fCK_PSC/(2^PSC)


#endif
