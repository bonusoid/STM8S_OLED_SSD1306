//writer : bonus adityas (bonusonic@gmail.com)
//21 august 2016 (revised in 25 may 2020)


#ifndef __STM8S_TIM2_H
#define __STM8S_TIM2_H


// TIMER 2 REGISTERS
//	16-bit General Purpose Timer

#define TIM2_BASE_ADDRESS 0x5300
#define TIM2_PRODUCT_OFFSET 0x02

#define TIM2_CR1	*(volatile unsigned char*)(TIM2_BASE_ADDRESS + 0x00)				// TIM2 Control Register 1
#define TIM2_IER	*(volatile unsigned char*)(TIM2_BASE_ADDRESS + 0x01 + TIM2_PRODUCT_OFFSET)	// TIM2 Interrupt Enable Register
#define TIM2_SR1	*(volatile unsigned char*)(TIM2_BASE_ADDRESS + 0x02 + TIM2_PRODUCT_OFFSET)	// TIM2 Status Register 1
#define TIM2_SR2	*(volatile unsigned char*)(TIM2_BASE_ADDRESS + 0x03 + TIM2_PRODUCT_OFFSET)	// TIM2 Status Register 2
#define TIM2_EGR	*(volatile unsigned char*)(TIM2_BASE_ADDRESS + 0x04 + TIM2_PRODUCT_OFFSET)	// TIM2 Event Generation Register
#define TIM2_CCMR1	*(volatile unsigned char*)(TIM2_BASE_ADDRESS + 0x05 + TIM2_PRODUCT_OFFSET)	// TIM2 Capture/Compare Mode Register 1
#define TIM2_CCMR2	*(volatile unsigned char*)(TIM2_BASE_ADDRESS + 0x06 + TIM2_PRODUCT_OFFSET)	// TIM2 Capture/Compare Mode Register 2
#define TIM2_CCMR3	*(volatile unsigned char*)(TIM2_BASE_ADDRESS + 0x07 + TIM2_PRODUCT_OFFSET)	// TIM2 Capture/Compare Mode Register 3
#define TIM2_CCER1	*(volatile unsigned char*)(TIM2_BASE_ADDRESS + 0x08 + TIM2_PRODUCT_OFFSET)	// TIM2 Capture/Compare Enable Register 1
#define TIM2_CCER2	*(volatile unsigned char*)(TIM2_BASE_ADDRESS + 0x09 + TIM2_PRODUCT_OFFSET)	// TIM2 Capture/Compare Enable Register 2
#define TIM2_CNTRH	*(volatile unsigned char*)(TIM2_BASE_ADDRESS + 0x0A + TIM2_PRODUCT_OFFSET)	// TIM2 Counter Register High
#define TIM2_CNTRL	*(volatile unsigned char*)(TIM2_BASE_ADDRESS + 0x0B + TIM2_PRODUCT_OFFSET)	// TIM2 Counter Register Low
#define TIM2_PSCR	*(volatile unsigned char*)(TIM2_BASE_ADDRESS + 0x0C + TIM2_PRODUCT_OFFSET)	// TIM2 Prescaler Register
#define TIM2_ARRH	*(volatile unsigned char*)(TIM2_BASE_ADDRESS + 0x0D + TIM2_PRODUCT_OFFSET)	// TIM2 Auto-Reload Register High
#define TIM2_ARRL	*(volatile unsigned char*)(TIM2_BASE_ADDRESS + 0x0E + TIM2_PRODUCT_OFFSET)	// TIM2 Auto-Reload Register Low
#define TIM2_CCR1H	*(volatile unsigned char*)(TIM2_BASE_ADDRESS + 0x0F + TIM2_PRODUCT_OFFSET)	// TIM2 Capture/Compare Register 1 High
#define TIM2_CCR1L	*(volatile unsigned char*)(TIM2_BASE_ADDRESS + 0x10 + TIM2_PRODUCT_OFFSET)	// TIM2 Capture/Compare Register 1 Low
#define TIM2_CCR2H	*(volatile unsigned char*)(TIM2_BASE_ADDRESS + 0x11 + TIM2_PRODUCT_OFFSET)	// TIM2 Capture/Compare Register 2 High
#define TIM2_CCR2L	*(volatile unsigned char*)(TIM2_BASE_ADDRESS + 0x12 + TIM2_PRODUCT_OFFSET)	// TIM2 Capture/Compare Register 2 Low
#define TIM2_CCR3H	*(volatile unsigned char*)(TIM2_BASE_ADDRESS + 0x13 + TIM2_PRODUCT_OFFSET)	// TIM2 Capture/Compare Register 3 High
#define TIM2_CCR3L	*(volatile unsigned char*)(TIM2_BASE_ADDRESS + 0x14 + TIM2_PRODUCT_OFFSET)	// TIM2 Capture/Compare Register 3 Low

//TIM2_CR1 bits
#define TIM2_CR1_CEN	0	// Counter Enable
#define TIM2_CR1_UDIS	1	// Update Disable
#define TIM2_CR1_URS	2	// Update Request Source
#define TIM2_CR1_OPM	3	// One-Pulse Mode
#define TIM2_CR1_ARPE	7	// Auto-Reload Preload Register

//TIM2_IER bits
#define TIM2_IER_UIE	0	// Update Interrupt Enable
#define TIM2_IER_CC1IE	1	// Capture/Compare 1 Interrupt Enable
#define TIM2_IER_CC2IE	2	// Capture/Compare 2 Interrupt Enable
#define TIM2_IER_CC3IE	3	// Capture/Compare 3 Interrupt Enable

//TIM2_SR1 bits
#define TIM2_SR1_UIF	0	// Update Interrupt Flag
#define TIM2_SR1_CC1IF	1	// Capture/Compare 1 Interrupt Flag
#define TIM2_SR1_CC2IF	2	// Capture/Compare 2 Interrupt Flag
#define TIM2_SR1_CC3IF	3	// Capture/Compare 3 Interrupt Flag

//TIM2_SR2 bits
#define TIM2_SR2_CC1OF	1	// Capture/Compare 1 Overcapture Flag 
#define TIM2_SR2_CC2OF	2	// Capture/Compare 2 Overcapture Flag
#define TIM2_SR2_CC3OF	3	// Capture/Compare 3 Overcapture Flag

//TIM2_EGR bits
#define TIM2_EGR_UG	0	// Update Generation
#define TIM2_EGR_CC1G	1	// Capture/Compare 1 Generation
#define TIM2_EGR_CC2G	2	// Capture/Compare 2 Generation
#define TIM2_EGR_CC3G	3	// Capture/Compare 3 Generation

//TIM2_CCMR1 bits
#define TIM2_CCMR1_CC1S		0	// Capture/Compare 1 Selection
//Output Mode
#define TIM2_CCMR1_OC1PE	3	// Output Compare 1 Preload Enable
#define TIM2_CCMR1_OC1M 	4	// Output Compare 1 Mode
//Input Mode
#define TIM2_CCMR1_IC1PSC 	2	// Input Capture 1 Prescaler
#define TIM2_CCMR1_IC1F 	4	// Input Capture 1 Filter

//TIM2_CCMR2 bits
#define TIM2_CCMR2_CC2S		0	// Capture/Compare 2 Selection
//Output Mode
#define TIM2_CCMR2_OC2PE	3	// Output Compare 2 Preload Enable
#define TIM2_CCMR2_OC2M 	4	// Output Compare 2 Mode
//Input Mode
#define TIM2_CCMR2_IC2PSC 	2	// Input Capture 2 Prescaler
#define TIM2_CCMR2_IC2F 	4	// Input Capture 2 Filter

//TIM2_CCMR3 bits
#define TIM2_CCMR3_CC3S		0	// Capture/Compare 3 Selection
//Output Mode
#define TIM2_CCMR3_OC3PE	3	// Output Compare 3 Preload Enable
#define TIM2_CCMR3_OC3M 	4	// Output Compare 3 Mode
//Input Mode
#define TIM2_CCMR3_IC3PSC 	2	// Input Capture 3 Prescaler
#define TIM2_CCMR3_IC3F 	4	// Input Capture 3 Filter

//Capture/Compare Selection -> for CC1S
#define TIM2_CC1_output		0	// CC1 Channel Configured as Output
#define TIM2_CC1_input_TI1FP1	1	// CC1 Channel Configured as Input, IC1 is mapped on TI1FP1
#define TIM2_CC1_input_TI2FP1	2	// CC1 Channel Configured as Input, IC1 is mapped on TI2FP1
#define TIM2_CC1_input_TRC	3	// CC1 Channel Configured as Input, IC1 is mapped on TRC
//Capture/Compare Selection -> for CC2S
#define TIM2_CC2_output		0	// CC2 Channel Configured as Output
#define TIM2_CC2_input_TI2FP2	1	// CC2 Channel Configured as Input, IC2 is mapped on TI2FP2
#define TIM2_CC2_input_TI1FP2	2	// CC2 Channel Configured as Input, IC2 is mapped on TI1FP2
#define TIM2_CC2_input_TRC	3	// CC2 Channel Configured as Input, IC2 is mapped on TRC
//Capture/Compare Selection -> for CC3S
#define TIM2_CC3_output		0	// CC3 Channel Configured as Output
#define TIM2_CC3_input_TI3FP3	1	// CC3 Channel Configured as Input, IC3 is mapped on TI3FP3

//Output Compare Mode -> for OCxM
#define TIM2_OCxREF_frozen		0	// The comparison has no effect on the outputs
#define TIM2_OCxREF_active		1	// OCxREF is forced high when TIM1_CNT matches TIM1_CCRx
#define TIM2_OCxREF_inactive		2	// OCxREF is forced low when TIM1_CNT matches TIM1_CCRx
#define TIM2_OCxREF_toggle		3	// OCxREF toggles when TIM1_CNT matches TIM1_CCRx
#define TIM2_OCxREF_force_inactive	4	// OCxREF is forced low
#define TIM2_OCxREF_force_active	5	// OCxREF is forced high
#define TIM2_OCxREF_PWM_mode1		6	// PWM Mode 1
#define TIM2_OCxREF_PWM_mode2		7	// PWM Mode 2

//Input Capture Filter -> for ICxPSC
#define TIM2_ICxPSC_noprescaler	0	// Capture is made each time an edge is detected
#define TIM2_ICxPSC_2events	1	// Capture is made once every 2 events
#define TIM2_ICxPSC_4events	2	// Capture is made once every 4 events
#define TIM2_ICxPSC_8events	3	// Capture is made once every 8 events

//Input Capture Filter -> for ICxF
#define TIM2_fSAMPLING_nofilter		0	// No Filter
#define TIM2_fSAMPLING_fMASTER_N2 	1	// fSAMPLING = fMASTER, N=2
#define TIM2_fSAMPLING_fMASTER_N4 	2	// fSAMPLING = fMASTER, N=4
#define TIM2_fSAMPLING_fMASTER_N8 	3	// fSAMPLING = fMASTER, N=8
#define TIM2_fSAMPLING_fMASTER_2_N6 	4	// fSAMPLING = fMASTER/2, N=6
#define TIM2_fSAMPLING_fMASTER_2_N8 	5	// fSAMPLING = fMASTER/2, N=8
#define TIM2_fSAMPLING_fMASTER_4_N6 	6	// fSAMPLING = fMASTER/4, N=6
#define TIM2_fSAMPLING_fMASTER_4_N8 	7	// fSAMPLING = fMASTER/4, N=8
#define TIM2_fSAMPLING_fMASTER_8_N6 	8	// fSAMPLING = fMASTER/8, N=6
#define TIM2_fSAMPLING_fMASTER_8_N8 	9	// fSAMPLING = fMASTER/8, N=8
#define TIM2_fSAMPLING_fMASTER_16_N5 	10	// fSAMPLING = fMASTER/16, N=5
#define TIM2_fSAMPLING_fMASTER_16_N6 	11	// fSAMPLING = fMASTER/16, N=6
#define TIM2_fSAMPLING_fMASTER_16_N8 	12	// fSAMPLING = fMASTER/16, N=8
#define TIM2_fSAMPLING_fMASTER_32_N5 	13	// fSAMPLING = fMASTER/16, N=5
#define TIM2_fSAMPLING_fMASTER_32_N6 	14	// fSAMPLING = fMASTER/16, N=6
#define TIM2_fSAMPLING_fMASTER_32_N8 	15	// fSAMPLING = fMASTER/16, N=8

//TIM2_CCER1 bits
#define TIM2_CCER1_CC1E	0	// Capture/Compare 1 Output Enable 
#define TIM2_CCER1_CC1P	1	// Capture/Compare 1 Output Polarity
#define TIM2_CCER1_CC2E	4	// Capture/Compare 2 Output Enable
#define TIM2_CCER1_CC2P	5	// Capture/Compare 2 Output Polarity

//TIM2_CCER2 bits
#define TIM2_CCER2_CC3E	0	// Capture/Compare 3 Output Enable
#define TIM2_CCER2_CC3P	1	// Capture/Compare 3 Output Polarity

//TIM2_PSCR bits
#define TIM2_PSCR_PSC 	0	// Prescaler Value -> PSC[3:0]

//Prescaler Value -> for PSC
//fCK_CNT = fCK_PSC/(2^PSC)


#endif
