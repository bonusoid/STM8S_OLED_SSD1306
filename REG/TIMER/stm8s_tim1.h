//writer : bonus adityas (bonusonic@gmail.com)
//20 august 2016 (revised in 24 may 2020)


#ifndef __STM8S_TIM1_H
#define __STM8S_TIM1_H


// TIMER 1 REGISTERS
//	16-bit Advanced Control Timer

#define TIM1_BASE_ADDRESS 0x5250

#define TIM1_CR1  	*(volatile unsigned char*)(TIM1_BASE_ADDRESS + 0x00)	// TIM1 Control Register 1
#define TIM1_CR2  	*(volatile unsigned char*)(TIM1_BASE_ADDRESS + 0x01)	// TIM1 Control Register 2
#define TIM1_SMCR 	*(volatile unsigned char*)(TIM1_BASE_ADDRESS + 0x02)	// TIM1 Slave Mode Control Register
#define TIM1_ETR  	*(volatile unsigned char*)(TIM1_BASE_ADDRESS + 0x03)	// TIM1 External Trigger Register
#define TIM1_IER	*(volatile unsigned char*)(TIM1_BASE_ADDRESS + 0x04)	// TIM1 Interrupt Enable Register
#define TIM1_SR1	*(volatile unsigned char*)(TIM1_BASE_ADDRESS + 0x05)	// TIM1 Status Register 1
#define TIM1_SR2	*(volatile unsigned char*)(TIM1_BASE_ADDRESS + 0x06)	// TIM1 Status Register 2
#define TIM1_EGR	*(volatile unsigned char*)(TIM1_BASE_ADDRESS + 0x07)	// TIM1 Event Generation Register
#define TIM1_CCMR1	*(volatile unsigned char*)(TIM1_BASE_ADDRESS + 0x08)	// TIM1 Capture/Compare Mode Register 1
#define TIM1_CCMR2	*(volatile unsigned char*)(TIM1_BASE_ADDRESS + 0x09)	// TIM1 Capture/Compare Mode Register 2
#define TIM1_CCMR3	*(volatile unsigned char*)(TIM1_BASE_ADDRESS + 0x0A)	// TIM1 Capture/Compare Mode Register 3
#define TIM1_CCMR4	*(volatile unsigned char*)(TIM1_BASE_ADDRESS + 0x0B)	// TIM1 Capture/Compare Mode Register 4
#define TIM1_CCER1	*(volatile unsigned char*)(TIM1_BASE_ADDRESS + 0x0C)	// TIM1 Capture/Compare Enable Register 1 
#define TIM1_CCER2	*(volatile unsigned char*)(TIM1_BASE_ADDRESS + 0x0D)	// TIM1 Capture/Compare Enable Register 2
#define TIM1_CNTRH	*(volatile unsigned char*)(TIM1_BASE_ADDRESS + 0x0E)	// TIM1 Counter Register High
#define TIM1_CNTRL	*(volatile unsigned char*)(TIM1_BASE_ADDRESS + 0x0F)	// TIM1 Counter Register Low
#define TIM1_PSCRH	*(volatile unsigned char*)(TIM1_BASE_ADDRESS + 0x10)	// TIM1 Prescaler Register High
#define TIM1_PSCRL	*(volatile unsigned char*)(TIM1_BASE_ADDRESS + 0x11)	// TIM1 Prescaler Register Low
#define TIM1_ARRH	*(volatile unsigned char*)(TIM1_BASE_ADDRESS + 0x12)	// TIM1 Auto-Reload Register High
#define TIM1_ARRL	*(volatile unsigned char*)(TIM1_BASE_ADDRESS + 0x13)	// TIM1 Auto-Reload Register Low
#define TIM1_RCR	*(volatile unsigned char*)(TIM1_BASE_ADDRESS + 0x14)	// TIM1 Repetition Counter Register
#define TIM1_CCR1H	*(volatile unsigned char*)(TIM1_BASE_ADDRESS + 0x15)	// TIM1 Capture/Compare Register 1 High
#define TIM1_CCR1L	*(volatile unsigned char*)(TIM1_BASE_ADDRESS + 0x16)	// TIM1 Capture/Compare Register 1 Low
#define TIM1_CCR2H	*(volatile unsigned char*)(TIM1_BASE_ADDRESS + 0x17)	// TIM1 Capture/Compare Register 2 High
#define TIM1_CCR2L	*(volatile unsigned char*)(TIM1_BASE_ADDRESS + 0x18)	// TIM1 Capture/Compare Register 2 Low
#define TIM1_CCR3H	*(volatile unsigned char*)(TIM1_BASE_ADDRESS + 0x19)	// TIM1 Capture/Compare Register 3 High
#define TIM1_CCR3L	*(volatile unsigned char*)(TIM1_BASE_ADDRESS + 0x1A)	// TIM1 Capture/Compare Register 3 Low
#define TIM1_CCR4H	*(volatile unsigned char*)(TIM1_BASE_ADDRESS + 0x1B)	// TIM1 Capture/Compare Register 4 High
#define TIM1_CCR4L	*(volatile unsigned char*)(TIM1_BASE_ADDRESS + 0x1C)	// TIM1 Capture/Compare Register 4 Low
#define TIM1_BKR	*(volatile unsigned char*)(TIM1_BASE_ADDRESS + 0x1D)	// TIM1 Break Register
#define TIM1_DTR	*(volatile unsigned char*)(TIM1_BASE_ADDRESS + 0x1E)	// TIM1 Deadtime Register
#define TIM1_OISR	*(volatile unsigned char*)(TIM1_BASE_ADDRESS + 0x1F)	// TIM1 Output Idle State Register

//TIM1_CR1 bits
#define TIM1_CR1_CEN	0	// Counter Enable
#define TIM1_CR1_UDIS	1	// Update Disable
#define TIM1_CR1_URS	2	// Update Request Source
#define TIM1_CR1_OPM	3	// One-Pulse Mode
#define TIM1_CR1_DIR	4	// Direction
#define TIM1_CR1_CMS	5	// Center-Aligned Mode Selection
#define TIM1_CR1_ARPE	7	// Auto-Reload Preload Register

//TIM1_CR2 bits
#define TIM1_CR2_CCPC	0	// Capture/Compare Preloaded Control
#define TIM1_CR2_COMS	2	// Capture/Compare Control Update Selection
#define TIM1_CR2_MMS	4	// Master Mode Selection

//Master Mode Selection -> for MMS
#define TIM1_MMS_reset	0	// TIM1_EGR_UG as TRGO
#define TIM1_MMS_enable	1	// Counter Enable Signal as TRGO
#define TIM1_MMS_update	2	// Update Event as TRGO
#define TIM1_MMS_match1	3	// Positive Pulse when CC1IF set
#define TIM1_MMS_OC1REF	4	// OC1REF as TRGO
#define TIM1_MMS_OC2REF	5	// OC2REF as TRGO
#define TIM1_MMS_OC3REF	6	// OC3REF as TRGO
#define TIM1_MMS_OC4REF	7	// OC4REF as TRGO

//TIM1_SMCR bits
#define TIM1_SMCR_SMS	0	// Clock/Trigger/Slave Mode Selection
#define TIM1_SMCR_TS	4	// Trigger Selection
#define TIM1_SMCR_MSM	7	// Master/Slave Mode

//Clock/Trigger/Slave Mode Selection -> for SMS
#define TIM1_SMS_disabled		0	// Prescaler is clocked directly by internal clock
#define TIM1_SMS_encoder_mode1		1	// Counts on TI2FP2 edge depending on TI1FP1 level
#define TIM1_SMS_encoder_mode2		2	// Counts on TI1FP1 edge depending on TI2FP2 level
#define TIM1_SMS_encoder_mode3		3	// Counts on TI1FP1 and TI2FP2 edges depending on the level of the other input
#define TIM1_SMS_trigger_reset_mode	4	// Rising edge of TRGI reinitializes the counter
#define TIM1_SMS_trigger_gated_mode	5	// Counter clock is enabled when TRGI is high
#define TIM1_SMS_trigger_standard_mode	6	// Counter starts at a rising edge of the TRGI
#define TIM1_SMS_ext_clk_mode1		7	// Rising edges of TRGI clock the signal

//Trigger Selection -> for TS
#define TIM1_TS_ITR0_TIM6_TRGO	1	// ITR1 connected to TIM1 TRGO
#define TIM1_TS_ITR3_TIM5_TRGO	3	// ITR3 connected to TIM5 TRGO
#define TIM1_TS_TI1F_ED		4	// TI1 Edge Detector
#define TIM1_TS_TI1FP1		5	// Filtered Timer Input 1
#define TIM1_TS_TI2FP2		6	// Filtered Timer Input 2
#define TIM1_TS_ETRF		7	// External Trigger Input

//TIM1_ETR bits
#define TIM1_ETR_EFT	0	// External Trigger Filter
#define TIM1_ETR_ETPS	4	// External Trigger Prescaler
#define TIM1_ETR_ECE	6	// External Clock Enable
#define TIM1_ETR_ETP	7	// External Trigger Polarity

//External Trigger Filter -> for EFT
//Input Capture Filter -> for ICxF
#define TIM1_fSAMPLING_nofilter		0	// No Filter
#define TIM1_fSAMPLING_fMASTER_N2 	1	// fSAMPLING = fMASTER, N=2
#define TIM1_fSAMPLING_fMASTER_N4 	2	// fSAMPLING = fMASTER, N=4
#define TIM1_fSAMPLING_fMASTER_N8 	3	// fSAMPLING = fMASTER, N=8
#define TIM1_fSAMPLING_fMASTER_2_N6 	4	// fSAMPLING = fMASTER/2, N=6
#define TIM1_fSAMPLING_fMASTER_2_N8 	5	// fSAMPLING = fMASTER/2, N=8
#define TIM1_fSAMPLING_fMASTER_4_N6 	6	// fSAMPLING = fMASTER/4, N=6
#define TIM1_fSAMPLING_fMASTER_4_N8 	7	// fSAMPLING = fMASTER/4, N=8
#define TIM1_fSAMPLING_fMASTER_8_N6 	8	// fSAMPLING = fMASTER/8, N=6
#define TIM1_fSAMPLING_fMASTER_8_N8 	9	// fSAMPLING = fMASTER/8, N=8
#define TIM1_fSAMPLING_fMASTER_16_N5 	10	// fSAMPLING = fMASTER/16, N=5
#define TIM1_fSAMPLING_fMASTER_16_N6 	11	// fSAMPLING = fMASTER/16, N=6
#define TIM1_fSAMPLING_fMASTER_16_N8 	12	// fSAMPLING = fMASTER/16, N=8
#define TIM1_fSAMPLING_fMASTER_32_N5 	13	// fSAMPLING = fMASTER/16, N=5
#define TIM1_fSAMPLING_fMASTER_32_N6 	14	// fSAMPLING = fMASTER/16, N=6
#define TIM1_fSAMPLING_fMASTER_32_N8 	15	// fSAMPLING = fMASTER/16, N=8

//TIM1_IER bits
#define TIM1_IER_UIE	0	// Update Interrupt Enable
#define TIM1_IER_CC1IE	1	// Capture/Compare 1 Interrupt Enable
#define TIM1_IER_CC2IE	2	// Capture/Compare 2 Interrupt Enable
#define TIM1_IER_CC3IE	3	// Capture/Compare 3 Interrupt Enable
#define TIM1_IER_CC4IE	4	// Capture/Compare 4 Interrupt Enable
#define TIM1_IER_COMIE	5	// Commutation Interrupt Enable
#define TIM1_IER_TIE	6	// Trigger Interrupt Enable
#define TIM1_IER_BIE	7	// Break Interrupt Enable

//TIM1_SR1 bits
#define TIM1_SR1_UIF	0	// Update Interrupt Flag
#define TIM1_SR1_CC1IF	1	// Capture/Compare 1 Interrupt Flag
#define TIM1_SR1_CC2IF	2	// Capture/Compare 2 Interrupt Flag
#define TIM1_SR1_CC3IF	3	// Capture/Compare 3 Interrupt Flag
#define TIM1_SR1_CC4IF	4	// Capture/Compare 4 Interrupt Flag
#define TIM1_SR1_COMIF	5	// Commutation Interrupt Flag
#define TIM1_SR1_TIF	6	// Trigger Interrupt Flag
#define TIM1_SR1_BIF	7	// Break Interrupt Flag

//TIM1_SR2 bits
#define TIM1_SR2_CC1OF	1	// Capture/Compare 1 Overcapture Flag 
#define TIM1_SR2_CC2OF	2	// Capture/Compare 2 Overcapture Flag
#define TIM1_SR2_CC3OF	3	// Capture/Compare 3 Overcapture Flag
#define TIM1_SR2_CC4OF	4	// Capture/Compare 4 Overcapture Flag

//TIM1_EGR bits
#define TIM1_EGR_UG	0	// Update Generation
#define TIM1_EGR_CC1G	1	// Capture/Compare 1 Generation
#define TIM1_EGR_CC2G	2	// Capture/Compare 2 Generation
#define TIM1_EGR_CC3G	3	// Capture/Compare 3 Generation
#define TIM1_EGR_CC4G	4	// Capture/Compare 4 Generation 
#define TIM1_EGR_COMG	5	// Capture/Compare Control Update Generation
#define TIM1_EGR_TG	6	// Trigger Generation
#define TIM1_EGR_BG	7	// Break Generation

//TIM1_CCMR1 bits
#define TIM1_CCMR1_CC1S		0	// Capture/Compare 1 Selection
//Output Mode
#define TIM1_CCMR1_OC1FE	2	// Output Compare 1 Fast Enable
#define TIM1_CCMR1_OC1PE	3	// Output Compare 1 Preload Enable
#define TIM1_CCMR1_OC1M		4	// Output Compare 1 Mode
#define TIM1_CCMR1_OC1CE	7	// Output Compare 1 Clear Enable
//Input Mode
#define TIM1_CCMR1_IC1PSC 	2	// Input Capture 1 Prescaler
#define TIM1_CCMR1_IC1F 	4	// Input Capture 1 Filter

//TIM1_CCMR2 bits
#define TIM1_CCMR2_CC2S 	0	// Capture/Compare 2 Selection
//Output Mode
#define TIM1_CCMR2_OC2FE	2	// Output Compare 2 Fast Enable
#define TIM1_CCMR2_OC2PE	3	// Output Compare 2 Preload Enable
#define TIM1_CCMR2_OC2M 	4	// Output Compare 2 Mode
#define TIM1_CCMR2_OC2CE	7	// Output Compare 2 Clear Enable
//Input Mode
#define TIM1_CCMR2_IC2PSC 	2	// Input Capture 2 Prescaler
#define TIM1_CCMR2_IC2F 	4	// Input Capture 2 Filter

//TIM1_CCMR3 bits
#define TIM1_CCMR3_CC3S 	0	// Capture/Compare 3 Selection
//Output Mode
#define TIM1_CCMR3_OC3FE	2	// Output Compare 3 Fast Enable
#define TIM1_CCMR3_OC3PE	3	// Output Compare 3 Preload Enable
#define TIM1_CCMR3_OC3M 	4	// Output Compare 3 Mode
#define TIM1_CCMR3_OC3CE	7	// Output Compare 3 Clear Enable
//Input Mode
#define TIM1_CCMR3_IC3PSC 	2	// Input Capture 3 Prescaler
#define TIM1_CCMR3_IC3F 	4	// Input Capture 3 Filter

//TIM1_CCMR4 bits
#define TIM1_CCMR4_CC4S 	0	// Capture/Compare 4 Selection
//Output Mode
#define TIM1_CCMR4_OC4FE	2	// Output Compare 4 Fast Enable
#define TIM1_CCMR4_OC4PE	3	// Output Compare 4 Preload Enable
#define TIM1_CCMR4_OC4M 	4	// Output Compare 4 Mode
#define TIM1_CCMR4_OC4CE	7	// Output Compare 4 Clear Enable
//Input Mode
#define TIM1_CCMR4_IC4PSC 	2	// Input Capture 4 Prescaler
#define TIM1_CCMR4_IC4F 	4	// Input Capture 4 Filter

//Capture/Compare Selection -> for CC1S
#define TIM1_CC1_output		0	// CC1 Channel Configured as Output
#define TIM1_CC1_input_TI1FP1	1	// CC1 Channel Configured as Input, IC1 is mapped on TI1FP1
#define TIM1_CC1_input_TI2FP1	2	// CC1 Channel Configured as Input, IC1 is mapped on TI2FP1
#define TIM1_CC1_input_TRC	3	// CC1 Channel Configured as Input, IC1 is mapped on TRC
//Capture/Compare Selection -> for CC2S
#define TIM1_CC2_output		0	// CC2 Channel Configured as Output
#define TIM1_CC2_input_TI2FP2	1	// CC2 Channel Configured as Input, IC2 is mapped on TI2FP2
#define TIM1_CC2_input_TI1FP2	2	// CC2 Channel Configured as Input, IC2 is mapped on TI1FP2
#define TIM1_CC2_input_TRC	3	// CC2 Channel Configured as Input, IC2 is mapped on TRC
//Capture/Compare Selection -> for CC3S
#define TIM1_CC3_output		0	// CC3 Channel Configured as Output
#define TIM1_CC3_input_TI3FP3	1	// CC3 Channel Configured as Input, IC3 is mapped on TI3FP3
#define TIM1_CC3_input_TI4FP3	2	// CC3 Channel Configured as Input, IC3 is mapped on TI4FP3
//Capture/Compare Selection -> for CC4S
#define TIM1_CC4_output		0	// CC4 Channel Configured as Output
#define TIM1_CC4_input_TI4FP4	1	// CC4 Channel Configured as Input, IC4 is mapped on TI4FP4
#define TIM1_CC4_input_TI3FP4	2	// CC4 Channel Configured as Input, IC4 is mapped on TI3FP4

//Output Compare Mode -> for OCxM
#define TIM1_OCxREF_frozen		0	// The comparison has no effect on the outputs
#define TIM1_OCxREF_active		1	// OCxREF is forced high when TIM1_CNT matches TIM1_CCRx
#define TIM1_OCxREF_inactive		2	// OCxREF is forced low when TIM1_CNT matches TIM1_CCRx
#define TIM1_OCxREF_toggle		3	// OCxREF toggles when TIM1_CNT matches TIM1_CCRx
#define TIM1_OCxREF_force_inactive	4	// OCxREF is forced low
#define TIM1_OCxREF_force_active	5	// OCxREF is forced high
#define TIM1_OCxREF_PWM_mode1		6	// PWM Mode 1
#define TIM1_OCxREF_PWM_mode2		7	// PWM Mode 2

//Input Capture Filter -> for ICxPSC
#define TIM1_ICxPSC_noprescaler	0	// Capture is made each time an edge is detected
#define TIM1_ICxPSC_2events	1	// Capture is made once every 2 events
#define TIM1_ICxPSC_4events	2	// Capture is made once every 4 events
#define TIM1_ICxPSC_8events	3	// Capture is made once every 8 events

//Input Capture Filter -> for ICxF
//options same as EFT

//TIM1_CCER1 bits
#define TIM1_CCER1_CC1E		0	// Capture/Compare 1 Output Enable 
#define TIM1_CCER1_CC1P		1	// Capture/Compare 1 Output Polarity
#define TIM1_CCER1_CC1NE	2	// Capture/Compare 1 Complementary Output Enable
#define TIM1_CCER1_CC1NP	3	// Capture/Compare 1 Complementary Output Polarity
#define TIM1_CCER1_CC2E		4	// Capture/Compare 2 Output Enable
#define TIM1_CCER1_CC2P		5	// Capture/Compare 2 Output Polarity
#define TIM1_CCER1_CC2NE	6	// Capture/Compare 2 Complementary Output Enable
#define TIM1_CCER1_CC2NP	7	// Capture/Compare 2 Complementary Output Polarity

//TIM1_CCER2 bits
#define TIM1_CCER2_CC3E		0	// Capture/Compare 3 Output Enable
#define TIM1_CCER2_CC3P		1	// Capture/Compare 3 Output Polarity
#define TIM1_CCER2_CC3NE	2	// Capture/Compare 3 Complementary Output Enable
#define TIM1_CCER2_CC3NP	3	// Capture/Compare 3 Complementary Output Polarity
#define TIM1_CCER2_CC4E		4	// Capture/Compare 4 Output Enable
#define TIM1_CCER2_CC4P		5	// Capture/Compare 4 Output Polarity
#define TIM1_CCER2_CC4NE	6	// Capture/Compare 4 Complementary Output Enable
#define TIM1_CCER2_CC4NP	7	// Capture/Compare 4 Complementary Output Polarity

//TIM1_PSCRH & TIM1_PSCRL
//fCK_CNT = fCK_PSC/(PSCR[15:0]+1)

//TIM1_BKR bits
#define TIM1_BKR_LOCK 	0	// Lock Configuration
#define TIM1_BKR_OSSI	2	// Off State Selection for Idle Mode
#define TIM1_BKR_OSSR	3	// Off State Selection for Run Mode
#define TIM1_BKR_BKE	4	// Break Enable
#define TIM1_BKR_BKP	5	// Break Polarity
#define TIM1_BKR_AOE	6	// Automatic Output Enable
#define TIM1_BKR_MOE	7	// Main Output Enable

//Lock Configuration -> for LOCK
#define TIM1_LOCK_off		0	// No bits are write protected
#define TIM1_LOCK_level1	1	// OIS & BKE/BKP/AOE can no longer be written
#define TIM1_LOCK_level2	2	// LOCK level 1 + CC polarity bits
#define TIM1_LOCK_level3	3	// LOCK level 2 + CC control bits

//TIM1_DTR bits
#define TIM1_DTR_DTG 	0	// Deadtime Generator Set-Up
//DTG[7:5]=0XX -> DT = DTG[7:0] x tDTG 		with tDTG = tCK_PSC
//DTG[7:5]=10X -> DT = (64+DTG[5:0]) x tDTG 	with tDTG = 2xtCK_PSC
//DTG[7:5]=110 -> DT = (32+DTG[4:0]) x tDTG 	with tDTG = 8xtCK_PSC
//DTG[7:5]=111 -> DT = (32+DTG[4:0]) x tDTG 	with tDTG = 16xtCK_PSC

//TIM1_OISR bits
#define TIM1_OISR_OIS1	0	// Output Idle State 1 (OC1)
#define TIM1_OISR_OIS1N	1	// Output Idle State 1 (OC1N)
#define TIM1_OISR_OIS2	2	// Output Idle State 2 (OC2)
#define TIM1_OISR_OIS2N	3	// Output Idle State 2 (OC2N)
#define TIM1_OISR_OIS3	4	// Output Idle State 3 (OC3)
#define TIM1_OISR_OIS3N	5	// Output Idle State 3 (OC3N)
#define TIM1_OISR_OIS4	6	// Output Idle State 4 (OC4)


#endif
