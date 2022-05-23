//writer : bonus adityas (bonusonic@gmail.com)
//22 august 2016 (revised in 20 may 2020)


#ifndef __STM8S_ADC2_H
#define __STM8S_ADC2_H


// ADC2 REGISTERS

#define ADC2_BASE_ADDRESS 0x5400

#define ADC2_CSR	*(volatile unsigned char*)(ADC2_BASE_ADDRESS + 0x00)	// ADC2 Control/Status Register
#define ADC2_CR1	*(volatile unsigned char*)(ADC2_BASE_ADDRESS + 0x01)	// ADC2 Configuration Register 1
#define ADC2_CR2	*(volatile unsigned char*)(ADC2_BASE_ADDRESS + 0x02)	// ADC2 Configuration Register 2
#define ADC2_CR3	*(volatile unsigned char*)(ADC2_BASE_ADDRESS + 0x03)	// ADC2 Configuration Register 3
#define ADC2_DRH	*(volatile unsigned char*)(ADC2_BASE_ADDRESS + 0x04)	// ADC2 Data Register High
#define ADC2_DRL	*(volatile unsigned char*)(ADC2_BASE_ADDRESS + 0x05)	// ADC2 Data Register Low
#define ADC2_TDRH	*(volatile unsigned char*)(ADC2_BASE_ADDRESS + 0x06)	// ADC2 Schmitt Trigger Disable Register High
#define ADC2_TDRL	*(volatile unsigned char*)(ADC2_BASE_ADDRESS + 0x07)	// ADC2 Schmitt Trigger Disable Register Low

//ADC2_CSR bits
#define ADC2_CSR_CH	0	// Channel Selection Bit	
#define ADC2_CSR_AWDIE	4	// Analog Watchdog Interrupt Enable
#define ADC2_CSR_EOCIE	5	// Interrupt Enable for EOC
#define ADC2_CSR_AWD	6	// Analog Watchdog Flag
#define ADC2_CSR_EOC	7	// End of Conversion

//ADC2_CR1 bits
#define ADC2_CR1_ADON	0	// A/D Converter On/Off
#define ADC2_CR1_CONT	1	// Continuous Conversion
#define ADC2_CR1_SPSEL	4	// Prescaler Selection

//Prescaler Selection -> for SPSEL
#define fADC_fMASTER_2	0	// fADC = fMASTER/2
#define fADC_fMASTER_3	1	// fADC = fMASTER/3
#define fADC_fMASTER_4	2	// fADC = fMASTER/4
#define fADC_fMASTER_6	3	// fADC = fMASTER/6
#define fADC_fMASTER_8	4	// fADC = fMASTER/8
#define fADC_fMASTER_10	5	// fADC = fMASTER/10
#define fADC_fMASTER_12	6	// fADC = fMASTER/12
#define fADC_fMASTER_18	7	// fADC = fMASTER/18

//ADC2_CR2 bits
#define ADC2_CR2_SCAN		1	// Scan Mode Enable
#define ADC2_CR2_ALIGN		3	// Data Alignment
#define ADC2_CR2_EXTSEL		4	// External Event Selection
#define ADC2_CR2_EXTTRIG	6	// External Trigger Enable

//External Event Selection -> for EXTSEL
#define EXT_TIM1_TRGO	0	// Trigger from TIM1 TRGO Event
#define EXT_ADC_ETR	1	// Trigger from ADC_ETR pin

//ADC2_CR3 bits
#define ADC2_CR3_OVR 	6	// Overrun Flag
#define ADC2_CR3_DBUF	7	// Data Buffer Enable


#endif
