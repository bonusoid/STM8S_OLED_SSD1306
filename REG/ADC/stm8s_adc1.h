//writer : bonus adityas (bonusonic@gmail.com)
//22 august 2016 (revised in 20 may 2020)


#ifndef __STM8S_ADC1_H
#define __STM8S_ADC1_H


// ADC1 REGISTERS

#define ADC1_BASE_ADDRESS 0x53E0

#define ADC1_DB0RH	*(volatile unsigned char*)(ADC1_BASE_ADDRESS + 0x00)	// ADC1 Data Buffer Register 0 High
#define ADC1_DB0RL	*(volatile unsigned char*)(ADC1_BASE_ADDRESS + 0x01)	// ADC1 Data Buffer Register 0 Low
#define ADC1_DB1RH	*(volatile unsigned char*)(ADC1_BASE_ADDRESS + 0x02)	// ADC1 Data Buffer Register 1 High
#define ADC1_DB1RL	*(volatile unsigned char*)(ADC1_BASE_ADDRESS + 0x03)	// ADC1 Data Buffer Register 1 Low
#define ADC1_DB2RH	*(volatile unsigned char*)(ADC1_BASE_ADDRESS + 0x04)	// ADC1 Data Buffer Register 2 High
#define ADC1_DB2RL	*(volatile unsigned char*)(ADC1_BASE_ADDRESS + 0x05)	// ADC1 Data Buffer Register 2 Low
#define ADC1_DB3RH	*(volatile unsigned char*)(ADC1_BASE_ADDRESS + 0x06)	// ADC1 Data Buffer Register 3 High
#define ADC1_DB3RL	*(volatile unsigned char*)(ADC1_BASE_ADDRESS + 0x07)	// ADC1 Data Buffer Register 3 Low
#define ADC1_DB4RH	*(volatile unsigned char*)(ADC1_BASE_ADDRESS + 0x08)	// ADC1 Data Buffer Register 4 High
#define ADC1_DB4RL	*(volatile unsigned char*)(ADC1_BASE_ADDRESS + 0x09)	// ADC1 Data Buffer Register 4 Low
#define ADC1_DB5RH	*(volatile unsigned char*)(ADC1_BASE_ADDRESS + 0x0A)	// ADC1 Data Buffer Register 5 High
#define ADC1_DB5RL	*(volatile unsigned char*)(ADC1_BASE_ADDRESS + 0x0B)	// ADC1 Data Buffer Register 5 Low
#define ADC1_DB6RH	*(volatile unsigned char*)(ADC1_BASE_ADDRESS + 0x0C)	// ADC1 Data Buffer Register 6 High
#define ADC1_DB6RL	*(volatile unsigned char*)(ADC1_BASE_ADDRESS + 0x0D)	// ADC1 Data Buffer Register 6 Low
#define ADC1_DB7RH	*(volatile unsigned char*)(ADC1_BASE_ADDRESS + 0x0E)	// ADC1 Data Buffer Register 7 High
#define ADC1_DB7RL	*(volatile unsigned char*)(ADC1_BASE_ADDRESS + 0x0F)	// ADC1 Data Buffer Register 7 Low
#define ADC1_DB8RH	*(volatile unsigned char*)(ADC1_BASE_ADDRESS + 0x10)	// ADC1 Data Buffer Register 8 High
#define ADC1_DB8RL	*(volatile unsigned char*)(ADC1_BASE_ADDRESS + 0x11)	// ADC1 Data Buffer Register 8 Low
#define ADC1_DB9RH	*(volatile unsigned char*)(ADC1_BASE_ADDRESS + 0x12)	// ADC1 Data Buffer Register 9 High
#define ADC1_DB9RL	*(volatile unsigned char*)(ADC1_BASE_ADDRESS + 0x13)	// ADC1 Data Buffer Register 9 Low
#define ADC1_CSR	*(volatile unsigned char*)(ADC1_BASE_ADDRESS + 0x20)	// ADC1 Control/Status Register
#define ADC1_CR1	*(volatile unsigned char*)(ADC1_BASE_ADDRESS + 0x21)	// ADC1 Configuration Register 1
#define ADC1_CR2	*(volatile unsigned char*)(ADC1_BASE_ADDRESS + 0x22)	// ADC1 Configuration Register 2
#define ADC1_CR3	*(volatile unsigned char*)(ADC1_BASE_ADDRESS + 0x23)	// ADC1 Configuration Register 3
#define ADC1_DRH	*(volatile unsigned char*)(ADC1_BASE_ADDRESS + 0x24)	// ADC1 Data Register High
#define ADC1_DRL	*(volatile unsigned char*)(ADC1_BASE_ADDRESS + 0x25)	// ADC1 Data Register Low
#define ADC1_TDRH	*(volatile unsigned char*)(ADC1_BASE_ADDRESS + 0x26)	// ADC1 Schmitt Trigger Disable Register High
#define ADC1_TDRL	*(volatile unsigned char*)(ADC1_BASE_ADDRESS + 0x27)	// ADC1 Schmitt Trigger Disable Register Low
#define ADC1_HTRH	*(volatile unsigned char*)(ADC1_BASE_ADDRESS + 0x28)	// ADC1 High Threshold Register High
#define ADC1_HTRL	*(volatile unsigned char*)(ADC1_BASE_ADDRESS + 0x29)	// ADC1 High Threshold Register Low
#define ADC1_LTRH	*(volatile unsigned char*)(ADC1_BASE_ADDRESS + 0x2A)	// ADC1 Low Threshold Register High 
#define ADC1_LTRL	*(volatile unsigned char*)(ADC1_BASE_ADDRESS + 0x2B)	// ADC1 Low Threshold Register Low
#define ADC1_AWSRH	*(volatile unsigned char*)(ADC1_BASE_ADDRESS + 0x2C)	// ADC1 Watchdog Status Register High
#define ADC1_AWSRL	*(volatile unsigned char*)(ADC1_BASE_ADDRESS + 0x2D)	// ADC1 Watchdog Status Register Low
#define ADC1_AWCRH	*(volatile unsigned char*)(ADC1_BASE_ADDRESS + 0x2E)	// ADC1 Watchdog Control Register High
#define ADC1_AWCRL	*(volatile unsigned char*)(ADC1_BASE_ADDRESS + 0x2F)	// ADC1 Watchdog Control Register Low

//ADC1_CSR bits
#define ADC1_CSR_CH	0	// Channel Selection Bit	
#define ADC1_CSR_AWDIE	4	// Analog Watchdog Interrupt Enable
#define ADC1_CSR_EOCIE	5	// Interrupt Enable for EOC
#define ADC1_CSR_AWD	6	// Analog Watchdog Flag
#define ADC1_CSR_EOC	7	// End of Conversion

//ADC1_CR1 bits
#define ADC1_CR1_ADON	0	// A/D Converter On/Off
#define ADC1_CR1_CONT	1	// Continuous Conversion
#define ADC1_CR1_SPSEL	4	// Prescaler Selection

//Prescaler Selection -> for SPSEL
#define fADC_fMASTER_2	0	// fADC = fMASTER/2
#define fADC_fMASTER_3	1	// fADC = fMASTER/3
#define fADC_fMASTER_4	2	// fADC = fMASTER/4
#define fADC_fMASTER_6	3	// fADC = fMASTER/6
#define fADC_fMASTER_8	4	// fADC = fMASTER/8
#define fADC_fMASTER_10	5	// fADC = fMASTER/10
#define fADC_fMASTER_12	6	// fADC = fMASTER/12
#define fADC_fMASTER_18	7	// fADC = fMASTER/18

//ADC1_CR2 bits
#define ADC1_CR2_SCAN		1	// Scan Mode Enable
#define ADC1_CR2_ALIGN		3	// Data Alignment
#define ADC1_CR2_EXTSEL		4	// External Event Selection
#define ADC1_CR2_EXTTRIG	6	// External Trigger Enable

//External Event Selection -> for EXTSEL
#define EXT_TIM1_TRGO	0	// Trigger from TIM1 TRGO Event
#define EXT_ADC_ETR	1	// Trigger from ADC_ETR pin

//ADC1_CR3 bits
#define ADC1_CR3_OVR 	6	// Overrun Flag
#define ADC1_CR3_DBUF	7	// Data Buffer Enable


#endif
