//writer : bonus adityas (bonusonic@gmail.com)
//23 august 2016 (revised in 15 may 2020)


#ifndef __STM8S_CLK_H
#define __STM8S_CLK_H


// CLOCK CONTROL REGISTERS

#define CLK_BASE_ADDRESS 0x50C0

#define CLK_ICKR	*(volatile unsigned char*)(CLK_BASE_ADDRESS + 0x00)	// Internal Clock Register
#define CLK_ECKR	*(volatile unsigned char*)(CLK_BASE_ADDRESS + 0x01)	// External Clock Register
#define CLK_CMSR	*(volatile unsigned char*)(CLK_BASE_ADDRESS + 0x03)	// Clock Master Status Register
#define CLK_SWR		*(volatile unsigned char*)(CLK_BASE_ADDRESS + 0x04)	// Clock Master Switch Register
#define CLK_SWCR	*(volatile unsigned char*)(CLK_BASE_ADDRESS + 0x05)	// Switch Control Register
#define CLK_CKDIVR	*(volatile unsigned char*)(CLK_BASE_ADDRESS + 0x06)	// Clock Divider Register
#define CLK_PCKENR1	*(volatile unsigned char*)(CLK_BASE_ADDRESS + 0x07)	// Peripheral Clock Gating Register 1
#define CLK_CSSR	*(volatile unsigned char*)(CLK_BASE_ADDRESS + 0x08)	// Clock Security System Register
#define CLK_CCOR	*(volatile unsigned char*)(CLK_BASE_ADDRESS + 0x09)	// Configurable Clock Output Register
#define CLK_PCKENR2	*(volatile unsigned char*)(CLK_BASE_ADDRESS + 0x0A)	// Peripheral Clock Gating Register 2
#define CLK_HSITRIMR	*(volatile unsigned char*)(CLK_BASE_ADDRESS + 0x0C)	// HSI Clock Calibration Trimming Register
#define CLK_SWIMCCR	*(volatile unsigned char*)(CLK_BASE_ADDRESS + 0x0D)	// SWIM Clock Control Register

//CLK_ICKR bits
#define CLK_ICKR_HSIEN	0	// High Speed Internal RC Oscillator Enable
#define CLK_ICKR_HSIRDY	1	// High Speed Internal Oscillator Ready
#define CLK_ICKR_FHWU	2	// Fast Wakeup from Halt/Active-Halt Modes
#define CLK_ICKR_LSIEN	3	// Low Speed Internal RC Oscillator Enable
#define CLK_ICKR_LSIRDY	4	// Low Speed Internal RC Oscillator Ready
#define CLK_ICKR_REGAH	5	// Regulator Power Off in Active-Halt Mode

//CLK_ECKR bits
#define CLK_ECKR_HSEEN	0	// High Speed External Crystal Oscillator Enable
#define CLK_ECKR_HSERDY	1	// High Speed External Crystal Oscillator Ready

//CLK_CMSR Status
#define CKM_HSI	0xE1
#define CKM_LSI	0xD2
#define CKM_HSE	0xB4

//CLK_SWR Status 
#define SWI_HSI	0xE1
#define SWI_LSI	0xD2
#define SWI_HSE	0xB4

//CLK_SWCR bits
#define CLK_SWCR_SWBSY	0	// Switch Busy
#define CLK_SWCR_SWEN	1	// Switch Start/Stop
#define CLK_SWCR_SWIEN	2	// Clock Switch Interrupt Enable
#define CLK_SWCR_SWIF	3	// Clock Switch Interrupt Flag

//CLK_CKDIVR bits
#define CLK_CKDIVR_CPUDIV	0	// CPU Clock Prescaler bits
#define CLK_CKDIVR_HSIDIV	3	// High Speed Internal Clock Prescaler bits

//CPU Prescaler -> for CPUDIV
#define fCPU_fMASTER		0	// fCPU = fMASTER
#define fCPU_fMASTER_2		1	// fCPU = fMASTER/2
#define fCPU_fMASTER_4		2	// fCPU = fMASTER/4
#define fCPU_fMASTER_8		3	// fCPU = fMASTER/8
#define fCPU_fMASTER_16		4	// fCPU = fMASTER/16
#define fCPU_fMASTER_32		5	// fCPU = fMASTER/32
#define fCPU_fMASTER_64		6	// fCPU = fMASTER/64
#define fCPU_fMASTER_128	7	// fCPU = fMASTER/128

//HSI Prescaler -> for HSIDIV
#define fHSI_RC		0	// fHSI = fHSI_RC
#define fHSI_RC_2	1	// fHSI = fHSI_RC/2 
#define fHSI_RC_4	2	// fHSI = fHSI_RC/4 
#define fHSI_RC_8	3	// fHSI = fHSI_RC/8 

//CLK_PCKENR1 bits
#define CLK_PCKEN10	0	// fMASTER for I2C
#define CLK_PCKEN11	1	// fMASTER for SPI
#define CLK_PCKEN12	2	// fMASTER for UART (product dependent)
#define CLK_PCKEN13	3	// fMASTER for UART (product dependent)
#define CLK_PCKEN14	4	// fMASTER for TIM4/TIM6 (product dependent)
#define CLK_PCKEN15	5	// fMASTER for TIM2/TIM5 (product dependent)
#define CLK_PCKEN16	6	// fMASTER for TIM3
#define CLK_PCKEN17	7	// fMASTER for TIM1

//PCKENx Setting
#define fMASTER_disabled	0	//fMASTER to peripheral disabled
#define fMASTER_enabled		1	//fMASTER to peripheral enabled

//CLK_CSSR bits
#define CLK_CSSR_CSSEN	0	// Clock Security Sistem Enale
#define CLK_CSSR_AUX	1	// Auxiliary Oscillator Connected to Master Clock
#define CLK_CSSR_CSSDIE	2	// Clock Security System Detection Interrupt Enable
#define CLK_CSSR_CSSD	3	// Clock Security System Detection

//CLK_CCOR bits
#define CLK_CCOR_CCOEN		0	// Configurable Clock Output Enable
#define CLK_CCOR_CCOSEL		1	// Configurable Clock Output Selection bits
#define CLK_CCOR_CCORDY		5	// Configurale Clock Output Ready
#define CLK_CCOR_CCOBSY		6	// Configurable Clock Output Busy

//CCO Source -> for CCOSEL
#define fCCO_fHSIDIV		 0	// fCCO = fHSIDIV
#define fCCO_fLSI		 1	// fCCO = fLSI
#define fCCO_fHSE		 2	// fCCO = fHSE
#define fCCO_fCPU		 4	// fCCO = fCPU
#define fCCO_fCPU_2		 5	// fCCO = fCPU/2
#define fCCO_fCPU_4		 6	// fCCO = fCPU/4
#define fCCO_fCPU_8		 7	// fCCO = fCPU/8
#define fCCO_fCPU_16		 8	// fCCO = fCPU/16
#define fCCO_fCPU_32		 9	// fCCO = fCPU/32
#define fCCO_fCPU_64	        10	// fCCO = fCPU/64
#define fCCO_fHSI		11	// fCCO = fHSI
#define fCCO_fMASTER		12	// fCCO = fMASTER

//CLK_PCKENR2 bits
#define CLK_PCKEN22	2	// fMASTER for AWU
#define CLK_PCKEN23	3	// fMASTER for ADC
#define CLK_PCKEN27	7	// fMASTER for CAN (product dependent)

//CLK_SWIMCCR
#define CLK_SWIMCCR_SWIMCLK	0	// SWIM Clock Divider


#endif
