//writer : bonus adityas (bonusonic@gmail.com)
//18 may 2020


#ifndef __STM8S_BEEP_H
#define __STM8S_BEEP_H


// BEEP REGISTERS

#define BEEP_BASE_ADDRESS 0x50F3

#define BEEP_CSR *(volatile unsigned char*)(I2C_BASE_ADDRESS + 0x00)	// BEEP Control/Status Register

//BEEP_CSR bits
#define BEEP_CSR_BEEPDIV	0	// Beep Prescaler Divider
#define BEEP_CSR_BEEPEN		5	// Beep Enable
#define BEEP_CSR_BEEPSEL	6	// Beep Selection

//Beep Prescaler Divider
//BEEPDIV = DIV-2

//Beep Selection
#define fBEEP_fLS_8XBEEPDIV	0	// fBEEP_OUT = fLS/(8xBEEPDIV) kHz
#define fBEEP_fLS_4XBEEPDIV	1	// fBEEP_OUT = fLS/(4xBEEPDIV) kHz
#define fBEEP_fLS_2XBEEPDIV	2	// fBEEP_OUT = fLS/(2xBEEPDIV) kHz


#endif

