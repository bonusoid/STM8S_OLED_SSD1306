//writer : bonus adityas (bonusonic@gmail.com)
//11 january 2020

#include"REG/stm8s_clk.h"
#include"REG/stm8s_itc.h"
#include"REG/stm8s_i2c.h"
#include"REG/ADC/stm8s_adc1.h"
#include"REG/UART/stm8s_uart1.h"
#include"REG/TIMER/stm8s_tim1.h"
#include"REG/TIMER/stm8s_tim2.h"
#include"periph_stm8s.h"

unsigned char readreg;

//^^^^^^^^^^ CLOCK SETTING ^^^^^^^^^^//
void clock_init()
{
	CLK_CKDIVR = 0x00; //fMASTER = fCPU = fHSI = 16MHz
	CLK_ICKR = (1<<CLK_ICKR_HSIEN);	//High Speed Internal RC Oscillator enabled
}
//__________ CLOCK SETTING __________//

//^^^^^^^^^^ I2C FUNCTION ^^^^^^^^^^//
void i2c_init()
{
	I2C_CR1 = (0<<I2C_CR1_PE); //Disable I2C before configuration starts
	I2C_FREQR = 16;	//fCLK = 16 MHz
	I2C_CCRH = (0<<I2C_CCRH_FS)|(0<<I2C_CCRH_CCR11)|(0<<I2C_CCRH_CCR10)|(0<<I2C_CCRH_CCR9)|(0<<I2C_CCRH_CCR8); //Standard Mode
	I2C_CCRL = 0x80;  //Clock Speed = 100 kHz

	I2C_OARH = (0<<I2C_OARH_ADDMODE)|(1<<I2C_OARH_ADDCONF); //7-bit address mode, ADDCONF always must be 1
	I2C_TRISER = 17;  //Setup Bus Characteristic

	I2C_CR1 = (1<<I2C_CR1_PE);  //Enable I2C after configuration complete
}

void i2c_set_start()
{
	I2C_CR2 |= (1<<I2C_CR2_START);
}

void i2c_set_address(unsigned char addr, unsigned char dir)
{
	if(dir==I2C_READ) I2C_DR = (addr<<1)|dir;
	else if(dir==I2C_WRITE) I2C_DR = (addr<<1)&dir;
	else {}
}

void i2c_set_stop()
{
	I2C_CR2 |= (1<<I2C_CR2_STOP);
}

void i2c_clear_ack()
{
	I2C_CR2 &= ~(1<<I2C_CR2_ACK); //Disable Acknowledge
}

void i2c_set_ack()
{
	I2C_CR2 |= (1<<I2C_CR2_ACK); //Enable Acknowledge
}

void i2c_ack_pos_current()
{
	I2C_CR2 &= ~(1<<I2C_CR2_POS); //ACK bit controls the (N)ACK of the current byte being received in the shift register
}

void i2c_ack_pos_next()
{
	I2C_CR2 |= (1<<I2C_CR2_POS); //ACK bit controls the (N)ACK of the next byte which will be received in the shift register
}

void i2c_poll_SB()
{
	while((I2C_SR1&(1<<I2C_SR1_SB))!=(1<<I2C_SR1_SB)){}
}

void i2c_poll_ADDR()
{
	while((I2C_SR1&(1<<I2C_SR1_ADDR))!=(1<<I2C_SR1_ADDR)){}
}

void i2c_poll_BTF()
{
	while((I2C_SR1&(1<<I2C_SR1_BTF))!=(1<<I2C_SR1_BTF)){}
}

void i2c_poll_TXE()
{
	while((I2C_SR1&(1<<I2C_SR1_TXE))!=(1<<I2C_SR1_TXE)){}
}

void i2c_poll_RXNE()
{
	while((I2C_SR1&(1<<I2C_SR1_RXNE))!=(1<<I2C_SR1_RXNE)){}
}

void i2c_clear_bits()
{
	readreg = I2C_SR1;
}

void i2c_clear_ADDR()
{
	readreg = I2C_SR1;
	readreg = I2C_SR3;
}

void i2c_enable_interrupts()
{
	I2C_ITR = 0x07;
}
void i2c_disable_interrupts()
{
	I2C_ITR = 0x00;
}

void i2c_write_1byte(unsigned char devaddr, unsigned char dbyte1)
{
	i2c_set_start(); //Send Start Condition
	i2c_poll_SB(); //Wait until Start Bit is set --> Start Condition generated
	i2c_clear_bits(); //Clear Start Bit

	i2c_set_address(devaddr,I2C_WRITE); //Write Address w Direction : Write
	i2c_poll_ADDR(); //Wait until Address Flag is set --> Address matched
	i2c_clear_ADDR(); //Clear Address Flag

	i2c_poll_TXE(); //Wait until Data Register is empty. In practice, this step is optional
	I2C_DR = dbyte1; //Command or Data
	i2c_poll_BTF(); //Wait until Byte Transfer Flag is set --> 1 Byte Data Transfer is complete
	i2c_clear_bits(); //Clear Byte Transfer Flag

	i2c_set_stop(); //Send Stop Condition
	i2c_clear_bits(); //Clear Stop Bit
}

void i2c_write_2byte(unsigned char devaddr, unsigned char dbyte1, unsigned char dbyte2)
{
	i2c_set_start(); //Send Start Condition
	i2c_poll_SB(); //Wait until Start Bit is set --> Start Condition generated
	i2c_clear_bits(); //Clear Start Bit

	i2c_set_address(devaddr,I2C_WRITE); //Write Address w Direction : Write
	i2c_poll_ADDR(); //Wait until Address Flag is set --> Address matched
	i2c_clear_ADDR(); //Clear Address Flag

	i2c_poll_TXE(); //Wait until Data Register is empty. In practice, this step is optional
	I2C_DR = dbyte1; //1st Byte of Command or Data
	i2c_poll_BTF(); //Wait until Byte Transfer Flag is set --> 1 Byte Data Transfer is complete
	i2c_clear_bits(); //Clear Byte Transfer Flag

	I2C_DR = dbyte2; //2nd Byte of Command or Data
	i2c_poll_BTF(); //Wait until Byte Transfer Flag is set --> 1 Byte Data Transfer is complete
	i2c_clear_bits(); //Clear Byte Transfer Flag

	i2c_set_stop(); //Send Stop Condition
	i2c_clear_bits(); //Clear Stop Bit
}
//__________ I2C FUNCTION __________//

//^^^^^^^^^^ ADC FUNCTION ^^^^^^^^^^//
void adc_init()
{
	ADC1_CR1 = fADC_fMASTER_8<<ADC1_CR1_SPSEL; // ADCCLK = MCLK/8
	ADC1_CR2 = (1<<ADC1_CR2_ALIGN);  // right alignment adc data

	ADC1_CR1 |= (1<<ADC1_CR1_ADON);  // turn on ADC
	//delay_ms(10);		// wait for the ADC to be ready to use
}

unsigned int read_adc(unsigned char adcch)
{
	unsigned int adcval;

	ADC1_CSR &= 0xF0;  // select
	ADC1_CSR |= adcch; // channel
	//ADC1_TDRL = (1<<adcch);

	ADC1_CR1 |= (1<<ADC1_CR1_ADON); // start conversion
	while(!((ADC1_CSR)&(1<<ADC1_CSR_EOC)));; // conversion is in progress
	adcval = (ADC1_DRH<<8) + ADC1_DRL;
	ADC1_CSR |= (0<<ADC1_CSR_EOC); // reset EOC

	return adcval;
}
//__________ ADC FUNCTION __________//

//^^^^^^^^^^ UART FUNCTION ^^^^^^^^^^//
void uart1_init(unsigned char rxien) //UART Initialization
{
	//Baud Rate = 9600
	//CLK-DIV = fMASTER/BR = 16000000/9600 = 1667 = 0x0683 = 0b 0000 0110 1000 0011
	UART1_BRR1 = 0x68;
	UART1_BRR2 = 0x03;

	UART1_CR1 |= (0<<UART1_CR1_M)|(0<<UART1_CR1_PCEN); //8 bit Data; No Parity
	UART1_CR3 |= (0<<UART1_CR3_STOP); //Stop Bit = 1

	if(rxien==1) 
	{
		UART1_CR2 |= (1<<UART1_CR2_RIEN); //Enable Interrupt on Receiver Mode
		ITC_SPR5 = (level_2<<ITC_SPR5_VECT18); //UART Interrupt Setting
	}

	UART1_CR2 |= (1<<UART1_CR2_TEN); //Enable Transmitter Mode
	UART1_CR2 |= (1<<UART1_CR2_REN); //Enable Receiver Mode
}

void uart1_send(unsigned char usend) //UART Transmit a Byte
{
	UART1_DR = usend; //Write to UART Data Register
	while((UART1_SR&(1<<UART1_SR_TXE))!=(1<<UART1_SR_TXE)); //Wait until Transmission complete
}

unsigned char uart1_recv() //UART Receive a Byte (using Polling)
{
	unsigned char urecv;
	if((UART1_SR&(1<<UART1_SR_RXNE))==(1<<UART1_SR_RXNE)) //Check if any data in Data Register
	{
		urecv = UART1_DR; //Read Data Register (RXNE cleared automatically)
	}
	else urecv=0;
	return urecv;
}

unsigned char uart1_recv_i() //UART Receive a Byte (using Interrupt)
{
	unsigned char urecv;
	urecv = UART1_DR; //Read Data Register (RXNE cleared automatically)
	return urecv;
}
//__________ UART FUNCTION __________//

