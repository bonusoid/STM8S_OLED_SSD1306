//writer : bonus adityas (bonusonic@gmail.com)
//11 january 2020


#ifndef __PERIPH_STM8S_H
#define __PERIPH_STM8S_H

#define UART_RX_INTERRUPT_VECTOR	18
#define UART_RX_INTERRUPT_DISABLED	0
#define UART_RX_INTERRUPT_ENABLED	1

//Clock
void clock_init();	//Clock initialization

//I2C
void i2c_init();	//I2C initialization
void i2c_set_start();	//Send I2C START condition
void i2c_set_address(unsigned char addr, unsigned char dir);	//Send I2C Address and Data Direction
void i2c_set_stop();	//Send I2C STOP condition
void i2c_clear_ack();	//No Acknowledge returned
void i2c_set_ack();	//Acknowledge returned after 1 byte received
void i2c_ack_pos_current(); 	//ACK bit controls the (N)ACK of the current byte being received in the shift register
void i2c_ack_pos_next();	//ACK bit controls the (N)ACK of the next byte which will be received in the shift register
void i2c_poll_SB();	//Wait until Start Bit is set
void i2c_poll_ADDR();	//Wait until Address Flag is set
void i2c_poll_BTF();	//Wait until Byte Transfer Flag is set
void i2c_poll_TXE();	//Wait until TXE Bit is set -> Data Register Empty (Transmitter)
void i2c_poll_RXNE();   //Wait until RXNE Bit is set -> Data Register Not Empty (Receiver)
void i2c_clear_bits();	//Clear Start Bit/Stop Bit/BTF by read SR1
void i2c_clear_ADDR();	//Clear Address Flag by read SR1 & SR3
void i2c_enable_interrupts();	//Enable all I2C Interrupts
void i2c_disable_interrupts();	//Disable all I2C Interrupts
void i2c_write_1byte(unsigned char devaddr, unsigned char dbyte1); //Send 1 byte
void i2c_write_2byte(unsigned char devaddr, unsigned char dbyte1, unsigned char dbyte2); //Send 2 bytes

//ADC
void adc_init();	//ADC initialization
unsigned int read_adc(unsigned char adcch);	//Read specific ADC channel

//UART
void uart1_init(unsigned char rxien);	//UART initialization
void uart1_send(unsigned char usend);	//UART transmit a byte
unsigned char uart1_recv();		//UART receive a byte (using Polling)
unsigned char uart1_recv_i();		//UART receive a byte (using Interrupt)

#endif
