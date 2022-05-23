;--------------------------------------------------------
; File Created by SDCC : free open source ANSI-C Compiler
; Version 3.5.0 #9253 (Apr  3 2018) (Linux)
; This file was generated Mon May 23 21:01:18 2022
;--------------------------------------------------------
	.module main
	.optsdcc -mstm8
	
;--------------------------------------------------------
; Public variables in this module
;--------------------------------------------------------
	.globl _font_arr
	.globl _main
	.globl _dtri
	.globl _dsine
	.globl _readreg
	.globl _delay_init
	.globl _delay_us
	.globl _delay_ms
	.globl _delay_timer
	.globl _clock_init
	.globl _i2c_init
	.globl _i2c_set_start
	.globl _i2c_set_address
	.globl _i2c_set_stop
	.globl _i2c_clear_ack
	.globl _i2c_set_ack
	.globl _i2c_ack_pos_current
	.globl _i2c_ack_pos_next
	.globl _i2c_poll_SB
	.globl _i2c_poll_ADDR
	.globl _i2c_poll_BTF
	.globl _i2c_poll_TXE
	.globl _i2c_poll_RXNE
	.globl _i2c_clear_bits
	.globl _i2c_clear_ADDR
	.globl _i2c_enable_interrupts
	.globl _i2c_disable_interrupts
	.globl _i2c_write_1byte
	.globl _i2c_write_2byte
	.globl _adc_init
	.globl _read_adc
	.globl _uart1_init
	.globl _uart1_send
	.globl _uart1_recv
	.globl _uart1_recv_i
	.globl _ssd1306_init
	.globl _ssd1306_sendcom
	.globl _ssd1306_senddat
	.globl _ssd1306_setpos
	.globl _ssd1306_clear
	.globl _OLED_setpos
	.globl _OLED_drawbyte
	.globl _OLED_drawchar
	.globl _OLED_drawtext
	.globl _OLED_drawint
	.globl _OLED_clear
	.globl _OLED_clearblock
	.globl _OLED_normal
	.globl _OLED_reverse
	.globl _loop
	.globl _gpio_init
	.globl _drawInt
	.globl _drawAlphanum
	.globl _drawPunct
	.globl _drawFrame
	.globl _drawArrow
	.globl _drawBytes
	.globl _drawLoadingBar
;--------------------------------------------------------
; ram data
;--------------------------------------------------------
	.area DATA
_readreg::
	.ds 1
;--------------------------------------------------------
; ram data
;--------------------------------------------------------
	.area INITIALIZED
_dsine::
	.ds 10
_dtri::
	.ds 14
;--------------------------------------------------------
; Stack segment in internal ram 
;--------------------------------------------------------
	.area	SSEG
__start__stack:
	.ds	1

;--------------------------------------------------------
; absolute external ram data
;--------------------------------------------------------
	.area DABS (ABS)
;--------------------------------------------------------
; interrupt vector 
;--------------------------------------------------------
	.area HOME
__interrupt_vect:
	int s_GSINIT ;reset
	int 0x0000 ;trap
	int 0x0000 ;int0
	int 0x0000 ;int1
	int 0x0000 ;int2
	int 0x0000 ;int3
	int 0x0000 ;int4
	int 0x0000 ;int5
	int 0x0000 ;int6
	int 0x0000 ;int7
	int 0x0000 ;int8
	int 0x0000 ;int9
	int 0x0000 ;int10
	int 0x0000 ;int11
	int 0x0000 ;int12
	int 0x0000 ;int13
	int 0x0000 ;int14
	int 0x0000 ;int15
	int 0x0000 ;int16
	int 0x0000 ;int17
	int 0x0000 ;int18
	int 0x0000 ;int19
	int 0x0000 ;int20
	int 0x0000 ;int21
	int 0x0000 ;int22
	int 0x0000 ;int23
	int 0x0000 ;int24
	int 0x0000 ;int25
	int 0x0000 ;int26
	int 0x0000 ;int27
	int 0x0000 ;int28
	int 0x0000 ;int29
;--------------------------------------------------------
; global & static initialisations
;--------------------------------------------------------
	.area HOME
	.area GSINIT
	.area GSFINAL
	.area GSINIT
__sdcc_gs_init_startup:
__sdcc_init_data:
; stm8_genXINIT() start
	ldw x, #l_DATA
	jreq	00002$
00001$:
	clr (s_DATA - 1, x)
	decw x
	jrne	00001$
00002$:
	ldw	x, #l_INITIALIZER
	jreq	00004$
00003$:
	ld	a, (s_INITIALIZER - 1, x)
	ld	(s_INITIALIZED - 1, x), a
	decw	x
	jrne	00003$
00004$:
; stm8_genXINIT() end
	.area GSFINAL
	jp	__sdcc_program_startup
;--------------------------------------------------------
; Home
;--------------------------------------------------------
	.area HOME
	.area HOME
__sdcc_program_startup:
	jp	_main
;	return from main will return to caller
;--------------------------------------------------------
; code
;--------------------------------------------------------
	.area CODE
;	delay.c: 7: void delay_init()
;	-----------------------------------------
;	 function delay_init
;	-----------------------------------------
_delay_init:
;	delay.c: 9: TIM4_PSCR = 4; // CLK/16
	mov	0x5347+0, #0x04
	ret
;	delay.c: 12: void delay_us(unsigned long delus)
;	-----------------------------------------
;	 function delay_us
;	-----------------------------------------
_delay_us:
	sub	sp, #6
;	delay.c: 16: for(du=0;du<(delus/10);du++)
	push	#0x0a
	clrw	x
	pushw	x
	push	#0x00
	ldw	x, (0x0f, sp)
	pushw	x
	ldw	x, (0x0f, sp)
	pushw	x
	call	__divulong
	addw	sp, #8
	ldw	(0x05, sp), x
	ldw	(0x03, sp), y
	clrw	x
	ldw	(0x01, sp), x
00103$:
	ldw	x, (0x01, sp)
	clrw	y
	cpw	x, (0x05, sp)
	ld	a, yl
	sbc	a, (0x04, sp)
	ld	a, yh
	sbc	a, (0x03, sp)
	jrnc	00101$
;	delay.c: 18: delay_timer(100);
	push	#0x64
	call	_delay_timer
	pop	a
;	delay.c: 16: for(du=0;du<(delus/10);du++)
	ldw	x, (0x01, sp)
	incw	x
	ldw	(0x01, sp), x
	jra	00103$
00101$:
;	delay.c: 20: delay_timer(delus%10);
	push	#0x0a
	clrw	x
	pushw	x
	push	#0x00
	ldw	x, (0x0f, sp)
	pushw	x
	ldw	x, (0x0f, sp)
	pushw	x
	call	__modulong
	addw	sp, #8
	ld	a, xl
	push	a
	call	_delay_timer
	addw	sp, #7
	ret
;	delay.c: 23: void delay_ms(unsigned long delms)
;	-----------------------------------------
;	 function delay_ms
;	-----------------------------------------
_delay_ms:
	sub	sp, #8
;	delay.c: 27: for(dm=0;dm<(delms*100);dm++)
	ldw	x, (0x0d, sp)
	pushw	x
	ldw	x, (0x0d, sp)
	pushw	x
	push	#0x64
	clrw	x
	pushw	x
	push	#0x00
	call	__mullong
	addw	sp, #8
	ldw	(0x07, sp), x
	ldw	(0x05, sp), y
	clrw	x
	clr	a
	clr	(0x01, sp)
00103$:
	push	a
	cpw	x, (0x08, sp)
	ld	a, (1, sp)
	sbc	a, (0x07, sp)
	ld	a, (0x02, sp)
	sbc	a, (0x06, sp)
	pop	a
	jrnc	00105$
;	delay.c: 29: delay_timer(100);
	push	a
	pushw	x
	push	#0x64
	call	_delay_timer
	pop	a
	popw	x
	pop	a
;	delay.c: 27: for(dm=0;dm<(delms*100);dm++)
	addw	x, #0x0001
	adc	a, #0x00
	push	a
	ld	a, (0x02, sp)
	adc	a, #0x00
	ld	(0x02, sp), a
	pop	a
	jra	00103$
00105$:
	addw	sp, #8
	ret
;	delay.c: 33: void delay_timer(unsigned char deltim)
;	-----------------------------------------
;	 function delay_timer
;	-----------------------------------------
_delay_timer:
;	delay.c: 35: TIM4_CR1 = (1<<TIM4_CR1_CEN);
	mov	0x5340+0, #0x01
;	delay.c: 36: while(TIM4_CNTR<deltim);
00101$:
	ldw	x, #0x5346
	ld	a, (x)
	cp	a, (0x03, sp)
	jrc	00101$
;	delay.c: 37: TIM4_CR1 = (0<<TIM4_CR1_CEN);
	mov	0x5340+0, #0x00
;	delay.c: 38: TIM4_CNTR = 0; //reset timer	
	mov	0x5346+0, #0x00
	ret
;	periph_stm8s.c: 16: void clock_init()
;	-----------------------------------------
;	 function clock_init
;	-----------------------------------------
_clock_init:
;	periph_stm8s.c: 18: CLK_CKDIVR = 0x00; //fMASTER = fCPU = fHSI = 16MHz
	mov	0x50c6+0, #0x00
;	periph_stm8s.c: 19: CLK_ICKR = (1<<CLK_ICKR_HSIEN);	//High Speed Internal RC Oscillator enabled
	mov	0x50c0+0, #0x01
	ret
;	periph_stm8s.c: 24: void i2c_init()
;	-----------------------------------------
;	 function i2c_init
;	-----------------------------------------
_i2c_init:
;	periph_stm8s.c: 26: I2C_CR1 = (0<<I2C_CR1_PE); //Disable I2C before configuration starts
	mov	0x5210+0, #0x00
;	periph_stm8s.c: 27: I2C_FREQR = 16;	//fCLK = 16 MHz
	mov	0x5212+0, #0x10
;	periph_stm8s.c: 28: I2C_CCRH = (0<<I2C_CCRH_FS)|(0<<I2C_CCRH_CCR11)|(0<<I2C_CCRH_CCR10)|(0<<I2C_CCRH_CCR9)|(0<<I2C_CCRH_CCR8); //Standard Mode
	mov	0x521c+0, #0x00
;	periph_stm8s.c: 29: I2C_CCRL = 0x80;  //Clock Speed = 100 kHz
	mov	0x521b+0, #0x80
;	periph_stm8s.c: 31: I2C_OARH = (0<<I2C_OARH_ADDMODE)|(1<<I2C_OARH_ADDCONF); //7-bit address mode, ADDCONF always must be 1
	mov	0x5214+0, #0x40
;	periph_stm8s.c: 32: I2C_TRISER = 17;  //Setup Bus Characteristic
	mov	0x521d+0, #0x11
;	periph_stm8s.c: 34: I2C_CR1 = (1<<I2C_CR1_PE);  //Enable I2C after configuration complete
	mov	0x5210+0, #0x01
	ret
;	periph_stm8s.c: 37: void i2c_set_start()
;	-----------------------------------------
;	 function i2c_set_start
;	-----------------------------------------
_i2c_set_start:
;	periph_stm8s.c: 39: I2C_CR2 |= (1<<I2C_CR2_START);
	bset	0x5211, #0
	ret
;	periph_stm8s.c: 42: void i2c_set_address(unsigned char addr, unsigned char dir)
;	-----------------------------------------
;	 function i2c_set_address
;	-----------------------------------------
_i2c_set_address:
;	periph_stm8s.c: 44: if(dir==I2C_READ) I2C_DR = (addr<<1)|dir;
	ld	a, (0x03, sp)
	ld	xl, a
	sllw	x
	ld	a, (0x04, sp)
	cp	a, #0x01
	jrne	00104$
	ld	a, xl
	or	a, (0x04, sp)
	ldw	x, #0x5216
	ld	(x), a
	jra	00106$
00104$:
;	periph_stm8s.c: 45: else if(dir==I2C_WRITE) I2C_DR = (addr<<1)&dir;
	ld	a, (0x04, sp)
	cp	a, #0xfe
	jrne	00106$
	ld	a, xl
	and	a, (0x04, sp)
	ldw	x, #0x5216
	ld	(x), a
00106$:
	ret
;	periph_stm8s.c: 49: void i2c_set_stop()
;	-----------------------------------------
;	 function i2c_set_stop
;	-----------------------------------------
_i2c_set_stop:
;	periph_stm8s.c: 51: I2C_CR2 |= (1<<I2C_CR2_STOP);
	ldw	x, #0x5211
	ld	a, (x)
	or	a, #0x02
	ld	(x), a
	ret
;	periph_stm8s.c: 54: void i2c_clear_ack()
;	-----------------------------------------
;	 function i2c_clear_ack
;	-----------------------------------------
_i2c_clear_ack:
;	periph_stm8s.c: 56: I2C_CR2 &= ~(1<<I2C_CR2_ACK); //Disable Acknowledge
	ldw	x, #0x5211
	ld	a, (x)
	and	a, #0xfb
	ld	(x), a
	ret
;	periph_stm8s.c: 59: void i2c_set_ack()
;	-----------------------------------------
;	 function i2c_set_ack
;	-----------------------------------------
_i2c_set_ack:
;	periph_stm8s.c: 61: I2C_CR2 |= (1<<I2C_CR2_ACK); //Enable Acknowledge
	ldw	x, #0x5211
	ld	a, (x)
	or	a, #0x04
	ld	(x), a
	ret
;	periph_stm8s.c: 64: void i2c_ack_pos_current()
;	-----------------------------------------
;	 function i2c_ack_pos_current
;	-----------------------------------------
_i2c_ack_pos_current:
;	periph_stm8s.c: 66: I2C_CR2 &= ~(1<<I2C_CR2_POS); //ACK bit controls the (N)ACK of the current byte being received in the shift register
	ldw	x, #0x5211
	ld	a, (x)
	and	a, #0xf7
	ld	(x), a
	ret
;	periph_stm8s.c: 69: void i2c_ack_pos_next()
;	-----------------------------------------
;	 function i2c_ack_pos_next
;	-----------------------------------------
_i2c_ack_pos_next:
;	periph_stm8s.c: 71: I2C_CR2 |= (1<<I2C_CR2_POS); //ACK bit controls the (N)ACK of the next byte which will be received in the shift register
	ldw	x, #0x5211
	ld	a, (x)
	or	a, #0x08
	ld	(x), a
	ret
;	periph_stm8s.c: 74: void i2c_poll_SB()
;	-----------------------------------------
;	 function i2c_poll_SB
;	-----------------------------------------
_i2c_poll_SB:
;	periph_stm8s.c: 76: while((I2C_SR1&(1<<I2C_SR1_SB))!=(1<<I2C_SR1_SB)){}
00101$:
	ldw	x, #0x5217
	ld	a, (x)
	and	a, #0x01
	cp	a, #0x01
	jrne	00101$
	ret
;	periph_stm8s.c: 79: void i2c_poll_ADDR()
;	-----------------------------------------
;	 function i2c_poll_ADDR
;	-----------------------------------------
_i2c_poll_ADDR:
;	periph_stm8s.c: 81: while((I2C_SR1&(1<<I2C_SR1_ADDR))!=(1<<I2C_SR1_ADDR)){}
00101$:
	ldw	x, #0x5217
	ld	a, (x)
	and	a, #0x02
	cp	a, #0x02
	jrne	00101$
	ret
;	periph_stm8s.c: 84: void i2c_poll_BTF()
;	-----------------------------------------
;	 function i2c_poll_BTF
;	-----------------------------------------
_i2c_poll_BTF:
;	periph_stm8s.c: 86: while((I2C_SR1&(1<<I2C_SR1_BTF))!=(1<<I2C_SR1_BTF)){}
00101$:
	ldw	x, #0x5217
	ld	a, (x)
	and	a, #0x04
	cp	a, #0x04
	jrne	00101$
	ret
;	periph_stm8s.c: 89: void i2c_poll_TXE()
;	-----------------------------------------
;	 function i2c_poll_TXE
;	-----------------------------------------
_i2c_poll_TXE:
;	periph_stm8s.c: 91: while((I2C_SR1&(1<<I2C_SR1_TXE))!=(1<<I2C_SR1_TXE)){}
00101$:
	ldw	x, #0x5217
	ld	a, (x)
	and	a, #0x80
	cp	a, #0x80
	jrne	00101$
	ret
;	periph_stm8s.c: 94: void i2c_poll_RXNE()
;	-----------------------------------------
;	 function i2c_poll_RXNE
;	-----------------------------------------
_i2c_poll_RXNE:
;	periph_stm8s.c: 96: while((I2C_SR1&(1<<I2C_SR1_RXNE))!=(1<<I2C_SR1_RXNE)){}
00101$:
	ldw	x, #0x5217
	ld	a, (x)
	and	a, #0x40
	cp	a, #0x40
	jrne	00101$
	ret
;	periph_stm8s.c: 99: void i2c_clear_bits()
;	-----------------------------------------
;	 function i2c_clear_bits
;	-----------------------------------------
_i2c_clear_bits:
;	periph_stm8s.c: 101: readreg = I2C_SR1;
	ldw	x, #0x5217
	ld	a, (x)
	ld	_readreg+0, a
	ret
;	periph_stm8s.c: 104: void i2c_clear_ADDR()
;	-----------------------------------------
;	 function i2c_clear_ADDR
;	-----------------------------------------
_i2c_clear_ADDR:
;	periph_stm8s.c: 106: readreg = I2C_SR1;
	ldw	x, #0x5217
	ld	a, (x)
;	periph_stm8s.c: 107: readreg = I2C_SR3;
	ldw	x, #0x5219
	ld	a, (x)
	ld	_readreg+0, a
	ret
;	periph_stm8s.c: 110: void i2c_enable_interrupts()
;	-----------------------------------------
;	 function i2c_enable_interrupts
;	-----------------------------------------
_i2c_enable_interrupts:
;	periph_stm8s.c: 112: I2C_ITR = 0x07;
	mov	0x521a+0, #0x07
	ret
;	periph_stm8s.c: 114: void i2c_disable_interrupts()
;	-----------------------------------------
;	 function i2c_disable_interrupts
;	-----------------------------------------
_i2c_disable_interrupts:
;	periph_stm8s.c: 116: I2C_ITR = 0x00;
	mov	0x521a+0, #0x00
	ret
;	periph_stm8s.c: 119: void i2c_write_1byte(unsigned char devaddr, unsigned char dbyte1)
;	-----------------------------------------
;	 function i2c_write_1byte
;	-----------------------------------------
_i2c_write_1byte:
;	periph_stm8s.c: 121: i2c_set_start(); //Send Start Condition
	call	_i2c_set_start
;	periph_stm8s.c: 122: i2c_poll_SB(); //Wait until Start Bit is set --> Start Condition generated
	call	_i2c_poll_SB
;	periph_stm8s.c: 123: i2c_clear_bits(); //Clear Start Bit
	call	_i2c_clear_bits
;	periph_stm8s.c: 125: i2c_set_address(devaddr,I2C_WRITE); //Write Address w Direction : Write
	push	#0xfe
	ld	a, (0x04, sp)
	push	a
	call	_i2c_set_address
	addw	sp, #2
;	periph_stm8s.c: 126: i2c_poll_ADDR(); //Wait until Address Flag is set --> Address matched
	call	_i2c_poll_ADDR
;	periph_stm8s.c: 127: i2c_clear_ADDR(); //Clear Address Flag
	call	_i2c_clear_ADDR
;	periph_stm8s.c: 129: i2c_poll_TXE(); //Wait until Data Register is empty. In practice, this step is optional
	call	_i2c_poll_TXE
;	periph_stm8s.c: 130: I2C_DR = dbyte1; //Command or Data
	ldw	x, #0x5216
	ld	a, (0x04, sp)
	ld	(x), a
;	periph_stm8s.c: 131: i2c_poll_BTF(); //Wait until Byte Transfer Flag is set --> 1 Byte Data Transfer is complete
	call	_i2c_poll_BTF
;	periph_stm8s.c: 132: i2c_clear_bits(); //Clear Byte Transfer Flag
	call	_i2c_clear_bits
;	periph_stm8s.c: 134: i2c_set_stop(); //Send Stop Condition
	call	_i2c_set_stop
;	periph_stm8s.c: 135: i2c_clear_bits(); //Clear Stop Bit
	jp	_i2c_clear_bits
;	periph_stm8s.c: 138: void i2c_write_2byte(unsigned char devaddr, unsigned char dbyte1, unsigned char dbyte2)
;	-----------------------------------------
;	 function i2c_write_2byte
;	-----------------------------------------
_i2c_write_2byte:
;	periph_stm8s.c: 140: i2c_set_start(); //Send Start Condition
	call	_i2c_set_start
;	periph_stm8s.c: 141: i2c_poll_SB(); //Wait until Start Bit is set --> Start Condition generated
	call	_i2c_poll_SB
;	periph_stm8s.c: 142: i2c_clear_bits(); //Clear Start Bit
	call	_i2c_clear_bits
;	periph_stm8s.c: 144: i2c_set_address(devaddr,I2C_WRITE); //Write Address w Direction : Write
	push	#0xfe
	ld	a, (0x04, sp)
	push	a
	call	_i2c_set_address
	addw	sp, #2
;	periph_stm8s.c: 145: i2c_poll_ADDR(); //Wait until Address Flag is set --> Address matched
	call	_i2c_poll_ADDR
;	periph_stm8s.c: 146: i2c_clear_ADDR(); //Clear Address Flag
	call	_i2c_clear_ADDR
;	periph_stm8s.c: 148: i2c_poll_TXE(); //Wait until Data Register is empty. In practice, this step is optional
	call	_i2c_poll_TXE
;	periph_stm8s.c: 149: I2C_DR = dbyte1; //1st Byte of Command or Data
	ldw	x, #0x5216
	ld	a, (0x04, sp)
	ld	(x), a
;	periph_stm8s.c: 150: i2c_poll_BTF(); //Wait until Byte Transfer Flag is set --> 1 Byte Data Transfer is complete
	call	_i2c_poll_BTF
;	periph_stm8s.c: 151: i2c_clear_bits(); //Clear Byte Transfer Flag
	call	_i2c_clear_bits
;	periph_stm8s.c: 153: I2C_DR = dbyte2; //2nd Byte of Command or Data
	ldw	x, #0x5216
	ld	a, (0x05, sp)
	ld	(x), a
;	periph_stm8s.c: 154: i2c_poll_BTF(); //Wait until Byte Transfer Flag is set --> 1 Byte Data Transfer is complete
	call	_i2c_poll_BTF
;	periph_stm8s.c: 155: i2c_clear_bits(); //Clear Byte Transfer Flag
	call	_i2c_clear_bits
;	periph_stm8s.c: 157: i2c_set_stop(); //Send Stop Condition
	call	_i2c_set_stop
;	periph_stm8s.c: 158: i2c_clear_bits(); //Clear Stop Bit
	jp	_i2c_clear_bits
;	periph_stm8s.c: 163: void adc_init()
;	-----------------------------------------
;	 function adc_init
;	-----------------------------------------
_adc_init:
;	periph_stm8s.c: 165: ADC1_CR1 = fADC_fMASTER_8<<ADC1_CR1_SPSEL; // ADCCLK = MCLK/8
	mov	0x5401+0, #0x40
;	periph_stm8s.c: 166: ADC1_CR2 = (1<<ADC1_CR2_ALIGN);  // right alignment adc data
	mov	0x5402+0, #0x08
;	periph_stm8s.c: 168: ADC1_CR1 |= (1<<ADC1_CR1_ADON);  // turn on ADC
	bset	0x5401, #0
	ret
;	periph_stm8s.c: 172: unsigned int read_adc(unsigned char adcch)
;	-----------------------------------------
;	 function read_adc
;	-----------------------------------------
_read_adc:
	sub	sp, #4
;	periph_stm8s.c: 176: ADC1_CSR &= 0xF0;  // select
	ldw	x, #0x5400
	ld	a, (x)
	and	a, #0xf0
	ld	(x), a
;	periph_stm8s.c: 177: ADC1_CSR |= adcch; // channel
	ldw	x, #0x5400
	ld	a, (x)
	or	a, (0x07, sp)
	ldw	x, #0x5400
	ld	(x), a
;	periph_stm8s.c: 180: ADC1_CR1 |= (1<<ADC1_CR1_ADON); // start conversion
	bset	0x5401, #0
;	periph_stm8s.c: 181: while(!((ADC1_CSR)&(1<<ADC1_CSR_EOC)));; // conversion is in progress
00101$:
	ldw	x, #0x5400
	ld	a, (x)
	tnz	a
	jrpl	00101$
;	periph_stm8s.c: 182: adcval = (ADC1_DRH<<8) + ADC1_DRL;
	ldw	x, #0x5404
	ld	a, (x)
	clr	(0x03, sp)
	ld	(0x01, sp), a
	clr	(0x02, sp)
	ldw	x, #0x5405
	ld	a, (x)
	clrw	x
	ld	xl, a
	addw	x, (0x01, sp)
;	periph_stm8s.c: 183: ADC1_CSR |= (0<<ADC1_CSR_EOC); // reset EOC
	ldw	y, #0x5400
	ld	a, (y)
	ldw	y, #0x5400
	ld	(y), a
;	periph_stm8s.c: 185: return adcval;
	addw	sp, #4
	ret
;	periph_stm8s.c: 190: void uart1_init(unsigned char rxien) //UART Initialization
;	-----------------------------------------
;	 function uart1_init
;	-----------------------------------------
_uart1_init:
;	periph_stm8s.c: 194: UART1_BRR1 = 0x68;
	mov	0x5232+0, #0x68
;	periph_stm8s.c: 195: UART1_BRR2 = 0x03;
	mov	0x5233+0, #0x03
;	periph_stm8s.c: 197: UART1_CR1 |= (0<<UART1_CR1_M)|(0<<UART1_CR1_PCEN); //8 bit Data; No Parity
	ldw	x, #0x5234
	ld	a, (x)
	ldw	x, #0x5234
	ld	(x), a
;	periph_stm8s.c: 198: UART1_CR3 |= (0<<UART1_CR3_STOP); //Stop Bit = 1
	ldw	x, #0x5236
	ld	a, (x)
	ldw	x, #0x5236
	ld	(x), a
;	periph_stm8s.c: 200: if(rxien==1) 
	ld	a, (0x03, sp)
	cp	a, #0x01
	jrne	00102$
;	periph_stm8s.c: 202: UART1_CR2 |= (1<<UART1_CR2_RIEN); //Enable Interrupt on Receiver Mode
	ldw	x, #0x5235
	ld	a, (x)
	or	a, #0x20
	ld	(x), a
;	periph_stm8s.c: 203: ITC_SPR5 = (level_2<<ITC_SPR5_VECT18); //UART Interrupt Setting
	mov	0x7f74+0, #0x00
00102$:
;	periph_stm8s.c: 206: UART1_CR2 |= (1<<UART1_CR2_TEN); //Enable Transmitter Mode
	ldw	x, #0x5235
	ld	a, (x)
	or	a, #0x08
	ld	(x), a
;	periph_stm8s.c: 207: UART1_CR2 |= (1<<UART1_CR2_REN); //Enable Receiver Mode
	ldw	x, #0x5235
	ld	a, (x)
	or	a, #0x04
	ld	(x), a
	ret
;	periph_stm8s.c: 210: void uart1_send(unsigned char usend) //UART Transmit a Byte
;	-----------------------------------------
;	 function uart1_send
;	-----------------------------------------
_uart1_send:
;	periph_stm8s.c: 212: UART1_DR = usend; //Write to UART Data Register
	ldw	x, #0x5231
	ld	a, (0x03, sp)
	ld	(x), a
;	periph_stm8s.c: 213: while((UART1_SR&(1<<UART1_SR_TXE))!=(1<<UART1_SR_TXE)); //Wait until Transmission complete
00101$:
	ldw	x, #0x5230
	ld	a, (x)
	and	a, #0x80
	cp	a, #0x80
	jrne	00101$
	ret
;	periph_stm8s.c: 216: unsigned char uart1_recv() //UART Receive a Byte (using Polling)
;	-----------------------------------------
;	 function uart1_recv
;	-----------------------------------------
_uart1_recv:
;	periph_stm8s.c: 219: if((UART1_SR&(1<<UART1_SR_RXNE))==(1<<UART1_SR_RXNE)) //Check if any data in Data Register
	ldw	x, #0x5230
	ld	a, (x)
	and	a, #0x20
	cp	a, #0x20
	jrne	00102$
;	periph_stm8s.c: 221: urecv = UART1_DR; //Read Data Register (RXNE cleared automatically)
	ldw	x, #0x5231
	ld	a, (x)
;	periph_stm8s.c: 223: else urecv=0;
	.byte 0x21
00102$:
	clr	a
00103$:
;	periph_stm8s.c: 224: return urecv;
	ret
;	periph_stm8s.c: 227: unsigned char uart1_recv_i() //UART Receive a Byte (using Interrupt)
;	-----------------------------------------
;	 function uart1_recv_i
;	-----------------------------------------
_uart1_recv_i:
;	periph_stm8s.c: 230: urecv = UART1_DR; //Read Data Register (RXNE cleared automatically)
	ldw	x, #0x5231
	ld	a, (x)
;	periph_stm8s.c: 231: return urecv;
	ret
;	oled_ssd1306.c: 7: void ssd1306_init(unsigned char olednum)
;	-----------------------------------------
;	 function ssd1306_init
;	-----------------------------------------
_ssd1306_init:
;	oled_ssd1306.c: 9: ssd1306_sendcom(olednum,0xAE); //Set Display Off
	push	#0xae
	ld	a, (0x04, sp)
	push	a
	call	_ssd1306_sendcom
	addw	sp, #2
;	oled_ssd1306.c: 10: ssd1306_sendcom(olednum,0xD5); //Set Display Clock Divider Ratio/Oscillator Frequency
	push	#0xd5
	ld	a, (0x04, sp)
	push	a
	call	_ssd1306_sendcom
	addw	sp, #2
;	oled_ssd1306.c: 11: ssd1306_sendcom(olednum,0x80);
	push	#0x80
	ld	a, (0x04, sp)
	push	a
	call	_ssd1306_sendcom
	addw	sp, #2
;	oled_ssd1306.c: 12: ssd1306_sendcom(olednum,0xA8); //Set Multiplex Ratio
	push	#0xa8
	ld	a, (0x04, sp)
	push	a
	call	_ssd1306_sendcom
	addw	sp, #2
;	oled_ssd1306.c: 13: ssd1306_sendcom(olednum,0x3F);
	push	#0x3f
	ld	a, (0x04, sp)
	push	a
	call	_ssd1306_sendcom
	addw	sp, #2
;	oled_ssd1306.c: 14: ssd1306_sendcom(olednum,0xD3); //Set Display Offset
	push	#0xd3
	ld	a, (0x04, sp)
	push	a
	call	_ssd1306_sendcom
	addw	sp, #2
;	oled_ssd1306.c: 15: ssd1306_sendcom(olednum,0x00);
	push	#0x00
	ld	a, (0x04, sp)
	push	a
	call	_ssd1306_sendcom
	addw	sp, #2
;	oled_ssd1306.c: 16: ssd1306_sendcom(olednum,0x40); //Set Display Start Line
	push	#0x40
	ld	a, (0x04, sp)
	push	a
	call	_ssd1306_sendcom
	addw	sp, #2
;	oled_ssd1306.c: 17: ssd1306_sendcom(olednum,0x8D); //Set Charge Pump
	push	#0x8d
	ld	a, (0x04, sp)
	push	a
	call	_ssd1306_sendcom
	addw	sp, #2
;	oled_ssd1306.c: 18: ssd1306_sendcom(olednum,0x14); //Internal VCC
	push	#0x14
	ld	a, (0x04, sp)
	push	a
	call	_ssd1306_sendcom
	addw	sp, #2
;	oled_ssd1306.c: 19: ssd1306_sendcom(olednum,0x20); //Set Memory Mode
	push	#0x20
	ld	a, (0x04, sp)
	push	a
	call	_ssd1306_sendcom
	addw	sp, #2
;	oled_ssd1306.c: 20: ssd1306_sendcom(olednum,0x00); //Horizontal Addressing
	push	#0x00
	ld	a, (0x04, sp)
	push	a
	call	_ssd1306_sendcom
	addw	sp, #2
;	oled_ssd1306.c: 21: ssd1306_sendcom(olednum,0xA1); //Set Segment Re-Map
	push	#0xa1
	ld	a, (0x04, sp)
	push	a
	call	_ssd1306_sendcom
	addw	sp, #2
;	oled_ssd1306.c: 22: ssd1306_sendcom(olednum,0xC8); //Set COM Output Scan Direction
	push	#0xc8
	ld	a, (0x04, sp)
	push	a
	call	_ssd1306_sendcom
	addw	sp, #2
;	oled_ssd1306.c: 23: ssd1306_sendcom(olednum,0xDA); //Set COM Pins HW Config
	push	#0xda
	ld	a, (0x04, sp)
	push	a
	call	_ssd1306_sendcom
	addw	sp, #2
;	oled_ssd1306.c: 24: ssd1306_sendcom(olednum,0x12);
	push	#0x12
	ld	a, (0x04, sp)
	push	a
	call	_ssd1306_sendcom
	addw	sp, #2
;	oled_ssd1306.c: 25: ssd1306_sendcom(olednum,0x81); //Set Contrast Control
	push	#0x81
	ld	a, (0x04, sp)
	push	a
	call	_ssd1306_sendcom
	addw	sp, #2
;	oled_ssd1306.c: 26: ssd1306_sendcom(olednum,0xCF);
	push	#0xcf
	ld	a, (0x04, sp)
	push	a
	call	_ssd1306_sendcom
	addw	sp, #2
;	oled_ssd1306.c: 27: ssd1306_sendcom(olednum,0xD9); //Set Pre-Charge Period
	push	#0xd9
	ld	a, (0x04, sp)
	push	a
	call	_ssd1306_sendcom
	addw	sp, #2
;	oled_ssd1306.c: 28: ssd1306_sendcom(olednum,0xF1);
	push	#0xf1
	ld	a, (0x04, sp)
	push	a
	call	_ssd1306_sendcom
	addw	sp, #2
;	oled_ssd1306.c: 29: ssd1306_sendcom(olednum,0xDB); //Set VCOMH Deselect Level
	push	#0xdb
	ld	a, (0x04, sp)
	push	a
	call	_ssd1306_sendcom
	addw	sp, #2
;	oled_ssd1306.c: 30: ssd1306_sendcom(olednum,0x40);
	push	#0x40
	ld	a, (0x04, sp)
	push	a
	call	_ssd1306_sendcom
	addw	sp, #2
;	oled_ssd1306.c: 31: ssd1306_sendcom(olednum,0xA4); //Set Entire Display On/Off
	push	#0xa4
	ld	a, (0x04, sp)
	push	a
	call	_ssd1306_sendcom
	addw	sp, #2
;	oled_ssd1306.c: 32: ssd1306_sendcom(olednum,0xA6); //Set Normal/Inverse Display
	push	#0xa6
	ld	a, (0x04, sp)
	push	a
	call	_ssd1306_sendcom
	addw	sp, #2
;	oled_ssd1306.c: 33: ssd1306_sendcom(olednum,0xAF); //Set Display On
	push	#0xaf
	ld	a, (0x04, sp)
	push	a
	call	_ssd1306_sendcom
	addw	sp, #2
	ret
;	oled_ssd1306.c: 36: void ssd1306_sendcom(unsigned char olednum, unsigned char ssd1306com)
;	-----------------------------------------
;	 function ssd1306_sendcom
;	-----------------------------------------
_ssd1306_sendcom:
;	oled_ssd1306.c: 38: i2c_write_2byte(olednum,commode,ssd1306com); //Send Command
	ld	a, (0x04, sp)
	push	a
	push	#0x00
	ld	a, (0x05, sp)
	push	a
	call	_i2c_write_2byte
	addw	sp, #3
	ret
;	oled_ssd1306.c: 41: void ssd1306_senddat(unsigned char olednum, unsigned char ssd1306dat)
;	-----------------------------------------
;	 function ssd1306_senddat
;	-----------------------------------------
_ssd1306_senddat:
;	oled_ssd1306.c: 43: i2c_write_2byte(olednum,datmode,ssd1306dat); //Send Data
	ld	a, (0x04, sp)
	push	a
	push	#0x40
	ld	a, (0x05, sp)
	push	a
	call	_i2c_write_2byte
	addw	sp, #3
	ret
;	oled_ssd1306.c: 46: void ssd1306_setpos(unsigned char olednum, unsigned char row, unsigned char col)
;	-----------------------------------------
;	 function ssd1306_setpos
;	-----------------------------------------
_ssd1306_setpos:
;	oled_ssd1306.c: 48: ssd1306_sendcom(olednum,(0xB0|(row&0x0F))); //Set page of row
	ld	a, (0x04, sp)
	and	a, #0x0f
	or	a, #0xb0
	push	a
	ld	a, (0x04, sp)
	push	a
	call	_ssd1306_sendcom
	addw	sp, #2
;	oled_ssd1306.c: 49: ssd1306_sendcom(olednum,(0x00|(col&0x0F))); //Set lower nibble of column
	ld	a, (0x05, sp)
	and	a, #0x0f
	push	a
	ld	a, (0x04, sp)
	push	a
	call	_ssd1306_sendcom
	addw	sp, #2
;	oled_ssd1306.c: 50: ssd1306_sendcom(olednum,(0x10|((col>>4)&0x0F))); //Set upper nibble of column
	ld	a, (0x05, sp)
	swap	a
	and	a, #0x0f
	and	a, #0x0f
	or	a, #0x10
	push	a
	ld	a, (0x04, sp)
	push	a
	call	_ssd1306_sendcom
	addw	sp, #2
	ret
;	oled_ssd1306.c: 53: void ssd1306_clear(unsigned char olednum) 
;	-----------------------------------------
;	 function ssd1306_clear
;	-----------------------------------------
_ssd1306_clear:
	sub	sp, #2
;	oled_ssd1306.c: 56: ssd1306_setpos(olednum,0,0);
	push	#0x00
	push	#0x00
	ld	a, (0x07, sp)
	push	a
	call	_ssd1306_setpos
	addw	sp, #3
;	oled_ssd1306.c: 57: for(row=0;row<OLED_ROW+1;row++)	//Scan rows, add 1 row for completely flush.
	clr	(0x01, sp)
;	oled_ssd1306.c: 59: for(col=0;col<OLED_COL;col++)	//Scan columns
00109$:
	clr	(0x02, sp)
00103$:
;	oled_ssd1306.c: 61: ssd1306_senddat(olednum,0);	//Send 0 to every pixel
	push	#0x00
	ld	a, (0x06, sp)
	push	a
	call	_ssd1306_senddat
	addw	sp, #2
;	oled_ssd1306.c: 59: for(col=0;col<OLED_COL;col++)	//Scan columns
	inc	(0x02, sp)
	ld	a, (0x02, sp)
	cp	a, #0x80
	jrc	00103$
;	oled_ssd1306.c: 57: for(row=0;row<OLED_ROW+1;row++)	//Scan rows, add 1 row for completely flush.
	inc	(0x01, sp)
	ld	a, (0x01, sp)
	cp	a, #0x09
	jrc	00109$
	addw	sp, #2
	ret
;	oled_ssd1306.c: 66: void OLED_setpos(unsigned char olednum, unsigned char row, unsigned char col)
;	-----------------------------------------
;	 function OLED_setpos
;	-----------------------------------------
_OLED_setpos:
;	oled_ssd1306.c: 68: ssd1306_setpos(olednum,row,col); //Set coordinate (for LCD_drawbyte)
	ld	a, (0x05, sp)
	push	a
	ld	a, (0x05, sp)
	push	a
	ld	a, (0x05, sp)
	push	a
	call	_ssd1306_setpos
	addw	sp, #3
	ret
;	oled_ssd1306.c: 71: void OLED_drawbyte(unsigned char olednum, unsigned char dbyte)
;	-----------------------------------------
;	 function OLED_drawbyte
;	-----------------------------------------
_OLED_drawbyte:
;	oled_ssd1306.c: 73: ssd1306_senddat(olednum,dbyte); //Send 1 byte data only
	ld	a, (0x04, sp)
	push	a
	ld	a, (0x04, sp)
	push	a
	call	_ssd1306_senddat
	addw	sp, #2
	ret
;	oled_ssd1306.c: 76: void OLED_drawchar(unsigned char olednum, unsigned char chr, unsigned char chrrow, unsigned char chrcol)
;	-----------------------------------------
;	 function OLED_drawchar
;	-----------------------------------------
_OLED_drawchar:
	sub	sp, #11
;	oled_ssd1306.c: 81: ssd1306_setpos(olednum,chrrow,chrcol);
	ld	a, (0x11, sp)
	push	a
	ld	a, (0x11, sp)
	push	a
	ld	a, (0x10, sp)
	push	a
	call	_ssd1306_setpos
	addw	sp, #3
;	oled_ssd1306.c: 86: chridx=(chr-32)*5; //Start at character 32 (Space). 5 columns for each character
	ld	a, (0x0f, sp)
	ld	(0x03, sp), a
	clr	(0x02, sp)
;	oled_ssd1306.c: 83: if((chr>31)&&(chr<128))	//Alphanumeric & Punctuation Area
	ld	a, (0x0f, sp)
	cp	a, #0x1f
	jrule	00107$
	ld	a, (0x0f, sp)
	cp	a, #0x80
	jrnc	00107$
;	oled_ssd1306.c: 85: ssd1306_senddat(olednum,0x00);
	push	#0x00
	ld	a, (0x0f, sp)
	push	a
	call	_ssd1306_senddat
	addw	sp, #2
;	oled_ssd1306.c: 86: chridx=(chr-32)*5; //Start at character 32 (Space). 5 columns for each character
	ldw	x, (0x02, sp)
	subw	x, #0x0020
	pushw	x
	push	#0x05
	push	#0x00
	call	__mulint
	addw	sp, #4
	ldw	(0x0a, sp), x
;	oled_ssd1306.c: 87: for(ci=0;ci<5;ci++)
	ldw	x, #_font_arr+0
	ldw	(0x08, sp), x
	clr	(0x01, sp)
00110$:
;	oled_ssd1306.c: 89: fchar = font_arr[chridx+ci]; //Get character pattern from Font Array
	ld	a, (0x01, sp)
	ld	xl, a
	clr	a
	ld	xh, a
	addw	x, (0x0a, sp)
	addw	x, (0x08, sp)
	ld	a, (x)
;	oled_ssd1306.c: 90: ssd1306_senddat(olednum,fchar); //Send pattern 1 byte at a time
	push	a
	ld	a, (0x0f, sp)
	push	a
	call	_ssd1306_senddat
	addw	sp, #2
;	oled_ssd1306.c: 87: for(ci=0;ci<5;ci++)
	inc	(0x01, sp)
	ld	a, (0x01, sp)
	cp	a, #0x05
	jrc	00110$
	jra	00114$
00107$:
;	oled_ssd1306.c: 93: else if((chr>127)&&(chr<148)) //Frame & Arrow Area
	ld	a, (0x0f, sp)
	cp	a, #0x7f
	jrule	00114$
	ld	a, (0x0f, sp)
	cp	a, #0x94
	jrnc	00114$
;	oled_ssd1306.c: 95: chridx=(chr-128)*8; //Start at index 128. 5 columns for each symbol
	ldw	x, (0x02, sp)
	subw	x, #0x0080
	sllw	x
	sllw	x
	sllw	x
;	oled_ssd1306.c: 96: for(ci=0;ci<8;ci++)
	ldw	y, #_font_arr+0
	ldw	(0x06, sp), y
	addw	x, #0x01e0
	ldw	(0x04, sp), x
	clr	(0x01, sp)
00112$:
;	oled_ssd1306.c: 98: fchar = font_arr[chridx+480+ci]; //Get symbol pattern from Font Array
	clrw	x
	ld	a, (0x01, sp)
	ld	xl, a
	addw	x, (0x04, sp)
	addw	x, (0x06, sp)
	ld	a, (x)
;	oled_ssd1306.c: 99: ssd1306_senddat(olednum,fchar); //Send pattern 1 byte at a time		   
	push	a
	ld	a, (0x0f, sp)
	push	a
	call	_ssd1306_senddat
	addw	sp, #2
;	oled_ssd1306.c: 96: for(ci=0;ci<8;ci++)
	inc	(0x01, sp)
	ld	a, (0x01, sp)
	cp	a, #0x08
	jrc	00112$
00114$:
	addw	sp, #11
	ret
;	oled_ssd1306.c: 105: void OLED_drawtext(unsigned char olednum, unsigned char *text, unsigned char txtrow, unsigned char txtcol)
;	-----------------------------------------
;	 function OLED_drawtext
;	-----------------------------------------
_OLED_drawtext:
	sub	sp, #2
;	oled_ssd1306.c: 109: while(text[stridx] != 0) //Scan characters in string
	clrw	x
	ldw	(0x01, sp), x
00101$:
	ldw	x, (0x06, sp)
	addw	x, (0x01, sp)
	ld	a, (x)
	ld	xl, a
	tnz	a
	jreq	00104$
;	oled_ssd1306.c: 111: OLED_drawchar(olednum,text[stridx],txtrow,txtcol+(8*stridx)); //Display each character
	ld	a, (0x02, sp)
	sll	a
	sll	a
	sll	a
	add	a, (0x09, sp)
	push	a
	ld	a, (0x09, sp)
	push	a
	ld	a, xl
	push	a
	ld	a, (0x08, sp)
	push	a
	call	_OLED_drawchar
	addw	sp, #4
;	oled_ssd1306.c: 112: stridx++;
	ldw	x, (0x01, sp)
	incw	x
	ldw	(0x01, sp), x
	jra	00101$
00104$:
	addw	sp, #2
	ret
;	oled_ssd1306.c: 116: void OLED_drawint(unsigned char olednum, unsigned int num, unsigned char numrow, unsigned char numcol)
;	-----------------------------------------
;	 function OLED_drawint
;	-----------------------------------------
_OLED_drawint:
	sub	sp, #12
;	oled_ssd1306.c: 123: numb = num;
	ldw	x, (0x10, sp)
;	oled_ssd1306.c: 124: while(numb!=0) //Counting digit
	clr	a
00101$:
	tnzw	x
	jreq	00114$
;	oled_ssd1306.c: 126: ndigit++;
	inc	a
;	oled_ssd1306.c: 127: numb /= 10;	
	ldw	y, #0x000a
	divw	x, y
	jra	00101$
00114$:
	ld	(0x0b, sp), a
;	oled_ssd1306.c: 129: for(nd=0;nd<ndigit;nd++) //Converting each digit
	clr	a
	ldw	x, sp
	incw	x
	ldw	(0x09, sp), x
00106$:
	cp	a, (0x0b, sp)
	jrnc	00104$
;	oled_ssd1306.c: 131: numb = num%10;
	ldw	x, (0x10, sp)
	ldw	y, #0x000a
	divw	x, y
	ldw	(0x07, sp), y
;	oled_ssd1306.c: 132: num = num/10;
	ldw	x, (0x10, sp)
	ldw	y, #0x000a
	divw	x, y
	ldw	(0x10, sp), x
;	oled_ssd1306.c: 133: ibuff[ndigit-(nd+1)] = numb + '0'; //Start from last_index-1
	inc	a
	ld	(0x0c, sp), a
	ld	a, (0x0b, sp)
	sub	a, (0x0c, sp)
	clrw	x
	ld	xl, a
	addw	x, (0x09, sp)
	ld	a, (0x08, sp)
	add	a, #0x30
	ld	(x), a
;	oled_ssd1306.c: 129: for(nd=0;nd<ndigit;nd++) //Converting each digit
	ld	a, (0x0c, sp)
	jra	00106$
00104$:
;	oled_ssd1306.c: 135: ibuff[ndigit] = '\0'; //Last character is null
	clrw	x
	ld	a, (0x0b, sp)
	ld	xl, a
	addw	x, (0x09, sp)
	clr	(x)
;	oled_ssd1306.c: 137: OLED_drawtext(olednum,ibuff,numrow,numcol); //Display number as text
	ldw	x, (0x09, sp)
	ld	a, (0x13, sp)
	push	a
	ld	a, (0x13, sp)
	push	a
	pushw	x
	ld	a, (0x13, sp)
	push	a
	call	_OLED_drawtext
	addw	sp, #17
	ret
;	oled_ssd1306.c: 140: void OLED_clear(unsigned char olednum)
;	-----------------------------------------
;	 function OLED_clear
;	-----------------------------------------
_OLED_clear:
;	oled_ssd1306.c: 142: ssd1306_sendcom(olednum,0xAE); //Set Display off
	push	#0xae
	ld	a, (0x04, sp)
	push	a
	call	_ssd1306_sendcom
	addw	sp, #2
;	oled_ssd1306.c: 143: ssd1306_clear(olednum); //Clear Display
	ld	a, (0x03, sp)
	push	a
	call	_ssd1306_clear
	pop	a
;	oled_ssd1306.c: 144: ssd1306_sendcom(olednum,0xAF); //Set Display on
	push	#0xaf
	ld	a, (0x04, sp)
	push	a
	call	_ssd1306_sendcom
	addw	sp, #2
	ret
;	oled_ssd1306.c: 147: void OLED_clearblock(unsigned char olednum, unsigned char row, unsigned char col_start, unsigned char col_fin)
;	-----------------------------------------
;	 function OLED_clearblock
;	-----------------------------------------
_OLED_clearblock:
	push	a
;	oled_ssd1306.c: 151: ssd1306_setpos(olednum,row,col_start); 	//Set start position
	ld	a, (0x06, sp)
	push	a
	ld	a, (0x06, sp)
	push	a
	ld	a, (0x06, sp)
	push	a
	call	_ssd1306_setpos
	addw	sp, #3
;	oled_ssd1306.c: 152: for(col=col_start;col<=col_fin;col++) 	//Scan columns
	ld	a, (0x06, sp)
	ld	(0x01, sp), a
00103$:
	ld	a, (0x01, sp)
	cp	a, (0x07, sp)
	jrugt	00105$
;	oled_ssd1306.c: 154: ssd1306_senddat(olednum,0);	//Send 0 to every pixel in a column
	push	#0x00
	ld	a, (0x05, sp)
	push	a
	call	_ssd1306_senddat
	addw	sp, #2
;	oled_ssd1306.c: 152: for(col=col_start;col<=col_fin;col++) 	//Scan columns
	inc	(0x01, sp)
	jra	00103$
00105$:
	pop	a
	ret
;	oled_ssd1306.c: 158: void OLED_normal(unsigned char olednum)
;	-----------------------------------------
;	 function OLED_normal
;	-----------------------------------------
_OLED_normal:
;	oled_ssd1306.c: 160: ssd1306_sendcom(olednum,0xA6);	//On Pixel in Off Background
	push	#0xa6
	ld	a, (0x04, sp)
	push	a
	call	_ssd1306_sendcom
	addw	sp, #2
	ret
;	oled_ssd1306.c: 163: void OLED_reverse(unsigned char olednum)
;	-----------------------------------------
;	 function OLED_reverse
;	-----------------------------------------
_OLED_reverse:
;	oled_ssd1306.c: 165: ssd1306_sendcom(olednum,0xA7);	//Off Pixel in On Background
	push	#0xa7
	ld	a, (0x04, sp)
	push	a
	call	_ssd1306_sendcom
	addw	sp, #2
	ret
;	main.c: 28: int main()
;	-----------------------------------------
;	 function main
;	-----------------------------------------
_main:
;	main.c: 30: clock_init();
	call	_clock_init
;	main.c: 31: delay_init();
	call	_delay_init
;	main.c: 32: gpio_init();
	call	_gpio_init
;	main.c: 33: i2c_init();
	call	_i2c_init
;	main.c: 35: ssd1306_init(OLED1);
	push	#0x3c
	call	_ssd1306_init
	pop	a
;	main.c: 36: ssd1306_init(OLED2);
	push	#0x3d
	call	_ssd1306_init
	pop	a
;	main.c: 37: OLED_clear(OLED1);
	push	#0x3c
	call	_OLED_clear
	pop	a
;	main.c: 38: OLED_clear(OLED2);
	push	#0x3d
	call	_OLED_clear
	pop	a
;	main.c: 40: drawLoadingBar(OLED1);
	push	#0x3c
	call	_drawLoadingBar
	pop	a
;	main.c: 41: drawLoadingBar(OLED2);
	push	#0x3d
	call	_drawLoadingBar
	pop	a
;	main.c: 43: loop();
	call	_loop
;	main.c: 44: return 0;
	clrw	x
	ret
;	main.c: 50: void loop()
;	-----------------------------------------
;	 function loop
;	-----------------------------------------
_loop:
;	main.c: 52: while(OLED1)
00102$:
;	main.c: 54: drawBytes(OLED1);
	push	#0x3c
	call	_drawBytes
	pop	a
;	main.c: 55: delay_ms(1000);
	push	#0xe8
	push	#0x03
	clrw	x
	pushw	x
	call	_delay_ms
	addw	sp, #4
;	main.c: 56: OLED_clearblock(OLED1,3,5,114); //Finish column = 5 + 11*10 - 1
	push	#0x72
	push	#0x05
	push	#0x03
	push	#0x3c
	call	_OLED_clearblock
	addw	sp, #4
;	main.c: 57: delay_ms(500);
	push	#0xf4
	push	#0x01
	clrw	x
	pushw	x
	call	_delay_ms
	addw	sp, #4
;	main.c: 58: OLED_clearblock(OLED1,5,3,114); //Finish column = 3 + 8*14 - 1
	push	#0x72
	push	#0x03
	push	#0x05
	push	#0x3c
	call	_OLED_clearblock
	addw	sp, #4
;	main.c: 59: delay_ms(500);
	push	#0xf4
	push	#0x01
	clrw	x
	pushw	x
	call	_delay_ms
	addw	sp, #4
;	main.c: 61: drawInt(OLED2);
	push	#0x3d
	call	_drawInt
	pop	a
;	main.c: 62: delay_ms(1000); 
	push	#0xe8
	push	#0x03
	clrw	x
	pushw	x
	call	_delay_ms
	addw	sp, #4
;	main.c: 63: OLED_clear(OLED2);
	push	#0x3d
	call	_OLED_clear
	pop	a
;	main.c: 65: drawAlphanum(OLED1);
	push	#0x3c
	call	_drawAlphanum
	pop	a
;	main.c: 66: delay_ms(1000); 
	push	#0xe8
	push	#0x03
	clrw	x
	pushw	x
	call	_delay_ms
	addw	sp, #4
;	main.c: 67: OLED_reverse(OLED1);
	push	#0x3c
	call	_OLED_reverse
	pop	a
;	main.c: 68: delay_ms(1000);
	push	#0xe8
	push	#0x03
	clrw	x
	pushw	x
	call	_delay_ms
	addw	sp, #4
;	main.c: 69: OLED_clear(OLED1);
	push	#0x3c
	call	_OLED_clear
	pop	a
;	main.c: 70: OLED_normal(OLED1);
	push	#0x3c
	call	_OLED_normal
	pop	a
;	main.c: 72: drawPunct(OLED2);
	push	#0x3d
	call	_drawPunct
	pop	a
;	main.c: 73: delay_ms(1000); 
	push	#0xe8
	push	#0x03
	clrw	x
	pushw	x
	call	_delay_ms
	addw	sp, #4
;	main.c: 74: OLED_reverse(OLED2);
	push	#0x3d
	call	_OLED_reverse
	pop	a
;	main.c: 75: delay_ms(1000);
	push	#0xe8
	push	#0x03
	clrw	x
	pushw	x
	call	_delay_ms
	addw	sp, #4
;	main.c: 76: OLED_clear(OLED2);
	push	#0x3d
	call	_OLED_clear
	pop	a
;	main.c: 77: OLED_normal(OLED2);
	push	#0x3d
	call	_OLED_normal
	pop	a
;	main.c: 79: drawFrame(OLED1);
	push	#0x3c
	call	_drawFrame
	pop	a
;	main.c: 80: delay_ms(700); 
	push	#0xbc
	push	#0x02
	clrw	x
	pushw	x
	call	_delay_ms
	addw	sp, #4
;	main.c: 81: OLED_clearblock(OLED1,3,36,43); //Finish column = 36 + 8 - 1
	push	#0x2b
	push	#0x24
	push	#0x03
	push	#0x3c
	call	_OLED_clearblock
	addw	sp, #4
;	main.c: 82: delay_ms(700);
	push	#0xbc
	push	#0x02
	clrw	x
	pushw	x
	call	_delay_ms
	addw	sp, #4
;	main.c: 83: OLED_clear(OLED1);
	push	#0x3c
	call	_OLED_clear
	pop	a
;	main.c: 85: drawArrow(OLED2);
	push	#0x3d
	call	_drawArrow
	pop	a
;	main.c: 86: delay_ms(700); 
	push	#0xbc
	push	#0x02
	clrw	x
	pushw	x
	call	_delay_ms
	addw	sp, #4
;	main.c: 87: OLED_clearblock(OLED2,3,36,43); //Finish column = 36 + 8 - 1
	push	#0x2b
	push	#0x24
	push	#0x03
	push	#0x3d
	call	_OLED_clearblock
	addw	sp, #4
;	main.c: 88: delay_ms(700);
	push	#0xbc
	push	#0x02
	clrw	x
	pushw	x
	call	_delay_ms
	addw	sp, #4
;	main.c: 89: OLED_clear(OLED2);
	push	#0x3d
	call	_OLED_clear
	pop	a
	jp	00102$
	ret
;	main.c: 94: void gpio_init()
;	-----------------------------------------
;	 function gpio_init
;	-----------------------------------------
_gpio_init:
;	main.c: 97: }
	ret
;	main.c: 99: void drawInt(unsigned char olednum)
;	-----------------------------------------
;	 function drawInt
;	-----------------------------------------
_drawInt:
;	main.c: 101: OLED_drawint(olednum, 64, 0, 8);   //Decimal
	push	#0x08
	push	#0x00
	push	#0x40
	push	#0x00
	ld	a, (0x07, sp)
	push	a
	call	_OLED_drawint
	addw	sp, #5
;	main.c: 102: OLED_drawint(olednum, 064, 0, 48); //Octal displayed as Decimal
	push	#0x30
	push	#0x00
	push	#0x34
	push	#0x00
	ld	a, (0x07, sp)
	push	a
	call	_OLED_drawint
	addw	sp, #5
;	main.c: 103: OLED_drawint(olednum, 0x64, 0, 88); //Hexadecimal displayed as Decimal
	push	#0x58
	push	#0x00
	push	#0x64
	push	#0x00
	ld	a, (0x07, sp)
	push	a
	call	_OLED_drawint
	addw	sp, #5
;	main.c: 105: OLED_drawint(olednum, -64, 1, 8); //Negative number is not supported
	push	#0x08
	push	#0x01
	push	#0xc0
	push	#0xff
	ld	a, (0x07, sp)
	push	a
	call	_OLED_drawint
	addw	sp, #5
;	main.c: 108: OLED_drawint(olednum, 65535, 3, 8); //Max. is 65535
	push	#0x08
	push	#0x03
	push	#0xff
	push	#0xff
	ld	a, (0x07, sp)
	push	a
	call	_OLED_drawint
	addw	sp, #5
;	main.c: 110: OLED_drawint(olednum, 100, 5, 8);
	push	#0x08
	push	#0x05
	push	#0x64
	push	#0x00
	ld	a, (0x07, sp)
	push	a
	call	_OLED_drawint
	addw	sp, #5
;	main.c: 111: OLED_drawchar(olednum, SYM_DEGREE, 5, 32);
	push	#0x20
	push	#0x05
	push	#0x7f
	ld	a, (0x06, sp)
	push	a
	call	_OLED_drawchar
	addw	sp, #4
;	main.c: 112: OLED_drawchar(olednum, 'C', 5, 40);
	push	#0x28
	push	#0x05
	push	#0x43
	ld	a, (0x06, sp)
	push	a
	call	_OLED_drawchar
	addw	sp, #4
;	main.c: 114: OLED_drawtext(olednum, " OLED TEST : INT",7,0);
	ldw	x, #___str_0+0
	push	#0x00
	push	#0x07
	pushw	x
	ld	a, (0x07, sp)
	push	a
	call	_OLED_drawtext
	addw	sp, #5
	ret
;	main.c: 117: void drawAlphanum(unsigned char olednum)
;	-----------------------------------------
;	 function drawAlphanum
;	-----------------------------------------
_drawAlphanum:
;	main.c: 119: OLED_drawtext(olednum, "ABCDEFGHIJKLM",0,0);
	ldw	x, #___str_1+0
	push	#0x00
	push	#0x00
	pushw	x
	ld	a, (0x07, sp)
	push	a
	call	_OLED_drawtext
	addw	sp, #5
;	main.c: 120: OLED_drawtext(olednum, "NOPQRSTUVWXYZ",1,0);
	ldw	x, #___str_2+0
	push	#0x00
	push	#0x01
	pushw	x
	ld	a, (0x07, sp)
	push	a
	call	_OLED_drawtext
	addw	sp, #5
;	main.c: 122: OLED_drawtext(olednum, "abcdefghijklm",3,0);
	ldw	x, #___str_3+0
	push	#0x00
	push	#0x03
	pushw	x
	ld	a, (0x07, sp)
	push	a
	call	_OLED_drawtext
	addw	sp, #5
;	main.c: 123: OLED_drawtext(olednum, "nopqrstuvwxyz",4,0);
	ldw	x, #___str_4+0
	push	#0x00
	push	#0x04
	pushw	x
	ld	a, (0x07, sp)
	push	a
	call	_OLED_drawtext
	addw	sp, #5
;	main.c: 125: OLED_drawtext(olednum, "0123456789",6,0);
	ldw	x, #___str_5+0
	push	#0x00
	push	#0x06
	pushw	x
	ld	a, (0x07, sp)
	push	a
	call	_OLED_drawtext
	addw	sp, #5
;	main.c: 127: OLED_drawtext(olednum, "OLED TEST : CHAR",7,0);
	ldw	x, #___str_6+0
	push	#0x00
	push	#0x07
	pushw	x
	ld	a, (0x07, sp)
	push	a
	call	_OLED_drawtext
	addw	sp, #5
	ret
;	main.c: 130: void drawPunct(unsigned char olednum)
;	-----------------------------------------
;	 function drawPunct
;	-----------------------------------------
_drawPunct:
;	main.c: 132: OLED_drawtext(olednum, "<{([+_-=])}>",0,0);
	ldw	x, #___str_7+0
	push	#0x00
	push	#0x00
	pushw	x
	ld	a, (0x07, sp)
	push	a
	call	_OLED_drawtext
	addw	sp, #5
;	main.c: 133: OLED_drawtext(olednum, "!@#$%^&*`|~?",2,0);
	ldw	x, #___str_8+0
	push	#0x00
	push	#0x02
	pushw	x
	ld	a, (0x07, sp)
	push	a
	call	_OLED_drawtext
	addw	sp, #5
;	main.c: 134: OLED_drawtext(olednum, ".\,\"\'\\/ :;",4,0);
	ldw	x, #___str_9+0
	push	#0x00
	push	#0x04
	pushw	x
	ld	a, (0x07, sp)
	push	a
	call	_OLED_drawtext
	addw	sp, #5
;	main.c: 136: OLED_drawtext(olednum, "OLED TEST : CHAR",7,0);
	ldw	x, #___str_6+0
	push	#0x00
	push	#0x07
	pushw	x
	ld	a, (0x07, sp)
	push	a
	call	_OLED_drawtext
	addw	sp, #5
	ret
;	main.c: 139: void drawFrame(unsigned char olednum)
;	-----------------------------------------
;	 function drawFrame
;	-----------------------------------------
_drawFrame:
;	main.c: 143: OLED_drawchar(olednum, FRAME_TOP_LEFT,1,startcol);
	push	#0x14
	push	#0x01
	push	#0x80
	ld	a, (0x06, sp)
	push	a
	call	_OLED_drawchar
	addw	sp, #4
;	main.c: 144: OLED_drawchar(olednum, FRAME_LINE_HOR,1,startcol+8);
	push	#0x1c
	push	#0x01
	push	#0x89
	ld	a, (0x06, sp)
	push	a
	call	_OLED_drawchar
	addw	sp, #4
;	main.c: 145: OLED_drawchar(olednum, FRAME_TOP,1,startcol+16);
	push	#0x24
	push	#0x01
	push	#0x81
	ld	a, (0x06, sp)
	push	a
	call	_OLED_drawchar
	addw	sp, #4
;	main.c: 146: OLED_drawchar(olednum, FRAME_LINE_HOR,1,startcol+24);
	push	#0x2c
	push	#0x01
	push	#0x89
	ld	a, (0x06, sp)
	push	a
	call	_OLED_drawchar
	addw	sp, #4
;	main.c: 147: OLED_drawchar(olednum, FRAME_TOP_RIGHT,1,startcol+32);
	push	#0x34
	push	#0x01
	push	#0x82
	ld	a, (0x06, sp)
	push	a
	call	_OLED_drawchar
	addw	sp, #4
;	main.c: 149: OLED_drawchar(olednum, FRAME_LINE_VER,2,startcol);
	push	#0x14
	push	#0x02
	push	#0x8a
	ld	a, (0x06, sp)
	push	a
	call	_OLED_drawchar
	addw	sp, #4
;	main.c: 150: OLED_drawchar(olednum, FRAME_LINE_VER,2,startcol+16);
	push	#0x24
	push	#0x02
	push	#0x8a
	ld	a, (0x06, sp)
	push	a
	call	_OLED_drawchar
	addw	sp, #4
;	main.c: 151: OLED_drawchar(olednum, FRAME_LINE_VER,2,startcol+32);
	push	#0x34
	push	#0x02
	push	#0x8a
	ld	a, (0x06, sp)
	push	a
	call	_OLED_drawchar
	addw	sp, #4
;	main.c: 153: OLED_drawchar(olednum, FRAME_MID_LEFT,3,startcol);
	push	#0x14
	push	#0x03
	push	#0x83
	ld	a, (0x06, sp)
	push	a
	call	_OLED_drawchar
	addw	sp, #4
;	main.c: 154: OLED_drawchar(olednum, FRAME_LINE_HOR,3,startcol+8);
	push	#0x1c
	push	#0x03
	push	#0x89
	ld	a, (0x06, sp)
	push	a
	call	_OLED_drawchar
	addw	sp, #4
;	main.c: 155: OLED_drawchar(olednum, FRAME_CENTER,3,startcol+16);
	push	#0x24
	push	#0x03
	push	#0x84
	ld	a, (0x06, sp)
	push	a
	call	_OLED_drawchar
	addw	sp, #4
;	main.c: 156: OLED_drawchar(olednum, FRAME_LINE_HOR,3,startcol+24);
	push	#0x2c
	push	#0x03
	push	#0x89
	ld	a, (0x06, sp)
	push	a
	call	_OLED_drawchar
	addw	sp, #4
;	main.c: 157: OLED_drawchar(olednum, FRAME_MID_RIGHT,3,startcol+32);
	push	#0x34
	push	#0x03
	push	#0x85
	ld	a, (0x06, sp)
	push	a
	call	_OLED_drawchar
	addw	sp, #4
;	main.c: 159: OLED_drawchar(olednum, FRAME_LINE_VER,4,startcol);
	push	#0x14
	push	#0x04
	push	#0x8a
	ld	a, (0x06, sp)
	push	a
	call	_OLED_drawchar
	addw	sp, #4
;	main.c: 160: OLED_drawchar(olednum, FRAME_LINE_VER,4,startcol+16);
	push	#0x24
	push	#0x04
	push	#0x8a
	ld	a, (0x06, sp)
	push	a
	call	_OLED_drawchar
	addw	sp, #4
;	main.c: 161: OLED_drawchar(olednum, FRAME_LINE_VER,4,startcol+32);
	push	#0x34
	push	#0x04
	push	#0x8a
	ld	a, (0x06, sp)
	push	a
	call	_OLED_drawchar
	addw	sp, #4
;	main.c: 163: OLED_drawchar(olednum, FRAME_BOT_LEFT,5,startcol);
	push	#0x14
	push	#0x05
	push	#0x86
	ld	a, (0x06, sp)
	push	a
	call	_OLED_drawchar
	addw	sp, #4
;	main.c: 164: OLED_drawchar(olednum, FRAME_LINE_HOR,5,startcol+8);
	push	#0x1c
	push	#0x05
	push	#0x89
	ld	a, (0x06, sp)
	push	a
	call	_OLED_drawchar
	addw	sp, #4
;	main.c: 165: OLED_drawchar(olednum, FRAME_BOT,5,startcol+16);
	push	#0x24
	push	#0x05
	push	#0x87
	ld	a, (0x06, sp)
	push	a
	call	_OLED_drawchar
	addw	sp, #4
;	main.c: 166: OLED_drawchar(olednum, FRAME_LINE_HOR,5,startcol+24);
	push	#0x2c
	push	#0x05
	push	#0x89
	ld	a, (0x06, sp)
	push	a
	call	_OLED_drawchar
	addw	sp, #4
;	main.c: 167: OLED_drawchar(olednum, FRAME_BOT_RIGHT,5,startcol+32);
	push	#0x34
	push	#0x05
	push	#0x88
	ld	a, (0x06, sp)
	push	a
	call	_OLED_drawchar
	addw	sp, #4
;	main.c: 169: OLED_drawtext(olednum, " OLED TEST : SYM",7,0);
	ldw	x, #___str_10+0
	push	#0x00
	push	#0x07
	pushw	x
	ld	a, (0x07, sp)
	push	a
	call	_OLED_drawtext
	addw	sp, #5
	ret
;	main.c: 171: void drawArrow(unsigned char olednum)
;	-----------------------------------------
;	 function drawArrow
;	-----------------------------------------
_drawArrow:
;	main.c: 175: OLED_drawchar(olednum, ARROW_UP_LEFT,1,startcol);
	push	#0x14
	push	#0x01
	push	#0x8f
	ld	a, (0x06, sp)
	push	a
	call	_OLED_drawchar
	addw	sp, #4
;	main.c: 176: OLED_drawchar(olednum, ARROW_UP,1,startcol+16);
	push	#0x24
	push	#0x01
	push	#0x8b
	ld	a, (0x06, sp)
	push	a
	call	_OLED_drawchar
	addw	sp, #4
;	main.c: 177: OLED_drawchar(olednum, ARROW_UP_RIGHT,1,startcol+32);
	push	#0x34
	push	#0x01
	push	#0x90
	ld	a, (0x06, sp)
	push	a
	call	_OLED_drawchar
	addw	sp, #4
;	main.c: 179: OLED_drawchar(olednum, ARROW_LEFT,3,startcol);
	push	#0x14
	push	#0x03
	push	#0x8d
	ld	a, (0x06, sp)
	push	a
	call	_OLED_drawchar
	addw	sp, #4
;	main.c: 180: OLED_drawchar(olednum, ARROW_POINT,3,startcol+16);
	push	#0x24
	push	#0x03
	push	#0x93
	ld	a, (0x06, sp)
	push	a
	call	_OLED_drawchar
	addw	sp, #4
;	main.c: 181: OLED_drawchar(olednum, ARROW_RIGHT,3,startcol+32);
	push	#0x34
	push	#0x03
	push	#0x8e
	ld	a, (0x06, sp)
	push	a
	call	_OLED_drawchar
	addw	sp, #4
;	main.c: 183: OLED_drawchar(olednum, ARROW_DOWN_LEFT,5,startcol);
	push	#0x14
	push	#0x05
	push	#0x91
	ld	a, (0x06, sp)
	push	a
	call	_OLED_drawchar
	addw	sp, #4
;	main.c: 184: OLED_drawchar(olednum, ARROW_DOWN,5,startcol+16);
	push	#0x24
	push	#0x05
	push	#0x8c
	ld	a, (0x06, sp)
	push	a
	call	_OLED_drawchar
	addw	sp, #4
;	main.c: 185: OLED_drawchar(olednum, ARROW_DOWN_RIGHT,5,startcol+32);
	push	#0x34
	push	#0x05
	push	#0x92
	ld	a, (0x06, sp)
	push	a
	call	_OLED_drawchar
	addw	sp, #4
;	main.c: 187: OLED_drawtext(olednum, " OLED TEST : SYM",7,0);
	ldw	x, #___str_10+0
	push	#0x00
	push	#0x07
	pushw	x
	ld	a, (0x07, sp)
	push	a
	call	_OLED_drawtext
	addw	sp, #5
	ret
;	main.c: 190: void drawBytes(unsigned char olednum)
;	-----------------------------------------
;	 function drawBytes
;	-----------------------------------------
_drawBytes:
	sub	sp, #6
;	main.c: 194: OLED_setpos(olednum,3,5);
	push	#0x05
	push	#0x03
	ld	a, (0x0b, sp)
	push	a
	call	_OLED_setpos
	addw	sp, #3
;	main.c: 195: for(Ts=0;Ts<11;Ts++) //Draw pattern 11 times
	ldw	x, #_dsine+0
	ldw	(0x03, sp), x
	clr	(0x02, sp)
;	main.c: 197: for(ds=0;ds<10;ds++)
00115$:
	clr	(0x01, sp)
00105$:
;	main.c: 199: OLED_drawbyte(olednum, dsine[ds]);
	clrw	x
	ld	a, (0x01, sp)
	ld	xl, a
	addw	x, (0x03, sp)
	ld	a, (x)
	push	a
	ld	a, (0x0a, sp)
	push	a
	call	_OLED_drawbyte
	addw	sp, #2
;	main.c: 197: for(ds=0;ds<10;ds++)
	inc	(0x01, sp)
	ld	a, (0x01, sp)
	cp	a, #0x0a
	jrc	00105$
;	main.c: 195: for(Ts=0;Ts<11;Ts++) //Draw pattern 11 times
	inc	(0x02, sp)
	ld	a, (0x02, sp)
	cp	a, #0x0b
	jrc	00115$
;	main.c: 203: OLED_setpos(olednum,5,3);
	push	#0x03
	push	#0x05
	ld	a, (0x0b, sp)
	push	a
	call	_OLED_setpos
	addw	sp, #3
;	main.c: 204: for(Ts=0;Ts<8;Ts++) //Draw pattern 8 times
	ldw	x, #_dtri+0
	ldw	(0x05, sp), x
	clr	(0x02, sp)
;	main.c: 206: for(ds=0;ds<14;ds++)
00119$:
	clr	(0x01, sp)
00109$:
;	main.c: 208: OLED_drawbyte(olednum, dtri[ds]);
	clrw	x
	ld	a, (0x01, sp)
	ld	xl, a
	addw	x, (0x05, sp)
	ld	a, (x)
	push	a
	ld	a, (0x0a, sp)
	push	a
	call	_OLED_drawbyte
	addw	sp, #2
;	main.c: 206: for(ds=0;ds<14;ds++)
	inc	(0x01, sp)
	ld	a, (0x01, sp)
	cp	a, #0x0e
	jrc	00109$
;	main.c: 204: for(Ts=0;Ts<8;Ts++) //Draw pattern 8 times
	inc	(0x02, sp)
	ld	a, (0x02, sp)
	cp	a, #0x08
	jrc	00119$
;	main.c: 212: OLED_drawtext(olednum, "  DRAW PATTERN  ",7,0);
	ldw	x, #___str_11+0
	push	#0x00
	push	#0x07
	pushw	x
	ld	a, (0x0d, sp)
	push	a
	call	_OLED_drawtext
	addw	sp, #11
	ret
;	main.c: 215: void drawLoadingBar(unsigned char olednum)
;	-----------------------------------------
;	 function drawLoadingBar
;	-----------------------------------------
_drawLoadingBar:
	push	a
;	main.c: 219: OLED_setpos(olednum, 4,5);
	push	#0x05
	push	#0x04
	ld	a, (0x06, sp)
	push	a
	call	_OLED_setpos
	addw	sp, #3
;	main.c: 221: for(lb=5;lb<123;lb++)
	ld	a, #0x05
	ld	(0x01, sp), a
00102$:
;	main.c: 223: OLED_drawbyte(olednum, 0xFF);
	push	#0xff
	ld	a, (0x05, sp)
	push	a
	call	_OLED_drawbyte
	addw	sp, #2
;	main.c: 224: delay_ms(10);
	push	#0x0a
	clrw	x
	pushw	x
	push	#0x00
	call	_delay_ms
	addw	sp, #4
;	main.c: 221: for(lb=5;lb<123;lb++)
	inc	(0x01, sp)
	ld	a, (0x01, sp)
	cp	a, #0x7b
	jrc	00102$
;	main.c: 226: delay_ms(1000);
	push	#0xe8
	push	#0x03
	clrw	x
	pushw	x
	call	_delay_ms
	addw	sp, #4
;	main.c: 227: OLED_clearblock(olednum,4,5,122); //Start & finish column = start & finish lb
	push	#0x7a
	push	#0x05
	push	#0x04
	ld	a, (0x07, sp)
	push	a
	call	_OLED_clearblock
	addw	sp, #5
	ret
	.area CODE
_font_arr:
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x5F	; 95
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x05	; 5
	.db #0x03	; 3
	.db #0x00	; 0
	.db #0x05	; 5
	.db #0x03	; 3
	.db #0x14	; 20
	.db #0x7F	; 127
	.db #0x14	; 20
	.db #0x7F	; 127
	.db #0x14	; 20
	.db #0x24	; 36
	.db #0x2A	; 42
	.db #0x7F	; 127
	.db #0x2A	; 42
	.db #0x12	; 18
	.db #0x23	; 35
	.db #0x13	; 19
	.db #0x08	; 8
	.db #0x64	; 100	'd'
	.db #0x62	; 98	'b'
	.db #0x36	; 54	'6'
	.db #0x49	; 73	'I'
	.db #0x55	; 85	'U'
	.db #0x22	; 34
	.db #0x50	; 80	'P'
	.db #0x00	; 0
	.db #0x05	; 5
	.db #0x03	; 3
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x1C	; 28
	.db #0x22	; 34
	.db #0x41	; 65	'A'
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x41	; 65	'A'
	.db #0x22	; 34
	.db #0x1C	; 28
	.db #0x00	; 0
	.db #0x0A	; 10
	.db #0x04	; 4
	.db #0x1F	; 31
	.db #0x04	; 4
	.db #0x0A	; 10
	.db #0x08	; 8
	.db #0x08	; 8
	.db #0x3E	; 62
	.db #0x08	; 8
	.db #0x08	; 8
	.db #0x00	; 0
	.db #0x50	; 80	'P'
	.db #0x30	; 48	'0'
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x08	; 8
	.db #0x08	; 8
	.db #0x08	; 8
	.db #0x08	; 8
	.db #0x08	; 8
	.db #0x00	; 0
	.db #0x60	; 96
	.db #0x60	; 96
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x20	; 32
	.db #0x10	; 16
	.db #0x08	; 8
	.db #0x04	; 4
	.db #0x02	; 2
	.db #0x3E	; 62
	.db #0x51	; 81	'Q'
	.db #0x49	; 73	'I'
	.db #0x45	; 69	'E'
	.db #0x3E	; 62
	.db #0x00	; 0
	.db #0x42	; 66	'B'
	.db #0x7F	; 127
	.db #0x40	; 64
	.db #0x00	; 0
	.db #0x42	; 66	'B'
	.db #0x61	; 97	'a'
	.db #0x51	; 81	'Q'
	.db #0x49	; 73	'I'
	.db #0x46	; 70	'F'
	.db #0x22	; 34
	.db #0x41	; 65	'A'
	.db #0x49	; 73	'I'
	.db #0x49	; 73	'I'
	.db #0x36	; 54	'6'
	.db #0x18	; 24
	.db #0x14	; 20
	.db #0x12	; 18
	.db #0x7F	; 127
	.db #0x10	; 16
	.db #0x27	; 39
	.db #0x45	; 69	'E'
	.db #0x45	; 69	'E'
	.db #0x45	; 69	'E'
	.db #0x39	; 57	'9'
	.db #0x3E	; 62
	.db #0x49	; 73	'I'
	.db #0x49	; 73	'I'
	.db #0x49	; 73	'I'
	.db #0x32	; 50	'2'
	.db #0x61	; 97	'a'
	.db #0x11	; 17
	.db #0x09	; 9
	.db #0x05	; 5
	.db #0x03	; 3
	.db #0x36	; 54	'6'
	.db #0x49	; 73	'I'
	.db #0x49	; 73	'I'
	.db #0x49	; 73	'I'
	.db #0x36	; 54	'6'
	.db #0x26	; 38
	.db #0x49	; 73	'I'
	.db #0x49	; 73	'I'
	.db #0x49	; 73	'I'
	.db #0x3E	; 62
	.db #0x00	; 0
	.db #0x36	; 54	'6'
	.db #0x36	; 54	'6'
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x56	; 86	'V'
	.db #0x36	; 54	'6'
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x08	; 8
	.db #0x14	; 20
	.db #0x22	; 34
	.db #0x00	; 0
	.db #0x14	; 20
	.db #0x14	; 20
	.db #0x14	; 20
	.db #0x14	; 20
	.db #0x14	; 20
	.db #0x00	; 0
	.db #0x22	; 34
	.db #0x14	; 20
	.db #0x08	; 8
	.db #0x00	; 0
	.db #0x02	; 2
	.db #0x01	; 1
	.db #0x51	; 81	'Q'
	.db #0x09	; 9
	.db #0x06	; 6
	.db #0x32	; 50	'2'
	.db #0x49	; 73	'I'
	.db #0x79	; 121	'y'
	.db #0x41	; 65	'A'
	.db #0x3E	; 62
	.db #0x7C	; 124
	.db #0x12	; 18
	.db #0x11	; 17
	.db #0x12	; 18
	.db #0x7C	; 124
	.db #0x7F	; 127
	.db #0x49	; 73	'I'
	.db #0x49	; 73	'I'
	.db #0x49	; 73	'I'
	.db #0x36	; 54	'6'
	.db #0x3E	; 62
	.db #0x41	; 65	'A'
	.db #0x41	; 65	'A'
	.db #0x41	; 65	'A'
	.db #0x22	; 34
	.db #0x7F	; 127
	.db #0x41	; 65	'A'
	.db #0x41	; 65	'A'
	.db #0x22	; 34
	.db #0x1C	; 28
	.db #0x7F	; 127
	.db #0x49	; 73	'I'
	.db #0x49	; 73	'I'
	.db #0x49	; 73	'I'
	.db #0x49	; 73	'I'
	.db #0x7F	; 127
	.db #0x09	; 9
	.db #0x09	; 9
	.db #0x09	; 9
	.db #0x09	; 9
	.db #0x3E	; 62
	.db #0x41	; 65	'A'
	.db #0x49	; 73	'I'
	.db #0x49	; 73	'I'
	.db #0x3A	; 58
	.db #0x7F	; 127
	.db #0x08	; 8
	.db #0x08	; 8
	.db #0x08	; 8
	.db #0x7F	; 127
	.db #0x00	; 0
	.db #0x41	; 65	'A'
	.db #0x7F	; 127
	.db #0x41	; 65	'A'
	.db #0x00	; 0
	.db #0x20	; 32
	.db #0x40	; 64
	.db #0x41	; 65	'A'
	.db #0x3F	; 63
	.db #0x01	; 1
	.db #0x7F	; 127
	.db #0x08	; 8
	.db #0x14	; 20
	.db #0x22	; 34
	.db #0x41	; 65	'A'
	.db #0x7F	; 127
	.db #0x40	; 64
	.db #0x40	; 64
	.db #0x40	; 64
	.db #0x40	; 64
	.db #0x7F	; 127
	.db #0x02	; 2
	.db #0x0C	; 12
	.db #0x02	; 2
	.db #0x7F	; 127
	.db #0x7F	; 127
	.db #0x04	; 4
	.db #0x08	; 8
	.db #0x10	; 16
	.db #0x7F	; 127
	.db #0x3E	; 62
	.db #0x41	; 65	'A'
	.db #0x41	; 65	'A'
	.db #0x41	; 65	'A'
	.db #0x3E	; 62
	.db #0x7F	; 127
	.db #0x09	; 9
	.db #0x09	; 9
	.db #0x09	; 9
	.db #0x06	; 6
	.db #0x3E	; 62
	.db #0x41	; 65	'A'
	.db #0x51	; 81	'Q'
	.db #0x21	; 33
	.db #0x5E	; 94
	.db #0x7F	; 127
	.db #0x09	; 9
	.db #0x19	; 25
	.db #0x29	; 41
	.db #0x46	; 70	'F'
	.db #0x26	; 38
	.db #0x49	; 73	'I'
	.db #0x49	; 73	'I'
	.db #0x49	; 73	'I'
	.db #0x32	; 50	'2'
	.db #0x01	; 1
	.db #0x01	; 1
	.db #0x7F	; 127
	.db #0x01	; 1
	.db #0x01	; 1
	.db #0x3F	; 63
	.db #0x40	; 64
	.db #0x40	; 64
	.db #0x40	; 64
	.db #0x3F	; 63
	.db #0x1F	; 31
	.db #0x20	; 32
	.db #0x40	; 64
	.db #0x20	; 32
	.db #0x1F	; 31
	.db #0x3F	; 63
	.db #0x40	; 64
	.db #0x38	; 56	'8'
	.db #0x40	; 64
	.db #0x3F	; 63
	.db #0x63	; 99	'c'
	.db #0x14	; 20
	.db #0x08	; 8
	.db #0x14	; 20
	.db #0x63	; 99	'c'
	.db #0x07	; 7
	.db #0x08	; 8
	.db #0x70	; 112	'p'
	.db #0x08	; 8
	.db #0x07	; 7
	.db #0x61	; 97	'a'
	.db #0x51	; 81	'Q'
	.db #0x49	; 73	'I'
	.db #0x45	; 69	'E'
	.db #0x43	; 67	'C'
	.db #0x00	; 0
	.db #0x7F	; 127
	.db #0x41	; 65	'A'
	.db #0x41	; 65	'A'
	.db #0x00	; 0
	.db #0x02	; 2
	.db #0x04	; 4
	.db #0x08	; 8
	.db #0x10	; 16
	.db #0x20	; 32
	.db #0x00	; 0
	.db #0x41	; 65	'A'
	.db #0x41	; 65	'A'
	.db #0x7F	; 127
	.db #0x00	; 0
	.db #0x04	; 4
	.db #0x02	; 2
	.db #0x01	; 1
	.db #0x02	; 2
	.db #0x04	; 4
	.db #0x40	; 64
	.db #0x40	; 64
	.db #0x40	; 64
	.db #0x40	; 64
	.db #0x40	; 64
	.db #0x00	; 0
	.db #0x01	; 1
	.db #0x02	; 2
	.db #0x04	; 4
	.db #0x00	; 0
	.db #0x20	; 32
	.db #0x54	; 84	'T'
	.db #0x54	; 84	'T'
	.db #0x54	; 84	'T'
	.db #0x78	; 120	'x'
	.db #0x7F	; 127
	.db #0x50	; 80	'P'
	.db #0x48	; 72	'H'
	.db #0x48	; 72	'H'
	.db #0x30	; 48	'0'
	.db #0x38	; 56	'8'
	.db #0x44	; 68	'D'
	.db #0x44	; 68	'D'
	.db #0x44	; 68	'D'
	.db #0x28	; 40
	.db #0x30	; 48	'0'
	.db #0x48	; 72	'H'
	.db #0x48	; 72	'H'
	.db #0x50	; 80	'P'
	.db #0x7F	; 127
	.db #0x38	; 56	'8'
	.db #0x54	; 84	'T'
	.db #0x54	; 84	'T'
	.db #0x54	; 84	'T'
	.db #0x18	; 24
	.db #0x08	; 8
	.db #0x7E	; 126
	.db #0x09	; 9
	.db #0x09	; 9
	.db #0x02	; 2
	.db #0x08	; 8
	.db #0x54	; 84	'T'
	.db #0x54	; 84	'T'
	.db #0x54	; 84	'T'
	.db #0x3C	; 60
	.db #0x7F	; 127
	.db #0x10	; 16
	.db #0x08	; 8
	.db #0x08	; 8
	.db #0x70	; 112	'p'
	.db #0x00	; 0
	.db #0x48	; 72	'H'
	.db #0x7A	; 122	'z'
	.db #0x40	; 64
	.db #0x00	; 0
	.db #0x20	; 32
	.db #0x40	; 64
	.db #0x48	; 72	'H'
	.db #0x3A	; 58
	.db #0x00	; 0
	.db #0x7F	; 127
	.db #0x10	; 16
	.db #0x28	; 40
	.db #0x44	; 68	'D'
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x41	; 65	'A'
	.db #0x7F	; 127
	.db #0x40	; 64
	.db #0x00	; 0
	.db #0x7C	; 124
	.db #0x04	; 4
	.db #0x7C	; 124
	.db #0x04	; 4
	.db #0x78	; 120	'x'
	.db #0x7C	; 124
	.db #0x08	; 8
	.db #0x04	; 4
	.db #0x04	; 4
	.db #0x78	; 120	'x'
	.db #0x38	; 56	'8'
	.db #0x44	; 68	'D'
	.db #0x44	; 68	'D'
	.db #0x44	; 68	'D'
	.db #0x38	; 56	'8'
	.db #0x7C	; 124
	.db #0x14	; 20
	.db #0x14	; 20
	.db #0x14	; 20
	.db #0x08	; 8
	.db #0x08	; 8
	.db #0x14	; 20
	.db #0x14	; 20
	.db #0x18	; 24
	.db #0x7C	; 124
	.db #0x7C	; 124
	.db #0x08	; 8
	.db #0x04	; 4
	.db #0x04	; 4
	.db #0x08	; 8
	.db #0x48	; 72	'H'
	.db #0x54	; 84	'T'
	.db #0x54	; 84	'T'
	.db #0x54	; 84	'T'
	.db #0x20	; 32
	.db #0x04	; 4
	.db #0x3F	; 63
	.db #0x44	; 68	'D'
	.db #0x44	; 68	'D'
	.db #0x20	; 32
	.db #0x3C	; 60
	.db #0x40	; 64
	.db #0x40	; 64
	.db #0x20	; 32
	.db #0x7C	; 124
	.db #0x1C	; 28
	.db #0x20	; 32
	.db #0x40	; 64
	.db #0x20	; 32
	.db #0x1C	; 28
	.db #0x3C	; 60
	.db #0x40	; 64
	.db #0x38	; 56	'8'
	.db #0x40	; 64
	.db #0x3C	; 60
	.db #0x44	; 68	'D'
	.db #0x28	; 40
	.db #0x10	; 16
	.db #0x28	; 40
	.db #0x44	; 68	'D'
	.db #0x0C	; 12
	.db #0x50	; 80	'P'
	.db #0x50	; 80	'P'
	.db #0x50	; 80	'P'
	.db #0x3C	; 60
	.db #0x44	; 68	'D'
	.db #0x64	; 100	'd'
	.db #0x54	; 84	'T'
	.db #0x4C	; 76	'L'
	.db #0x44	; 68	'D'
	.db #0x00	; 0
	.db #0x08	; 8
	.db #0x36	; 54	'6'
	.db #0x41	; 65	'A'
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x7F	; 127
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x41	; 65	'A'
	.db #0x36	; 54	'6'
	.db #0x08	; 8
	.db #0x00	; 0
	.db #0x10	; 16
	.db #0x08	; 8
	.db #0x08	; 8
	.db #0x10	; 16
	.db #0x08	; 8
	.db #0x06	; 6
	.db #0x09	; 9
	.db #0x09	; 9
	.db #0x06	; 6
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0xF8	; 248
	.db #0xF8	; 248
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0xF8	; 248
	.db #0xF8	; 248
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0xF8	; 248
	.db #0xF8	; 248
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0xFF	; 255
	.db #0xFF	; 255
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0xFF	; 255
	.db #0xFF	; 255
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0xFF	; 255
	.db #0xFF	; 255
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x1F	; 31
	.db #0x1F	; 31
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x1F	; 31
	.db #0x1F	; 31
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x1F	; 31
	.db #0x1F	; 31
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0xFF	; 255
	.db #0xFF	; 255
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x18	; 24
	.db #0x0C	; 12
	.db #0x06	; 6
	.db #0xFF	; 255
	.db #0xFF	; 255
	.db #0x06	; 6
	.db #0x0C	; 12
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x30	; 48	'0'
	.db #0x60	; 96
	.db #0xFF	; 255
	.db #0xFF	; 255
	.db #0x60	; 96
	.db #0x30	; 48	'0'
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x3C	; 60
	.db #0x7E	; 126
	.db #0xDB	; 219
	.db #0x99	; 153
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x99	; 153
	.db #0xDB	; 219
	.db #0x7E	; 126
	.db #0x3C	; 60
	.db #0x18	; 24
	.db #0x7F	; 127
	.db #0x7F	; 127
	.db #0x0F	; 15
	.db #0x1F	; 31
	.db #0x3B	; 59
	.db #0x73	; 115	's'
	.db #0xE3	; 227
	.db #0x40	; 64
	.db #0x40	; 64
	.db #0xE3	; 227
	.db #0x73	; 115	's'
	.db #0x3B	; 59
	.db #0x1F	; 31
	.db #0x0F	; 15
	.db #0x7F	; 127
	.db #0x7F	; 127
	.db #0xFE	; 254
	.db #0xFE	; 254
	.db #0xF0	; 240
	.db #0xF8	; 248
	.db #0xDC	; 220
	.db #0xCE	; 206
	.db #0xC7	; 199
	.db #0x02	; 2
	.db #0x02	; 2
	.db #0xC7	; 199
	.db #0xCE	; 206
	.db #0xDC	; 220
	.db #0xF8	; 248
	.db #0xF0	; 240
	.db #0xFE	; 254
	.db #0xFE	; 254
	.db #0x3C	; 60
	.db #0x42	; 66	'B'
	.db #0x81	; 129
	.db #0x99	; 153
	.db #0x99	; 153
	.db #0x81	; 129
	.db #0x42	; 66	'B'
	.db #0x3C	; 60
___str_0:
	.ascii " OLED TEST : INT"
	.db 0x00
___str_1:
	.ascii "ABCDEFGHIJKLM"
	.db 0x00
___str_2:
	.ascii "NOPQRSTUVWXYZ"
	.db 0x00
___str_3:
	.ascii "abcdefghijklm"
	.db 0x00
___str_4:
	.ascii "nopqrstuvwxyz"
	.db 0x00
___str_5:
	.ascii "0123456789"
	.db 0x00
___str_6:
	.ascii "OLED TEST : CHAR"
	.db 0x00
___str_7:
	.ascii "<{([+_-=])}>"
	.db 0x00
___str_8:
	.ascii "!@#$%^&*`|~?"
	.db 0x00
___str_9:
	.ascii ".,"
	.db 0x22
	.ascii "'"
	.db 0x5C
	.ascii "/ :;"
	.db 0x00
___str_10:
	.ascii " OLED TEST : SYM"
	.db 0x00
___str_11:
	.ascii "  DRAW PATTERN  "
	.db 0x00
	.area INITIALIZER
__xinit__dsine:
	.db #0x18	; 24
	.db #0x06	; 6
	.db #0x01	; 1
	.db #0x01	; 1
	.db #0x06	; 6
	.db #0x18	; 24
	.db #0x60	; 96
	.db #0x80	; 128
	.db #0x80	; 128
	.db #0x60	; 96
__xinit__dtri:
	.db #0x08	; 8
	.db #0x04	; 4
	.db #0x02	; 2
	.db #0x01	; 1
	.db #0x02	; 2
	.db #0x04	; 4
	.db #0x08	; 8
	.db #0x10	; 16
	.db #0x20	; 32
	.db #0x40	; 64
	.db #0x80	; 128
	.db #0x40	; 64
	.db #0x20	; 32
	.db #0x10	; 16
	.area CABS (ABS)
