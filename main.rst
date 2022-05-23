                                      1 ;--------------------------------------------------------
                                      2 ; File Created by SDCC : free open source ANSI-C Compiler
                                      3 ; Version 3.5.0 #9253 (Apr  3 2018) (Linux)
                                      4 ; This file was generated Mon May 23 21:01:18 2022
                                      5 ;--------------------------------------------------------
                                      6 	.module main
                                      7 	.optsdcc -mstm8
                                      8 	
                                      9 ;--------------------------------------------------------
                                     10 ; Public variables in this module
                                     11 ;--------------------------------------------------------
                                     12 	.globl _font_arr
                                     13 	.globl _main
                                     14 	.globl _dtri
                                     15 	.globl _dsine
                                     16 	.globl _readreg
                                     17 	.globl _delay_init
                                     18 	.globl _delay_us
                                     19 	.globl _delay_ms
                                     20 	.globl _delay_timer
                                     21 	.globl _clock_init
                                     22 	.globl _i2c_init
                                     23 	.globl _i2c_set_start
                                     24 	.globl _i2c_set_address
                                     25 	.globl _i2c_set_stop
                                     26 	.globl _i2c_clear_ack
                                     27 	.globl _i2c_set_ack
                                     28 	.globl _i2c_ack_pos_current
                                     29 	.globl _i2c_ack_pos_next
                                     30 	.globl _i2c_poll_SB
                                     31 	.globl _i2c_poll_ADDR
                                     32 	.globl _i2c_poll_BTF
                                     33 	.globl _i2c_poll_TXE
                                     34 	.globl _i2c_poll_RXNE
                                     35 	.globl _i2c_clear_bits
                                     36 	.globl _i2c_clear_ADDR
                                     37 	.globl _i2c_enable_interrupts
                                     38 	.globl _i2c_disable_interrupts
                                     39 	.globl _i2c_write_1byte
                                     40 	.globl _i2c_write_2byte
                                     41 	.globl _adc_init
                                     42 	.globl _read_adc
                                     43 	.globl _uart1_init
                                     44 	.globl _uart1_send
                                     45 	.globl _uart1_recv
                                     46 	.globl _uart1_recv_i
                                     47 	.globl _ssd1306_init
                                     48 	.globl _ssd1306_sendcom
                                     49 	.globl _ssd1306_senddat
                                     50 	.globl _ssd1306_setpos
                                     51 	.globl _ssd1306_clear
                                     52 	.globl _OLED_setpos
                                     53 	.globl _OLED_drawbyte
                                     54 	.globl _OLED_drawchar
                                     55 	.globl _OLED_drawtext
                                     56 	.globl _OLED_drawint
                                     57 	.globl _OLED_clear
                                     58 	.globl _OLED_clearblock
                                     59 	.globl _OLED_normal
                                     60 	.globl _OLED_reverse
                                     61 	.globl _loop
                                     62 	.globl _gpio_init
                                     63 	.globl _drawInt
                                     64 	.globl _drawAlphanum
                                     65 	.globl _drawPunct
                                     66 	.globl _drawFrame
                                     67 	.globl _drawArrow
                                     68 	.globl _drawBytes
                                     69 	.globl _drawLoadingBar
                                     70 ;--------------------------------------------------------
                                     71 ; ram data
                                     72 ;--------------------------------------------------------
                                     73 	.area DATA
      000001                         74 _readreg::
      000001                         75 	.ds 1
                                     76 ;--------------------------------------------------------
                                     77 ; ram data
                                     78 ;--------------------------------------------------------
                                     79 	.area INITIALIZED
      000002                         80 _dsine::
      000002                         81 	.ds 10
      00000C                         82 _dtri::
      00000C                         83 	.ds 14
                                     84 ;--------------------------------------------------------
                                     85 ; Stack segment in internal ram 
                                     86 ;--------------------------------------------------------
                                     87 	.area	SSEG
      00001A                         88 __start__stack:
      00001A                         89 	.ds	1
                                     90 
                                     91 ;--------------------------------------------------------
                                     92 ; absolute external ram data
                                     93 ;--------------------------------------------------------
                                     94 	.area DABS (ABS)
                                     95 ;--------------------------------------------------------
                                     96 ; interrupt vector 
                                     97 ;--------------------------------------------------------
                                     98 	.area HOME
      008000                         99 __interrupt_vect:
      008000 82 00 80 83            100 	int s_GSINIT ;reset
      008004 82 00 00 00            101 	int 0x0000 ;trap
      008008 82 00 00 00            102 	int 0x0000 ;int0
      00800C 82 00 00 00            103 	int 0x0000 ;int1
      008010 82 00 00 00            104 	int 0x0000 ;int2
      008014 82 00 00 00            105 	int 0x0000 ;int3
      008018 82 00 00 00            106 	int 0x0000 ;int4
      00801C 82 00 00 00            107 	int 0x0000 ;int5
      008020 82 00 00 00            108 	int 0x0000 ;int6
      008024 82 00 00 00            109 	int 0x0000 ;int7
      008028 82 00 00 00            110 	int 0x0000 ;int8
      00802C 82 00 00 00            111 	int 0x0000 ;int9
      008030 82 00 00 00            112 	int 0x0000 ;int10
      008034 82 00 00 00            113 	int 0x0000 ;int11
      008038 82 00 00 00            114 	int 0x0000 ;int12
      00803C 82 00 00 00            115 	int 0x0000 ;int13
      008040 82 00 00 00            116 	int 0x0000 ;int14
      008044 82 00 00 00            117 	int 0x0000 ;int15
      008048 82 00 00 00            118 	int 0x0000 ;int16
      00804C 82 00 00 00            119 	int 0x0000 ;int17
      008050 82 00 00 00            120 	int 0x0000 ;int18
      008054 82 00 00 00            121 	int 0x0000 ;int19
      008058 82 00 00 00            122 	int 0x0000 ;int20
      00805C 82 00 00 00            123 	int 0x0000 ;int21
      008060 82 00 00 00            124 	int 0x0000 ;int22
      008064 82 00 00 00            125 	int 0x0000 ;int23
      008068 82 00 00 00            126 	int 0x0000 ;int24
      00806C 82 00 00 00            127 	int 0x0000 ;int25
      008070 82 00 00 00            128 	int 0x0000 ;int26
      008074 82 00 00 00            129 	int 0x0000 ;int27
      008078 82 00 00 00            130 	int 0x0000 ;int28
      00807C 82 00 00 00            131 	int 0x0000 ;int29
                                    132 ;--------------------------------------------------------
                                    133 ; global & static initialisations
                                    134 ;--------------------------------------------------------
                                    135 	.area HOME
                                    136 	.area GSINIT
                                    137 	.area GSFINAL
                                    138 	.area GSINIT
      008083                        139 __sdcc_gs_init_startup:
      008083                        140 __sdcc_init_data:
                                    141 ; stm8_genXINIT() start
      008083 AE 00 01         [ 2]  142 	ldw x, #l_DATA
      008086 27 07            [ 1]  143 	jreq	00002$
      008088                        144 00001$:
      008088 72 4F 00 00      [ 1]  145 	clr (s_DATA - 1, x)
      00808C 5A               [ 2]  146 	decw x
      00808D 26 F9            [ 1]  147 	jrne	00001$
      00808F                        148 00002$:
      00808F AE 00 18         [ 2]  149 	ldw	x, #l_INITIALIZER
      008092 27 09            [ 1]  150 	jreq	00004$
      008094                        151 00003$:
      008094 D6 8F DD         [ 1]  152 	ld	a, (s_INITIALIZER - 1, x)
      008097 D7 00 01         [ 1]  153 	ld	(s_INITIALIZED - 1, x), a
      00809A 5A               [ 2]  154 	decw	x
      00809B 26 F7            [ 1]  155 	jrne	00003$
      00809D                        156 00004$:
                                    157 ; stm8_genXINIT() end
                                    158 	.area GSFINAL
      00809D CC 80 80         [ 2]  159 	jp	__sdcc_program_startup
                                    160 ;--------------------------------------------------------
                                    161 ; Home
                                    162 ;--------------------------------------------------------
                                    163 	.area HOME
                                    164 	.area HOME
      008080                        165 __sdcc_program_startup:
      008080 CC 86 44         [ 2]  166 	jp	_main
                                    167 ;	return from main will return to caller
                                    168 ;--------------------------------------------------------
                                    169 ; code
                                    170 ;--------------------------------------------------------
                                    171 	.area CODE
                                    172 ;	delay.c: 7: void delay_init()
                                    173 ;	-----------------------------------------
                                    174 ;	 function delay_init
                                    175 ;	-----------------------------------------
      0080A0                        176 _delay_init:
                                    177 ;	delay.c: 9: TIM4_PSCR = 4; // CLK/16
      0080A0 35 04 53 47      [ 1]  178 	mov	0x5347+0, #0x04
      0080A4 81               [ 4]  179 	ret
                                    180 ;	delay.c: 12: void delay_us(unsigned long delus)
                                    181 ;	-----------------------------------------
                                    182 ;	 function delay_us
                                    183 ;	-----------------------------------------
      0080A5                        184 _delay_us:
      0080A5 52 06            [ 2]  185 	sub	sp, #6
                                    186 ;	delay.c: 16: for(du=0;du<(delus/10);du++)
      0080A7 4B 0A            [ 1]  187 	push	#0x0a
      0080A9 5F               [ 1]  188 	clrw	x
      0080AA 89               [ 2]  189 	pushw	x
      0080AB 4B 00            [ 1]  190 	push	#0x00
      0080AD 1E 0F            [ 2]  191 	ldw	x, (0x0f, sp)
      0080AF 89               [ 2]  192 	pushw	x
      0080B0 1E 0F            [ 2]  193 	ldw	x, (0x0f, sp)
      0080B2 89               [ 2]  194 	pushw	x
      0080B3 CD 8F 08         [ 4]  195 	call	__divulong
      0080B6 5B 08            [ 2]  196 	addw	sp, #8
      0080B8 1F 05            [ 2]  197 	ldw	(0x05, sp), x
      0080BA 17 03            [ 2]  198 	ldw	(0x03, sp), y
      0080BC 5F               [ 1]  199 	clrw	x
      0080BD 1F 01            [ 2]  200 	ldw	(0x01, sp), x
      0080BF                        201 00103$:
      0080BF 1E 01            [ 2]  202 	ldw	x, (0x01, sp)
      0080C1 90 5F            [ 1]  203 	clrw	y
      0080C3 13 05            [ 2]  204 	cpw	x, (0x05, sp)
      0080C5 90 9F            [ 1]  205 	ld	a, yl
      0080C7 12 04            [ 1]  206 	sbc	a, (0x04, sp)
      0080C9 90 9E            [ 1]  207 	ld	a, yh
      0080CB 12 03            [ 1]  208 	sbc	a, (0x03, sp)
      0080CD 24 0D            [ 1]  209 	jrnc	00101$
                                    210 ;	delay.c: 18: delay_timer(100);
      0080CF 4B 64            [ 1]  211 	push	#0x64
      0080D1 CD 81 3A         [ 4]  212 	call	_delay_timer
      0080D4 84               [ 1]  213 	pop	a
                                    214 ;	delay.c: 16: for(du=0;du<(delus/10);du++)
      0080D5 1E 01            [ 2]  215 	ldw	x, (0x01, sp)
      0080D7 5C               [ 2]  216 	incw	x
      0080D8 1F 01            [ 2]  217 	ldw	(0x01, sp), x
      0080DA 20 E3            [ 2]  218 	jra	00103$
      0080DC                        219 00101$:
                                    220 ;	delay.c: 20: delay_timer(delus%10);
      0080DC 4B 0A            [ 1]  221 	push	#0x0a
      0080DE 5F               [ 1]  222 	clrw	x
      0080DF 89               [ 2]  223 	pushw	x
      0080E0 4B 00            [ 1]  224 	push	#0x00
      0080E2 1E 0F            [ 2]  225 	ldw	x, (0x0f, sp)
      0080E4 89               [ 2]  226 	pushw	x
      0080E5 1E 0F            [ 2]  227 	ldw	x, (0x0f, sp)
      0080E7 89               [ 2]  228 	pushw	x
      0080E8 CD 8E 98         [ 4]  229 	call	__modulong
      0080EB 5B 08            [ 2]  230 	addw	sp, #8
      0080ED 9F               [ 1]  231 	ld	a, xl
      0080EE 88               [ 1]  232 	push	a
      0080EF CD 81 3A         [ 4]  233 	call	_delay_timer
      0080F2 5B 07            [ 2]  234 	addw	sp, #7
      0080F4 81               [ 4]  235 	ret
                                    236 ;	delay.c: 23: void delay_ms(unsigned long delms)
                                    237 ;	-----------------------------------------
                                    238 ;	 function delay_ms
                                    239 ;	-----------------------------------------
      0080F5                        240 _delay_ms:
      0080F5 52 08            [ 2]  241 	sub	sp, #8
                                    242 ;	delay.c: 27: for(dm=0;dm<(delms*100);dm++)
      0080F7 1E 0D            [ 2]  243 	ldw	x, (0x0d, sp)
      0080F9 89               [ 2]  244 	pushw	x
      0080FA 1E 0D            [ 2]  245 	ldw	x, (0x0d, sp)
      0080FC 89               [ 2]  246 	pushw	x
      0080FD 4B 64            [ 1]  247 	push	#0x64
      0080FF 5F               [ 1]  248 	clrw	x
      008100 89               [ 2]  249 	pushw	x
      008101 4B 00            [ 1]  250 	push	#0x00
      008103 CD 8F 62         [ 4]  251 	call	__mullong
      008106 5B 08            [ 2]  252 	addw	sp, #8
      008108 1F 07            [ 2]  253 	ldw	(0x07, sp), x
      00810A 17 05            [ 2]  254 	ldw	(0x05, sp), y
      00810C 5F               [ 1]  255 	clrw	x
      00810D 4F               [ 1]  256 	clr	a
      00810E 0F 01            [ 1]  257 	clr	(0x01, sp)
      008110                        258 00103$:
      008110 88               [ 1]  259 	push	a
      008111 13 08            [ 2]  260 	cpw	x, (0x08, sp)
      008113 7B 01            [ 1]  261 	ld	a, (1, sp)
      008115 12 07            [ 1]  262 	sbc	a, (0x07, sp)
      008117 7B 02            [ 1]  263 	ld	a, (0x02, sp)
      008119 12 06            [ 1]  264 	sbc	a, (0x06, sp)
      00811B 84               [ 1]  265 	pop	a
      00811C 24 19            [ 1]  266 	jrnc	00105$
                                    267 ;	delay.c: 29: delay_timer(100);
      00811E 88               [ 1]  268 	push	a
      00811F 89               [ 2]  269 	pushw	x
      008120 4B 64            [ 1]  270 	push	#0x64
      008122 CD 81 3A         [ 4]  271 	call	_delay_timer
      008125 84               [ 1]  272 	pop	a
      008126 85               [ 2]  273 	popw	x
      008127 84               [ 1]  274 	pop	a
                                    275 ;	delay.c: 27: for(dm=0;dm<(delms*100);dm++)
      008128 1C 00 01         [ 2]  276 	addw	x, #0x0001
      00812B A9 00            [ 1]  277 	adc	a, #0x00
      00812D 88               [ 1]  278 	push	a
      00812E 7B 02            [ 1]  279 	ld	a, (0x02, sp)
      008130 A9 00            [ 1]  280 	adc	a, #0x00
      008132 6B 02            [ 1]  281 	ld	(0x02, sp), a
      008134 84               [ 1]  282 	pop	a
      008135 20 D9            [ 2]  283 	jra	00103$
      008137                        284 00105$:
      008137 5B 08            [ 2]  285 	addw	sp, #8
      008139 81               [ 4]  286 	ret
                                    287 ;	delay.c: 33: void delay_timer(unsigned char deltim)
                                    288 ;	-----------------------------------------
                                    289 ;	 function delay_timer
                                    290 ;	-----------------------------------------
      00813A                        291 _delay_timer:
                                    292 ;	delay.c: 35: TIM4_CR1 = (1<<TIM4_CR1_CEN);
      00813A 35 01 53 40      [ 1]  293 	mov	0x5340+0, #0x01
                                    294 ;	delay.c: 36: while(TIM4_CNTR<deltim);
      00813E                        295 00101$:
      00813E AE 53 46         [ 2]  296 	ldw	x, #0x5346
      008141 F6               [ 1]  297 	ld	a, (x)
      008142 11 03            [ 1]  298 	cp	a, (0x03, sp)
      008144 25 F8            [ 1]  299 	jrc	00101$
                                    300 ;	delay.c: 37: TIM4_CR1 = (0<<TIM4_CR1_CEN);
      008146 35 00 53 40      [ 1]  301 	mov	0x5340+0, #0x00
                                    302 ;	delay.c: 38: TIM4_CNTR = 0; //reset timer	
      00814A 35 00 53 46      [ 1]  303 	mov	0x5346+0, #0x00
      00814E 81               [ 4]  304 	ret
                                    305 ;	periph_stm8s.c: 16: void clock_init()
                                    306 ;	-----------------------------------------
                                    307 ;	 function clock_init
                                    308 ;	-----------------------------------------
      00814F                        309 _clock_init:
                                    310 ;	periph_stm8s.c: 18: CLK_CKDIVR = 0x00; //fMASTER = fCPU = fHSI = 16MHz
      00814F 35 00 50 C6      [ 1]  311 	mov	0x50c6+0, #0x00
                                    312 ;	periph_stm8s.c: 19: CLK_ICKR = (1<<CLK_ICKR_HSIEN);	//High Speed Internal RC Oscillator enabled
      008153 35 01 50 C0      [ 1]  313 	mov	0x50c0+0, #0x01
      008157 81               [ 4]  314 	ret
                                    315 ;	periph_stm8s.c: 24: void i2c_init()
                                    316 ;	-----------------------------------------
                                    317 ;	 function i2c_init
                                    318 ;	-----------------------------------------
      008158                        319 _i2c_init:
                                    320 ;	periph_stm8s.c: 26: I2C_CR1 = (0<<I2C_CR1_PE); //Disable I2C before configuration starts
      008158 35 00 52 10      [ 1]  321 	mov	0x5210+0, #0x00
                                    322 ;	periph_stm8s.c: 27: I2C_FREQR = 16;	//fCLK = 16 MHz
      00815C 35 10 52 12      [ 1]  323 	mov	0x5212+0, #0x10
                                    324 ;	periph_stm8s.c: 28: I2C_CCRH = (0<<I2C_CCRH_FS)|(0<<I2C_CCRH_CCR11)|(0<<I2C_CCRH_CCR10)|(0<<I2C_CCRH_CCR9)|(0<<I2C_CCRH_CCR8); //Standard Mode
      008160 35 00 52 1C      [ 1]  325 	mov	0x521c+0, #0x00
                                    326 ;	periph_stm8s.c: 29: I2C_CCRL = 0x80;  //Clock Speed = 100 kHz
      008164 35 80 52 1B      [ 1]  327 	mov	0x521b+0, #0x80
                                    328 ;	periph_stm8s.c: 31: I2C_OARH = (0<<I2C_OARH_ADDMODE)|(1<<I2C_OARH_ADDCONF); //7-bit address mode, ADDCONF always must be 1
      008168 35 40 52 14      [ 1]  329 	mov	0x5214+0, #0x40
                                    330 ;	periph_stm8s.c: 32: I2C_TRISER = 17;  //Setup Bus Characteristic
      00816C 35 11 52 1D      [ 1]  331 	mov	0x521d+0, #0x11
                                    332 ;	periph_stm8s.c: 34: I2C_CR1 = (1<<I2C_CR1_PE);  //Enable I2C after configuration complete
      008170 35 01 52 10      [ 1]  333 	mov	0x5210+0, #0x01
      008174 81               [ 4]  334 	ret
                                    335 ;	periph_stm8s.c: 37: void i2c_set_start()
                                    336 ;	-----------------------------------------
                                    337 ;	 function i2c_set_start
                                    338 ;	-----------------------------------------
      008175                        339 _i2c_set_start:
                                    340 ;	periph_stm8s.c: 39: I2C_CR2 |= (1<<I2C_CR2_START);
      008175 72 10 52 11      [ 1]  341 	bset	0x5211, #0
      008179 81               [ 4]  342 	ret
                                    343 ;	periph_stm8s.c: 42: void i2c_set_address(unsigned char addr, unsigned char dir)
                                    344 ;	-----------------------------------------
                                    345 ;	 function i2c_set_address
                                    346 ;	-----------------------------------------
      00817A                        347 _i2c_set_address:
                                    348 ;	periph_stm8s.c: 44: if(dir==I2C_READ) I2C_DR = (addr<<1)|dir;
      00817A 7B 03            [ 1]  349 	ld	a, (0x03, sp)
      00817C 97               [ 1]  350 	ld	xl, a
      00817D 58               [ 2]  351 	sllw	x
      00817E 7B 04            [ 1]  352 	ld	a, (0x04, sp)
      008180 A1 01            [ 1]  353 	cp	a, #0x01
      008182 26 09            [ 1]  354 	jrne	00104$
      008184 9F               [ 1]  355 	ld	a, xl
      008185 1A 04            [ 1]  356 	or	a, (0x04, sp)
      008187 AE 52 16         [ 2]  357 	ldw	x, #0x5216
      00818A F7               [ 1]  358 	ld	(x), a
      00818B 20 0D            [ 2]  359 	jra	00106$
      00818D                        360 00104$:
                                    361 ;	periph_stm8s.c: 45: else if(dir==I2C_WRITE) I2C_DR = (addr<<1)&dir;
      00818D 7B 04            [ 1]  362 	ld	a, (0x04, sp)
      00818F A1 FE            [ 1]  363 	cp	a, #0xfe
      008191 26 07            [ 1]  364 	jrne	00106$
      008193 9F               [ 1]  365 	ld	a, xl
      008194 14 04            [ 1]  366 	and	a, (0x04, sp)
      008196 AE 52 16         [ 2]  367 	ldw	x, #0x5216
      008199 F7               [ 1]  368 	ld	(x), a
      00819A                        369 00106$:
      00819A 81               [ 4]  370 	ret
                                    371 ;	periph_stm8s.c: 49: void i2c_set_stop()
                                    372 ;	-----------------------------------------
                                    373 ;	 function i2c_set_stop
                                    374 ;	-----------------------------------------
      00819B                        375 _i2c_set_stop:
                                    376 ;	periph_stm8s.c: 51: I2C_CR2 |= (1<<I2C_CR2_STOP);
      00819B AE 52 11         [ 2]  377 	ldw	x, #0x5211
      00819E F6               [ 1]  378 	ld	a, (x)
      00819F AA 02            [ 1]  379 	or	a, #0x02
      0081A1 F7               [ 1]  380 	ld	(x), a
      0081A2 81               [ 4]  381 	ret
                                    382 ;	periph_stm8s.c: 54: void i2c_clear_ack()
                                    383 ;	-----------------------------------------
                                    384 ;	 function i2c_clear_ack
                                    385 ;	-----------------------------------------
      0081A3                        386 _i2c_clear_ack:
                                    387 ;	periph_stm8s.c: 56: I2C_CR2 &= ~(1<<I2C_CR2_ACK); //Disable Acknowledge
      0081A3 AE 52 11         [ 2]  388 	ldw	x, #0x5211
      0081A6 F6               [ 1]  389 	ld	a, (x)
      0081A7 A4 FB            [ 1]  390 	and	a, #0xfb
      0081A9 F7               [ 1]  391 	ld	(x), a
      0081AA 81               [ 4]  392 	ret
                                    393 ;	periph_stm8s.c: 59: void i2c_set_ack()
                                    394 ;	-----------------------------------------
                                    395 ;	 function i2c_set_ack
                                    396 ;	-----------------------------------------
      0081AB                        397 _i2c_set_ack:
                                    398 ;	periph_stm8s.c: 61: I2C_CR2 |= (1<<I2C_CR2_ACK); //Enable Acknowledge
      0081AB AE 52 11         [ 2]  399 	ldw	x, #0x5211
      0081AE F6               [ 1]  400 	ld	a, (x)
      0081AF AA 04            [ 1]  401 	or	a, #0x04
      0081B1 F7               [ 1]  402 	ld	(x), a
      0081B2 81               [ 4]  403 	ret
                                    404 ;	periph_stm8s.c: 64: void i2c_ack_pos_current()
                                    405 ;	-----------------------------------------
                                    406 ;	 function i2c_ack_pos_current
                                    407 ;	-----------------------------------------
      0081B3                        408 _i2c_ack_pos_current:
                                    409 ;	periph_stm8s.c: 66: I2C_CR2 &= ~(1<<I2C_CR2_POS); //ACK bit controls the (N)ACK of the current byte being received in the shift register
      0081B3 AE 52 11         [ 2]  410 	ldw	x, #0x5211
      0081B6 F6               [ 1]  411 	ld	a, (x)
      0081B7 A4 F7            [ 1]  412 	and	a, #0xf7
      0081B9 F7               [ 1]  413 	ld	(x), a
      0081BA 81               [ 4]  414 	ret
                                    415 ;	periph_stm8s.c: 69: void i2c_ack_pos_next()
                                    416 ;	-----------------------------------------
                                    417 ;	 function i2c_ack_pos_next
                                    418 ;	-----------------------------------------
      0081BB                        419 _i2c_ack_pos_next:
                                    420 ;	periph_stm8s.c: 71: I2C_CR2 |= (1<<I2C_CR2_POS); //ACK bit controls the (N)ACK of the next byte which will be received in the shift register
      0081BB AE 52 11         [ 2]  421 	ldw	x, #0x5211
      0081BE F6               [ 1]  422 	ld	a, (x)
      0081BF AA 08            [ 1]  423 	or	a, #0x08
      0081C1 F7               [ 1]  424 	ld	(x), a
      0081C2 81               [ 4]  425 	ret
                                    426 ;	periph_stm8s.c: 74: void i2c_poll_SB()
                                    427 ;	-----------------------------------------
                                    428 ;	 function i2c_poll_SB
                                    429 ;	-----------------------------------------
      0081C3                        430 _i2c_poll_SB:
                                    431 ;	periph_stm8s.c: 76: while((I2C_SR1&(1<<I2C_SR1_SB))!=(1<<I2C_SR1_SB)){}
      0081C3                        432 00101$:
      0081C3 AE 52 17         [ 2]  433 	ldw	x, #0x5217
      0081C6 F6               [ 1]  434 	ld	a, (x)
      0081C7 A4 01            [ 1]  435 	and	a, #0x01
      0081C9 A1 01            [ 1]  436 	cp	a, #0x01
      0081CB 26 F6            [ 1]  437 	jrne	00101$
      0081CD 81               [ 4]  438 	ret
                                    439 ;	periph_stm8s.c: 79: void i2c_poll_ADDR()
                                    440 ;	-----------------------------------------
                                    441 ;	 function i2c_poll_ADDR
                                    442 ;	-----------------------------------------
      0081CE                        443 _i2c_poll_ADDR:
                                    444 ;	periph_stm8s.c: 81: while((I2C_SR1&(1<<I2C_SR1_ADDR))!=(1<<I2C_SR1_ADDR)){}
      0081CE                        445 00101$:
      0081CE AE 52 17         [ 2]  446 	ldw	x, #0x5217
      0081D1 F6               [ 1]  447 	ld	a, (x)
      0081D2 A4 02            [ 1]  448 	and	a, #0x02
      0081D4 A1 02            [ 1]  449 	cp	a, #0x02
      0081D6 26 F6            [ 1]  450 	jrne	00101$
      0081D8 81               [ 4]  451 	ret
                                    452 ;	periph_stm8s.c: 84: void i2c_poll_BTF()
                                    453 ;	-----------------------------------------
                                    454 ;	 function i2c_poll_BTF
                                    455 ;	-----------------------------------------
      0081D9                        456 _i2c_poll_BTF:
                                    457 ;	periph_stm8s.c: 86: while((I2C_SR1&(1<<I2C_SR1_BTF))!=(1<<I2C_SR1_BTF)){}
      0081D9                        458 00101$:
      0081D9 AE 52 17         [ 2]  459 	ldw	x, #0x5217
      0081DC F6               [ 1]  460 	ld	a, (x)
      0081DD A4 04            [ 1]  461 	and	a, #0x04
      0081DF A1 04            [ 1]  462 	cp	a, #0x04
      0081E1 26 F6            [ 1]  463 	jrne	00101$
      0081E3 81               [ 4]  464 	ret
                                    465 ;	periph_stm8s.c: 89: void i2c_poll_TXE()
                                    466 ;	-----------------------------------------
                                    467 ;	 function i2c_poll_TXE
                                    468 ;	-----------------------------------------
      0081E4                        469 _i2c_poll_TXE:
                                    470 ;	periph_stm8s.c: 91: while((I2C_SR1&(1<<I2C_SR1_TXE))!=(1<<I2C_SR1_TXE)){}
      0081E4                        471 00101$:
      0081E4 AE 52 17         [ 2]  472 	ldw	x, #0x5217
      0081E7 F6               [ 1]  473 	ld	a, (x)
      0081E8 A4 80            [ 1]  474 	and	a, #0x80
      0081EA A1 80            [ 1]  475 	cp	a, #0x80
      0081EC 26 F6            [ 1]  476 	jrne	00101$
      0081EE 81               [ 4]  477 	ret
                                    478 ;	periph_stm8s.c: 94: void i2c_poll_RXNE()
                                    479 ;	-----------------------------------------
                                    480 ;	 function i2c_poll_RXNE
                                    481 ;	-----------------------------------------
      0081EF                        482 _i2c_poll_RXNE:
                                    483 ;	periph_stm8s.c: 96: while((I2C_SR1&(1<<I2C_SR1_RXNE))!=(1<<I2C_SR1_RXNE)){}
      0081EF                        484 00101$:
      0081EF AE 52 17         [ 2]  485 	ldw	x, #0x5217
      0081F2 F6               [ 1]  486 	ld	a, (x)
      0081F3 A4 40            [ 1]  487 	and	a, #0x40
      0081F5 A1 40            [ 1]  488 	cp	a, #0x40
      0081F7 26 F6            [ 1]  489 	jrne	00101$
      0081F9 81               [ 4]  490 	ret
                                    491 ;	periph_stm8s.c: 99: void i2c_clear_bits()
                                    492 ;	-----------------------------------------
                                    493 ;	 function i2c_clear_bits
                                    494 ;	-----------------------------------------
      0081FA                        495 _i2c_clear_bits:
                                    496 ;	periph_stm8s.c: 101: readreg = I2C_SR1;
      0081FA AE 52 17         [ 2]  497 	ldw	x, #0x5217
      0081FD F6               [ 1]  498 	ld	a, (x)
      0081FE C7 00 01         [ 1]  499 	ld	_readreg+0, a
      008201 81               [ 4]  500 	ret
                                    501 ;	periph_stm8s.c: 104: void i2c_clear_ADDR()
                                    502 ;	-----------------------------------------
                                    503 ;	 function i2c_clear_ADDR
                                    504 ;	-----------------------------------------
      008202                        505 _i2c_clear_ADDR:
                                    506 ;	periph_stm8s.c: 106: readreg = I2C_SR1;
      008202 AE 52 17         [ 2]  507 	ldw	x, #0x5217
      008205 F6               [ 1]  508 	ld	a, (x)
                                    509 ;	periph_stm8s.c: 107: readreg = I2C_SR3;
      008206 AE 52 19         [ 2]  510 	ldw	x, #0x5219
      008209 F6               [ 1]  511 	ld	a, (x)
      00820A C7 00 01         [ 1]  512 	ld	_readreg+0, a
      00820D 81               [ 4]  513 	ret
                                    514 ;	periph_stm8s.c: 110: void i2c_enable_interrupts()
                                    515 ;	-----------------------------------------
                                    516 ;	 function i2c_enable_interrupts
                                    517 ;	-----------------------------------------
      00820E                        518 _i2c_enable_interrupts:
                                    519 ;	periph_stm8s.c: 112: I2C_ITR = 0x07;
      00820E 35 07 52 1A      [ 1]  520 	mov	0x521a+0, #0x07
      008212 81               [ 4]  521 	ret
                                    522 ;	periph_stm8s.c: 114: void i2c_disable_interrupts()
                                    523 ;	-----------------------------------------
                                    524 ;	 function i2c_disable_interrupts
                                    525 ;	-----------------------------------------
      008213                        526 _i2c_disable_interrupts:
                                    527 ;	periph_stm8s.c: 116: I2C_ITR = 0x00;
      008213 35 00 52 1A      [ 1]  528 	mov	0x521a+0, #0x00
      008217 81               [ 4]  529 	ret
                                    530 ;	periph_stm8s.c: 119: void i2c_write_1byte(unsigned char devaddr, unsigned char dbyte1)
                                    531 ;	-----------------------------------------
                                    532 ;	 function i2c_write_1byte
                                    533 ;	-----------------------------------------
      008218                        534 _i2c_write_1byte:
                                    535 ;	periph_stm8s.c: 121: i2c_set_start(); //Send Start Condition
      008218 CD 81 75         [ 4]  536 	call	_i2c_set_start
                                    537 ;	periph_stm8s.c: 122: i2c_poll_SB(); //Wait until Start Bit is set --> Start Condition generated
      00821B CD 81 C3         [ 4]  538 	call	_i2c_poll_SB
                                    539 ;	periph_stm8s.c: 123: i2c_clear_bits(); //Clear Start Bit
      00821E CD 81 FA         [ 4]  540 	call	_i2c_clear_bits
                                    541 ;	periph_stm8s.c: 125: i2c_set_address(devaddr,I2C_WRITE); //Write Address w Direction : Write
      008221 4B FE            [ 1]  542 	push	#0xfe
      008223 7B 04            [ 1]  543 	ld	a, (0x04, sp)
      008225 88               [ 1]  544 	push	a
      008226 CD 81 7A         [ 4]  545 	call	_i2c_set_address
      008229 5B 02            [ 2]  546 	addw	sp, #2
                                    547 ;	periph_stm8s.c: 126: i2c_poll_ADDR(); //Wait until Address Flag is set --> Address matched
      00822B CD 81 CE         [ 4]  548 	call	_i2c_poll_ADDR
                                    549 ;	periph_stm8s.c: 127: i2c_clear_ADDR(); //Clear Address Flag
      00822E CD 82 02         [ 4]  550 	call	_i2c_clear_ADDR
                                    551 ;	periph_stm8s.c: 129: i2c_poll_TXE(); //Wait until Data Register is empty. In practice, this step is optional
      008231 CD 81 E4         [ 4]  552 	call	_i2c_poll_TXE
                                    553 ;	periph_stm8s.c: 130: I2C_DR = dbyte1; //Command or Data
      008234 AE 52 16         [ 2]  554 	ldw	x, #0x5216
      008237 7B 04            [ 1]  555 	ld	a, (0x04, sp)
      008239 F7               [ 1]  556 	ld	(x), a
                                    557 ;	periph_stm8s.c: 131: i2c_poll_BTF(); //Wait until Byte Transfer Flag is set --> 1 Byte Data Transfer is complete
      00823A CD 81 D9         [ 4]  558 	call	_i2c_poll_BTF
                                    559 ;	periph_stm8s.c: 132: i2c_clear_bits(); //Clear Byte Transfer Flag
      00823D CD 81 FA         [ 4]  560 	call	_i2c_clear_bits
                                    561 ;	periph_stm8s.c: 134: i2c_set_stop(); //Send Stop Condition
      008240 CD 81 9B         [ 4]  562 	call	_i2c_set_stop
                                    563 ;	periph_stm8s.c: 135: i2c_clear_bits(); //Clear Stop Bit
      008243 CC 81 FA         [ 2]  564 	jp	_i2c_clear_bits
                                    565 ;	periph_stm8s.c: 138: void i2c_write_2byte(unsigned char devaddr, unsigned char dbyte1, unsigned char dbyte2)
                                    566 ;	-----------------------------------------
                                    567 ;	 function i2c_write_2byte
                                    568 ;	-----------------------------------------
      008246                        569 _i2c_write_2byte:
                                    570 ;	periph_stm8s.c: 140: i2c_set_start(); //Send Start Condition
      008246 CD 81 75         [ 4]  571 	call	_i2c_set_start
                                    572 ;	periph_stm8s.c: 141: i2c_poll_SB(); //Wait until Start Bit is set --> Start Condition generated
      008249 CD 81 C3         [ 4]  573 	call	_i2c_poll_SB
                                    574 ;	periph_stm8s.c: 142: i2c_clear_bits(); //Clear Start Bit
      00824C CD 81 FA         [ 4]  575 	call	_i2c_clear_bits
                                    576 ;	periph_stm8s.c: 144: i2c_set_address(devaddr,I2C_WRITE); //Write Address w Direction : Write
      00824F 4B FE            [ 1]  577 	push	#0xfe
      008251 7B 04            [ 1]  578 	ld	a, (0x04, sp)
      008253 88               [ 1]  579 	push	a
      008254 CD 81 7A         [ 4]  580 	call	_i2c_set_address
      008257 5B 02            [ 2]  581 	addw	sp, #2
                                    582 ;	periph_stm8s.c: 145: i2c_poll_ADDR(); //Wait until Address Flag is set --> Address matched
      008259 CD 81 CE         [ 4]  583 	call	_i2c_poll_ADDR
                                    584 ;	periph_stm8s.c: 146: i2c_clear_ADDR(); //Clear Address Flag
      00825C CD 82 02         [ 4]  585 	call	_i2c_clear_ADDR
                                    586 ;	periph_stm8s.c: 148: i2c_poll_TXE(); //Wait until Data Register is empty. In practice, this step is optional
      00825F CD 81 E4         [ 4]  587 	call	_i2c_poll_TXE
                                    588 ;	periph_stm8s.c: 149: I2C_DR = dbyte1; //1st Byte of Command or Data
      008262 AE 52 16         [ 2]  589 	ldw	x, #0x5216
      008265 7B 04            [ 1]  590 	ld	a, (0x04, sp)
      008267 F7               [ 1]  591 	ld	(x), a
                                    592 ;	periph_stm8s.c: 150: i2c_poll_BTF(); //Wait until Byte Transfer Flag is set --> 1 Byte Data Transfer is complete
      008268 CD 81 D9         [ 4]  593 	call	_i2c_poll_BTF
                                    594 ;	periph_stm8s.c: 151: i2c_clear_bits(); //Clear Byte Transfer Flag
      00826B CD 81 FA         [ 4]  595 	call	_i2c_clear_bits
                                    596 ;	periph_stm8s.c: 153: I2C_DR = dbyte2; //2nd Byte of Command or Data
      00826E AE 52 16         [ 2]  597 	ldw	x, #0x5216
      008271 7B 05            [ 1]  598 	ld	a, (0x05, sp)
      008273 F7               [ 1]  599 	ld	(x), a
                                    600 ;	periph_stm8s.c: 154: i2c_poll_BTF(); //Wait until Byte Transfer Flag is set --> 1 Byte Data Transfer is complete
      008274 CD 81 D9         [ 4]  601 	call	_i2c_poll_BTF
                                    602 ;	periph_stm8s.c: 155: i2c_clear_bits(); //Clear Byte Transfer Flag
      008277 CD 81 FA         [ 4]  603 	call	_i2c_clear_bits
                                    604 ;	periph_stm8s.c: 157: i2c_set_stop(); //Send Stop Condition
      00827A CD 81 9B         [ 4]  605 	call	_i2c_set_stop
                                    606 ;	periph_stm8s.c: 158: i2c_clear_bits(); //Clear Stop Bit
      00827D CC 81 FA         [ 2]  607 	jp	_i2c_clear_bits
                                    608 ;	periph_stm8s.c: 163: void adc_init()
                                    609 ;	-----------------------------------------
                                    610 ;	 function adc_init
                                    611 ;	-----------------------------------------
      008280                        612 _adc_init:
                                    613 ;	periph_stm8s.c: 165: ADC1_CR1 = fADC_fMASTER_8<<ADC1_CR1_SPSEL; // ADCCLK = MCLK/8
      008280 35 40 54 01      [ 1]  614 	mov	0x5401+0, #0x40
                                    615 ;	periph_stm8s.c: 166: ADC1_CR2 = (1<<ADC1_CR2_ALIGN);  // right alignment adc data
      008284 35 08 54 02      [ 1]  616 	mov	0x5402+0, #0x08
                                    617 ;	periph_stm8s.c: 168: ADC1_CR1 |= (1<<ADC1_CR1_ADON);  // turn on ADC
      008288 72 10 54 01      [ 1]  618 	bset	0x5401, #0
      00828C 81               [ 4]  619 	ret
                                    620 ;	periph_stm8s.c: 172: unsigned int read_adc(unsigned char adcch)
                                    621 ;	-----------------------------------------
                                    622 ;	 function read_adc
                                    623 ;	-----------------------------------------
      00828D                        624 _read_adc:
      00828D 52 04            [ 2]  625 	sub	sp, #4
                                    626 ;	periph_stm8s.c: 176: ADC1_CSR &= 0xF0;  // select
      00828F AE 54 00         [ 2]  627 	ldw	x, #0x5400
      008292 F6               [ 1]  628 	ld	a, (x)
      008293 A4 F0            [ 1]  629 	and	a, #0xf0
      008295 F7               [ 1]  630 	ld	(x), a
                                    631 ;	periph_stm8s.c: 177: ADC1_CSR |= adcch; // channel
      008296 AE 54 00         [ 2]  632 	ldw	x, #0x5400
      008299 F6               [ 1]  633 	ld	a, (x)
      00829A 1A 07            [ 1]  634 	or	a, (0x07, sp)
      00829C AE 54 00         [ 2]  635 	ldw	x, #0x5400
      00829F F7               [ 1]  636 	ld	(x), a
                                    637 ;	periph_stm8s.c: 180: ADC1_CR1 |= (1<<ADC1_CR1_ADON); // start conversion
      0082A0 72 10 54 01      [ 1]  638 	bset	0x5401, #0
                                    639 ;	periph_stm8s.c: 181: while(!((ADC1_CSR)&(1<<ADC1_CSR_EOC)));; // conversion is in progress
      0082A4                        640 00101$:
      0082A4 AE 54 00         [ 2]  641 	ldw	x, #0x5400
      0082A7 F6               [ 1]  642 	ld	a, (x)
      0082A8 4D               [ 1]  643 	tnz	a
      0082A9 2A F9            [ 1]  644 	jrpl	00101$
                                    645 ;	periph_stm8s.c: 182: adcval = (ADC1_DRH<<8) + ADC1_DRL;
      0082AB AE 54 04         [ 2]  646 	ldw	x, #0x5404
      0082AE F6               [ 1]  647 	ld	a, (x)
      0082AF 0F 03            [ 1]  648 	clr	(0x03, sp)
      0082B1 6B 01            [ 1]  649 	ld	(0x01, sp), a
      0082B3 0F 02            [ 1]  650 	clr	(0x02, sp)
      0082B5 AE 54 05         [ 2]  651 	ldw	x, #0x5405
      0082B8 F6               [ 1]  652 	ld	a, (x)
      0082B9 5F               [ 1]  653 	clrw	x
      0082BA 97               [ 1]  654 	ld	xl, a
      0082BB 72 FB 01         [ 2]  655 	addw	x, (0x01, sp)
                                    656 ;	periph_stm8s.c: 183: ADC1_CSR |= (0<<ADC1_CSR_EOC); // reset EOC
      0082BE 90 AE 54 00      [ 2]  657 	ldw	y, #0x5400
      0082C2 90 F6            [ 1]  658 	ld	a, (y)
      0082C4 90 AE 54 00      [ 2]  659 	ldw	y, #0x5400
      0082C8 90 F7            [ 1]  660 	ld	(y), a
                                    661 ;	periph_stm8s.c: 185: return adcval;
      0082CA 5B 04            [ 2]  662 	addw	sp, #4
      0082CC 81               [ 4]  663 	ret
                                    664 ;	periph_stm8s.c: 190: void uart1_init(unsigned char rxien) //UART Initialization
                                    665 ;	-----------------------------------------
                                    666 ;	 function uart1_init
                                    667 ;	-----------------------------------------
      0082CD                        668 _uart1_init:
                                    669 ;	periph_stm8s.c: 194: UART1_BRR1 = 0x68;
      0082CD 35 68 52 32      [ 1]  670 	mov	0x5232+0, #0x68
                                    671 ;	periph_stm8s.c: 195: UART1_BRR2 = 0x03;
      0082D1 35 03 52 33      [ 1]  672 	mov	0x5233+0, #0x03
                                    673 ;	periph_stm8s.c: 197: UART1_CR1 |= (0<<UART1_CR1_M)|(0<<UART1_CR1_PCEN); //8 bit Data; No Parity
      0082D5 AE 52 34         [ 2]  674 	ldw	x, #0x5234
      0082D8 F6               [ 1]  675 	ld	a, (x)
      0082D9 AE 52 34         [ 2]  676 	ldw	x, #0x5234
      0082DC F7               [ 1]  677 	ld	(x), a
                                    678 ;	periph_stm8s.c: 198: UART1_CR3 |= (0<<UART1_CR3_STOP); //Stop Bit = 1
      0082DD AE 52 36         [ 2]  679 	ldw	x, #0x5236
      0082E0 F6               [ 1]  680 	ld	a, (x)
      0082E1 AE 52 36         [ 2]  681 	ldw	x, #0x5236
      0082E4 F7               [ 1]  682 	ld	(x), a
                                    683 ;	periph_stm8s.c: 200: if(rxien==1) 
      0082E5 7B 03            [ 1]  684 	ld	a, (0x03, sp)
      0082E7 A1 01            [ 1]  685 	cp	a, #0x01
      0082E9 26 0B            [ 1]  686 	jrne	00102$
                                    687 ;	periph_stm8s.c: 202: UART1_CR2 |= (1<<UART1_CR2_RIEN); //Enable Interrupt on Receiver Mode
      0082EB AE 52 35         [ 2]  688 	ldw	x, #0x5235
      0082EE F6               [ 1]  689 	ld	a, (x)
      0082EF AA 20            [ 1]  690 	or	a, #0x20
      0082F1 F7               [ 1]  691 	ld	(x), a
                                    692 ;	periph_stm8s.c: 203: ITC_SPR5 = (level_2<<ITC_SPR5_VECT18); //UART Interrupt Setting
      0082F2 35 00 7F 74      [ 1]  693 	mov	0x7f74+0, #0x00
      0082F6                        694 00102$:
                                    695 ;	periph_stm8s.c: 206: UART1_CR2 |= (1<<UART1_CR2_TEN); //Enable Transmitter Mode
      0082F6 AE 52 35         [ 2]  696 	ldw	x, #0x5235
      0082F9 F6               [ 1]  697 	ld	a, (x)
      0082FA AA 08            [ 1]  698 	or	a, #0x08
      0082FC F7               [ 1]  699 	ld	(x), a
                                    700 ;	periph_stm8s.c: 207: UART1_CR2 |= (1<<UART1_CR2_REN); //Enable Receiver Mode
      0082FD AE 52 35         [ 2]  701 	ldw	x, #0x5235
      008300 F6               [ 1]  702 	ld	a, (x)
      008301 AA 04            [ 1]  703 	or	a, #0x04
      008303 F7               [ 1]  704 	ld	(x), a
      008304 81               [ 4]  705 	ret
                                    706 ;	periph_stm8s.c: 210: void uart1_send(unsigned char usend) //UART Transmit a Byte
                                    707 ;	-----------------------------------------
                                    708 ;	 function uart1_send
                                    709 ;	-----------------------------------------
      008305                        710 _uart1_send:
                                    711 ;	periph_stm8s.c: 212: UART1_DR = usend; //Write to UART Data Register
      008305 AE 52 31         [ 2]  712 	ldw	x, #0x5231
      008308 7B 03            [ 1]  713 	ld	a, (0x03, sp)
      00830A F7               [ 1]  714 	ld	(x), a
                                    715 ;	periph_stm8s.c: 213: while((UART1_SR&(1<<UART1_SR_TXE))!=(1<<UART1_SR_TXE)); //Wait until Transmission complete
      00830B                        716 00101$:
      00830B AE 52 30         [ 2]  717 	ldw	x, #0x5230
      00830E F6               [ 1]  718 	ld	a, (x)
      00830F A4 80            [ 1]  719 	and	a, #0x80
      008311 A1 80            [ 1]  720 	cp	a, #0x80
      008313 26 F6            [ 1]  721 	jrne	00101$
      008315 81               [ 4]  722 	ret
                                    723 ;	periph_stm8s.c: 216: unsigned char uart1_recv() //UART Receive a Byte (using Polling)
                                    724 ;	-----------------------------------------
                                    725 ;	 function uart1_recv
                                    726 ;	-----------------------------------------
      008316                        727 _uart1_recv:
                                    728 ;	periph_stm8s.c: 219: if((UART1_SR&(1<<UART1_SR_RXNE))==(1<<UART1_SR_RXNE)) //Check if any data in Data Register
      008316 AE 52 30         [ 2]  729 	ldw	x, #0x5230
      008319 F6               [ 1]  730 	ld	a, (x)
      00831A A4 20            [ 1]  731 	and	a, #0x20
      00831C A1 20            [ 1]  732 	cp	a, #0x20
      00831E 26 05            [ 1]  733 	jrne	00102$
                                    734 ;	periph_stm8s.c: 221: urecv = UART1_DR; //Read Data Register (RXNE cleared automatically)
      008320 AE 52 31         [ 2]  735 	ldw	x, #0x5231
      008323 F6               [ 1]  736 	ld	a, (x)
                                    737 ;	periph_stm8s.c: 223: else urecv=0;
      008324 21                     738 	.byte 0x21
      008325                        739 00102$:
      008325 4F               [ 1]  740 	clr	a
      008326                        741 00103$:
                                    742 ;	periph_stm8s.c: 224: return urecv;
      008326 81               [ 4]  743 	ret
                                    744 ;	periph_stm8s.c: 227: unsigned char uart1_recv_i() //UART Receive a Byte (using Interrupt)
                                    745 ;	-----------------------------------------
                                    746 ;	 function uart1_recv_i
                                    747 ;	-----------------------------------------
      008327                        748 _uart1_recv_i:
                                    749 ;	periph_stm8s.c: 230: urecv = UART1_DR; //Read Data Register (RXNE cleared automatically)
      008327 AE 52 31         [ 2]  750 	ldw	x, #0x5231
      00832A F6               [ 1]  751 	ld	a, (x)
                                    752 ;	periph_stm8s.c: 231: return urecv;
      00832B 81               [ 4]  753 	ret
                                    754 ;	oled_ssd1306.c: 7: void ssd1306_init(unsigned char olednum)
                                    755 ;	-----------------------------------------
                                    756 ;	 function ssd1306_init
                                    757 ;	-----------------------------------------
      00832C                        758 _ssd1306_init:
                                    759 ;	oled_ssd1306.c: 9: ssd1306_sendcom(olednum,0xAE); //Set Display Off
      00832C 4B AE            [ 1]  760 	push	#0xae
      00832E 7B 04            [ 1]  761 	ld	a, (0x04, sp)
      008330 88               [ 1]  762 	push	a
      008331 CD 84 27         [ 4]  763 	call	_ssd1306_sendcom
      008334 5B 02            [ 2]  764 	addw	sp, #2
                                    765 ;	oled_ssd1306.c: 10: ssd1306_sendcom(olednum,0xD5); //Set Display Clock Divider Ratio/Oscillator Frequency
      008336 4B D5            [ 1]  766 	push	#0xd5
      008338 7B 04            [ 1]  767 	ld	a, (0x04, sp)
      00833A 88               [ 1]  768 	push	a
      00833B CD 84 27         [ 4]  769 	call	_ssd1306_sendcom
      00833E 5B 02            [ 2]  770 	addw	sp, #2
                                    771 ;	oled_ssd1306.c: 11: ssd1306_sendcom(olednum,0x80);
      008340 4B 80            [ 1]  772 	push	#0x80
      008342 7B 04            [ 1]  773 	ld	a, (0x04, sp)
      008344 88               [ 1]  774 	push	a
      008345 CD 84 27         [ 4]  775 	call	_ssd1306_sendcom
      008348 5B 02            [ 2]  776 	addw	sp, #2
                                    777 ;	oled_ssd1306.c: 12: ssd1306_sendcom(olednum,0xA8); //Set Multiplex Ratio
      00834A 4B A8            [ 1]  778 	push	#0xa8
      00834C 7B 04            [ 1]  779 	ld	a, (0x04, sp)
      00834E 88               [ 1]  780 	push	a
      00834F CD 84 27         [ 4]  781 	call	_ssd1306_sendcom
      008352 5B 02            [ 2]  782 	addw	sp, #2
                                    783 ;	oled_ssd1306.c: 13: ssd1306_sendcom(olednum,0x3F);
      008354 4B 3F            [ 1]  784 	push	#0x3f
      008356 7B 04            [ 1]  785 	ld	a, (0x04, sp)
      008358 88               [ 1]  786 	push	a
      008359 CD 84 27         [ 4]  787 	call	_ssd1306_sendcom
      00835C 5B 02            [ 2]  788 	addw	sp, #2
                                    789 ;	oled_ssd1306.c: 14: ssd1306_sendcom(olednum,0xD3); //Set Display Offset
      00835E 4B D3            [ 1]  790 	push	#0xd3
      008360 7B 04            [ 1]  791 	ld	a, (0x04, sp)
      008362 88               [ 1]  792 	push	a
      008363 CD 84 27         [ 4]  793 	call	_ssd1306_sendcom
      008366 5B 02            [ 2]  794 	addw	sp, #2
                                    795 ;	oled_ssd1306.c: 15: ssd1306_sendcom(olednum,0x00);
      008368 4B 00            [ 1]  796 	push	#0x00
      00836A 7B 04            [ 1]  797 	ld	a, (0x04, sp)
      00836C 88               [ 1]  798 	push	a
      00836D CD 84 27         [ 4]  799 	call	_ssd1306_sendcom
      008370 5B 02            [ 2]  800 	addw	sp, #2
                                    801 ;	oled_ssd1306.c: 16: ssd1306_sendcom(olednum,0x40); //Set Display Start Line
      008372 4B 40            [ 1]  802 	push	#0x40
      008374 7B 04            [ 1]  803 	ld	a, (0x04, sp)
      008376 88               [ 1]  804 	push	a
      008377 CD 84 27         [ 4]  805 	call	_ssd1306_sendcom
      00837A 5B 02            [ 2]  806 	addw	sp, #2
                                    807 ;	oled_ssd1306.c: 17: ssd1306_sendcom(olednum,0x8D); //Set Charge Pump
      00837C 4B 8D            [ 1]  808 	push	#0x8d
      00837E 7B 04            [ 1]  809 	ld	a, (0x04, sp)
      008380 88               [ 1]  810 	push	a
      008381 CD 84 27         [ 4]  811 	call	_ssd1306_sendcom
      008384 5B 02            [ 2]  812 	addw	sp, #2
                                    813 ;	oled_ssd1306.c: 18: ssd1306_sendcom(olednum,0x14); //Internal VCC
      008386 4B 14            [ 1]  814 	push	#0x14
      008388 7B 04            [ 1]  815 	ld	a, (0x04, sp)
      00838A 88               [ 1]  816 	push	a
      00838B CD 84 27         [ 4]  817 	call	_ssd1306_sendcom
      00838E 5B 02            [ 2]  818 	addw	sp, #2
                                    819 ;	oled_ssd1306.c: 19: ssd1306_sendcom(olednum,0x20); //Set Memory Mode
      008390 4B 20            [ 1]  820 	push	#0x20
      008392 7B 04            [ 1]  821 	ld	a, (0x04, sp)
      008394 88               [ 1]  822 	push	a
      008395 CD 84 27         [ 4]  823 	call	_ssd1306_sendcom
      008398 5B 02            [ 2]  824 	addw	sp, #2
                                    825 ;	oled_ssd1306.c: 20: ssd1306_sendcom(olednum,0x00); //Horizontal Addressing
      00839A 4B 00            [ 1]  826 	push	#0x00
      00839C 7B 04            [ 1]  827 	ld	a, (0x04, sp)
      00839E 88               [ 1]  828 	push	a
      00839F CD 84 27         [ 4]  829 	call	_ssd1306_sendcom
      0083A2 5B 02            [ 2]  830 	addw	sp, #2
                                    831 ;	oled_ssd1306.c: 21: ssd1306_sendcom(olednum,0xA1); //Set Segment Re-Map
      0083A4 4B A1            [ 1]  832 	push	#0xa1
      0083A6 7B 04            [ 1]  833 	ld	a, (0x04, sp)
      0083A8 88               [ 1]  834 	push	a
      0083A9 CD 84 27         [ 4]  835 	call	_ssd1306_sendcom
      0083AC 5B 02            [ 2]  836 	addw	sp, #2
                                    837 ;	oled_ssd1306.c: 22: ssd1306_sendcom(olednum,0xC8); //Set COM Output Scan Direction
      0083AE 4B C8            [ 1]  838 	push	#0xc8
      0083B0 7B 04            [ 1]  839 	ld	a, (0x04, sp)
      0083B2 88               [ 1]  840 	push	a
      0083B3 CD 84 27         [ 4]  841 	call	_ssd1306_sendcom
      0083B6 5B 02            [ 2]  842 	addw	sp, #2
                                    843 ;	oled_ssd1306.c: 23: ssd1306_sendcom(olednum,0xDA); //Set COM Pins HW Config
      0083B8 4B DA            [ 1]  844 	push	#0xda
      0083BA 7B 04            [ 1]  845 	ld	a, (0x04, sp)
      0083BC 88               [ 1]  846 	push	a
      0083BD CD 84 27         [ 4]  847 	call	_ssd1306_sendcom
      0083C0 5B 02            [ 2]  848 	addw	sp, #2
                                    849 ;	oled_ssd1306.c: 24: ssd1306_sendcom(olednum,0x12);
      0083C2 4B 12            [ 1]  850 	push	#0x12
      0083C4 7B 04            [ 1]  851 	ld	a, (0x04, sp)
      0083C6 88               [ 1]  852 	push	a
      0083C7 CD 84 27         [ 4]  853 	call	_ssd1306_sendcom
      0083CA 5B 02            [ 2]  854 	addw	sp, #2
                                    855 ;	oled_ssd1306.c: 25: ssd1306_sendcom(olednum,0x81); //Set Contrast Control
      0083CC 4B 81            [ 1]  856 	push	#0x81
      0083CE 7B 04            [ 1]  857 	ld	a, (0x04, sp)
      0083D0 88               [ 1]  858 	push	a
      0083D1 CD 84 27         [ 4]  859 	call	_ssd1306_sendcom
      0083D4 5B 02            [ 2]  860 	addw	sp, #2
                                    861 ;	oled_ssd1306.c: 26: ssd1306_sendcom(olednum,0xCF);
      0083D6 4B CF            [ 1]  862 	push	#0xcf
      0083D8 7B 04            [ 1]  863 	ld	a, (0x04, sp)
      0083DA 88               [ 1]  864 	push	a
      0083DB CD 84 27         [ 4]  865 	call	_ssd1306_sendcom
      0083DE 5B 02            [ 2]  866 	addw	sp, #2
                                    867 ;	oled_ssd1306.c: 27: ssd1306_sendcom(olednum,0xD9); //Set Pre-Charge Period
      0083E0 4B D9            [ 1]  868 	push	#0xd9
      0083E2 7B 04            [ 1]  869 	ld	a, (0x04, sp)
      0083E4 88               [ 1]  870 	push	a
      0083E5 CD 84 27         [ 4]  871 	call	_ssd1306_sendcom
      0083E8 5B 02            [ 2]  872 	addw	sp, #2
                                    873 ;	oled_ssd1306.c: 28: ssd1306_sendcom(olednum,0xF1);
      0083EA 4B F1            [ 1]  874 	push	#0xf1
      0083EC 7B 04            [ 1]  875 	ld	a, (0x04, sp)
      0083EE 88               [ 1]  876 	push	a
      0083EF CD 84 27         [ 4]  877 	call	_ssd1306_sendcom
      0083F2 5B 02            [ 2]  878 	addw	sp, #2
                                    879 ;	oled_ssd1306.c: 29: ssd1306_sendcom(olednum,0xDB); //Set VCOMH Deselect Level
      0083F4 4B DB            [ 1]  880 	push	#0xdb
      0083F6 7B 04            [ 1]  881 	ld	a, (0x04, sp)
      0083F8 88               [ 1]  882 	push	a
      0083F9 CD 84 27         [ 4]  883 	call	_ssd1306_sendcom
      0083FC 5B 02            [ 2]  884 	addw	sp, #2
                                    885 ;	oled_ssd1306.c: 30: ssd1306_sendcom(olednum,0x40);
      0083FE 4B 40            [ 1]  886 	push	#0x40
      008400 7B 04            [ 1]  887 	ld	a, (0x04, sp)
      008402 88               [ 1]  888 	push	a
      008403 CD 84 27         [ 4]  889 	call	_ssd1306_sendcom
      008406 5B 02            [ 2]  890 	addw	sp, #2
                                    891 ;	oled_ssd1306.c: 31: ssd1306_sendcom(olednum,0xA4); //Set Entire Display On/Off
      008408 4B A4            [ 1]  892 	push	#0xa4
      00840A 7B 04            [ 1]  893 	ld	a, (0x04, sp)
      00840C 88               [ 1]  894 	push	a
      00840D CD 84 27         [ 4]  895 	call	_ssd1306_sendcom
      008410 5B 02            [ 2]  896 	addw	sp, #2
                                    897 ;	oled_ssd1306.c: 32: ssd1306_sendcom(olednum,0xA6); //Set Normal/Inverse Display
      008412 4B A6            [ 1]  898 	push	#0xa6
      008414 7B 04            [ 1]  899 	ld	a, (0x04, sp)
      008416 88               [ 1]  900 	push	a
      008417 CD 84 27         [ 4]  901 	call	_ssd1306_sendcom
      00841A 5B 02            [ 2]  902 	addw	sp, #2
                                    903 ;	oled_ssd1306.c: 33: ssd1306_sendcom(olednum,0xAF); //Set Display On
      00841C 4B AF            [ 1]  904 	push	#0xaf
      00841E 7B 04            [ 1]  905 	ld	a, (0x04, sp)
      008420 88               [ 1]  906 	push	a
      008421 CD 84 27         [ 4]  907 	call	_ssd1306_sendcom
      008424 5B 02            [ 2]  908 	addw	sp, #2
      008426 81               [ 4]  909 	ret
                                    910 ;	oled_ssd1306.c: 36: void ssd1306_sendcom(unsigned char olednum, unsigned char ssd1306com)
                                    911 ;	-----------------------------------------
                                    912 ;	 function ssd1306_sendcom
                                    913 ;	-----------------------------------------
      008427                        914 _ssd1306_sendcom:
                                    915 ;	oled_ssd1306.c: 38: i2c_write_2byte(olednum,commode,ssd1306com); //Send Command
      008427 7B 04            [ 1]  916 	ld	a, (0x04, sp)
      008429 88               [ 1]  917 	push	a
      00842A 4B 00            [ 1]  918 	push	#0x00
      00842C 7B 05            [ 1]  919 	ld	a, (0x05, sp)
      00842E 88               [ 1]  920 	push	a
      00842F CD 82 46         [ 4]  921 	call	_i2c_write_2byte
      008432 5B 03            [ 2]  922 	addw	sp, #3
      008434 81               [ 4]  923 	ret
                                    924 ;	oled_ssd1306.c: 41: void ssd1306_senddat(unsigned char olednum, unsigned char ssd1306dat)
                                    925 ;	-----------------------------------------
                                    926 ;	 function ssd1306_senddat
                                    927 ;	-----------------------------------------
      008435                        928 _ssd1306_senddat:
                                    929 ;	oled_ssd1306.c: 43: i2c_write_2byte(olednum,datmode,ssd1306dat); //Send Data
      008435 7B 04            [ 1]  930 	ld	a, (0x04, sp)
      008437 88               [ 1]  931 	push	a
      008438 4B 40            [ 1]  932 	push	#0x40
      00843A 7B 05            [ 1]  933 	ld	a, (0x05, sp)
      00843C 88               [ 1]  934 	push	a
      00843D CD 82 46         [ 4]  935 	call	_i2c_write_2byte
      008440 5B 03            [ 2]  936 	addw	sp, #3
      008442 81               [ 4]  937 	ret
                                    938 ;	oled_ssd1306.c: 46: void ssd1306_setpos(unsigned char olednum, unsigned char row, unsigned char col)
                                    939 ;	-----------------------------------------
                                    940 ;	 function ssd1306_setpos
                                    941 ;	-----------------------------------------
      008443                        942 _ssd1306_setpos:
                                    943 ;	oled_ssd1306.c: 48: ssd1306_sendcom(olednum,(0xB0|(row&0x0F))); //Set page of row
      008443 7B 04            [ 1]  944 	ld	a, (0x04, sp)
      008445 A4 0F            [ 1]  945 	and	a, #0x0f
      008447 AA B0            [ 1]  946 	or	a, #0xb0
      008449 88               [ 1]  947 	push	a
      00844A 7B 04            [ 1]  948 	ld	a, (0x04, sp)
      00844C 88               [ 1]  949 	push	a
      00844D CD 84 27         [ 4]  950 	call	_ssd1306_sendcom
      008450 5B 02            [ 2]  951 	addw	sp, #2
                                    952 ;	oled_ssd1306.c: 49: ssd1306_sendcom(olednum,(0x00|(col&0x0F))); //Set lower nibble of column
      008452 7B 05            [ 1]  953 	ld	a, (0x05, sp)
      008454 A4 0F            [ 1]  954 	and	a, #0x0f
      008456 88               [ 1]  955 	push	a
      008457 7B 04            [ 1]  956 	ld	a, (0x04, sp)
      008459 88               [ 1]  957 	push	a
      00845A CD 84 27         [ 4]  958 	call	_ssd1306_sendcom
      00845D 5B 02            [ 2]  959 	addw	sp, #2
                                    960 ;	oled_ssd1306.c: 50: ssd1306_sendcom(olednum,(0x10|((col>>4)&0x0F))); //Set upper nibble of column
      00845F 7B 05            [ 1]  961 	ld	a, (0x05, sp)
      008461 4E               [ 1]  962 	swap	a
      008462 A4 0F            [ 1]  963 	and	a, #0x0f
      008464 A4 0F            [ 1]  964 	and	a, #0x0f
      008466 AA 10            [ 1]  965 	or	a, #0x10
      008468 88               [ 1]  966 	push	a
      008469 7B 04            [ 1]  967 	ld	a, (0x04, sp)
      00846B 88               [ 1]  968 	push	a
      00846C CD 84 27         [ 4]  969 	call	_ssd1306_sendcom
      00846F 5B 02            [ 2]  970 	addw	sp, #2
      008471 81               [ 4]  971 	ret
                                    972 ;	oled_ssd1306.c: 53: void ssd1306_clear(unsigned char olednum) 
                                    973 ;	-----------------------------------------
                                    974 ;	 function ssd1306_clear
                                    975 ;	-----------------------------------------
      008472                        976 _ssd1306_clear:
      008472 52 02            [ 2]  977 	sub	sp, #2
                                    978 ;	oled_ssd1306.c: 56: ssd1306_setpos(olednum,0,0);
      008474 4B 00            [ 1]  979 	push	#0x00
      008476 4B 00            [ 1]  980 	push	#0x00
      008478 7B 07            [ 1]  981 	ld	a, (0x07, sp)
      00847A 88               [ 1]  982 	push	a
      00847B CD 84 43         [ 4]  983 	call	_ssd1306_setpos
      00847E 5B 03            [ 2]  984 	addw	sp, #3
                                    985 ;	oled_ssd1306.c: 57: for(row=0;row<OLED_ROW+1;row++)	//Scan rows, add 1 row for completely flush.
      008480 0F 01            [ 1]  986 	clr	(0x01, sp)
                                    987 ;	oled_ssd1306.c: 59: for(col=0;col<OLED_COL;col++)	//Scan columns
      008482                        988 00109$:
      008482 0F 02            [ 1]  989 	clr	(0x02, sp)
      008484                        990 00103$:
                                    991 ;	oled_ssd1306.c: 61: ssd1306_senddat(olednum,0);	//Send 0 to every pixel
      008484 4B 00            [ 1]  992 	push	#0x00
      008486 7B 06            [ 1]  993 	ld	a, (0x06, sp)
      008488 88               [ 1]  994 	push	a
      008489 CD 84 35         [ 4]  995 	call	_ssd1306_senddat
      00848C 5B 02            [ 2]  996 	addw	sp, #2
                                    997 ;	oled_ssd1306.c: 59: for(col=0;col<OLED_COL;col++)	//Scan columns
      00848E 0C 02            [ 1]  998 	inc	(0x02, sp)
      008490 7B 02            [ 1]  999 	ld	a, (0x02, sp)
      008492 A1 80            [ 1] 1000 	cp	a, #0x80
      008494 25 EE            [ 1] 1001 	jrc	00103$
                                   1002 ;	oled_ssd1306.c: 57: for(row=0;row<OLED_ROW+1;row++)	//Scan rows, add 1 row for completely flush.
      008496 0C 01            [ 1] 1003 	inc	(0x01, sp)
      008498 7B 01            [ 1] 1004 	ld	a, (0x01, sp)
      00849A A1 09            [ 1] 1005 	cp	a, #0x09
      00849C 25 E4            [ 1] 1006 	jrc	00109$
      00849E 5B 02            [ 2] 1007 	addw	sp, #2
      0084A0 81               [ 4] 1008 	ret
                                   1009 ;	oled_ssd1306.c: 66: void OLED_setpos(unsigned char olednum, unsigned char row, unsigned char col)
                                   1010 ;	-----------------------------------------
                                   1011 ;	 function OLED_setpos
                                   1012 ;	-----------------------------------------
      0084A1                       1013 _OLED_setpos:
                                   1014 ;	oled_ssd1306.c: 68: ssd1306_setpos(olednum,row,col); //Set coordinate (for LCD_drawbyte)
      0084A1 7B 05            [ 1] 1015 	ld	a, (0x05, sp)
      0084A3 88               [ 1] 1016 	push	a
      0084A4 7B 05            [ 1] 1017 	ld	a, (0x05, sp)
      0084A6 88               [ 1] 1018 	push	a
      0084A7 7B 05            [ 1] 1019 	ld	a, (0x05, sp)
      0084A9 88               [ 1] 1020 	push	a
      0084AA CD 84 43         [ 4] 1021 	call	_ssd1306_setpos
      0084AD 5B 03            [ 2] 1022 	addw	sp, #3
      0084AF 81               [ 4] 1023 	ret
                                   1024 ;	oled_ssd1306.c: 71: void OLED_drawbyte(unsigned char olednum, unsigned char dbyte)
                                   1025 ;	-----------------------------------------
                                   1026 ;	 function OLED_drawbyte
                                   1027 ;	-----------------------------------------
      0084B0                       1028 _OLED_drawbyte:
                                   1029 ;	oled_ssd1306.c: 73: ssd1306_senddat(olednum,dbyte); //Send 1 byte data only
      0084B0 7B 04            [ 1] 1030 	ld	a, (0x04, sp)
      0084B2 88               [ 1] 1031 	push	a
      0084B3 7B 04            [ 1] 1032 	ld	a, (0x04, sp)
      0084B5 88               [ 1] 1033 	push	a
      0084B6 CD 84 35         [ 4] 1034 	call	_ssd1306_senddat
      0084B9 5B 02            [ 2] 1035 	addw	sp, #2
      0084BB 81               [ 4] 1036 	ret
                                   1037 ;	oled_ssd1306.c: 76: void OLED_drawchar(unsigned char olednum, unsigned char chr, unsigned char chrrow, unsigned char chrcol)
                                   1038 ;	-----------------------------------------
                                   1039 ;	 function OLED_drawchar
                                   1040 ;	-----------------------------------------
      0084BC                       1041 _OLED_drawchar:
      0084BC 52 0B            [ 2] 1042 	sub	sp, #11
                                   1043 ;	oled_ssd1306.c: 81: ssd1306_setpos(olednum,chrrow,chrcol);
      0084BE 7B 11            [ 1] 1044 	ld	a, (0x11, sp)
      0084C0 88               [ 1] 1045 	push	a
      0084C1 7B 11            [ 1] 1046 	ld	a, (0x11, sp)
      0084C3 88               [ 1] 1047 	push	a
      0084C4 7B 10            [ 1] 1048 	ld	a, (0x10, sp)
      0084C6 88               [ 1] 1049 	push	a
      0084C7 CD 84 43         [ 4] 1050 	call	_ssd1306_setpos
      0084CA 5B 03            [ 2] 1051 	addw	sp, #3
                                   1052 ;	oled_ssd1306.c: 86: chridx=(chr-32)*5; //Start at character 32 (Space). 5 columns for each character
      0084CC 7B 0F            [ 1] 1053 	ld	a, (0x0f, sp)
      0084CE 6B 03            [ 1] 1054 	ld	(0x03, sp), a
      0084D0 0F 02            [ 1] 1055 	clr	(0x02, sp)
                                   1056 ;	oled_ssd1306.c: 83: if((chr>31)&&(chr<128))	//Alphanumeric & Punctuation Area
      0084D2 7B 0F            [ 1] 1057 	ld	a, (0x0f, sp)
      0084D4 A1 1F            [ 1] 1058 	cp	a, #0x1f
      0084D6 23 47            [ 2] 1059 	jrule	00107$
      0084D8 7B 0F            [ 1] 1060 	ld	a, (0x0f, sp)
      0084DA A1 80            [ 1] 1061 	cp	a, #0x80
      0084DC 24 41            [ 1] 1062 	jrnc	00107$
                                   1063 ;	oled_ssd1306.c: 85: ssd1306_senddat(olednum,0x00);
      0084DE 4B 00            [ 1] 1064 	push	#0x00
      0084E0 7B 0F            [ 1] 1065 	ld	a, (0x0f, sp)
      0084E2 88               [ 1] 1066 	push	a
      0084E3 CD 84 35         [ 4] 1067 	call	_ssd1306_senddat
      0084E6 5B 02            [ 2] 1068 	addw	sp, #2
                                   1069 ;	oled_ssd1306.c: 86: chridx=(chr-32)*5; //Start at character 32 (Space). 5 columns for each character
      0084E8 1E 02            [ 2] 1070 	ldw	x, (0x02, sp)
      0084EA 1D 00 20         [ 2] 1071 	subw	x, #0x0020
      0084ED 89               [ 2] 1072 	pushw	x
      0084EE 4B 05            [ 1] 1073 	push	#0x05
      0084F0 4B 00            [ 1] 1074 	push	#0x00
      0084F2 CD 8E 77         [ 4] 1075 	call	__mulint
      0084F5 5B 04            [ 2] 1076 	addw	sp, #4
      0084F7 1F 0A            [ 2] 1077 	ldw	(0x0a, sp), x
                                   1078 ;	oled_ssd1306.c: 87: for(ci=0;ci<5;ci++)
      0084F9 AE 8B 4C         [ 2] 1079 	ldw	x, #_font_arr+0
      0084FC 1F 08            [ 2] 1080 	ldw	(0x08, sp), x
      0084FE 0F 01            [ 1] 1081 	clr	(0x01, sp)
      008500                       1082 00110$:
                                   1083 ;	oled_ssd1306.c: 89: fchar = font_arr[chridx+ci]; //Get character pattern from Font Array
      008500 7B 01            [ 1] 1084 	ld	a, (0x01, sp)
      008502 97               [ 1] 1085 	ld	xl, a
      008503 4F               [ 1] 1086 	clr	a
      008504 95               [ 1] 1087 	ld	xh, a
      008505 72 FB 0A         [ 2] 1088 	addw	x, (0x0a, sp)
      008508 72 FB 08         [ 2] 1089 	addw	x, (0x08, sp)
      00850B F6               [ 1] 1090 	ld	a, (x)
                                   1091 ;	oled_ssd1306.c: 90: ssd1306_senddat(olednum,fchar); //Send pattern 1 byte at a time
      00850C 88               [ 1] 1092 	push	a
      00850D 7B 0F            [ 1] 1093 	ld	a, (0x0f, sp)
      00850F 88               [ 1] 1094 	push	a
      008510 CD 84 35         [ 4] 1095 	call	_ssd1306_senddat
      008513 5B 02            [ 2] 1096 	addw	sp, #2
                                   1097 ;	oled_ssd1306.c: 87: for(ci=0;ci<5;ci++)
      008515 0C 01            [ 1] 1098 	inc	(0x01, sp)
      008517 7B 01            [ 1] 1099 	ld	a, (0x01, sp)
      008519 A1 05            [ 1] 1100 	cp	a, #0x05
      00851B 25 E3            [ 1] 1101 	jrc	00110$
      00851D 20 3D            [ 2] 1102 	jra	00114$
      00851F                       1103 00107$:
                                   1104 ;	oled_ssd1306.c: 93: else if((chr>127)&&(chr<148)) //Frame & Arrow Area
      00851F 7B 0F            [ 1] 1105 	ld	a, (0x0f, sp)
      008521 A1 7F            [ 1] 1106 	cp	a, #0x7f
      008523 23 37            [ 2] 1107 	jrule	00114$
      008525 7B 0F            [ 1] 1108 	ld	a, (0x0f, sp)
      008527 A1 94            [ 1] 1109 	cp	a, #0x94
      008529 24 31            [ 1] 1110 	jrnc	00114$
                                   1111 ;	oled_ssd1306.c: 95: chridx=(chr-128)*8; //Start at index 128. 5 columns for each symbol
      00852B 1E 02            [ 2] 1112 	ldw	x, (0x02, sp)
      00852D 1D 00 80         [ 2] 1113 	subw	x, #0x0080
      008530 58               [ 2] 1114 	sllw	x
      008531 58               [ 2] 1115 	sllw	x
      008532 58               [ 2] 1116 	sllw	x
                                   1117 ;	oled_ssd1306.c: 96: for(ci=0;ci<8;ci++)
      008533 90 AE 8B 4C      [ 2] 1118 	ldw	y, #_font_arr+0
      008537 17 06            [ 2] 1119 	ldw	(0x06, sp), y
      008539 1C 01 E0         [ 2] 1120 	addw	x, #0x01e0
      00853C 1F 04            [ 2] 1121 	ldw	(0x04, sp), x
      00853E 0F 01            [ 1] 1122 	clr	(0x01, sp)
      008540                       1123 00112$:
                                   1124 ;	oled_ssd1306.c: 98: fchar = font_arr[chridx+480+ci]; //Get symbol pattern from Font Array
      008540 5F               [ 1] 1125 	clrw	x
      008541 7B 01            [ 1] 1126 	ld	a, (0x01, sp)
      008543 97               [ 1] 1127 	ld	xl, a
      008544 72 FB 04         [ 2] 1128 	addw	x, (0x04, sp)
      008547 72 FB 06         [ 2] 1129 	addw	x, (0x06, sp)
      00854A F6               [ 1] 1130 	ld	a, (x)
                                   1131 ;	oled_ssd1306.c: 99: ssd1306_senddat(olednum,fchar); //Send pattern 1 byte at a time		   
      00854B 88               [ 1] 1132 	push	a
      00854C 7B 0F            [ 1] 1133 	ld	a, (0x0f, sp)
      00854E 88               [ 1] 1134 	push	a
      00854F CD 84 35         [ 4] 1135 	call	_ssd1306_senddat
      008552 5B 02            [ 2] 1136 	addw	sp, #2
                                   1137 ;	oled_ssd1306.c: 96: for(ci=0;ci<8;ci++)
      008554 0C 01            [ 1] 1138 	inc	(0x01, sp)
      008556 7B 01            [ 1] 1139 	ld	a, (0x01, sp)
      008558 A1 08            [ 1] 1140 	cp	a, #0x08
      00855A 25 E4            [ 1] 1141 	jrc	00112$
      00855C                       1142 00114$:
      00855C 5B 0B            [ 2] 1143 	addw	sp, #11
      00855E 81               [ 4] 1144 	ret
                                   1145 ;	oled_ssd1306.c: 105: void OLED_drawtext(unsigned char olednum, unsigned char *text, unsigned char txtrow, unsigned char txtcol)
                                   1146 ;	-----------------------------------------
                                   1147 ;	 function OLED_drawtext
                                   1148 ;	-----------------------------------------
      00855F                       1149 _OLED_drawtext:
      00855F 52 02            [ 2] 1150 	sub	sp, #2
                                   1151 ;	oled_ssd1306.c: 109: while(text[stridx] != 0) //Scan characters in string
      008561 5F               [ 1] 1152 	clrw	x
      008562 1F 01            [ 2] 1153 	ldw	(0x01, sp), x
      008564                       1154 00101$:
      008564 1E 06            [ 2] 1155 	ldw	x, (0x06, sp)
      008566 72 FB 01         [ 2] 1156 	addw	x, (0x01, sp)
      008569 F6               [ 1] 1157 	ld	a, (x)
      00856A 97               [ 1] 1158 	ld	xl, a
      00856B 4D               [ 1] 1159 	tnz	a
      00856C 27 1C            [ 1] 1160 	jreq	00104$
                                   1161 ;	oled_ssd1306.c: 111: OLED_drawchar(olednum,text[stridx],txtrow,txtcol+(8*stridx)); //Display each character
      00856E 7B 02            [ 1] 1162 	ld	a, (0x02, sp)
      008570 48               [ 1] 1163 	sll	a
      008571 48               [ 1] 1164 	sll	a
      008572 48               [ 1] 1165 	sll	a
      008573 1B 09            [ 1] 1166 	add	a, (0x09, sp)
      008575 88               [ 1] 1167 	push	a
      008576 7B 09            [ 1] 1168 	ld	a, (0x09, sp)
      008578 88               [ 1] 1169 	push	a
      008579 9F               [ 1] 1170 	ld	a, xl
      00857A 88               [ 1] 1171 	push	a
      00857B 7B 08            [ 1] 1172 	ld	a, (0x08, sp)
      00857D 88               [ 1] 1173 	push	a
      00857E CD 84 BC         [ 4] 1174 	call	_OLED_drawchar
      008581 5B 04            [ 2] 1175 	addw	sp, #4
                                   1176 ;	oled_ssd1306.c: 112: stridx++;
      008583 1E 01            [ 2] 1177 	ldw	x, (0x01, sp)
      008585 5C               [ 2] 1178 	incw	x
      008586 1F 01            [ 2] 1179 	ldw	(0x01, sp), x
      008588 20 DA            [ 2] 1180 	jra	00101$
      00858A                       1181 00104$:
      00858A 5B 02            [ 2] 1182 	addw	sp, #2
      00858C 81               [ 4] 1183 	ret
                                   1184 ;	oled_ssd1306.c: 116: void OLED_drawint(unsigned char olednum, unsigned int num, unsigned char numrow, unsigned char numcol)
                                   1185 ;	-----------------------------------------
                                   1186 ;	 function OLED_drawint
                                   1187 ;	-----------------------------------------
      00858D                       1188 _OLED_drawint:
      00858D 52 0C            [ 2] 1189 	sub	sp, #12
                                   1190 ;	oled_ssd1306.c: 123: numb = num;
      00858F 1E 10            [ 2] 1191 	ldw	x, (0x10, sp)
                                   1192 ;	oled_ssd1306.c: 124: while(numb!=0) //Counting digit
      008591 4F               [ 1] 1193 	clr	a
      008592                       1194 00101$:
      008592 5D               [ 2] 1195 	tnzw	x
      008593 27 08            [ 1] 1196 	jreq	00114$
                                   1197 ;	oled_ssd1306.c: 126: ndigit++;
      008595 4C               [ 1] 1198 	inc	a
                                   1199 ;	oled_ssd1306.c: 127: numb /= 10;	
      008596 90 AE 00 0A      [ 2] 1200 	ldw	y, #0x000a
      00859A 65               [ 2] 1201 	divw	x, y
      00859B 20 F5            [ 2] 1202 	jra	00101$
      00859D                       1203 00114$:
      00859D 6B 0B            [ 1] 1204 	ld	(0x0b, sp), a
                                   1205 ;	oled_ssd1306.c: 129: for(nd=0;nd<ndigit;nd++) //Converting each digit
      00859F 4F               [ 1] 1206 	clr	a
      0085A0 96               [ 1] 1207 	ldw	x, sp
      0085A1 5C               [ 2] 1208 	incw	x
      0085A2 1F 09            [ 2] 1209 	ldw	(0x09, sp), x
      0085A4                       1210 00106$:
      0085A4 11 0B            [ 1] 1211 	cp	a, (0x0b, sp)
      0085A6 24 27            [ 1] 1212 	jrnc	00104$
                                   1213 ;	oled_ssd1306.c: 131: numb = num%10;
      0085A8 1E 10            [ 2] 1214 	ldw	x, (0x10, sp)
      0085AA 90 AE 00 0A      [ 2] 1215 	ldw	y, #0x000a
      0085AE 65               [ 2] 1216 	divw	x, y
      0085AF 17 07            [ 2] 1217 	ldw	(0x07, sp), y
                                   1218 ;	oled_ssd1306.c: 132: num = num/10;
      0085B1 1E 10            [ 2] 1219 	ldw	x, (0x10, sp)
      0085B3 90 AE 00 0A      [ 2] 1220 	ldw	y, #0x000a
      0085B7 65               [ 2] 1221 	divw	x, y
      0085B8 1F 10            [ 2] 1222 	ldw	(0x10, sp), x
                                   1223 ;	oled_ssd1306.c: 133: ibuff[ndigit-(nd+1)] = numb + '0'; //Start from last_index-1
      0085BA 4C               [ 1] 1224 	inc	a
      0085BB 6B 0C            [ 1] 1225 	ld	(0x0c, sp), a
      0085BD 7B 0B            [ 1] 1226 	ld	a, (0x0b, sp)
      0085BF 10 0C            [ 1] 1227 	sub	a, (0x0c, sp)
      0085C1 5F               [ 1] 1228 	clrw	x
      0085C2 97               [ 1] 1229 	ld	xl, a
      0085C3 72 FB 09         [ 2] 1230 	addw	x, (0x09, sp)
      0085C6 7B 08            [ 1] 1231 	ld	a, (0x08, sp)
      0085C8 AB 30            [ 1] 1232 	add	a, #0x30
      0085CA F7               [ 1] 1233 	ld	(x), a
                                   1234 ;	oled_ssd1306.c: 129: for(nd=0;nd<ndigit;nd++) //Converting each digit
      0085CB 7B 0C            [ 1] 1235 	ld	a, (0x0c, sp)
      0085CD 20 D5            [ 2] 1236 	jra	00106$
      0085CF                       1237 00104$:
                                   1238 ;	oled_ssd1306.c: 135: ibuff[ndigit] = '\0'; //Last character is null
      0085CF 5F               [ 1] 1239 	clrw	x
      0085D0 7B 0B            [ 1] 1240 	ld	a, (0x0b, sp)
      0085D2 97               [ 1] 1241 	ld	xl, a
      0085D3 72 FB 09         [ 2] 1242 	addw	x, (0x09, sp)
      0085D6 7F               [ 1] 1243 	clr	(x)
                                   1244 ;	oled_ssd1306.c: 137: OLED_drawtext(olednum,ibuff,numrow,numcol); //Display number as text
      0085D7 1E 09            [ 2] 1245 	ldw	x, (0x09, sp)
      0085D9 7B 13            [ 1] 1246 	ld	a, (0x13, sp)
      0085DB 88               [ 1] 1247 	push	a
      0085DC 7B 13            [ 1] 1248 	ld	a, (0x13, sp)
      0085DE 88               [ 1] 1249 	push	a
      0085DF 89               [ 2] 1250 	pushw	x
      0085E0 7B 13            [ 1] 1251 	ld	a, (0x13, sp)
      0085E2 88               [ 1] 1252 	push	a
      0085E3 CD 85 5F         [ 4] 1253 	call	_OLED_drawtext
      0085E6 5B 11            [ 2] 1254 	addw	sp, #17
      0085E8 81               [ 4] 1255 	ret
                                   1256 ;	oled_ssd1306.c: 140: void OLED_clear(unsigned char olednum)
                                   1257 ;	-----------------------------------------
                                   1258 ;	 function OLED_clear
                                   1259 ;	-----------------------------------------
      0085E9                       1260 _OLED_clear:
                                   1261 ;	oled_ssd1306.c: 142: ssd1306_sendcom(olednum,0xAE); //Set Display off
      0085E9 4B AE            [ 1] 1262 	push	#0xae
      0085EB 7B 04            [ 1] 1263 	ld	a, (0x04, sp)
      0085ED 88               [ 1] 1264 	push	a
      0085EE CD 84 27         [ 4] 1265 	call	_ssd1306_sendcom
      0085F1 5B 02            [ 2] 1266 	addw	sp, #2
                                   1267 ;	oled_ssd1306.c: 143: ssd1306_clear(olednum); //Clear Display
      0085F3 7B 03            [ 1] 1268 	ld	a, (0x03, sp)
      0085F5 88               [ 1] 1269 	push	a
      0085F6 CD 84 72         [ 4] 1270 	call	_ssd1306_clear
      0085F9 84               [ 1] 1271 	pop	a
                                   1272 ;	oled_ssd1306.c: 144: ssd1306_sendcom(olednum,0xAF); //Set Display on
      0085FA 4B AF            [ 1] 1273 	push	#0xaf
      0085FC 7B 04            [ 1] 1274 	ld	a, (0x04, sp)
      0085FE 88               [ 1] 1275 	push	a
      0085FF CD 84 27         [ 4] 1276 	call	_ssd1306_sendcom
      008602 5B 02            [ 2] 1277 	addw	sp, #2
      008604 81               [ 4] 1278 	ret
                                   1279 ;	oled_ssd1306.c: 147: void OLED_clearblock(unsigned char olednum, unsigned char row, unsigned char col_start, unsigned char col_fin)
                                   1280 ;	-----------------------------------------
                                   1281 ;	 function OLED_clearblock
                                   1282 ;	-----------------------------------------
      008605                       1283 _OLED_clearblock:
      008605 88               [ 1] 1284 	push	a
                                   1285 ;	oled_ssd1306.c: 151: ssd1306_setpos(olednum,row,col_start); 	//Set start position
      008606 7B 06            [ 1] 1286 	ld	a, (0x06, sp)
      008608 88               [ 1] 1287 	push	a
      008609 7B 06            [ 1] 1288 	ld	a, (0x06, sp)
      00860B 88               [ 1] 1289 	push	a
      00860C 7B 06            [ 1] 1290 	ld	a, (0x06, sp)
      00860E 88               [ 1] 1291 	push	a
      00860F CD 84 43         [ 4] 1292 	call	_ssd1306_setpos
      008612 5B 03            [ 2] 1293 	addw	sp, #3
                                   1294 ;	oled_ssd1306.c: 152: for(col=col_start;col<=col_fin;col++) 	//Scan columns
      008614 7B 06            [ 1] 1295 	ld	a, (0x06, sp)
      008616 6B 01            [ 1] 1296 	ld	(0x01, sp), a
      008618                       1297 00103$:
      008618 7B 01            [ 1] 1298 	ld	a, (0x01, sp)
      00861A 11 07            [ 1] 1299 	cp	a, (0x07, sp)
      00861C 22 0E            [ 1] 1300 	jrugt	00105$
                                   1301 ;	oled_ssd1306.c: 154: ssd1306_senddat(olednum,0);	//Send 0 to every pixel in a column
      00861E 4B 00            [ 1] 1302 	push	#0x00
      008620 7B 05            [ 1] 1303 	ld	a, (0x05, sp)
      008622 88               [ 1] 1304 	push	a
      008623 CD 84 35         [ 4] 1305 	call	_ssd1306_senddat
      008626 5B 02            [ 2] 1306 	addw	sp, #2
                                   1307 ;	oled_ssd1306.c: 152: for(col=col_start;col<=col_fin;col++) 	//Scan columns
      008628 0C 01            [ 1] 1308 	inc	(0x01, sp)
      00862A 20 EC            [ 2] 1309 	jra	00103$
      00862C                       1310 00105$:
      00862C 84               [ 1] 1311 	pop	a
      00862D 81               [ 4] 1312 	ret
                                   1313 ;	oled_ssd1306.c: 158: void OLED_normal(unsigned char olednum)
                                   1314 ;	-----------------------------------------
                                   1315 ;	 function OLED_normal
                                   1316 ;	-----------------------------------------
      00862E                       1317 _OLED_normal:
                                   1318 ;	oled_ssd1306.c: 160: ssd1306_sendcom(olednum,0xA6);	//On Pixel in Off Background
      00862E 4B A6            [ 1] 1319 	push	#0xa6
      008630 7B 04            [ 1] 1320 	ld	a, (0x04, sp)
      008632 88               [ 1] 1321 	push	a
      008633 CD 84 27         [ 4] 1322 	call	_ssd1306_sendcom
      008636 5B 02            [ 2] 1323 	addw	sp, #2
      008638 81               [ 4] 1324 	ret
                                   1325 ;	oled_ssd1306.c: 163: void OLED_reverse(unsigned char olednum)
                                   1326 ;	-----------------------------------------
                                   1327 ;	 function OLED_reverse
                                   1328 ;	-----------------------------------------
      008639                       1329 _OLED_reverse:
                                   1330 ;	oled_ssd1306.c: 165: ssd1306_sendcom(olednum,0xA7);	//Off Pixel in On Background
      008639 4B A7            [ 1] 1331 	push	#0xa7
      00863B 7B 04            [ 1] 1332 	ld	a, (0x04, sp)
      00863D 88               [ 1] 1333 	push	a
      00863E CD 84 27         [ 4] 1334 	call	_ssd1306_sendcom
      008641 5B 02            [ 2] 1335 	addw	sp, #2
      008643 81               [ 4] 1336 	ret
                                   1337 ;	main.c: 28: int main()
                                   1338 ;	-----------------------------------------
                                   1339 ;	 function main
                                   1340 ;	-----------------------------------------
      008644                       1341 _main:
                                   1342 ;	main.c: 30: clock_init();
      008644 CD 81 4F         [ 4] 1343 	call	_clock_init
                                   1344 ;	main.c: 31: delay_init();
      008647 CD 80 A0         [ 4] 1345 	call	_delay_init
                                   1346 ;	main.c: 32: gpio_init();
      00864A CD 87 8F         [ 4] 1347 	call	_gpio_init
                                   1348 ;	main.c: 33: i2c_init();
      00864D CD 81 58         [ 4] 1349 	call	_i2c_init
                                   1350 ;	main.c: 35: ssd1306_init(OLED1);
      008650 4B 3C            [ 1] 1351 	push	#0x3c
      008652 CD 83 2C         [ 4] 1352 	call	_ssd1306_init
      008655 84               [ 1] 1353 	pop	a
                                   1354 ;	main.c: 36: ssd1306_init(OLED2);
      008656 4B 3D            [ 1] 1355 	push	#0x3d
      008658 CD 83 2C         [ 4] 1356 	call	_ssd1306_init
      00865B 84               [ 1] 1357 	pop	a
                                   1358 ;	main.c: 37: OLED_clear(OLED1);
      00865C 4B 3C            [ 1] 1359 	push	#0x3c
      00865E CD 85 E9         [ 4] 1360 	call	_OLED_clear
      008661 84               [ 1] 1361 	pop	a
                                   1362 ;	main.c: 38: OLED_clear(OLED2);
      008662 4B 3D            [ 1] 1363 	push	#0x3d
      008664 CD 85 E9         [ 4] 1364 	call	_OLED_clear
      008667 84               [ 1] 1365 	pop	a
                                   1366 ;	main.c: 40: drawLoadingBar(OLED1);
      008668 4B 3C            [ 1] 1367 	push	#0x3c
      00866A CD 8B 04         [ 4] 1368 	call	_drawLoadingBar
      00866D 84               [ 1] 1369 	pop	a
                                   1370 ;	main.c: 41: drawLoadingBar(OLED2);
      00866E 4B 3D            [ 1] 1371 	push	#0x3d
      008670 CD 8B 04         [ 4] 1372 	call	_drawLoadingBar
      008673 84               [ 1] 1373 	pop	a
                                   1374 ;	main.c: 43: loop();
      008674 CD 86 79         [ 4] 1375 	call	_loop
                                   1376 ;	main.c: 44: return 0;
      008677 5F               [ 1] 1377 	clrw	x
      008678 81               [ 4] 1378 	ret
                                   1379 ;	main.c: 50: void loop()
                                   1380 ;	-----------------------------------------
                                   1381 ;	 function loop
                                   1382 ;	-----------------------------------------
      008679                       1383 _loop:
                                   1384 ;	main.c: 52: while(OLED1)
      008679                       1385 00102$:
                                   1386 ;	main.c: 54: drawBytes(OLED1);
      008679 4B 3C            [ 1] 1387 	push	#0x3c
      00867B CD 8A 85         [ 4] 1388 	call	_drawBytes
      00867E 84               [ 1] 1389 	pop	a
                                   1390 ;	main.c: 55: delay_ms(1000);
      00867F 4B E8            [ 1] 1391 	push	#0xe8
      008681 4B 03            [ 1] 1392 	push	#0x03
      008683 5F               [ 1] 1393 	clrw	x
      008684 89               [ 2] 1394 	pushw	x
      008685 CD 80 F5         [ 4] 1395 	call	_delay_ms
      008688 5B 04            [ 2] 1396 	addw	sp, #4
                                   1397 ;	main.c: 56: OLED_clearblock(OLED1,3,5,114); //Finish column = 5 + 11*10 - 1
      00868A 4B 72            [ 1] 1398 	push	#0x72
      00868C 4B 05            [ 1] 1399 	push	#0x05
      00868E 4B 03            [ 1] 1400 	push	#0x03
      008690 4B 3C            [ 1] 1401 	push	#0x3c
      008692 CD 86 05         [ 4] 1402 	call	_OLED_clearblock
      008695 5B 04            [ 2] 1403 	addw	sp, #4
                                   1404 ;	main.c: 57: delay_ms(500);
      008697 4B F4            [ 1] 1405 	push	#0xf4
      008699 4B 01            [ 1] 1406 	push	#0x01
      00869B 5F               [ 1] 1407 	clrw	x
      00869C 89               [ 2] 1408 	pushw	x
      00869D CD 80 F5         [ 4] 1409 	call	_delay_ms
      0086A0 5B 04            [ 2] 1410 	addw	sp, #4
                                   1411 ;	main.c: 58: OLED_clearblock(OLED1,5,3,114); //Finish column = 3 + 8*14 - 1
      0086A2 4B 72            [ 1] 1412 	push	#0x72
      0086A4 4B 03            [ 1] 1413 	push	#0x03
      0086A6 4B 05            [ 1] 1414 	push	#0x05
      0086A8 4B 3C            [ 1] 1415 	push	#0x3c
      0086AA CD 86 05         [ 4] 1416 	call	_OLED_clearblock
      0086AD 5B 04            [ 2] 1417 	addw	sp, #4
                                   1418 ;	main.c: 59: delay_ms(500);
      0086AF 4B F4            [ 1] 1419 	push	#0xf4
      0086B1 4B 01            [ 1] 1420 	push	#0x01
      0086B3 5F               [ 1] 1421 	clrw	x
      0086B4 89               [ 2] 1422 	pushw	x
      0086B5 CD 80 F5         [ 4] 1423 	call	_delay_ms
      0086B8 5B 04            [ 2] 1424 	addw	sp, #4
                                   1425 ;	main.c: 61: drawInt(OLED2);
      0086BA 4B 3D            [ 1] 1426 	push	#0x3d
      0086BC CD 87 90         [ 4] 1427 	call	_drawInt
      0086BF 84               [ 1] 1428 	pop	a
                                   1429 ;	main.c: 62: delay_ms(1000); 
      0086C0 4B E8            [ 1] 1430 	push	#0xe8
      0086C2 4B 03            [ 1] 1431 	push	#0x03
      0086C4 5F               [ 1] 1432 	clrw	x
      0086C5 89               [ 2] 1433 	pushw	x
      0086C6 CD 80 F5         [ 4] 1434 	call	_delay_ms
      0086C9 5B 04            [ 2] 1435 	addw	sp, #4
                                   1436 ;	main.c: 63: OLED_clear(OLED2);
      0086CB 4B 3D            [ 1] 1437 	push	#0x3d
      0086CD CD 85 E9         [ 4] 1438 	call	_OLED_clear
      0086D0 84               [ 1] 1439 	pop	a
                                   1440 ;	main.c: 65: drawAlphanum(OLED1);
      0086D1 4B 3C            [ 1] 1441 	push	#0x3c
      0086D3 CD 88 1D         [ 4] 1442 	call	_drawAlphanum
      0086D6 84               [ 1] 1443 	pop	a
                                   1444 ;	main.c: 66: delay_ms(1000); 
      0086D7 4B E8            [ 1] 1445 	push	#0xe8
      0086D9 4B 03            [ 1] 1446 	push	#0x03
      0086DB 5F               [ 1] 1447 	clrw	x
      0086DC 89               [ 2] 1448 	pushw	x
      0086DD CD 80 F5         [ 4] 1449 	call	_delay_ms
      0086E0 5B 04            [ 2] 1450 	addw	sp, #4
                                   1451 ;	main.c: 67: OLED_reverse(OLED1);
      0086E2 4B 3C            [ 1] 1452 	push	#0x3c
      0086E4 CD 86 39         [ 4] 1453 	call	_OLED_reverse
      0086E7 84               [ 1] 1454 	pop	a
                                   1455 ;	main.c: 68: delay_ms(1000);
      0086E8 4B E8            [ 1] 1456 	push	#0xe8
      0086EA 4B 03            [ 1] 1457 	push	#0x03
      0086EC 5F               [ 1] 1458 	clrw	x
      0086ED 89               [ 2] 1459 	pushw	x
      0086EE CD 80 F5         [ 4] 1460 	call	_delay_ms
      0086F1 5B 04            [ 2] 1461 	addw	sp, #4
                                   1462 ;	main.c: 69: OLED_clear(OLED1);
      0086F3 4B 3C            [ 1] 1463 	push	#0x3c
      0086F5 CD 85 E9         [ 4] 1464 	call	_OLED_clear
      0086F8 84               [ 1] 1465 	pop	a
                                   1466 ;	main.c: 70: OLED_normal(OLED1);
      0086F9 4B 3C            [ 1] 1467 	push	#0x3c
      0086FB CD 86 2E         [ 4] 1468 	call	_OLED_normal
      0086FE 84               [ 1] 1469 	pop	a
                                   1470 ;	main.c: 72: drawPunct(OLED2);
      0086FF 4B 3D            [ 1] 1471 	push	#0x3d
      008701 CD 88 7E         [ 4] 1472 	call	_drawPunct
      008704 84               [ 1] 1473 	pop	a
                                   1474 ;	main.c: 73: delay_ms(1000); 
      008705 4B E8            [ 1] 1475 	push	#0xe8
      008707 4B 03            [ 1] 1476 	push	#0x03
      008709 5F               [ 1] 1477 	clrw	x
      00870A 89               [ 2] 1478 	pushw	x
      00870B CD 80 F5         [ 4] 1479 	call	_delay_ms
      00870E 5B 04            [ 2] 1480 	addw	sp, #4
                                   1481 ;	main.c: 74: OLED_reverse(OLED2);
      008710 4B 3D            [ 1] 1482 	push	#0x3d
      008712 CD 86 39         [ 4] 1483 	call	_OLED_reverse
      008715 84               [ 1] 1484 	pop	a
                                   1485 ;	main.c: 75: delay_ms(1000);
      008716 4B E8            [ 1] 1486 	push	#0xe8
      008718 4B 03            [ 1] 1487 	push	#0x03
      00871A 5F               [ 1] 1488 	clrw	x
      00871B 89               [ 2] 1489 	pushw	x
      00871C CD 80 F5         [ 4] 1490 	call	_delay_ms
      00871F 5B 04            [ 2] 1491 	addw	sp, #4
                                   1492 ;	main.c: 76: OLED_clear(OLED2);
      008721 4B 3D            [ 1] 1493 	push	#0x3d
      008723 CD 85 E9         [ 4] 1494 	call	_OLED_clear
      008726 84               [ 1] 1495 	pop	a
                                   1496 ;	main.c: 77: OLED_normal(OLED2);
      008727 4B 3D            [ 1] 1497 	push	#0x3d
      008729 CD 86 2E         [ 4] 1498 	call	_OLED_normal
      00872C 84               [ 1] 1499 	pop	a
                                   1500 ;	main.c: 79: drawFrame(OLED1);
      00872D 4B 3C            [ 1] 1501 	push	#0x3c
      00872F CD 88 BF         [ 4] 1502 	call	_drawFrame
      008732 84               [ 1] 1503 	pop	a
                                   1504 ;	main.c: 80: delay_ms(700); 
      008733 4B BC            [ 1] 1505 	push	#0xbc
      008735 4B 02            [ 1] 1506 	push	#0x02
      008737 5F               [ 1] 1507 	clrw	x
      008738 89               [ 2] 1508 	pushw	x
      008739 CD 80 F5         [ 4] 1509 	call	_delay_ms
      00873C 5B 04            [ 2] 1510 	addw	sp, #4
                                   1511 ;	main.c: 81: OLED_clearblock(OLED1,3,36,43); //Finish column = 36 + 8 - 1
      00873E 4B 2B            [ 1] 1512 	push	#0x2b
      008740 4B 24            [ 1] 1513 	push	#0x24
      008742 4B 03            [ 1] 1514 	push	#0x03
      008744 4B 3C            [ 1] 1515 	push	#0x3c
      008746 CD 86 05         [ 4] 1516 	call	_OLED_clearblock
      008749 5B 04            [ 2] 1517 	addw	sp, #4
                                   1518 ;	main.c: 82: delay_ms(700);
      00874B 4B BC            [ 1] 1519 	push	#0xbc
      00874D 4B 02            [ 1] 1520 	push	#0x02
      00874F 5F               [ 1] 1521 	clrw	x
      008750 89               [ 2] 1522 	pushw	x
      008751 CD 80 F5         [ 4] 1523 	call	_delay_ms
      008754 5B 04            [ 2] 1524 	addw	sp, #4
                                   1525 ;	main.c: 83: OLED_clear(OLED1);
      008756 4B 3C            [ 1] 1526 	push	#0x3c
      008758 CD 85 E9         [ 4] 1527 	call	_OLED_clear
      00875B 84               [ 1] 1528 	pop	a
                                   1529 ;	main.c: 85: drawArrow(OLED2);
      00875C 4B 3D            [ 1] 1530 	push	#0x3d
      00875E CD 89 F6         [ 4] 1531 	call	_drawArrow
      008761 84               [ 1] 1532 	pop	a
                                   1533 ;	main.c: 86: delay_ms(700); 
      008762 4B BC            [ 1] 1534 	push	#0xbc
      008764 4B 02            [ 1] 1535 	push	#0x02
      008766 5F               [ 1] 1536 	clrw	x
      008767 89               [ 2] 1537 	pushw	x
      008768 CD 80 F5         [ 4] 1538 	call	_delay_ms
      00876B 5B 04            [ 2] 1539 	addw	sp, #4
                                   1540 ;	main.c: 87: OLED_clearblock(OLED2,3,36,43); //Finish column = 36 + 8 - 1
      00876D 4B 2B            [ 1] 1541 	push	#0x2b
      00876F 4B 24            [ 1] 1542 	push	#0x24
      008771 4B 03            [ 1] 1543 	push	#0x03
      008773 4B 3D            [ 1] 1544 	push	#0x3d
      008775 CD 86 05         [ 4] 1545 	call	_OLED_clearblock
      008778 5B 04            [ 2] 1546 	addw	sp, #4
                                   1547 ;	main.c: 88: delay_ms(700);
      00877A 4B BC            [ 1] 1548 	push	#0xbc
      00877C 4B 02            [ 1] 1549 	push	#0x02
      00877E 5F               [ 1] 1550 	clrw	x
      00877F 89               [ 2] 1551 	pushw	x
      008780 CD 80 F5         [ 4] 1552 	call	_delay_ms
      008783 5B 04            [ 2] 1553 	addw	sp, #4
                                   1554 ;	main.c: 89: OLED_clear(OLED2);
      008785 4B 3D            [ 1] 1555 	push	#0x3d
      008787 CD 85 E9         [ 4] 1556 	call	_OLED_clear
      00878A 84               [ 1] 1557 	pop	a
      00878B CC 86 79         [ 2] 1558 	jp	00102$
      00878E 81               [ 4] 1559 	ret
                                   1560 ;	main.c: 94: void gpio_init()
                                   1561 ;	-----------------------------------------
                                   1562 ;	 function gpio_init
                                   1563 ;	-----------------------------------------
      00878F                       1564 _gpio_init:
                                   1565 ;	main.c: 97: }
      00878F 81               [ 4] 1566 	ret
                                   1567 ;	main.c: 99: void drawInt(unsigned char olednum)
                                   1568 ;	-----------------------------------------
                                   1569 ;	 function drawInt
                                   1570 ;	-----------------------------------------
      008790                       1571 _drawInt:
                                   1572 ;	main.c: 101: OLED_drawint(olednum, 64, 0, 8);   //Decimal
      008790 4B 08            [ 1] 1573 	push	#0x08
      008792 4B 00            [ 1] 1574 	push	#0x00
      008794 4B 40            [ 1] 1575 	push	#0x40
      008796 4B 00            [ 1] 1576 	push	#0x00
      008798 7B 07            [ 1] 1577 	ld	a, (0x07, sp)
      00879A 88               [ 1] 1578 	push	a
      00879B CD 85 8D         [ 4] 1579 	call	_OLED_drawint
      00879E 5B 05            [ 2] 1580 	addw	sp, #5
                                   1581 ;	main.c: 102: OLED_drawint(olednum, 064, 0, 48); //Octal displayed as Decimal
      0087A0 4B 30            [ 1] 1582 	push	#0x30
      0087A2 4B 00            [ 1] 1583 	push	#0x00
      0087A4 4B 34            [ 1] 1584 	push	#0x34
      0087A6 4B 00            [ 1] 1585 	push	#0x00
      0087A8 7B 07            [ 1] 1586 	ld	a, (0x07, sp)
      0087AA 88               [ 1] 1587 	push	a
      0087AB CD 85 8D         [ 4] 1588 	call	_OLED_drawint
      0087AE 5B 05            [ 2] 1589 	addw	sp, #5
                                   1590 ;	main.c: 103: OLED_drawint(olednum, 0x64, 0, 88); //Hexadecimal displayed as Decimal
      0087B0 4B 58            [ 1] 1591 	push	#0x58
      0087B2 4B 00            [ 1] 1592 	push	#0x00
      0087B4 4B 64            [ 1] 1593 	push	#0x64
      0087B6 4B 00            [ 1] 1594 	push	#0x00
      0087B8 7B 07            [ 1] 1595 	ld	a, (0x07, sp)
      0087BA 88               [ 1] 1596 	push	a
      0087BB CD 85 8D         [ 4] 1597 	call	_OLED_drawint
      0087BE 5B 05            [ 2] 1598 	addw	sp, #5
                                   1599 ;	main.c: 105: OLED_drawint(olednum, -64, 1, 8); //Negative number is not supported
      0087C0 4B 08            [ 1] 1600 	push	#0x08
      0087C2 4B 01            [ 1] 1601 	push	#0x01
      0087C4 4B C0            [ 1] 1602 	push	#0xc0
      0087C6 4B FF            [ 1] 1603 	push	#0xff
      0087C8 7B 07            [ 1] 1604 	ld	a, (0x07, sp)
      0087CA 88               [ 1] 1605 	push	a
      0087CB CD 85 8D         [ 4] 1606 	call	_OLED_drawint
      0087CE 5B 05            [ 2] 1607 	addw	sp, #5
                                   1608 ;	main.c: 108: OLED_drawint(olednum, 65535, 3, 8); //Max. is 65535
      0087D0 4B 08            [ 1] 1609 	push	#0x08
      0087D2 4B 03            [ 1] 1610 	push	#0x03
      0087D4 4B FF            [ 1] 1611 	push	#0xff
      0087D6 4B FF            [ 1] 1612 	push	#0xff
      0087D8 7B 07            [ 1] 1613 	ld	a, (0x07, sp)
      0087DA 88               [ 1] 1614 	push	a
      0087DB CD 85 8D         [ 4] 1615 	call	_OLED_drawint
      0087DE 5B 05            [ 2] 1616 	addw	sp, #5
                                   1617 ;	main.c: 110: OLED_drawint(olednum, 100, 5, 8);
      0087E0 4B 08            [ 1] 1618 	push	#0x08
      0087E2 4B 05            [ 1] 1619 	push	#0x05
      0087E4 4B 64            [ 1] 1620 	push	#0x64
      0087E6 4B 00            [ 1] 1621 	push	#0x00
      0087E8 7B 07            [ 1] 1622 	ld	a, (0x07, sp)
      0087EA 88               [ 1] 1623 	push	a
      0087EB CD 85 8D         [ 4] 1624 	call	_OLED_drawint
      0087EE 5B 05            [ 2] 1625 	addw	sp, #5
                                   1626 ;	main.c: 111: OLED_drawchar(olednum, SYM_DEGREE, 5, 32);
      0087F0 4B 20            [ 1] 1627 	push	#0x20
      0087F2 4B 05            [ 1] 1628 	push	#0x05
      0087F4 4B 7F            [ 1] 1629 	push	#0x7f
      0087F6 7B 06            [ 1] 1630 	ld	a, (0x06, sp)
      0087F8 88               [ 1] 1631 	push	a
      0087F9 CD 84 BC         [ 4] 1632 	call	_OLED_drawchar
      0087FC 5B 04            [ 2] 1633 	addw	sp, #4
                                   1634 ;	main.c: 112: OLED_drawchar(olednum, 'C', 5, 40);
      0087FE 4B 28            [ 1] 1635 	push	#0x28
      008800 4B 05            [ 1] 1636 	push	#0x05
      008802 4B 43            [ 1] 1637 	push	#0x43
      008804 7B 06            [ 1] 1638 	ld	a, (0x06, sp)
      008806 88               [ 1] 1639 	push	a
      008807 CD 84 BC         [ 4] 1640 	call	_OLED_drawchar
      00880A 5B 04            [ 2] 1641 	addw	sp, #4
                                   1642 ;	main.c: 114: OLED_drawtext(olednum, " OLED TEST : INT",7,0);
      00880C AE 8D CC         [ 2] 1643 	ldw	x, #___str_0+0
      00880F 4B 00            [ 1] 1644 	push	#0x00
      008811 4B 07            [ 1] 1645 	push	#0x07
      008813 89               [ 2] 1646 	pushw	x
      008814 7B 07            [ 1] 1647 	ld	a, (0x07, sp)
      008816 88               [ 1] 1648 	push	a
      008817 CD 85 5F         [ 4] 1649 	call	_OLED_drawtext
      00881A 5B 05            [ 2] 1650 	addw	sp, #5
      00881C 81               [ 4] 1651 	ret
                                   1652 ;	main.c: 117: void drawAlphanum(unsigned char olednum)
                                   1653 ;	-----------------------------------------
                                   1654 ;	 function drawAlphanum
                                   1655 ;	-----------------------------------------
      00881D                       1656 _drawAlphanum:
                                   1657 ;	main.c: 119: OLED_drawtext(olednum, "ABCDEFGHIJKLM",0,0);
      00881D AE 8D DD         [ 2] 1658 	ldw	x, #___str_1+0
      008820 4B 00            [ 1] 1659 	push	#0x00
      008822 4B 00            [ 1] 1660 	push	#0x00
      008824 89               [ 2] 1661 	pushw	x
      008825 7B 07            [ 1] 1662 	ld	a, (0x07, sp)
      008827 88               [ 1] 1663 	push	a
      008828 CD 85 5F         [ 4] 1664 	call	_OLED_drawtext
      00882B 5B 05            [ 2] 1665 	addw	sp, #5
                                   1666 ;	main.c: 120: OLED_drawtext(olednum, "NOPQRSTUVWXYZ",1,0);
      00882D AE 8D EB         [ 2] 1667 	ldw	x, #___str_2+0
      008830 4B 00            [ 1] 1668 	push	#0x00
      008832 4B 01            [ 1] 1669 	push	#0x01
      008834 89               [ 2] 1670 	pushw	x
      008835 7B 07            [ 1] 1671 	ld	a, (0x07, sp)
      008837 88               [ 1] 1672 	push	a
      008838 CD 85 5F         [ 4] 1673 	call	_OLED_drawtext
      00883B 5B 05            [ 2] 1674 	addw	sp, #5
                                   1675 ;	main.c: 122: OLED_drawtext(olednum, "abcdefghijklm",3,0);
      00883D AE 8D F9         [ 2] 1676 	ldw	x, #___str_3+0
      008840 4B 00            [ 1] 1677 	push	#0x00
      008842 4B 03            [ 1] 1678 	push	#0x03
      008844 89               [ 2] 1679 	pushw	x
      008845 7B 07            [ 1] 1680 	ld	a, (0x07, sp)
      008847 88               [ 1] 1681 	push	a
      008848 CD 85 5F         [ 4] 1682 	call	_OLED_drawtext
      00884B 5B 05            [ 2] 1683 	addw	sp, #5
                                   1684 ;	main.c: 123: OLED_drawtext(olednum, "nopqrstuvwxyz",4,0);
      00884D AE 8E 07         [ 2] 1685 	ldw	x, #___str_4+0
      008850 4B 00            [ 1] 1686 	push	#0x00
      008852 4B 04            [ 1] 1687 	push	#0x04
      008854 89               [ 2] 1688 	pushw	x
      008855 7B 07            [ 1] 1689 	ld	a, (0x07, sp)
      008857 88               [ 1] 1690 	push	a
      008858 CD 85 5F         [ 4] 1691 	call	_OLED_drawtext
      00885B 5B 05            [ 2] 1692 	addw	sp, #5
                                   1693 ;	main.c: 125: OLED_drawtext(olednum, "0123456789",6,0);
      00885D AE 8E 15         [ 2] 1694 	ldw	x, #___str_5+0
      008860 4B 00            [ 1] 1695 	push	#0x00
      008862 4B 06            [ 1] 1696 	push	#0x06
      008864 89               [ 2] 1697 	pushw	x
      008865 7B 07            [ 1] 1698 	ld	a, (0x07, sp)
      008867 88               [ 1] 1699 	push	a
      008868 CD 85 5F         [ 4] 1700 	call	_OLED_drawtext
      00886B 5B 05            [ 2] 1701 	addw	sp, #5
                                   1702 ;	main.c: 127: OLED_drawtext(olednum, "OLED TEST : CHAR",7,0);
      00886D AE 8E 20         [ 2] 1703 	ldw	x, #___str_6+0
      008870 4B 00            [ 1] 1704 	push	#0x00
      008872 4B 07            [ 1] 1705 	push	#0x07
      008874 89               [ 2] 1706 	pushw	x
      008875 7B 07            [ 1] 1707 	ld	a, (0x07, sp)
      008877 88               [ 1] 1708 	push	a
      008878 CD 85 5F         [ 4] 1709 	call	_OLED_drawtext
      00887B 5B 05            [ 2] 1710 	addw	sp, #5
      00887D 81               [ 4] 1711 	ret
                                   1712 ;	main.c: 130: void drawPunct(unsigned char olednum)
                                   1713 ;	-----------------------------------------
                                   1714 ;	 function drawPunct
                                   1715 ;	-----------------------------------------
      00887E                       1716 _drawPunct:
                                   1717 ;	main.c: 132: OLED_drawtext(olednum, "<{([+_-=])}>",0,0);
      00887E AE 8E 31         [ 2] 1718 	ldw	x, #___str_7+0
      008881 4B 00            [ 1] 1719 	push	#0x00
      008883 4B 00            [ 1] 1720 	push	#0x00
      008885 89               [ 2] 1721 	pushw	x
      008886 7B 07            [ 1] 1722 	ld	a, (0x07, sp)
      008888 88               [ 1] 1723 	push	a
      008889 CD 85 5F         [ 4] 1724 	call	_OLED_drawtext
      00888C 5B 05            [ 2] 1725 	addw	sp, #5
                                   1726 ;	main.c: 133: OLED_drawtext(olednum, "!@#$%^&*`|~?",2,0);
      00888E AE 8E 3E         [ 2] 1727 	ldw	x, #___str_8+0
      008891 4B 00            [ 1] 1728 	push	#0x00
      008893 4B 02            [ 1] 1729 	push	#0x02
      008895 89               [ 2] 1730 	pushw	x
      008896 7B 07            [ 1] 1731 	ld	a, (0x07, sp)
      008898 88               [ 1] 1732 	push	a
      008899 CD 85 5F         [ 4] 1733 	call	_OLED_drawtext
      00889C 5B 05            [ 2] 1734 	addw	sp, #5
                                   1735 ;	main.c: 134: OLED_drawtext(olednum, ".\,\"\'\\/ :;",4,0);
      00889E AE 8E 4B         [ 2] 1736 	ldw	x, #___str_9+0
      0088A1 4B 00            [ 1] 1737 	push	#0x00
      0088A3 4B 04            [ 1] 1738 	push	#0x04
      0088A5 89               [ 2] 1739 	pushw	x
      0088A6 7B 07            [ 1] 1740 	ld	a, (0x07, sp)
      0088A8 88               [ 1] 1741 	push	a
      0088A9 CD 85 5F         [ 4] 1742 	call	_OLED_drawtext
      0088AC 5B 05            [ 2] 1743 	addw	sp, #5
                                   1744 ;	main.c: 136: OLED_drawtext(olednum, "OLED TEST : CHAR",7,0);
      0088AE AE 8E 20         [ 2] 1745 	ldw	x, #___str_6+0
      0088B1 4B 00            [ 1] 1746 	push	#0x00
      0088B3 4B 07            [ 1] 1747 	push	#0x07
      0088B5 89               [ 2] 1748 	pushw	x
      0088B6 7B 07            [ 1] 1749 	ld	a, (0x07, sp)
      0088B8 88               [ 1] 1750 	push	a
      0088B9 CD 85 5F         [ 4] 1751 	call	_OLED_drawtext
      0088BC 5B 05            [ 2] 1752 	addw	sp, #5
      0088BE 81               [ 4] 1753 	ret
                                   1754 ;	main.c: 139: void drawFrame(unsigned char olednum)
                                   1755 ;	-----------------------------------------
                                   1756 ;	 function drawFrame
                                   1757 ;	-----------------------------------------
      0088BF                       1758 _drawFrame:
                                   1759 ;	main.c: 143: OLED_drawchar(olednum, FRAME_TOP_LEFT,1,startcol);
      0088BF 4B 14            [ 1] 1760 	push	#0x14
      0088C1 4B 01            [ 1] 1761 	push	#0x01
      0088C3 4B 80            [ 1] 1762 	push	#0x80
      0088C5 7B 06            [ 1] 1763 	ld	a, (0x06, sp)
      0088C7 88               [ 1] 1764 	push	a
      0088C8 CD 84 BC         [ 4] 1765 	call	_OLED_drawchar
      0088CB 5B 04            [ 2] 1766 	addw	sp, #4
                                   1767 ;	main.c: 144: OLED_drawchar(olednum, FRAME_LINE_HOR,1,startcol+8);
      0088CD 4B 1C            [ 1] 1768 	push	#0x1c
      0088CF 4B 01            [ 1] 1769 	push	#0x01
      0088D1 4B 89            [ 1] 1770 	push	#0x89
      0088D3 7B 06            [ 1] 1771 	ld	a, (0x06, sp)
      0088D5 88               [ 1] 1772 	push	a
      0088D6 CD 84 BC         [ 4] 1773 	call	_OLED_drawchar
      0088D9 5B 04            [ 2] 1774 	addw	sp, #4
                                   1775 ;	main.c: 145: OLED_drawchar(olednum, FRAME_TOP,1,startcol+16);
      0088DB 4B 24            [ 1] 1776 	push	#0x24
      0088DD 4B 01            [ 1] 1777 	push	#0x01
      0088DF 4B 81            [ 1] 1778 	push	#0x81
      0088E1 7B 06            [ 1] 1779 	ld	a, (0x06, sp)
      0088E3 88               [ 1] 1780 	push	a
      0088E4 CD 84 BC         [ 4] 1781 	call	_OLED_drawchar
      0088E7 5B 04            [ 2] 1782 	addw	sp, #4
                                   1783 ;	main.c: 146: OLED_drawchar(olednum, FRAME_LINE_HOR,1,startcol+24);
      0088E9 4B 2C            [ 1] 1784 	push	#0x2c
      0088EB 4B 01            [ 1] 1785 	push	#0x01
      0088ED 4B 89            [ 1] 1786 	push	#0x89
      0088EF 7B 06            [ 1] 1787 	ld	a, (0x06, sp)
      0088F1 88               [ 1] 1788 	push	a
      0088F2 CD 84 BC         [ 4] 1789 	call	_OLED_drawchar
      0088F5 5B 04            [ 2] 1790 	addw	sp, #4
                                   1791 ;	main.c: 147: OLED_drawchar(olednum, FRAME_TOP_RIGHT,1,startcol+32);
      0088F7 4B 34            [ 1] 1792 	push	#0x34
      0088F9 4B 01            [ 1] 1793 	push	#0x01
      0088FB 4B 82            [ 1] 1794 	push	#0x82
      0088FD 7B 06            [ 1] 1795 	ld	a, (0x06, sp)
      0088FF 88               [ 1] 1796 	push	a
      008900 CD 84 BC         [ 4] 1797 	call	_OLED_drawchar
      008903 5B 04            [ 2] 1798 	addw	sp, #4
                                   1799 ;	main.c: 149: OLED_drawchar(olednum, FRAME_LINE_VER,2,startcol);
      008905 4B 14            [ 1] 1800 	push	#0x14
      008907 4B 02            [ 1] 1801 	push	#0x02
      008909 4B 8A            [ 1] 1802 	push	#0x8a
      00890B 7B 06            [ 1] 1803 	ld	a, (0x06, sp)
      00890D 88               [ 1] 1804 	push	a
      00890E CD 84 BC         [ 4] 1805 	call	_OLED_drawchar
      008911 5B 04            [ 2] 1806 	addw	sp, #4
                                   1807 ;	main.c: 150: OLED_drawchar(olednum, FRAME_LINE_VER,2,startcol+16);
      008913 4B 24            [ 1] 1808 	push	#0x24
      008915 4B 02            [ 1] 1809 	push	#0x02
      008917 4B 8A            [ 1] 1810 	push	#0x8a
      008919 7B 06            [ 1] 1811 	ld	a, (0x06, sp)
      00891B 88               [ 1] 1812 	push	a
      00891C CD 84 BC         [ 4] 1813 	call	_OLED_drawchar
      00891F 5B 04            [ 2] 1814 	addw	sp, #4
                                   1815 ;	main.c: 151: OLED_drawchar(olednum, FRAME_LINE_VER,2,startcol+32);
      008921 4B 34            [ 1] 1816 	push	#0x34
      008923 4B 02            [ 1] 1817 	push	#0x02
      008925 4B 8A            [ 1] 1818 	push	#0x8a
      008927 7B 06            [ 1] 1819 	ld	a, (0x06, sp)
      008929 88               [ 1] 1820 	push	a
      00892A CD 84 BC         [ 4] 1821 	call	_OLED_drawchar
      00892D 5B 04            [ 2] 1822 	addw	sp, #4
                                   1823 ;	main.c: 153: OLED_drawchar(olednum, FRAME_MID_LEFT,3,startcol);
      00892F 4B 14            [ 1] 1824 	push	#0x14
      008931 4B 03            [ 1] 1825 	push	#0x03
      008933 4B 83            [ 1] 1826 	push	#0x83
      008935 7B 06            [ 1] 1827 	ld	a, (0x06, sp)
      008937 88               [ 1] 1828 	push	a
      008938 CD 84 BC         [ 4] 1829 	call	_OLED_drawchar
      00893B 5B 04            [ 2] 1830 	addw	sp, #4
                                   1831 ;	main.c: 154: OLED_drawchar(olednum, FRAME_LINE_HOR,3,startcol+8);
      00893D 4B 1C            [ 1] 1832 	push	#0x1c
      00893F 4B 03            [ 1] 1833 	push	#0x03
      008941 4B 89            [ 1] 1834 	push	#0x89
      008943 7B 06            [ 1] 1835 	ld	a, (0x06, sp)
      008945 88               [ 1] 1836 	push	a
      008946 CD 84 BC         [ 4] 1837 	call	_OLED_drawchar
      008949 5B 04            [ 2] 1838 	addw	sp, #4
                                   1839 ;	main.c: 155: OLED_drawchar(olednum, FRAME_CENTER,3,startcol+16);
      00894B 4B 24            [ 1] 1840 	push	#0x24
      00894D 4B 03            [ 1] 1841 	push	#0x03
      00894F 4B 84            [ 1] 1842 	push	#0x84
      008951 7B 06            [ 1] 1843 	ld	a, (0x06, sp)
      008953 88               [ 1] 1844 	push	a
      008954 CD 84 BC         [ 4] 1845 	call	_OLED_drawchar
      008957 5B 04            [ 2] 1846 	addw	sp, #4
                                   1847 ;	main.c: 156: OLED_drawchar(olednum, FRAME_LINE_HOR,3,startcol+24);
      008959 4B 2C            [ 1] 1848 	push	#0x2c
      00895B 4B 03            [ 1] 1849 	push	#0x03
      00895D 4B 89            [ 1] 1850 	push	#0x89
      00895F 7B 06            [ 1] 1851 	ld	a, (0x06, sp)
      008961 88               [ 1] 1852 	push	a
      008962 CD 84 BC         [ 4] 1853 	call	_OLED_drawchar
      008965 5B 04            [ 2] 1854 	addw	sp, #4
                                   1855 ;	main.c: 157: OLED_drawchar(olednum, FRAME_MID_RIGHT,3,startcol+32);
      008967 4B 34            [ 1] 1856 	push	#0x34
      008969 4B 03            [ 1] 1857 	push	#0x03
      00896B 4B 85            [ 1] 1858 	push	#0x85
      00896D 7B 06            [ 1] 1859 	ld	a, (0x06, sp)
      00896F 88               [ 1] 1860 	push	a
      008970 CD 84 BC         [ 4] 1861 	call	_OLED_drawchar
      008973 5B 04            [ 2] 1862 	addw	sp, #4
                                   1863 ;	main.c: 159: OLED_drawchar(olednum, FRAME_LINE_VER,4,startcol);
      008975 4B 14            [ 1] 1864 	push	#0x14
      008977 4B 04            [ 1] 1865 	push	#0x04
      008979 4B 8A            [ 1] 1866 	push	#0x8a
      00897B 7B 06            [ 1] 1867 	ld	a, (0x06, sp)
      00897D 88               [ 1] 1868 	push	a
      00897E CD 84 BC         [ 4] 1869 	call	_OLED_drawchar
      008981 5B 04            [ 2] 1870 	addw	sp, #4
                                   1871 ;	main.c: 160: OLED_drawchar(olednum, FRAME_LINE_VER,4,startcol+16);
      008983 4B 24            [ 1] 1872 	push	#0x24
      008985 4B 04            [ 1] 1873 	push	#0x04
      008987 4B 8A            [ 1] 1874 	push	#0x8a
      008989 7B 06            [ 1] 1875 	ld	a, (0x06, sp)
      00898B 88               [ 1] 1876 	push	a
      00898C CD 84 BC         [ 4] 1877 	call	_OLED_drawchar
      00898F 5B 04            [ 2] 1878 	addw	sp, #4
                                   1879 ;	main.c: 161: OLED_drawchar(olednum, FRAME_LINE_VER,4,startcol+32);
      008991 4B 34            [ 1] 1880 	push	#0x34
      008993 4B 04            [ 1] 1881 	push	#0x04
      008995 4B 8A            [ 1] 1882 	push	#0x8a
      008997 7B 06            [ 1] 1883 	ld	a, (0x06, sp)
      008999 88               [ 1] 1884 	push	a
      00899A CD 84 BC         [ 4] 1885 	call	_OLED_drawchar
      00899D 5B 04            [ 2] 1886 	addw	sp, #4
                                   1887 ;	main.c: 163: OLED_drawchar(olednum, FRAME_BOT_LEFT,5,startcol);
      00899F 4B 14            [ 1] 1888 	push	#0x14
      0089A1 4B 05            [ 1] 1889 	push	#0x05
      0089A3 4B 86            [ 1] 1890 	push	#0x86
      0089A5 7B 06            [ 1] 1891 	ld	a, (0x06, sp)
      0089A7 88               [ 1] 1892 	push	a
      0089A8 CD 84 BC         [ 4] 1893 	call	_OLED_drawchar
      0089AB 5B 04            [ 2] 1894 	addw	sp, #4
                                   1895 ;	main.c: 164: OLED_drawchar(olednum, FRAME_LINE_HOR,5,startcol+8);
      0089AD 4B 1C            [ 1] 1896 	push	#0x1c
      0089AF 4B 05            [ 1] 1897 	push	#0x05
      0089B1 4B 89            [ 1] 1898 	push	#0x89
      0089B3 7B 06            [ 1] 1899 	ld	a, (0x06, sp)
      0089B5 88               [ 1] 1900 	push	a
      0089B6 CD 84 BC         [ 4] 1901 	call	_OLED_drawchar
      0089B9 5B 04            [ 2] 1902 	addw	sp, #4
                                   1903 ;	main.c: 165: OLED_drawchar(olednum, FRAME_BOT,5,startcol+16);
      0089BB 4B 24            [ 1] 1904 	push	#0x24
      0089BD 4B 05            [ 1] 1905 	push	#0x05
      0089BF 4B 87            [ 1] 1906 	push	#0x87
      0089C1 7B 06            [ 1] 1907 	ld	a, (0x06, sp)
      0089C3 88               [ 1] 1908 	push	a
      0089C4 CD 84 BC         [ 4] 1909 	call	_OLED_drawchar
      0089C7 5B 04            [ 2] 1910 	addw	sp, #4
                                   1911 ;	main.c: 166: OLED_drawchar(olednum, FRAME_LINE_HOR,5,startcol+24);
      0089C9 4B 2C            [ 1] 1912 	push	#0x2c
      0089CB 4B 05            [ 1] 1913 	push	#0x05
      0089CD 4B 89            [ 1] 1914 	push	#0x89
      0089CF 7B 06            [ 1] 1915 	ld	a, (0x06, sp)
      0089D1 88               [ 1] 1916 	push	a
      0089D2 CD 84 BC         [ 4] 1917 	call	_OLED_drawchar
      0089D5 5B 04            [ 2] 1918 	addw	sp, #4
                                   1919 ;	main.c: 167: OLED_drawchar(olednum, FRAME_BOT_RIGHT,5,startcol+32);
      0089D7 4B 34            [ 1] 1920 	push	#0x34
      0089D9 4B 05            [ 1] 1921 	push	#0x05
      0089DB 4B 88            [ 1] 1922 	push	#0x88
      0089DD 7B 06            [ 1] 1923 	ld	a, (0x06, sp)
      0089DF 88               [ 1] 1924 	push	a
      0089E0 CD 84 BC         [ 4] 1925 	call	_OLED_drawchar
      0089E3 5B 04            [ 2] 1926 	addw	sp, #4
                                   1927 ;	main.c: 169: OLED_drawtext(olednum, " OLED TEST : SYM",7,0);
      0089E5 AE 8E 55         [ 2] 1928 	ldw	x, #___str_10+0
      0089E8 4B 00            [ 1] 1929 	push	#0x00
      0089EA 4B 07            [ 1] 1930 	push	#0x07
      0089EC 89               [ 2] 1931 	pushw	x
      0089ED 7B 07            [ 1] 1932 	ld	a, (0x07, sp)
      0089EF 88               [ 1] 1933 	push	a
      0089F0 CD 85 5F         [ 4] 1934 	call	_OLED_drawtext
      0089F3 5B 05            [ 2] 1935 	addw	sp, #5
      0089F5 81               [ 4] 1936 	ret
                                   1937 ;	main.c: 171: void drawArrow(unsigned char olednum)
                                   1938 ;	-----------------------------------------
                                   1939 ;	 function drawArrow
                                   1940 ;	-----------------------------------------
      0089F6                       1941 _drawArrow:
                                   1942 ;	main.c: 175: OLED_drawchar(olednum, ARROW_UP_LEFT,1,startcol);
      0089F6 4B 14            [ 1] 1943 	push	#0x14
      0089F8 4B 01            [ 1] 1944 	push	#0x01
      0089FA 4B 8F            [ 1] 1945 	push	#0x8f
      0089FC 7B 06            [ 1] 1946 	ld	a, (0x06, sp)
      0089FE 88               [ 1] 1947 	push	a
      0089FF CD 84 BC         [ 4] 1948 	call	_OLED_drawchar
      008A02 5B 04            [ 2] 1949 	addw	sp, #4
                                   1950 ;	main.c: 176: OLED_drawchar(olednum, ARROW_UP,1,startcol+16);
      008A04 4B 24            [ 1] 1951 	push	#0x24
      008A06 4B 01            [ 1] 1952 	push	#0x01
      008A08 4B 8B            [ 1] 1953 	push	#0x8b
      008A0A 7B 06            [ 1] 1954 	ld	a, (0x06, sp)
      008A0C 88               [ 1] 1955 	push	a
      008A0D CD 84 BC         [ 4] 1956 	call	_OLED_drawchar
      008A10 5B 04            [ 2] 1957 	addw	sp, #4
                                   1958 ;	main.c: 177: OLED_drawchar(olednum, ARROW_UP_RIGHT,1,startcol+32);
      008A12 4B 34            [ 1] 1959 	push	#0x34
      008A14 4B 01            [ 1] 1960 	push	#0x01
      008A16 4B 90            [ 1] 1961 	push	#0x90
      008A18 7B 06            [ 1] 1962 	ld	a, (0x06, sp)
      008A1A 88               [ 1] 1963 	push	a
      008A1B CD 84 BC         [ 4] 1964 	call	_OLED_drawchar
      008A1E 5B 04            [ 2] 1965 	addw	sp, #4
                                   1966 ;	main.c: 179: OLED_drawchar(olednum, ARROW_LEFT,3,startcol);
      008A20 4B 14            [ 1] 1967 	push	#0x14
      008A22 4B 03            [ 1] 1968 	push	#0x03
      008A24 4B 8D            [ 1] 1969 	push	#0x8d
      008A26 7B 06            [ 1] 1970 	ld	a, (0x06, sp)
      008A28 88               [ 1] 1971 	push	a
      008A29 CD 84 BC         [ 4] 1972 	call	_OLED_drawchar
      008A2C 5B 04            [ 2] 1973 	addw	sp, #4
                                   1974 ;	main.c: 180: OLED_drawchar(olednum, ARROW_POINT,3,startcol+16);
      008A2E 4B 24            [ 1] 1975 	push	#0x24
      008A30 4B 03            [ 1] 1976 	push	#0x03
      008A32 4B 93            [ 1] 1977 	push	#0x93
      008A34 7B 06            [ 1] 1978 	ld	a, (0x06, sp)
      008A36 88               [ 1] 1979 	push	a
      008A37 CD 84 BC         [ 4] 1980 	call	_OLED_drawchar
      008A3A 5B 04            [ 2] 1981 	addw	sp, #4
                                   1982 ;	main.c: 181: OLED_drawchar(olednum, ARROW_RIGHT,3,startcol+32);
      008A3C 4B 34            [ 1] 1983 	push	#0x34
      008A3E 4B 03            [ 1] 1984 	push	#0x03
      008A40 4B 8E            [ 1] 1985 	push	#0x8e
      008A42 7B 06            [ 1] 1986 	ld	a, (0x06, sp)
      008A44 88               [ 1] 1987 	push	a
      008A45 CD 84 BC         [ 4] 1988 	call	_OLED_drawchar
      008A48 5B 04            [ 2] 1989 	addw	sp, #4
                                   1990 ;	main.c: 183: OLED_drawchar(olednum, ARROW_DOWN_LEFT,5,startcol);
      008A4A 4B 14            [ 1] 1991 	push	#0x14
      008A4C 4B 05            [ 1] 1992 	push	#0x05
      008A4E 4B 91            [ 1] 1993 	push	#0x91
      008A50 7B 06            [ 1] 1994 	ld	a, (0x06, sp)
      008A52 88               [ 1] 1995 	push	a
      008A53 CD 84 BC         [ 4] 1996 	call	_OLED_drawchar
      008A56 5B 04            [ 2] 1997 	addw	sp, #4
                                   1998 ;	main.c: 184: OLED_drawchar(olednum, ARROW_DOWN,5,startcol+16);
      008A58 4B 24            [ 1] 1999 	push	#0x24
      008A5A 4B 05            [ 1] 2000 	push	#0x05
      008A5C 4B 8C            [ 1] 2001 	push	#0x8c
      008A5E 7B 06            [ 1] 2002 	ld	a, (0x06, sp)
      008A60 88               [ 1] 2003 	push	a
      008A61 CD 84 BC         [ 4] 2004 	call	_OLED_drawchar
      008A64 5B 04            [ 2] 2005 	addw	sp, #4
                                   2006 ;	main.c: 185: OLED_drawchar(olednum, ARROW_DOWN_RIGHT,5,startcol+32);
      008A66 4B 34            [ 1] 2007 	push	#0x34
      008A68 4B 05            [ 1] 2008 	push	#0x05
      008A6A 4B 92            [ 1] 2009 	push	#0x92
      008A6C 7B 06            [ 1] 2010 	ld	a, (0x06, sp)
      008A6E 88               [ 1] 2011 	push	a
      008A6F CD 84 BC         [ 4] 2012 	call	_OLED_drawchar
      008A72 5B 04            [ 2] 2013 	addw	sp, #4
                                   2014 ;	main.c: 187: OLED_drawtext(olednum, " OLED TEST : SYM",7,0);
      008A74 AE 8E 55         [ 2] 2015 	ldw	x, #___str_10+0
      008A77 4B 00            [ 1] 2016 	push	#0x00
      008A79 4B 07            [ 1] 2017 	push	#0x07
      008A7B 89               [ 2] 2018 	pushw	x
      008A7C 7B 07            [ 1] 2019 	ld	a, (0x07, sp)
      008A7E 88               [ 1] 2020 	push	a
      008A7F CD 85 5F         [ 4] 2021 	call	_OLED_drawtext
      008A82 5B 05            [ 2] 2022 	addw	sp, #5
      008A84 81               [ 4] 2023 	ret
                                   2024 ;	main.c: 190: void drawBytes(unsigned char olednum)
                                   2025 ;	-----------------------------------------
                                   2026 ;	 function drawBytes
                                   2027 ;	-----------------------------------------
      008A85                       2028 _drawBytes:
      008A85 52 06            [ 2] 2029 	sub	sp, #6
                                   2030 ;	main.c: 194: OLED_setpos(olednum,3,5);
      008A87 4B 05            [ 1] 2031 	push	#0x05
      008A89 4B 03            [ 1] 2032 	push	#0x03
      008A8B 7B 0B            [ 1] 2033 	ld	a, (0x0b, sp)
      008A8D 88               [ 1] 2034 	push	a
      008A8E CD 84 A1         [ 4] 2035 	call	_OLED_setpos
      008A91 5B 03            [ 2] 2036 	addw	sp, #3
                                   2037 ;	main.c: 195: for(Ts=0;Ts<11;Ts++) //Draw pattern 11 times
      008A93 AE 00 02         [ 2] 2038 	ldw	x, #_dsine+0
      008A96 1F 03            [ 2] 2039 	ldw	(0x03, sp), x
      008A98 0F 02            [ 1] 2040 	clr	(0x02, sp)
                                   2041 ;	main.c: 197: for(ds=0;ds<10;ds++)
      008A9A                       2042 00115$:
      008A9A 0F 01            [ 1] 2043 	clr	(0x01, sp)
      008A9C                       2044 00105$:
                                   2045 ;	main.c: 199: OLED_drawbyte(olednum, dsine[ds]);
      008A9C 5F               [ 1] 2046 	clrw	x
      008A9D 7B 01            [ 1] 2047 	ld	a, (0x01, sp)
      008A9F 97               [ 1] 2048 	ld	xl, a
      008AA0 72 FB 03         [ 2] 2049 	addw	x, (0x03, sp)
      008AA3 F6               [ 1] 2050 	ld	a, (x)
      008AA4 88               [ 1] 2051 	push	a
      008AA5 7B 0A            [ 1] 2052 	ld	a, (0x0a, sp)
      008AA7 88               [ 1] 2053 	push	a
      008AA8 CD 84 B0         [ 4] 2054 	call	_OLED_drawbyte
      008AAB 5B 02            [ 2] 2055 	addw	sp, #2
                                   2056 ;	main.c: 197: for(ds=0;ds<10;ds++)
      008AAD 0C 01            [ 1] 2057 	inc	(0x01, sp)
      008AAF 7B 01            [ 1] 2058 	ld	a, (0x01, sp)
      008AB1 A1 0A            [ 1] 2059 	cp	a, #0x0a
      008AB3 25 E7            [ 1] 2060 	jrc	00105$
                                   2061 ;	main.c: 195: for(Ts=0;Ts<11;Ts++) //Draw pattern 11 times
      008AB5 0C 02            [ 1] 2062 	inc	(0x02, sp)
      008AB7 7B 02            [ 1] 2063 	ld	a, (0x02, sp)
      008AB9 A1 0B            [ 1] 2064 	cp	a, #0x0b
      008ABB 25 DD            [ 1] 2065 	jrc	00115$
                                   2066 ;	main.c: 203: OLED_setpos(olednum,5,3);
      008ABD 4B 03            [ 1] 2067 	push	#0x03
      008ABF 4B 05            [ 1] 2068 	push	#0x05
      008AC1 7B 0B            [ 1] 2069 	ld	a, (0x0b, sp)
      008AC3 88               [ 1] 2070 	push	a
      008AC4 CD 84 A1         [ 4] 2071 	call	_OLED_setpos
      008AC7 5B 03            [ 2] 2072 	addw	sp, #3
                                   2073 ;	main.c: 204: for(Ts=0;Ts<8;Ts++) //Draw pattern 8 times
      008AC9 AE 00 0C         [ 2] 2074 	ldw	x, #_dtri+0
      008ACC 1F 05            [ 2] 2075 	ldw	(0x05, sp), x
      008ACE 0F 02            [ 1] 2076 	clr	(0x02, sp)
                                   2077 ;	main.c: 206: for(ds=0;ds<14;ds++)
      008AD0                       2078 00119$:
      008AD0 0F 01            [ 1] 2079 	clr	(0x01, sp)
      008AD2                       2080 00109$:
                                   2081 ;	main.c: 208: OLED_drawbyte(olednum, dtri[ds]);
      008AD2 5F               [ 1] 2082 	clrw	x
      008AD3 7B 01            [ 1] 2083 	ld	a, (0x01, sp)
      008AD5 97               [ 1] 2084 	ld	xl, a
      008AD6 72 FB 05         [ 2] 2085 	addw	x, (0x05, sp)
      008AD9 F6               [ 1] 2086 	ld	a, (x)
      008ADA 88               [ 1] 2087 	push	a
      008ADB 7B 0A            [ 1] 2088 	ld	a, (0x0a, sp)
      008ADD 88               [ 1] 2089 	push	a
      008ADE CD 84 B0         [ 4] 2090 	call	_OLED_drawbyte
      008AE1 5B 02            [ 2] 2091 	addw	sp, #2
                                   2092 ;	main.c: 206: for(ds=0;ds<14;ds++)
      008AE3 0C 01            [ 1] 2093 	inc	(0x01, sp)
      008AE5 7B 01            [ 1] 2094 	ld	a, (0x01, sp)
      008AE7 A1 0E            [ 1] 2095 	cp	a, #0x0e
      008AE9 25 E7            [ 1] 2096 	jrc	00109$
                                   2097 ;	main.c: 204: for(Ts=0;Ts<8;Ts++) //Draw pattern 8 times
      008AEB 0C 02            [ 1] 2098 	inc	(0x02, sp)
      008AED 7B 02            [ 1] 2099 	ld	a, (0x02, sp)
      008AEF A1 08            [ 1] 2100 	cp	a, #0x08
      008AF1 25 DD            [ 1] 2101 	jrc	00119$
                                   2102 ;	main.c: 212: OLED_drawtext(olednum, "  DRAW PATTERN  ",7,0);
      008AF3 AE 8E 66         [ 2] 2103 	ldw	x, #___str_11+0
      008AF6 4B 00            [ 1] 2104 	push	#0x00
      008AF8 4B 07            [ 1] 2105 	push	#0x07
      008AFA 89               [ 2] 2106 	pushw	x
      008AFB 7B 0D            [ 1] 2107 	ld	a, (0x0d, sp)
      008AFD 88               [ 1] 2108 	push	a
      008AFE CD 85 5F         [ 4] 2109 	call	_OLED_drawtext
      008B01 5B 0B            [ 2] 2110 	addw	sp, #11
      008B03 81               [ 4] 2111 	ret
                                   2112 ;	main.c: 215: void drawLoadingBar(unsigned char olednum)
                                   2113 ;	-----------------------------------------
                                   2114 ;	 function drawLoadingBar
                                   2115 ;	-----------------------------------------
      008B04                       2116 _drawLoadingBar:
      008B04 88               [ 1] 2117 	push	a
                                   2118 ;	main.c: 219: OLED_setpos(olednum, 4,5);
      008B05 4B 05            [ 1] 2119 	push	#0x05
      008B07 4B 04            [ 1] 2120 	push	#0x04
      008B09 7B 06            [ 1] 2121 	ld	a, (0x06, sp)
      008B0B 88               [ 1] 2122 	push	a
      008B0C CD 84 A1         [ 4] 2123 	call	_OLED_setpos
      008B0F 5B 03            [ 2] 2124 	addw	sp, #3
                                   2125 ;	main.c: 221: for(lb=5;lb<123;lb++)
      008B11 A6 05            [ 1] 2126 	ld	a, #0x05
      008B13 6B 01            [ 1] 2127 	ld	(0x01, sp), a
      008B15                       2128 00102$:
                                   2129 ;	main.c: 223: OLED_drawbyte(olednum, 0xFF);
      008B15 4B FF            [ 1] 2130 	push	#0xff
      008B17 7B 05            [ 1] 2131 	ld	a, (0x05, sp)
      008B19 88               [ 1] 2132 	push	a
      008B1A CD 84 B0         [ 4] 2133 	call	_OLED_drawbyte
      008B1D 5B 02            [ 2] 2134 	addw	sp, #2
                                   2135 ;	main.c: 224: delay_ms(10);
      008B1F 4B 0A            [ 1] 2136 	push	#0x0a
      008B21 5F               [ 1] 2137 	clrw	x
      008B22 89               [ 2] 2138 	pushw	x
      008B23 4B 00            [ 1] 2139 	push	#0x00
      008B25 CD 80 F5         [ 4] 2140 	call	_delay_ms
      008B28 5B 04            [ 2] 2141 	addw	sp, #4
                                   2142 ;	main.c: 221: for(lb=5;lb<123;lb++)
      008B2A 0C 01            [ 1] 2143 	inc	(0x01, sp)
      008B2C 7B 01            [ 1] 2144 	ld	a, (0x01, sp)
      008B2E A1 7B            [ 1] 2145 	cp	a, #0x7b
      008B30 25 E3            [ 1] 2146 	jrc	00102$
                                   2147 ;	main.c: 226: delay_ms(1000);
      008B32 4B E8            [ 1] 2148 	push	#0xe8
      008B34 4B 03            [ 1] 2149 	push	#0x03
      008B36 5F               [ 1] 2150 	clrw	x
      008B37 89               [ 2] 2151 	pushw	x
      008B38 CD 80 F5         [ 4] 2152 	call	_delay_ms
      008B3B 5B 04            [ 2] 2153 	addw	sp, #4
                                   2154 ;	main.c: 227: OLED_clearblock(olednum,4,5,122); //Start & finish column = start & finish lb
      008B3D 4B 7A            [ 1] 2155 	push	#0x7a
      008B3F 4B 05            [ 1] 2156 	push	#0x05
      008B41 4B 04            [ 1] 2157 	push	#0x04
      008B43 7B 07            [ 1] 2158 	ld	a, (0x07, sp)
      008B45 88               [ 1] 2159 	push	a
      008B46 CD 86 05         [ 4] 2160 	call	_OLED_clearblock
      008B49 5B 05            [ 2] 2161 	addw	sp, #5
      008B4B 81               [ 4] 2162 	ret
                                   2163 	.area CODE
      008B4C                       2164 _font_arr:
      008B4C 00                    2165 	.db #0x00	; 0
      008B4D 00                    2166 	.db #0x00	; 0
      008B4E 00                    2167 	.db #0x00	; 0
      008B4F 00                    2168 	.db #0x00	; 0
      008B50 00                    2169 	.db #0x00	; 0
      008B51 00                    2170 	.db #0x00	; 0
      008B52 00                    2171 	.db #0x00	; 0
      008B53 5F                    2172 	.db #0x5F	; 95
      008B54 00                    2173 	.db #0x00	; 0
      008B55 00                    2174 	.db #0x00	; 0
      008B56 05                    2175 	.db #0x05	; 5
      008B57 03                    2176 	.db #0x03	; 3
      008B58 00                    2177 	.db #0x00	; 0
      008B59 05                    2178 	.db #0x05	; 5
      008B5A 03                    2179 	.db #0x03	; 3
      008B5B 14                    2180 	.db #0x14	; 20
      008B5C 7F                    2181 	.db #0x7F	; 127
      008B5D 14                    2182 	.db #0x14	; 20
      008B5E 7F                    2183 	.db #0x7F	; 127
      008B5F 14                    2184 	.db #0x14	; 20
      008B60 24                    2185 	.db #0x24	; 36
      008B61 2A                    2186 	.db #0x2A	; 42
      008B62 7F                    2187 	.db #0x7F	; 127
      008B63 2A                    2188 	.db #0x2A	; 42
      008B64 12                    2189 	.db #0x12	; 18
      008B65 23                    2190 	.db #0x23	; 35
      008B66 13                    2191 	.db #0x13	; 19
      008B67 08                    2192 	.db #0x08	; 8
      008B68 64                    2193 	.db #0x64	; 100	'd'
      008B69 62                    2194 	.db #0x62	; 98	'b'
      008B6A 36                    2195 	.db #0x36	; 54	'6'
      008B6B 49                    2196 	.db #0x49	; 73	'I'
      008B6C 55                    2197 	.db #0x55	; 85	'U'
      008B6D 22                    2198 	.db #0x22	; 34
      008B6E 50                    2199 	.db #0x50	; 80	'P'
      008B6F 00                    2200 	.db #0x00	; 0
      008B70 05                    2201 	.db #0x05	; 5
      008B71 03                    2202 	.db #0x03	; 3
      008B72 00                    2203 	.db #0x00	; 0
      008B73 00                    2204 	.db #0x00	; 0
      008B74 00                    2205 	.db #0x00	; 0
      008B75 1C                    2206 	.db #0x1C	; 28
      008B76 22                    2207 	.db #0x22	; 34
      008B77 41                    2208 	.db #0x41	; 65	'A'
      008B78 00                    2209 	.db #0x00	; 0
      008B79 00                    2210 	.db #0x00	; 0
      008B7A 41                    2211 	.db #0x41	; 65	'A'
      008B7B 22                    2212 	.db #0x22	; 34
      008B7C 1C                    2213 	.db #0x1C	; 28
      008B7D 00                    2214 	.db #0x00	; 0
      008B7E 0A                    2215 	.db #0x0A	; 10
      008B7F 04                    2216 	.db #0x04	; 4
      008B80 1F                    2217 	.db #0x1F	; 31
      008B81 04                    2218 	.db #0x04	; 4
      008B82 0A                    2219 	.db #0x0A	; 10
      008B83 08                    2220 	.db #0x08	; 8
      008B84 08                    2221 	.db #0x08	; 8
      008B85 3E                    2222 	.db #0x3E	; 62
      008B86 08                    2223 	.db #0x08	; 8
      008B87 08                    2224 	.db #0x08	; 8
      008B88 00                    2225 	.db #0x00	; 0
      008B89 50                    2226 	.db #0x50	; 80	'P'
      008B8A 30                    2227 	.db #0x30	; 48	'0'
      008B8B 00                    2228 	.db #0x00	; 0
      008B8C 00                    2229 	.db #0x00	; 0
      008B8D 08                    2230 	.db #0x08	; 8
      008B8E 08                    2231 	.db #0x08	; 8
      008B8F 08                    2232 	.db #0x08	; 8
      008B90 08                    2233 	.db #0x08	; 8
      008B91 08                    2234 	.db #0x08	; 8
      008B92 00                    2235 	.db #0x00	; 0
      008B93 60                    2236 	.db #0x60	; 96
      008B94 60                    2237 	.db #0x60	; 96
      008B95 00                    2238 	.db #0x00	; 0
      008B96 00                    2239 	.db #0x00	; 0
      008B97 20                    2240 	.db #0x20	; 32
      008B98 10                    2241 	.db #0x10	; 16
      008B99 08                    2242 	.db #0x08	; 8
      008B9A 04                    2243 	.db #0x04	; 4
      008B9B 02                    2244 	.db #0x02	; 2
      008B9C 3E                    2245 	.db #0x3E	; 62
      008B9D 51                    2246 	.db #0x51	; 81	'Q'
      008B9E 49                    2247 	.db #0x49	; 73	'I'
      008B9F 45                    2248 	.db #0x45	; 69	'E'
      008BA0 3E                    2249 	.db #0x3E	; 62
      008BA1 00                    2250 	.db #0x00	; 0
      008BA2 42                    2251 	.db #0x42	; 66	'B'
      008BA3 7F                    2252 	.db #0x7F	; 127
      008BA4 40                    2253 	.db #0x40	; 64
      008BA5 00                    2254 	.db #0x00	; 0
      008BA6 42                    2255 	.db #0x42	; 66	'B'
      008BA7 61                    2256 	.db #0x61	; 97	'a'
      008BA8 51                    2257 	.db #0x51	; 81	'Q'
      008BA9 49                    2258 	.db #0x49	; 73	'I'
      008BAA 46                    2259 	.db #0x46	; 70	'F'
      008BAB 22                    2260 	.db #0x22	; 34
      008BAC 41                    2261 	.db #0x41	; 65	'A'
      008BAD 49                    2262 	.db #0x49	; 73	'I'
      008BAE 49                    2263 	.db #0x49	; 73	'I'
      008BAF 36                    2264 	.db #0x36	; 54	'6'
      008BB0 18                    2265 	.db #0x18	; 24
      008BB1 14                    2266 	.db #0x14	; 20
      008BB2 12                    2267 	.db #0x12	; 18
      008BB3 7F                    2268 	.db #0x7F	; 127
      008BB4 10                    2269 	.db #0x10	; 16
      008BB5 27                    2270 	.db #0x27	; 39
      008BB6 45                    2271 	.db #0x45	; 69	'E'
      008BB7 45                    2272 	.db #0x45	; 69	'E'
      008BB8 45                    2273 	.db #0x45	; 69	'E'
      008BB9 39                    2274 	.db #0x39	; 57	'9'
      008BBA 3E                    2275 	.db #0x3E	; 62
      008BBB 49                    2276 	.db #0x49	; 73	'I'
      008BBC 49                    2277 	.db #0x49	; 73	'I'
      008BBD 49                    2278 	.db #0x49	; 73	'I'
      008BBE 32                    2279 	.db #0x32	; 50	'2'
      008BBF 61                    2280 	.db #0x61	; 97	'a'
      008BC0 11                    2281 	.db #0x11	; 17
      008BC1 09                    2282 	.db #0x09	; 9
      008BC2 05                    2283 	.db #0x05	; 5
      008BC3 03                    2284 	.db #0x03	; 3
      008BC4 36                    2285 	.db #0x36	; 54	'6'
      008BC5 49                    2286 	.db #0x49	; 73	'I'
      008BC6 49                    2287 	.db #0x49	; 73	'I'
      008BC7 49                    2288 	.db #0x49	; 73	'I'
      008BC8 36                    2289 	.db #0x36	; 54	'6'
      008BC9 26                    2290 	.db #0x26	; 38
      008BCA 49                    2291 	.db #0x49	; 73	'I'
      008BCB 49                    2292 	.db #0x49	; 73	'I'
      008BCC 49                    2293 	.db #0x49	; 73	'I'
      008BCD 3E                    2294 	.db #0x3E	; 62
      008BCE 00                    2295 	.db #0x00	; 0
      008BCF 36                    2296 	.db #0x36	; 54	'6'
      008BD0 36                    2297 	.db #0x36	; 54	'6'
      008BD1 00                    2298 	.db #0x00	; 0
      008BD2 00                    2299 	.db #0x00	; 0
      008BD3 00                    2300 	.db #0x00	; 0
      008BD4 56                    2301 	.db #0x56	; 86	'V'
      008BD5 36                    2302 	.db #0x36	; 54	'6'
      008BD6 00                    2303 	.db #0x00	; 0
      008BD7 00                    2304 	.db #0x00	; 0
      008BD8 00                    2305 	.db #0x00	; 0
      008BD9 08                    2306 	.db #0x08	; 8
      008BDA 14                    2307 	.db #0x14	; 20
      008BDB 22                    2308 	.db #0x22	; 34
      008BDC 00                    2309 	.db #0x00	; 0
      008BDD 14                    2310 	.db #0x14	; 20
      008BDE 14                    2311 	.db #0x14	; 20
      008BDF 14                    2312 	.db #0x14	; 20
      008BE0 14                    2313 	.db #0x14	; 20
      008BE1 14                    2314 	.db #0x14	; 20
      008BE2 00                    2315 	.db #0x00	; 0
      008BE3 22                    2316 	.db #0x22	; 34
      008BE4 14                    2317 	.db #0x14	; 20
      008BE5 08                    2318 	.db #0x08	; 8
      008BE6 00                    2319 	.db #0x00	; 0
      008BE7 02                    2320 	.db #0x02	; 2
      008BE8 01                    2321 	.db #0x01	; 1
      008BE9 51                    2322 	.db #0x51	; 81	'Q'
      008BEA 09                    2323 	.db #0x09	; 9
      008BEB 06                    2324 	.db #0x06	; 6
      008BEC 32                    2325 	.db #0x32	; 50	'2'
      008BED 49                    2326 	.db #0x49	; 73	'I'
      008BEE 79                    2327 	.db #0x79	; 121	'y'
      008BEF 41                    2328 	.db #0x41	; 65	'A'
      008BF0 3E                    2329 	.db #0x3E	; 62
      008BF1 7C                    2330 	.db #0x7C	; 124
      008BF2 12                    2331 	.db #0x12	; 18
      008BF3 11                    2332 	.db #0x11	; 17
      008BF4 12                    2333 	.db #0x12	; 18
      008BF5 7C                    2334 	.db #0x7C	; 124
      008BF6 7F                    2335 	.db #0x7F	; 127
      008BF7 49                    2336 	.db #0x49	; 73	'I'
      008BF8 49                    2337 	.db #0x49	; 73	'I'
      008BF9 49                    2338 	.db #0x49	; 73	'I'
      008BFA 36                    2339 	.db #0x36	; 54	'6'
      008BFB 3E                    2340 	.db #0x3E	; 62
      008BFC 41                    2341 	.db #0x41	; 65	'A'
      008BFD 41                    2342 	.db #0x41	; 65	'A'
      008BFE 41                    2343 	.db #0x41	; 65	'A'
      008BFF 22                    2344 	.db #0x22	; 34
      008C00 7F                    2345 	.db #0x7F	; 127
      008C01 41                    2346 	.db #0x41	; 65	'A'
      008C02 41                    2347 	.db #0x41	; 65	'A'
      008C03 22                    2348 	.db #0x22	; 34
      008C04 1C                    2349 	.db #0x1C	; 28
      008C05 7F                    2350 	.db #0x7F	; 127
      008C06 49                    2351 	.db #0x49	; 73	'I'
      008C07 49                    2352 	.db #0x49	; 73	'I'
      008C08 49                    2353 	.db #0x49	; 73	'I'
      008C09 49                    2354 	.db #0x49	; 73	'I'
      008C0A 7F                    2355 	.db #0x7F	; 127
      008C0B 09                    2356 	.db #0x09	; 9
      008C0C 09                    2357 	.db #0x09	; 9
      008C0D 09                    2358 	.db #0x09	; 9
      008C0E 09                    2359 	.db #0x09	; 9
      008C0F 3E                    2360 	.db #0x3E	; 62
      008C10 41                    2361 	.db #0x41	; 65	'A'
      008C11 49                    2362 	.db #0x49	; 73	'I'
      008C12 49                    2363 	.db #0x49	; 73	'I'
      008C13 3A                    2364 	.db #0x3A	; 58
      008C14 7F                    2365 	.db #0x7F	; 127
      008C15 08                    2366 	.db #0x08	; 8
      008C16 08                    2367 	.db #0x08	; 8
      008C17 08                    2368 	.db #0x08	; 8
      008C18 7F                    2369 	.db #0x7F	; 127
      008C19 00                    2370 	.db #0x00	; 0
      008C1A 41                    2371 	.db #0x41	; 65	'A'
      008C1B 7F                    2372 	.db #0x7F	; 127
      008C1C 41                    2373 	.db #0x41	; 65	'A'
      008C1D 00                    2374 	.db #0x00	; 0
      008C1E 20                    2375 	.db #0x20	; 32
      008C1F 40                    2376 	.db #0x40	; 64
      008C20 41                    2377 	.db #0x41	; 65	'A'
      008C21 3F                    2378 	.db #0x3F	; 63
      008C22 01                    2379 	.db #0x01	; 1
      008C23 7F                    2380 	.db #0x7F	; 127
      008C24 08                    2381 	.db #0x08	; 8
      008C25 14                    2382 	.db #0x14	; 20
      008C26 22                    2383 	.db #0x22	; 34
      008C27 41                    2384 	.db #0x41	; 65	'A'
      008C28 7F                    2385 	.db #0x7F	; 127
      008C29 40                    2386 	.db #0x40	; 64
      008C2A 40                    2387 	.db #0x40	; 64
      008C2B 40                    2388 	.db #0x40	; 64
      008C2C 40                    2389 	.db #0x40	; 64
      008C2D 7F                    2390 	.db #0x7F	; 127
      008C2E 02                    2391 	.db #0x02	; 2
      008C2F 0C                    2392 	.db #0x0C	; 12
      008C30 02                    2393 	.db #0x02	; 2
      008C31 7F                    2394 	.db #0x7F	; 127
      008C32 7F                    2395 	.db #0x7F	; 127
      008C33 04                    2396 	.db #0x04	; 4
      008C34 08                    2397 	.db #0x08	; 8
      008C35 10                    2398 	.db #0x10	; 16
      008C36 7F                    2399 	.db #0x7F	; 127
      008C37 3E                    2400 	.db #0x3E	; 62
      008C38 41                    2401 	.db #0x41	; 65	'A'
      008C39 41                    2402 	.db #0x41	; 65	'A'
      008C3A 41                    2403 	.db #0x41	; 65	'A'
      008C3B 3E                    2404 	.db #0x3E	; 62
      008C3C 7F                    2405 	.db #0x7F	; 127
      008C3D 09                    2406 	.db #0x09	; 9
      008C3E 09                    2407 	.db #0x09	; 9
      008C3F 09                    2408 	.db #0x09	; 9
      008C40 06                    2409 	.db #0x06	; 6
      008C41 3E                    2410 	.db #0x3E	; 62
      008C42 41                    2411 	.db #0x41	; 65	'A'
      008C43 51                    2412 	.db #0x51	; 81	'Q'
      008C44 21                    2413 	.db #0x21	; 33
      008C45 5E                    2414 	.db #0x5E	; 94
      008C46 7F                    2415 	.db #0x7F	; 127
      008C47 09                    2416 	.db #0x09	; 9
      008C48 19                    2417 	.db #0x19	; 25
      008C49 29                    2418 	.db #0x29	; 41
      008C4A 46                    2419 	.db #0x46	; 70	'F'
      008C4B 26                    2420 	.db #0x26	; 38
      008C4C 49                    2421 	.db #0x49	; 73	'I'
      008C4D 49                    2422 	.db #0x49	; 73	'I'
      008C4E 49                    2423 	.db #0x49	; 73	'I'
      008C4F 32                    2424 	.db #0x32	; 50	'2'
      008C50 01                    2425 	.db #0x01	; 1
      008C51 01                    2426 	.db #0x01	; 1
      008C52 7F                    2427 	.db #0x7F	; 127
      008C53 01                    2428 	.db #0x01	; 1
      008C54 01                    2429 	.db #0x01	; 1
      008C55 3F                    2430 	.db #0x3F	; 63
      008C56 40                    2431 	.db #0x40	; 64
      008C57 40                    2432 	.db #0x40	; 64
      008C58 40                    2433 	.db #0x40	; 64
      008C59 3F                    2434 	.db #0x3F	; 63
      008C5A 1F                    2435 	.db #0x1F	; 31
      008C5B 20                    2436 	.db #0x20	; 32
      008C5C 40                    2437 	.db #0x40	; 64
      008C5D 20                    2438 	.db #0x20	; 32
      008C5E 1F                    2439 	.db #0x1F	; 31
      008C5F 3F                    2440 	.db #0x3F	; 63
      008C60 40                    2441 	.db #0x40	; 64
      008C61 38                    2442 	.db #0x38	; 56	'8'
      008C62 40                    2443 	.db #0x40	; 64
      008C63 3F                    2444 	.db #0x3F	; 63
      008C64 63                    2445 	.db #0x63	; 99	'c'
      008C65 14                    2446 	.db #0x14	; 20
      008C66 08                    2447 	.db #0x08	; 8
      008C67 14                    2448 	.db #0x14	; 20
      008C68 63                    2449 	.db #0x63	; 99	'c'
      008C69 07                    2450 	.db #0x07	; 7
      008C6A 08                    2451 	.db #0x08	; 8
      008C6B 70                    2452 	.db #0x70	; 112	'p'
      008C6C 08                    2453 	.db #0x08	; 8
      008C6D 07                    2454 	.db #0x07	; 7
      008C6E 61                    2455 	.db #0x61	; 97	'a'
      008C6F 51                    2456 	.db #0x51	; 81	'Q'
      008C70 49                    2457 	.db #0x49	; 73	'I'
      008C71 45                    2458 	.db #0x45	; 69	'E'
      008C72 43                    2459 	.db #0x43	; 67	'C'
      008C73 00                    2460 	.db #0x00	; 0
      008C74 7F                    2461 	.db #0x7F	; 127
      008C75 41                    2462 	.db #0x41	; 65	'A'
      008C76 41                    2463 	.db #0x41	; 65	'A'
      008C77 00                    2464 	.db #0x00	; 0
      008C78 02                    2465 	.db #0x02	; 2
      008C79 04                    2466 	.db #0x04	; 4
      008C7A 08                    2467 	.db #0x08	; 8
      008C7B 10                    2468 	.db #0x10	; 16
      008C7C 20                    2469 	.db #0x20	; 32
      008C7D 00                    2470 	.db #0x00	; 0
      008C7E 41                    2471 	.db #0x41	; 65	'A'
      008C7F 41                    2472 	.db #0x41	; 65	'A'
      008C80 7F                    2473 	.db #0x7F	; 127
      008C81 00                    2474 	.db #0x00	; 0
      008C82 04                    2475 	.db #0x04	; 4
      008C83 02                    2476 	.db #0x02	; 2
      008C84 01                    2477 	.db #0x01	; 1
      008C85 02                    2478 	.db #0x02	; 2
      008C86 04                    2479 	.db #0x04	; 4
      008C87 40                    2480 	.db #0x40	; 64
      008C88 40                    2481 	.db #0x40	; 64
      008C89 40                    2482 	.db #0x40	; 64
      008C8A 40                    2483 	.db #0x40	; 64
      008C8B 40                    2484 	.db #0x40	; 64
      008C8C 00                    2485 	.db #0x00	; 0
      008C8D 01                    2486 	.db #0x01	; 1
      008C8E 02                    2487 	.db #0x02	; 2
      008C8F 04                    2488 	.db #0x04	; 4
      008C90 00                    2489 	.db #0x00	; 0
      008C91 20                    2490 	.db #0x20	; 32
      008C92 54                    2491 	.db #0x54	; 84	'T'
      008C93 54                    2492 	.db #0x54	; 84	'T'
      008C94 54                    2493 	.db #0x54	; 84	'T'
      008C95 78                    2494 	.db #0x78	; 120	'x'
      008C96 7F                    2495 	.db #0x7F	; 127
      008C97 50                    2496 	.db #0x50	; 80	'P'
      008C98 48                    2497 	.db #0x48	; 72	'H'
      008C99 48                    2498 	.db #0x48	; 72	'H'
      008C9A 30                    2499 	.db #0x30	; 48	'0'
      008C9B 38                    2500 	.db #0x38	; 56	'8'
      008C9C 44                    2501 	.db #0x44	; 68	'D'
      008C9D 44                    2502 	.db #0x44	; 68	'D'
      008C9E 44                    2503 	.db #0x44	; 68	'D'
      008C9F 28                    2504 	.db #0x28	; 40
      008CA0 30                    2505 	.db #0x30	; 48	'0'
      008CA1 48                    2506 	.db #0x48	; 72	'H'
      008CA2 48                    2507 	.db #0x48	; 72	'H'
      008CA3 50                    2508 	.db #0x50	; 80	'P'
      008CA4 7F                    2509 	.db #0x7F	; 127
      008CA5 38                    2510 	.db #0x38	; 56	'8'
      008CA6 54                    2511 	.db #0x54	; 84	'T'
      008CA7 54                    2512 	.db #0x54	; 84	'T'
      008CA8 54                    2513 	.db #0x54	; 84	'T'
      008CA9 18                    2514 	.db #0x18	; 24
      008CAA 08                    2515 	.db #0x08	; 8
      008CAB 7E                    2516 	.db #0x7E	; 126
      008CAC 09                    2517 	.db #0x09	; 9
      008CAD 09                    2518 	.db #0x09	; 9
      008CAE 02                    2519 	.db #0x02	; 2
      008CAF 08                    2520 	.db #0x08	; 8
      008CB0 54                    2521 	.db #0x54	; 84	'T'
      008CB1 54                    2522 	.db #0x54	; 84	'T'
      008CB2 54                    2523 	.db #0x54	; 84	'T'
      008CB3 3C                    2524 	.db #0x3C	; 60
      008CB4 7F                    2525 	.db #0x7F	; 127
      008CB5 10                    2526 	.db #0x10	; 16
      008CB6 08                    2527 	.db #0x08	; 8
      008CB7 08                    2528 	.db #0x08	; 8
      008CB8 70                    2529 	.db #0x70	; 112	'p'
      008CB9 00                    2530 	.db #0x00	; 0
      008CBA 48                    2531 	.db #0x48	; 72	'H'
      008CBB 7A                    2532 	.db #0x7A	; 122	'z'
      008CBC 40                    2533 	.db #0x40	; 64
      008CBD 00                    2534 	.db #0x00	; 0
      008CBE 20                    2535 	.db #0x20	; 32
      008CBF 40                    2536 	.db #0x40	; 64
      008CC0 48                    2537 	.db #0x48	; 72	'H'
      008CC1 3A                    2538 	.db #0x3A	; 58
      008CC2 00                    2539 	.db #0x00	; 0
      008CC3 7F                    2540 	.db #0x7F	; 127
      008CC4 10                    2541 	.db #0x10	; 16
      008CC5 28                    2542 	.db #0x28	; 40
      008CC6 44                    2543 	.db #0x44	; 68	'D'
      008CC7 00                    2544 	.db #0x00	; 0
      008CC8 00                    2545 	.db #0x00	; 0
      008CC9 41                    2546 	.db #0x41	; 65	'A'
      008CCA 7F                    2547 	.db #0x7F	; 127
      008CCB 40                    2548 	.db #0x40	; 64
      008CCC 00                    2549 	.db #0x00	; 0
      008CCD 7C                    2550 	.db #0x7C	; 124
      008CCE 04                    2551 	.db #0x04	; 4
      008CCF 7C                    2552 	.db #0x7C	; 124
      008CD0 04                    2553 	.db #0x04	; 4
      008CD1 78                    2554 	.db #0x78	; 120	'x'
      008CD2 7C                    2555 	.db #0x7C	; 124
      008CD3 08                    2556 	.db #0x08	; 8
      008CD4 04                    2557 	.db #0x04	; 4
      008CD5 04                    2558 	.db #0x04	; 4
      008CD6 78                    2559 	.db #0x78	; 120	'x'
      008CD7 38                    2560 	.db #0x38	; 56	'8'
      008CD8 44                    2561 	.db #0x44	; 68	'D'
      008CD9 44                    2562 	.db #0x44	; 68	'D'
      008CDA 44                    2563 	.db #0x44	; 68	'D'
      008CDB 38                    2564 	.db #0x38	; 56	'8'
      008CDC 7C                    2565 	.db #0x7C	; 124
      008CDD 14                    2566 	.db #0x14	; 20
      008CDE 14                    2567 	.db #0x14	; 20
      008CDF 14                    2568 	.db #0x14	; 20
      008CE0 08                    2569 	.db #0x08	; 8
      008CE1 08                    2570 	.db #0x08	; 8
      008CE2 14                    2571 	.db #0x14	; 20
      008CE3 14                    2572 	.db #0x14	; 20
      008CE4 18                    2573 	.db #0x18	; 24
      008CE5 7C                    2574 	.db #0x7C	; 124
      008CE6 7C                    2575 	.db #0x7C	; 124
      008CE7 08                    2576 	.db #0x08	; 8
      008CE8 04                    2577 	.db #0x04	; 4
      008CE9 04                    2578 	.db #0x04	; 4
      008CEA 08                    2579 	.db #0x08	; 8
      008CEB 48                    2580 	.db #0x48	; 72	'H'
      008CEC 54                    2581 	.db #0x54	; 84	'T'
      008CED 54                    2582 	.db #0x54	; 84	'T'
      008CEE 54                    2583 	.db #0x54	; 84	'T'
      008CEF 20                    2584 	.db #0x20	; 32
      008CF0 04                    2585 	.db #0x04	; 4
      008CF1 3F                    2586 	.db #0x3F	; 63
      008CF2 44                    2587 	.db #0x44	; 68	'D'
      008CF3 44                    2588 	.db #0x44	; 68	'D'
      008CF4 20                    2589 	.db #0x20	; 32
      008CF5 3C                    2590 	.db #0x3C	; 60
      008CF6 40                    2591 	.db #0x40	; 64
      008CF7 40                    2592 	.db #0x40	; 64
      008CF8 20                    2593 	.db #0x20	; 32
      008CF9 7C                    2594 	.db #0x7C	; 124
      008CFA 1C                    2595 	.db #0x1C	; 28
      008CFB 20                    2596 	.db #0x20	; 32
      008CFC 40                    2597 	.db #0x40	; 64
      008CFD 20                    2598 	.db #0x20	; 32
      008CFE 1C                    2599 	.db #0x1C	; 28
      008CFF 3C                    2600 	.db #0x3C	; 60
      008D00 40                    2601 	.db #0x40	; 64
      008D01 38                    2602 	.db #0x38	; 56	'8'
      008D02 40                    2603 	.db #0x40	; 64
      008D03 3C                    2604 	.db #0x3C	; 60
      008D04 44                    2605 	.db #0x44	; 68	'D'
      008D05 28                    2606 	.db #0x28	; 40
      008D06 10                    2607 	.db #0x10	; 16
      008D07 28                    2608 	.db #0x28	; 40
      008D08 44                    2609 	.db #0x44	; 68	'D'
      008D09 0C                    2610 	.db #0x0C	; 12
      008D0A 50                    2611 	.db #0x50	; 80	'P'
      008D0B 50                    2612 	.db #0x50	; 80	'P'
      008D0C 50                    2613 	.db #0x50	; 80	'P'
      008D0D 3C                    2614 	.db #0x3C	; 60
      008D0E 44                    2615 	.db #0x44	; 68	'D'
      008D0F 64                    2616 	.db #0x64	; 100	'd'
      008D10 54                    2617 	.db #0x54	; 84	'T'
      008D11 4C                    2618 	.db #0x4C	; 76	'L'
      008D12 44                    2619 	.db #0x44	; 68	'D'
      008D13 00                    2620 	.db #0x00	; 0
      008D14 08                    2621 	.db #0x08	; 8
      008D15 36                    2622 	.db #0x36	; 54	'6'
      008D16 41                    2623 	.db #0x41	; 65	'A'
      008D17 00                    2624 	.db #0x00	; 0
      008D18 00                    2625 	.db #0x00	; 0
      008D19 00                    2626 	.db #0x00	; 0
      008D1A 7F                    2627 	.db #0x7F	; 127
      008D1B 00                    2628 	.db #0x00	; 0
      008D1C 00                    2629 	.db #0x00	; 0
      008D1D 00                    2630 	.db #0x00	; 0
      008D1E 41                    2631 	.db #0x41	; 65	'A'
      008D1F 36                    2632 	.db #0x36	; 54	'6'
      008D20 08                    2633 	.db #0x08	; 8
      008D21 00                    2634 	.db #0x00	; 0
      008D22 10                    2635 	.db #0x10	; 16
      008D23 08                    2636 	.db #0x08	; 8
      008D24 08                    2637 	.db #0x08	; 8
      008D25 10                    2638 	.db #0x10	; 16
      008D26 08                    2639 	.db #0x08	; 8
      008D27 06                    2640 	.db #0x06	; 6
      008D28 09                    2641 	.db #0x09	; 9
      008D29 09                    2642 	.db #0x09	; 9
      008D2A 06                    2643 	.db #0x06	; 6
      008D2B 00                    2644 	.db #0x00	; 0
      008D2C 00                    2645 	.db #0x00	; 0
      008D2D 00                    2646 	.db #0x00	; 0
      008D2E 00                    2647 	.db #0x00	; 0
      008D2F F8                    2648 	.db #0xF8	; 248
      008D30 F8                    2649 	.db #0xF8	; 248
      008D31 18                    2650 	.db #0x18	; 24
      008D32 18                    2651 	.db #0x18	; 24
      008D33 18                    2652 	.db #0x18	; 24
      008D34 18                    2653 	.db #0x18	; 24
      008D35 18                    2654 	.db #0x18	; 24
      008D36 18                    2655 	.db #0x18	; 24
      008D37 F8                    2656 	.db #0xF8	; 248
      008D38 F8                    2657 	.db #0xF8	; 248
      008D39 18                    2658 	.db #0x18	; 24
      008D3A 18                    2659 	.db #0x18	; 24
      008D3B 18                    2660 	.db #0x18	; 24
      008D3C 18                    2661 	.db #0x18	; 24
      008D3D 18                    2662 	.db #0x18	; 24
      008D3E 18                    2663 	.db #0x18	; 24
      008D3F F8                    2664 	.db #0xF8	; 248
      008D40 F8                    2665 	.db #0xF8	; 248
      008D41 00                    2666 	.db #0x00	; 0
      008D42 00                    2667 	.db #0x00	; 0
      008D43 00                    2668 	.db #0x00	; 0
      008D44 00                    2669 	.db #0x00	; 0
      008D45 00                    2670 	.db #0x00	; 0
      008D46 00                    2671 	.db #0x00	; 0
      008D47 FF                    2672 	.db #0xFF	; 255
      008D48 FF                    2673 	.db #0xFF	; 255
      008D49 18                    2674 	.db #0x18	; 24
      008D4A 18                    2675 	.db #0x18	; 24
      008D4B 18                    2676 	.db #0x18	; 24
      008D4C 18                    2677 	.db #0x18	; 24
      008D4D 18                    2678 	.db #0x18	; 24
      008D4E 18                    2679 	.db #0x18	; 24
      008D4F FF                    2680 	.db #0xFF	; 255
      008D50 FF                    2681 	.db #0xFF	; 255
      008D51 18                    2682 	.db #0x18	; 24
      008D52 18                    2683 	.db #0x18	; 24
      008D53 18                    2684 	.db #0x18	; 24
      008D54 18                    2685 	.db #0x18	; 24
      008D55 18                    2686 	.db #0x18	; 24
      008D56 18                    2687 	.db #0x18	; 24
      008D57 FF                    2688 	.db #0xFF	; 255
      008D58 FF                    2689 	.db #0xFF	; 255
      008D59 00                    2690 	.db #0x00	; 0
      008D5A 00                    2691 	.db #0x00	; 0
      008D5B 00                    2692 	.db #0x00	; 0
      008D5C 00                    2693 	.db #0x00	; 0
      008D5D 00                    2694 	.db #0x00	; 0
      008D5E 00                    2695 	.db #0x00	; 0
      008D5F 1F                    2696 	.db #0x1F	; 31
      008D60 1F                    2697 	.db #0x1F	; 31
      008D61 18                    2698 	.db #0x18	; 24
      008D62 18                    2699 	.db #0x18	; 24
      008D63 18                    2700 	.db #0x18	; 24
      008D64 18                    2701 	.db #0x18	; 24
      008D65 18                    2702 	.db #0x18	; 24
      008D66 18                    2703 	.db #0x18	; 24
      008D67 1F                    2704 	.db #0x1F	; 31
      008D68 1F                    2705 	.db #0x1F	; 31
      008D69 18                    2706 	.db #0x18	; 24
      008D6A 18                    2707 	.db #0x18	; 24
      008D6B 18                    2708 	.db #0x18	; 24
      008D6C 18                    2709 	.db #0x18	; 24
      008D6D 18                    2710 	.db #0x18	; 24
      008D6E 18                    2711 	.db #0x18	; 24
      008D6F 1F                    2712 	.db #0x1F	; 31
      008D70 1F                    2713 	.db #0x1F	; 31
      008D71 00                    2714 	.db #0x00	; 0
      008D72 00                    2715 	.db #0x00	; 0
      008D73 00                    2716 	.db #0x00	; 0
      008D74 18                    2717 	.db #0x18	; 24
      008D75 18                    2718 	.db #0x18	; 24
      008D76 18                    2719 	.db #0x18	; 24
      008D77 18                    2720 	.db #0x18	; 24
      008D78 18                    2721 	.db #0x18	; 24
      008D79 18                    2722 	.db #0x18	; 24
      008D7A 18                    2723 	.db #0x18	; 24
      008D7B 18                    2724 	.db #0x18	; 24
      008D7C 00                    2725 	.db #0x00	; 0
      008D7D 00                    2726 	.db #0x00	; 0
      008D7E 00                    2727 	.db #0x00	; 0
      008D7F FF                    2728 	.db #0xFF	; 255
      008D80 FF                    2729 	.db #0xFF	; 255
      008D81 00                    2730 	.db #0x00	; 0
      008D82 00                    2731 	.db #0x00	; 0
      008D83 00                    2732 	.db #0x00	; 0
      008D84 18                    2733 	.db #0x18	; 24
      008D85 0C                    2734 	.db #0x0C	; 12
      008D86 06                    2735 	.db #0x06	; 6
      008D87 FF                    2736 	.db #0xFF	; 255
      008D88 FF                    2737 	.db #0xFF	; 255
      008D89 06                    2738 	.db #0x06	; 6
      008D8A 0C                    2739 	.db #0x0C	; 12
      008D8B 18                    2740 	.db #0x18	; 24
      008D8C 18                    2741 	.db #0x18	; 24
      008D8D 30                    2742 	.db #0x30	; 48	'0'
      008D8E 60                    2743 	.db #0x60	; 96
      008D8F FF                    2744 	.db #0xFF	; 255
      008D90 FF                    2745 	.db #0xFF	; 255
      008D91 60                    2746 	.db #0x60	; 96
      008D92 30                    2747 	.db #0x30	; 48	'0'
      008D93 18                    2748 	.db #0x18	; 24
      008D94 18                    2749 	.db #0x18	; 24
      008D95 3C                    2750 	.db #0x3C	; 60
      008D96 7E                    2751 	.db #0x7E	; 126
      008D97 DB                    2752 	.db #0xDB	; 219
      008D98 99                    2753 	.db #0x99	; 153
      008D99 18                    2754 	.db #0x18	; 24
      008D9A 18                    2755 	.db #0x18	; 24
      008D9B 18                    2756 	.db #0x18	; 24
      008D9C 18                    2757 	.db #0x18	; 24
      008D9D 18                    2758 	.db #0x18	; 24
      008D9E 18                    2759 	.db #0x18	; 24
      008D9F 99                    2760 	.db #0x99	; 153
      008DA0 DB                    2761 	.db #0xDB	; 219
      008DA1 7E                    2762 	.db #0x7E	; 126
      008DA2 3C                    2763 	.db #0x3C	; 60
      008DA3 18                    2764 	.db #0x18	; 24
      008DA4 7F                    2765 	.db #0x7F	; 127
      008DA5 7F                    2766 	.db #0x7F	; 127
      008DA6 0F                    2767 	.db #0x0F	; 15
      008DA7 1F                    2768 	.db #0x1F	; 31
      008DA8 3B                    2769 	.db #0x3B	; 59
      008DA9 73                    2770 	.db #0x73	; 115	's'
      008DAA E3                    2771 	.db #0xE3	; 227
      008DAB 40                    2772 	.db #0x40	; 64
      008DAC 40                    2773 	.db #0x40	; 64
      008DAD E3                    2774 	.db #0xE3	; 227
      008DAE 73                    2775 	.db #0x73	; 115	's'
      008DAF 3B                    2776 	.db #0x3B	; 59
      008DB0 1F                    2777 	.db #0x1F	; 31
      008DB1 0F                    2778 	.db #0x0F	; 15
      008DB2 7F                    2779 	.db #0x7F	; 127
      008DB3 7F                    2780 	.db #0x7F	; 127
      008DB4 FE                    2781 	.db #0xFE	; 254
      008DB5 FE                    2782 	.db #0xFE	; 254
      008DB6 F0                    2783 	.db #0xF0	; 240
      008DB7 F8                    2784 	.db #0xF8	; 248
      008DB8 DC                    2785 	.db #0xDC	; 220
      008DB9 CE                    2786 	.db #0xCE	; 206
      008DBA C7                    2787 	.db #0xC7	; 199
      008DBB 02                    2788 	.db #0x02	; 2
      008DBC 02                    2789 	.db #0x02	; 2
      008DBD C7                    2790 	.db #0xC7	; 199
      008DBE CE                    2791 	.db #0xCE	; 206
      008DBF DC                    2792 	.db #0xDC	; 220
      008DC0 F8                    2793 	.db #0xF8	; 248
      008DC1 F0                    2794 	.db #0xF0	; 240
      008DC2 FE                    2795 	.db #0xFE	; 254
      008DC3 FE                    2796 	.db #0xFE	; 254
      008DC4 3C                    2797 	.db #0x3C	; 60
      008DC5 42                    2798 	.db #0x42	; 66	'B'
      008DC6 81                    2799 	.db #0x81	; 129
      008DC7 99                    2800 	.db #0x99	; 153
      008DC8 99                    2801 	.db #0x99	; 153
      008DC9 81                    2802 	.db #0x81	; 129
      008DCA 42                    2803 	.db #0x42	; 66	'B'
      008DCB 3C                    2804 	.db #0x3C	; 60
      008DCC                       2805 ___str_0:
      008DCC 20 4F 4C 45 44 20 54  2806 	.ascii " OLED TEST : INT"
             45 53 54 20 3A 20 49
             4E 54
      008DDC 00                    2807 	.db 0x00
      008DDD                       2808 ___str_1:
      008DDD 41 42 43 44 45 46 47  2809 	.ascii "ABCDEFGHIJKLM"
             48 49 4A 4B 4C 4D
      008DEA 00                    2810 	.db 0x00
      008DEB                       2811 ___str_2:
      008DEB 4E 4F 50 51 52 53 54  2812 	.ascii "NOPQRSTUVWXYZ"
             55 56 57 58 59 5A
      008DF8 00                    2813 	.db 0x00
      008DF9                       2814 ___str_3:
      008DF9 61 62 63 64 65 66 67  2815 	.ascii "abcdefghijklm"
             68 69 6A 6B 6C 6D
      008E06 00                    2816 	.db 0x00
      008E07                       2817 ___str_4:
      008E07 6E 6F 70 71 72 73 74  2818 	.ascii "nopqrstuvwxyz"
             75 76 77 78 79 7A
      008E14 00                    2819 	.db 0x00
      008E15                       2820 ___str_5:
      008E15 30 31 32 33 34 35 36  2821 	.ascii "0123456789"
             37 38 39
      008E1F 00                    2822 	.db 0x00
      008E20                       2823 ___str_6:
      008E20 4F 4C 45 44 20 54 45  2824 	.ascii "OLED TEST : CHAR"
             53 54 20 3A 20 43 48
             41 52
      008E30 00                    2825 	.db 0x00
      008E31                       2826 ___str_7:
      008E31 3C 7B 28 5B 2B 5F 2D  2827 	.ascii "<{([+_-=])}>"
             3D 5D 29 7D 3E
      008E3D 00                    2828 	.db 0x00
      008E3E                       2829 ___str_8:
      008E3E 21 40 23 24 25 5E 26  2830 	.ascii "!@#$%^&*`|~?"
             2A 60 7C 7E 3F
      008E4A 00                    2831 	.db 0x00
      008E4B                       2832 ___str_9:
      008E4B 2E 2C                 2833 	.ascii ".,"
      008E4D 22                    2834 	.db 0x22
      008E4E 27                    2835 	.ascii "'"
      008E4F 5C                    2836 	.db 0x5C
      008E50 2F 20 3A 3B           2837 	.ascii "/ :;"
      008E54 00                    2838 	.db 0x00
      008E55                       2839 ___str_10:
      008E55 20 4F 4C 45 44 20 54  2840 	.ascii " OLED TEST : SYM"
             45 53 54 20 3A 20 53
             59 4D
      008E65 00                    2841 	.db 0x00
      008E66                       2842 ___str_11:
      008E66 20 20 44 52 41 57 20  2843 	.ascii "  DRAW PATTERN  "
             50 41 54 54 45 52 4E
             20 20
      008E76 00                    2844 	.db 0x00
                                   2845 	.area INITIALIZER
      008FDE                       2846 __xinit__dsine:
      008FDE 18                    2847 	.db #0x18	; 24
      008FDF 06                    2848 	.db #0x06	; 6
      008FE0 01                    2849 	.db #0x01	; 1
      008FE1 01                    2850 	.db #0x01	; 1
      008FE2 06                    2851 	.db #0x06	; 6
      008FE3 18                    2852 	.db #0x18	; 24
      008FE4 60                    2853 	.db #0x60	; 96
      008FE5 80                    2854 	.db #0x80	; 128
      008FE6 80                    2855 	.db #0x80	; 128
      008FE7 60                    2856 	.db #0x60	; 96
      008FE8                       2857 __xinit__dtri:
      008FE8 08                    2858 	.db #0x08	; 8
      008FE9 04                    2859 	.db #0x04	; 4
      008FEA 02                    2860 	.db #0x02	; 2
      008FEB 01                    2861 	.db #0x01	; 1
      008FEC 02                    2862 	.db #0x02	; 2
      008FED 04                    2863 	.db #0x04	; 4
      008FEE 08                    2864 	.db #0x08	; 8
      008FEF 10                    2865 	.db #0x10	; 16
      008FF0 20                    2866 	.db #0x20	; 32
      008FF1 40                    2867 	.db #0x40	; 64
      008FF2 80                    2868 	.db #0x80	; 128
      008FF3 40                    2869 	.db #0x40	; 64
      008FF4 20                    2870 	.db #0x20	; 32
      008FF5 10                    2871 	.db #0x10	; 16
                                   2872 	.area CABS (ABS)
