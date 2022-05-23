//2020-01-11
//Created by : bonusoid
//OLED SSD1306 (128x64) Demo

#include"delay.h"
#include"delay.c"
#include"periph_stm8s.h"
#include"periph_stm8s.c"
#include"oled_ssd1306.h"
#include"oled_ssd1306.c"

unsigned char dsine[10] = {0x18,0x06,0x01,0x01,0x06,0x18,0x60,0x80,0x80,0x60}; //Sinewave pattern
unsigned char dtri[14] = {0x08,0x04,0x02,0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80,0x40,0x20,0x10}; //Trianglewave pattern

void gpio_init();
void loop();

void drawInt(unsigned char olednum);	//Draw Integer demo
void drawAlphanum(unsigned char olednum); //Draw Letter & Number demo
void drawPunct(unsigned char olednum); //Draw Punctuation demo
void drawFrame(unsigned char olednum); //Draw Frame demo
void drawArrow(unsigned char olednum); //Draw Arrow demo
void drawBytes(unsigned char olednum); //Draw Pattern demo

void drawLoadingBar(unsigned char olednum); //Draw Loading Bar animation

//^^^^^^^^^^ INIT ^^^^^^^^^^//
int main()
{
  clock_init();
  delay_init();
  gpio_init();
  i2c_init();
  //Up to 2 OLED. If only 1 OLED is connected, use one of : OLED1 or OLED2
  ssd1306_init(OLED1);
  ssd1306_init(OLED2);
  OLED_clear(OLED1);
  OLED_clear(OLED2);
  
  drawLoadingBar(OLED1);
  drawLoadingBar(OLED2);
  
  loop();
  return 0;
}
//__________ INIT __________//


//^^^^^^^^^^ LOOP ^^^^^^^^^^//
void loop()
{
	while(OLED1)
	{
		drawBytes(OLED1);
		delay_ms(1000);
                OLED_clearblock(OLED1,3,5,114); //Finish column = 5 + 11*10 - 1
   		delay_ms(500);
		OLED_clearblock(OLED1,5,3,114); //Finish column = 3 + 8*14 - 1
   		delay_ms(500);

		drawInt(OLED2);
		delay_ms(1000); 
		OLED_clear(OLED2);

		drawAlphanum(OLED1);
		delay_ms(1000); 
		OLED_reverse(OLED1);
		delay_ms(1000);
		OLED_clear(OLED1);
		OLED_normal(OLED1);

		drawPunct(OLED2);
		delay_ms(1000); 
		OLED_reverse(OLED2);
		delay_ms(1000);
		OLED_clear(OLED2);
		OLED_normal(OLED2);

		drawFrame(OLED1);
		delay_ms(700); 
		OLED_clearblock(OLED1,3,36,43); //Finish column = 36 + 8 - 1
		delay_ms(700);
		OLED_clear(OLED1);

		drawArrow(OLED2);
		delay_ms(700); 
		OLED_clearblock(OLED2,3,36,43); //Finish column = 36 + 8 - 1
		delay_ms(700);
		OLED_clear(OLED2);
	} 	
}
//__________ LOOP __________//

void gpio_init()
{
	
}

void drawInt(unsigned char olednum)
{
	OLED_drawint(olednum, 64, 0, 8);   //Decimal
	OLED_drawint(olednum, 064, 0, 48); //Octal displayed as Decimal
	OLED_drawint(olednum, 0x64, 0, 88); //Hexadecimal displayed as Decimal

	OLED_drawint(olednum, -64, 1, 8); //Negative number is not supported
				 	  //Its 2's complement will be displayed

	OLED_drawint(olednum, 65535, 3, 8); //Max. is 65535

	OLED_drawint(olednum, 100, 5, 8);
	OLED_drawchar(olednum, SYM_DEGREE, 5, 32);
	OLED_drawchar(olednum, 'C', 5, 40);

	OLED_drawtext(olednum, " OLED TEST : INT",7,0);
}

void drawAlphanum(unsigned char olednum)
{
	OLED_drawtext(olednum, "ABCDEFGHIJKLM",0,0);
	OLED_drawtext(olednum, "NOPQRSTUVWXYZ",1,0);
		
	OLED_drawtext(olednum, "abcdefghijklm",3,0);
	OLED_drawtext(olednum, "nopqrstuvwxyz",4,0);
		
	OLED_drawtext(olednum, "0123456789",6,0);

	OLED_drawtext(olednum, "OLED TEST : CHAR",7,0);
}

void drawPunct(unsigned char olednum)
{
	OLED_drawtext(olednum, "<{([+_-=])}>",0,0);
	OLED_drawtext(olednum, "!@#$%^&*`|~?",2,0);
	OLED_drawtext(olednum, ".\,\"\'\\/ :;",4,0);

	OLED_drawtext(olednum, "OLED TEST : CHAR",7,0);
}

void drawFrame(unsigned char olednum)
{
	unsigned char startcol=20;

	OLED_drawchar(olednum, FRAME_TOP_LEFT,1,startcol);
	OLED_drawchar(olednum, FRAME_LINE_HOR,1,startcol+8);
	OLED_drawchar(olednum, FRAME_TOP,1,startcol+16);
	OLED_drawchar(olednum, FRAME_LINE_HOR,1,startcol+24);
	OLED_drawchar(olednum, FRAME_TOP_RIGHT,1,startcol+32);
	
	OLED_drawchar(olednum, FRAME_LINE_VER,2,startcol);
	OLED_drawchar(olednum, FRAME_LINE_VER,2,startcol+16);
	OLED_drawchar(olednum, FRAME_LINE_VER,2,startcol+32);

	OLED_drawchar(olednum, FRAME_MID_LEFT,3,startcol);
	OLED_drawchar(olednum, FRAME_LINE_HOR,3,startcol+8);
	OLED_drawchar(olednum, FRAME_CENTER,3,startcol+16);
	OLED_drawchar(olednum, FRAME_LINE_HOR,3,startcol+24);
	OLED_drawchar(olednum, FRAME_MID_RIGHT,3,startcol+32);

	OLED_drawchar(olednum, FRAME_LINE_VER,4,startcol);
	OLED_drawchar(olednum, FRAME_LINE_VER,4,startcol+16);
	OLED_drawchar(olednum, FRAME_LINE_VER,4,startcol+32);

	OLED_drawchar(olednum, FRAME_BOT_LEFT,5,startcol);
	OLED_drawchar(olednum, FRAME_LINE_HOR,5,startcol+8);
	OLED_drawchar(olednum, FRAME_BOT,5,startcol+16);
	OLED_drawchar(olednum, FRAME_LINE_HOR,5,startcol+24);
	OLED_drawchar(olednum, FRAME_BOT_RIGHT,5,startcol+32);

	OLED_drawtext(olednum, " OLED TEST : SYM",7,0);
}
void drawArrow(unsigned char olednum)
{
	unsigned char startcol=20;

	OLED_drawchar(olednum, ARROW_UP_LEFT,1,startcol);
	OLED_drawchar(olednum, ARROW_UP,1,startcol+16);
	OLED_drawchar(olednum, ARROW_UP_RIGHT,1,startcol+32);

	OLED_drawchar(olednum, ARROW_LEFT,3,startcol);
	OLED_drawchar(olednum, ARROW_POINT,3,startcol+16);
	OLED_drawchar(olednum, ARROW_RIGHT,3,startcol+32);

	OLED_drawchar(olednum, ARROW_DOWN_LEFT,5,startcol);
	OLED_drawchar(olednum, ARROW_DOWN,5,startcol+16);
	OLED_drawchar(olednum, ARROW_DOWN_RIGHT,5,startcol+32);

	OLED_drawtext(olednum, " OLED TEST : SYM",7,0);
}

void drawBytes(unsigned char olednum)
{
	unsigned char Ts,ds;

	OLED_setpos(olednum,3,5);
	for(Ts=0;Ts<11;Ts++) //Draw pattern 11 times
	{
		for(ds=0;ds<10;ds++)
		{
			OLED_drawbyte(olednum, dsine[ds]);
		}
	}

	OLED_setpos(olednum,5,3);
	for(Ts=0;Ts<8;Ts++) //Draw pattern 8 times
	{
		for(ds=0;ds<14;ds++)
		{
			OLED_drawbyte(olednum, dtri[ds]);
		}
	}

	OLED_drawtext(olednum, "  DRAW PATTERN  ",7,0);
}

void drawLoadingBar(unsigned char olednum)
{
	unsigned char lb;

	OLED_setpos(olednum, 4,5);

	for(lb=5;lb<123;lb++)
	{
		OLED_drawbyte(olednum, 0xFF);
		delay_ms(10);
	}
	delay_ms(1000);
	OLED_clearblock(olednum,4,5,122); //Start & finish column = start & finish lb
}
