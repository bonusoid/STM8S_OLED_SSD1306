//writer : bonus adityas (bonusonic@gmail.com)
//11 january 2020

#ifndef __OLED_SSD1306_H
#define __OLED_SSD1306_H

#define OLED1	0x3C //addr 0 : 0b0111100[W/R]
#define OLED2	0x3D //addr 1 : 0b0111101[W/R]

#define commode	0x00 //Command Mode Identifier -> both 0x00 & 0x80 are working
#define datmode	0x40 //Data Mode Identifier

#define OLED_COL 128 //Column = 128 pixels (0-127)
#define OLED_ROW 8   //Row = 8 characters (0-7)

#define SYM_DEGREE       127

#define FRAME_TOP_LEFT   128
#define FRAME_TOP	 129
#define FRAME_TOP_RIGHT  130

#define FRAME_MID_LEFT   131
#define FRAME_CENTER	 132
#define FRAME_MID_RIGHT  133

#define FRAME_BOT_LEFT   134
#define FRAME_BOT	 135
#define FRAME_BOT_RIGHT  136

#define FRAME_LINE_HOR   137
#define FRAME_LINE_VER   138

#define ARROW_UP         139
#define ARROW_DOWN       140
#define ARROW_LEFT       141
#define ARROW_RIGHT      142
#define ARROW_UP_LEFT    143
#define ARROW_UP_RIGHT   144
#define ARROW_DOWN_LEFT  145
#define ARROW_DOWN_RIGHT 146
#define ARROW_POINT	 147

//OLED Operational Functions
void ssd1306_init(unsigned char olednum);	//OLED initialization
void ssd1306_sendcom(unsigned char olednum, unsigned char ssd1306com);	//Send Command
void ssd1306_senddat(unsigned char olednum, unsigned char ssd1306dat); 	//Send Data
void ssd1306_setpos(unsigned char olednum, unsigned char row, unsigned char col);	//Set Coordinate
void ssd1306_clear(unsigned char olednum);	//Clear all pixel

//OLED Draw Functions
void OLED_setpos(unsigned char olednum, unsigned char row, unsigned char col); //Set Coordinate (for OLED_drawbyte)
void OLED_drawbyte(unsigned char olednum, unsigned char dbyte);	//Draw 1 Byte
void OLED_drawchar(unsigned char olednum, unsigned char chr, unsigned char chrrow, unsigned char chrcol); //Draw 1 Character	
void OLED_drawtext(unsigned char olednum, unsigned char *text, unsigned char txtrow, unsigned char txtcol); //Draw Text
void OLED_drawint(unsigned char olednum, unsigned int num, unsigned char numrow, unsigned char numcol); //Draw Number (Integer), MAX=65535
void OLED_clear(unsigned char olednum); //Reset/Clear Screen
void OLED_clearblock(unsigned char olednum, unsigned char row, unsigned char col_start, unsigned char col_fin);	//Reset/Clear Area

//OLED Color Mode
void OLED_normal(unsigned char olednum);	//Normal Color
void OLED_reverse(unsigned char olednum); 	//Reverse Color

#endif
