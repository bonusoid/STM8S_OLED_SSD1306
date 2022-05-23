//writer : bonus adityas (bonusonic@gmail.com)
//11 january 2020

#include"oled_ssd1306.h"
#include"font.h"

void ssd1306_init(unsigned char olednum)
{
	ssd1306_sendcom(olednum,0xAE); //Set Display Off
	ssd1306_sendcom(olednum,0xD5); //Set Display Clock Divider Ratio/Oscillator Frequency
	ssd1306_sendcom(olednum,0x80);
	ssd1306_sendcom(olednum,0xA8); //Set Multiplex Ratio
	ssd1306_sendcom(olednum,0x3F);
	ssd1306_sendcom(olednum,0xD3); //Set Display Offset
	ssd1306_sendcom(olednum,0x00);
	ssd1306_sendcom(olednum,0x40); //Set Display Start Line
	ssd1306_sendcom(olednum,0x8D); //Set Charge Pump
	ssd1306_sendcom(olednum,0x14); //Internal VCC
	ssd1306_sendcom(olednum,0x20); //Set Memory Mode
	ssd1306_sendcom(olednum,0x00); //Horizontal Addressing
	ssd1306_sendcom(olednum,0xA1); //Set Segment Re-Map
	ssd1306_sendcom(olednum,0xC8); //Set COM Output Scan Direction
	ssd1306_sendcom(olednum,0xDA); //Set COM Pins HW Config
	ssd1306_sendcom(olednum,0x12);
	ssd1306_sendcom(olednum,0x81); //Set Contrast Control
	ssd1306_sendcom(olednum,0xCF);
	ssd1306_sendcom(olednum,0xD9); //Set Pre-Charge Period
	ssd1306_sendcom(olednum,0xF1);
	ssd1306_sendcom(olednum,0xDB); //Set VCOMH Deselect Level
	ssd1306_sendcom(olednum,0x40);
	ssd1306_sendcom(olednum,0xA4); //Set Entire Display On/Off
	ssd1306_sendcom(olednum,0xA6); //Set Normal/Inverse Display
	ssd1306_sendcom(olednum,0xAF); //Set Display On
}

void ssd1306_sendcom(unsigned char olednum, unsigned char ssd1306com)
{
	i2c_write_2byte(olednum,commode,ssd1306com); //Send Command
}

void ssd1306_senddat(unsigned char olednum, unsigned char ssd1306dat)
{
	i2c_write_2byte(olednum,datmode,ssd1306dat); //Send Data
}

void ssd1306_setpos(unsigned char olednum, unsigned char row, unsigned char col)
{
	ssd1306_sendcom(olednum,(0xB0|(row&0x0F))); //Set page of row
	ssd1306_sendcom(olednum,(0x00|(col&0x0F))); //Set lower nibble of column
	ssd1306_sendcom(olednum,(0x10|((col>>4)&0x0F))); //Set upper nibble of column
}

void ssd1306_clear(unsigned char olednum) 
{
  	unsigned char col,row;
	ssd1306_setpos(olednum,0,0);
  	for(row=0;row<OLED_ROW+1;row++)	//Scan rows, add 1 row for completely flush.
  	   {      
      		for(col=0;col<OLED_COL;col++)	//Scan columns
                   {
        		ssd1306_senddat(olednum,0);	//Send 0 to every pixel
                   }
           }
}

void OLED_setpos(unsigned char olednum, unsigned char row, unsigned char col)
{
	ssd1306_setpos(olednum,row,col); //Set coordinate (for LCD_drawbyte)
}

void OLED_drawbyte(unsigned char olednum, unsigned char dbyte)
{
	ssd1306_senddat(olednum,dbyte); //Send 1 byte data only
}

void OLED_drawchar(unsigned char olednum, unsigned char chr, unsigned char chrrow, unsigned char chrcol)
{
	unsigned char ci,fchar;
	unsigned int chridx;

	ssd1306_setpos(olednum,chrrow,chrcol);
	
	if((chr>31)&&(chr<128))	//Alphanumeric & Punctuation Area
	  {
	    ssd1306_senddat(olednum,0x00);
            chridx=(chr-32)*5; //Start at character 32 (Space). 5 columns for each character
            for(ci=0;ci<5;ci++)
		{
		   fchar = font_arr[chridx+ci]; //Get character pattern from Font Array
		   ssd1306_senddat(olednum,fchar); //Send pattern 1 byte at a time
		}
          }
 	else if((chr>127)&&(chr<148)) //Frame & Arrow Area
	  {
	    chridx=(chr-128)*8; //Start at index 128. 5 columns for each symbol
            for(ci=0;ci<8;ci++)
		{
 		   fchar = font_arr[chridx+480+ci]; //Get symbol pattern from Font Array
		   ssd1306_senddat(olednum,fchar); //Send pattern 1 byte at a time		   
		}
	  }
	else{}
}

void OLED_drawtext(unsigned char olednum, unsigned char *text, unsigned char txtrow, unsigned char txtcol)
{
	unsigned int stridx = 0;

	while(text[stridx] != 0) //Scan characters in string
	  {
		OLED_drawchar(olednum,text[stridx],txtrow,txtcol+(8*stridx)); //Display each character
		stridx++;
	  }
}

void OLED_drawint(unsigned char olednum, unsigned int num, unsigned char numrow, unsigned char numcol)
{
	unsigned char ibuff[6]; //MAX : 5 DIGIT : 65535

	unsigned char ndigit=0,nd;
	unsigned int numb; //Must be unsigned, so max. number can be 65535

	numb = num;
	while(numb!=0) //Counting digit
	  {
	  	ndigit++;
		numb /= 10;	
	  }
	for(nd=0;nd<ndigit;nd++) //Converting each digit
	  {
		numb = num%10;
		num = num/10;
		ibuff[ndigit-(nd+1)] = numb + '0'; //Start from last_index-1
	  }
	ibuff[ndigit] = '\0'; //Last character is null

	OLED_drawtext(olednum,ibuff,numrow,numcol); //Display number as text
}

void OLED_clear(unsigned char olednum)
{
	ssd1306_sendcom(olednum,0xAE); //Set Display off
  	ssd1306_clear(olednum); //Clear Display
  	ssd1306_sendcom(olednum,0xAF); //Set Display on
}

void OLED_clearblock(unsigned char olednum, unsigned char row, unsigned char col_start, unsigned char col_fin)
{
	unsigned char col;

	ssd1306_setpos(olednum,row,col_start); 	//Set start position
	for(col=col_start;col<=col_fin;col++) 	//Scan columns
	   {
		ssd1306_senddat(olednum,0);	//Send 0 to every pixel in a column
	   }
}

void OLED_normal(unsigned char olednum)
{
	ssd1306_sendcom(olednum,0xA6);	//On Pixel in Off Background
}

void OLED_reverse(unsigned char olednum)
{
	ssd1306_sendcom(olednum,0xA7);	//Off Pixel in On Background
}
