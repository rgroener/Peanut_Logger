#include "ssd1306.h"

void Display_Init(void)
{
	/*Init session according datasheet and sample code:
	 *https://www.buydisplay.com/serial-spi-0-71-inch-white-48x64-graphic-oled-display-ssd1306
	 * */
	DISP_RST_1;
	_delay_ms(10);
	DISP_RST_0;
	_delay_ms(20);
	DISP_RST_1;
	_delay_ms(10);
	DISP_COMM;
	CS_DISP_0;		
	send_command(0xae);//--turn off oled panel
	send_command(0xd5);//--set display clock divide ratio/oscillator frequency
	send_command(0x80);//--set divide ratio
	send_command(0xa8);//--set multiplex ratio(1 to 64)
	send_command(0x3f);//--1/64 duty orig 3f
	send_command(0xd3);//-set display offset
	send_command(0x00);//-not offset
	send_command(0x8d);//--set Charge Pump enable/disable
	send_command(0x14);//--set(0x10) disable
	send_command(0xa6);//--set normal display
	send_command(0xa4);// Disable Entire Display On orig a4
	send_command(0xa1);//--set segment re-map 128 to 0
	send_command(0xC8);//--Set COM Output Scan Direction 64 to 0
	send_command(0xda);//--set com pins hardware configuration
	send_command(0x12);
	send_command(0x81);//--set contrast control register
	send_command(0x08);//orig 80
	send_command(0xd9);//--set pre-charge period
	send_command(0xf1);
	send_command(0xdb);//--set vcomh
	send_command(0x40);//--set start line address orig 40
	send_command(0xaf);//--turn on oled panel
	CS_DISP_1;		   //release chip select
	DISP_DATA;
}
void send_data(char data)
{
	DISP_DATA;
	CS_DISP_0;
	SPI_MasterTransmit(data);
	CS_DISP_1;
}
void send_command(char command)
{
	DISP_COMM;
	CS_DISP_0;
	SPI_MasterTransmit(command);
	CS_DISP_1;
}
void Set_Page_Address(unsigned char add)
{
    add=0xb0|add;
    send_command(add);
	return;
}
void Set_Column_Address(unsigned char add)
{	 
	add+=40;
    send_command((0x10|(add>>4)));
	send_command((0x0f&add));
	return;
}
void Set_Contrast_Control_Register(unsigned char mod)
{
    send_command(0x81);
	send_command(mod);
	return;
}
void Display_Picture(uint8_t posx, uint8_t posy, uint8_t width, uint8_t height, const unsigned char data[])
{
	/* Oled Display 64*48  bit organisation
	 * array size full display [384]
	 * 
	 ======== ============>
	 |	P P		P		Column 0
	 | 	A A		A		Column 1
	 | 	G G ... G			|
	 | 	E E		E		Column ...
	 | 	  					|
	 | 	7 6		0		Column 47
	 v
	 * 
	 * LCD Image Converter Settings:
	 * https://lcd-image-converter.riuson.com
	 * 
	 * Main scan direction: Top to Bottom
	 * Line scan direction: Forward
	 * Bands:	yes (8px)
	 * Inverse:	yes 
	 * 
	*/
	for(unsigned char i=0;i<(width/8);i++)		//write Pages
	{
		Set_Page_Address(7-i-posx);			//start at desired page (8Bit wide)
		Set_Column_Address(posy);			//start at desired column 
        for(unsigned char j=0;j<(height);j++)	//write Column for current page
		{
		    send_data(pgm_read_byte(&data[i*height+j]));
		}
	}
	
    return;
}
void Display_Clear(void)
{
	//clear whole display
	unsigned char i,j;
	for(i=0;i<0x08;i++)
	{
	Set_Page_Address(i);
    Set_Column_Address(0x00);
        for(j=0;j<0x30;j++)
		{
		    send_data(0x00);
		}
	}
    return;
}
void Char_Position(uint8_t fontsize, uint8_t row, uint8_t pos)
{
		//set position for new char (font size 8x14)
		Set_Page_Address(7-pos);	//0-7 	(* 8 bit)
		Set_Column_Address(row*fontsize);	//0-3	(* 14 collums / char)
}
void Write_Char(uint8_t fontsize, uint8_t row, uint8_t pos, char n)
{
	const char *fontpointer=0;
	uint8_t x;
	
	switch(fontsize)		//variable height and fixed width (8 Pixel)
	{
		//pointer to right array[0]
		case 8:fontpointer=font8;break;
		case 14:fontpointer=font14;break;
		case 16:fontpointer=font16;break;
	}
				//jump to position in asci table
	if(fontsize==16)// char width / hight is 16 bit
	{
		n-=0x20;
		Char_Position(fontsize,row,pos);//first page (first half of digit)
		for(x=0;x<16;x++)
		{
			send_data(pgm_read_byte(&fontpointer[(n*32)+x]));
		}
		
		Char_Position(fontsize,row,pos+1);//second page (second half of digit)
		for(x=0;x<16;x++)
		{
			send_data(pgm_read_byte(&fontpointer[(n*32)+x+16]));
		}
			
	}else //char width is 8 bit
	{
		n-=0x20;
		Char_Position(fontsize,row,pos);//first page 
		for(x=0;x<fontsize;x++) 
		{
			send_data(pgm_read_byte(&fontpointer[(n*fontsize)+x]));
		}
	}//eof fontsize
	
	
	
		

	
}

void Write_String(uint8_t fontsize, uint8_t row, uint8_t pos, const char str[]) 
{
	
		while(*str)
		{
			Set_Page_Address(7-pos);			//0-7 	(* 8 bit)
			Set_Column_Address(row*fontsize);	//0-3	(* 14 collums / char)
			Write_Char(fontsize, row, pos, *str++);
			if(fontsize==16)
			{
				pos+=2;
			}else
			{
				pos++;
			}
			
		 }
	
}
uint8_t Eeprom(uint32_t data, uint32_t max, uint8_t reset)
{
	/*	Display bar graphic 
	 * 	Bar lenght corresponds to % of the max value.
	 * 	Hight of the bar / box is variable
	 * 
	 * 	Input: 	- Data to visualise in bar
	 * 			- Max value to calculate % from
	 * 	Return:	% value (0...100)%
	 * 	Output:	bar
	 * 
	 * 	grn Jan 21
	 * */
	/* Oled Display 64*48  bit organisation
	 * array size full display [384]
	 * 
	 ======== ============>
	 |	P P		P		Column 0
	 | 	A A		A		Column 1
	 | 	G G ... G			|
	 | 	E E		E		Column ...
	 | 	  					|
	 | 	7 6		0		Column 47
	 v*/
	uint8_t column=0;		//vertical display position
	uint8_t page=7;			//horizontal display position
	uint8_t xx=0;			//helper variable
	uint32_t proz=0;		//data size in % of total eeprom memory
	uint32_t bar=0;			//lenght of memory bar
	uint8_t max_page=8;		//full used pages to show bar
	uint8_t rest=0;			//bits for pattern
	uint8_t pattern=0;		//most forward pixel of bar
	uint8_t bar_hight=9;	//height o the memo box 2...
	//static uint8_t boxflag=0;		//draw box only the first time
	static uint8_t old_rest=0;		//save last rest
	static uint8_t old_max_page=8;	//save last max position
	/*delete saved values and reset
	 * variables to original / start settings*/
	if(reset==1)
	{
		column=0;		//vertical display position
		page=7;			//horizontal display position
		rest=0;			//bits for pattern
		pattern=0;		//most forward pixel of bar
		bar_hight=10;	//height o the memo box 2...
		//boxflag=0;		//draw box only the first time
		old_rest=0;		//save last rest
		old_max_page=8;	//save last max position
	}
	
	
	if(data>max)data=max;			//avoid bigger data than max
	proz=0.5+((100*data)/max); 		//calculate used memory in %
	bar= (proz*64)/100;	 	//is equal to how many pixels
	max_page=bar/8;			//page is 8 Bites
	rest=bar-(max_page*8);	//*8 due to 8 bit size of page
	
	if(proz!=100)//only draw if bar is not yet full
	{
		//Boxoutline needs to be drawn only once
		//if(boxflag==0)
		if(1)
		{
			//boxflag=1;	
			//draw left line of memory-box
			Set_Column_Address(1);
			Set_Page_Address(7);
			for(xx=0;xx<(bar_hight-2);xx++)
			{
				send_data(0x80);
			}
			//draw right line of memory-box
			Set_Column_Address(1);
			Set_Page_Address(0);
			for(xx=0;xx<(bar_hight-2);xx++)
			{
				send_data(0x01);
			}
			//draw upper line of memory-box
			for(page=0;page<8;page++)
			{
				Set_Column_Address(0);
				Set_Page_Address(page);
				send_data(0xff);
			}
			//draw lower line of memory-box
			page=0;
			column=(bar_hight-1);
			for(page=0;page<8;page++)
			{
				Set_Column_Address(column);
				Set_Page_Address(page);
				send_data(0xff);
			}
		}//eof boxflag
		/*draw full pages if not 
		already drawn the last time*/
		if(old_max_page!=max_page)
		{
			//Set_Column_Address(column);
			//Set_Page_Address(7-max_page-old_max_page);
			for(xx=0;xx<max_page;xx++)
			{
				for(column=1;column<(bar_hight-1);column++)
				{
					Set_Column_Address(column);
					Set_Page_Address(7-xx);
					send_data(0xff);
				}
			}
		}//end of max_page
		if(rest!=old_rest)
			{
				switch(rest)
				{
					case 1:	pattern=(0x80);
							break;
					case 2:	pattern=(0xC0);
							break;
					case 3:	pattern=(0xE0);
							break;
					case 4:	pattern=(0xF0);
							break;
					case 5:	pattern=(0xF8);
							break;
					case 6:	pattern=(0xFC);
							break;
					case 7:	pattern=(0xFE);
							break;
				}//end of switch
				//adds right line of box
				if(max_page==7)pattern |= 0x01;
				for(column=1;column<(bar_hight-1);column++)
				{
					Set_Column_Address(column);
					Set_Page_Address(7-max_page);
					send_data(pattern);
				}
			}//end of rest
		//save old values for next run
		old_rest=rest;
		old_max_page=max_page;
	}//eof if(proz!=100)
	//Display barscale
	
	//Display_Picture(64,5,barscale);
	return proz;
}//end of Display Eeprom
