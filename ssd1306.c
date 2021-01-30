#include "ssd1306.h"

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
uint8_t Display_Eeprom(uint16_t data, uint16_t max)
{
	uint8_t column=0;	//vertical display position
	uint8_t page=7;		//horizontal display position
	uint8_t xx=0;		//helper variable
	uint16_t proz=0;		//data size in % of total eeprom memory
	uint16_t bar=0;		//lenght of memory bar
	uint8_t max_page=0;	//full used pages to show bar
	uint8_t pattern=0;	//leading edge of bar
	uint8_t rest=0;		//bits for pattern
	static uint8_t old_data=0;
	static uint8_t old_max=0;
	static uint8_t old_max_page=8;
	proz=(100*data)/max; //calculate used memory in %
	bar= (proz*64)/100;	 //is equal to how many pixels
	max_page=bar/8;		//page is 8 Bites
	rest=data-max_page;	//
	
	//no need to draw same data twice
	if(!((data==old_data) && (max==old_max)))
	{
		
		
		//draw left line of memory-box
		Set_Column_Address(column);
		Set_Page_Address(page);
		for(xx=0;xx<16;xx++)
		{
			send_data(0x80);
		}
		//draw right line of memory-box
		Set_Column_Address(column);
		Set_Page_Address(0);
		for(xx=0;xx<16;xx++)
		{
			send_data(0x01);
		}
		//draw upper line of memory-box
		column=0;
		page=0;
		for(page=0;page<8;page++)
		{
			Set_Column_Address(column);
			Set_Page_Address(page);
			send_data(0xff);
		}
		//draw lower line of memory-box
		page=0;
		column=16;
		for(page=0;page<8;page++)
		{
			Set_Column_Address(column);
			Set_Page_Address(page);
			send_data(0xff);
		}
		
		/*draw full pages if not 
		already drawn the last time*/
		if(old_max_page!=max_page)
		{
			//Set_Column_Address(column);
			//Set_Page_Address(7-max_page-old_max_page);
			for(xx=0;xx<max_page;xx++)
			{
				for(column=1;column<16;column++)
				{
					Set_Column_Address(column);
					Set_Page_Address(7-xx);
					send_data(0xff);
				}
			}
		}
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
		for(column=1;column<16;column++)
		{
			Set_Column_Address(column);
			Set_Page_Address(7-max_page);
			send_data(pattern);
		}

		
	}//end of data==old_data
	old_data=data;
	old_max=max;
	old_max_page=max_page;
	
	
	/*for(column=1;column<16;column++)
	{
		Set_Column_Address(column);
		Set_Page_Address(page);
		send_data(0xff);
	}
	//draw filled bar representing used memory
	//start at column 1 to avoid gap in upper line
	
	for(column=1;column<16;column++)
	{
		Set_Column_Address(column);
		Set_Page_Address(page);
		send_data(0b11111000);
	}*/
	return 7-max_page;
}//end of Display Eeprom


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
	send_command(0x01);//orig 80
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
void Display_Picture(uint8_t width, uint8_t height, const unsigned char data[])
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
		Set_Page_Address(7-i);
		Set_Column_Address(0);			//start at column 0
        for(unsigned char j=0;j<(height-1);j++)	//write Column for current page
		{
		    send_data(pgm_read_byte(&data[i*0x30+j]));
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
void Write_Char(uint8_t fontsize, char n)
{
	const char *fontpointer=0;
	uint8_t x;
	
	switch(fontsize)		//variable height and fixed width (8 Pixel)
	{
		case 8:fontpointer=font8;break;
		case 14:fontpointer=font14;break;
	}
	n-=0x20;			//jump to position in asci table
	for(x=0;x<fontsize;x++) 
	{
		send_data(pgm_read_byte(&fontpointer[(n*fontsize)+x]));
	}
	
}
void Write_String(uint8_t fontsize, uint8_t row, uint8_t pos, const char str[]) 
{
	if(fontsize==26)
	{
		while(*str)
		{
			Set_Page_Address(7-pos*2);			//0-7 	(* 8 bit)
			Set_Column_Address(row*fontsize);	//0-3	(* 14 collums / char)
			Write_Char(fontsize, *str++);
			pos++;
		 }
	}else
	{
		while(*str)
		{
			Set_Page_Address(7-pos);			//0-7 	(* 8 bit)
			Set_Column_Address(row*fontsize);	//0-3	(* 14 collums / char)
			Write_Char(fontsize, *str++);
			pos++;
		 }
	}
}
