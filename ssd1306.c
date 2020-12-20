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
{	 add+=40;
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
void Display_Picture(const unsigned char pic[])
{
	//Display picture 48x64
    unsigned char i,j;
	for(i=0;i<0x08;i++)
	{
	Set_Page_Address(i);
    Set_Column_Address(0x00);
        for(j=0;j<0x30;j++)
		{
		    send_data(pgm_read_byte(&pic[i*0x30+j]));
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