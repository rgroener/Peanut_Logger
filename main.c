/*
 * Altitude logger for rubber powered models
 * 
 * grn Dec/20
 * 
 * avrdude: safemode: Fuses OK (E:F7, H:D9, L:62)
 * */
#define F_CPU 8000000UL// set the CPU clock
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <avr/io.h>
#include <stdio.h>                 
#include <avr/pgmspace.h>
#include "fonts.h"

//OLED Controling
#define CS_DISP_0 	PORTB &= ~(1<<PB0)	//select chip select display
#define CS_DISP_1 	PORTB |= (1<<PB0)	//deselect chip select display
#define DISP_DATA	PORTB |= (1<<PB2)	//set D/C to data
#define DISP_COMM	PORTB &= ~(1<<PB2)	//set D/C to command
#define DISP_RST_0	PORTB &= ~(1<<PB1)	//set Reset to 0
#define DISP_RST_1	PORTB |= (1<<PB1)	//set Reset to 1
#define 	Start_column	0x00
#define 	Start_page		0x00
#define	StartLine_set	0x00
/*
 * RES		PB1
 * CS_DISP	PB0
 * SCK		PB5
 * MISO		PB4
 * D/C		PB2
 * SDO		PD2
 * MOSI		PB3
 * SDA		PC4
 * SCL		PC5
 * 
 * */
 

void SPI_MasterInit(void);
void SPI_MasterTransmit(char cData);
void send_data(char data);
void send_command(char command);
void Set_Page_Address(unsigned char add);
void Set_Column_Address(unsigned char add);
void Set_Contrast_Control_Register(unsigned char mod);
void Display_Picture(const unsigned char pic[]);
void Display_Init(void);
void Display_Clear(void);
void Write_Char(uint8_t fontsize, char n);
void Char_Position(uint8_t fontsize, uint8_t row, uint8_t pos);
void Write_Char(uint8_t fontsize, char n);
void Write_String(uint8_t fontsize, uint8_t row, uint8_t pos, const char str[]); 
int main(void)
{
	DDRB 	|= (1<<PB0) | (1<<PB1) | (1<<PB2) | (1<<PB3) | (1<<PB5);//set CS_DISP and RES and D/C output
	PORTB	|= (1<<PB0) | (1<<PB1) | (1<<PB2) | (1<<PB3) | (1<<PB5);//set CS_DISP and RES and D/C high
		
	DDRC |= (1<<PC5);	//SCL
	PORTC |= (1<<PC5);
	PORTC &= ~(1<<PC5);
	
	//init SPI as master without interrupt
	SPI_MasterInit();
    // Enable Global Interrupts
    //sei();
		
	Display_Init();
	Display_Clear();
	Set_Page_Address(0);
    Set_Column_Address(0);
    
   
 
	Write_String(8,0,0,"Row1");
	Write_String(14,1,0,"rgroener@");
	Write_String(14,2,0,"mailbox.org");
	

	while(1)
	{ 		
		
	} //end while
}//end of main


void SPI_MasterInit(void)
{
	/* Set MOSI and SCK output, all others input */
	//DDRB = (1<<MOSI)|(1<<SCK);
	/* Enable SPI, Master, set clock rate fck/16 */
	SPCR0 = (1<<SPE)|(1<<MSTR)|(1<<SPR0);
}
void SPI_MasterTransmit(char cData)
{
	/* Start transmission */
	SPDR0 = cData;
	/* Wait for transmission complete */
	while(!(SPSR0 & (1<<SPIF)));
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
void Write_Char(uint8_t fontsize, char n)
{
	const char *fontpointer=0;
	uint8_t x;
	
	switch(fontsize)		//variable height and fixed width (8 Pixel)
	{
		case 8:fontpointer=font8;break;
		case 14:fontpointer=font14;break;
	}
	n-=0x21;			//jump to position in asci table
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

