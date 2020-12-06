/*
 * Altitude logger for rubber powered models
 * 
 * grn Dec/20
 * 
 * avrdude: safemode: Fuses OK (E:F7, H:D9, L:62)
 * */
#define F_CPU 1000000UL// set the CPU clock
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <avr/io.h>
#include <stdio.h>                 
#include <avr/pgmspace.h>

#define CS_DISP_ON 	PORTB &= ~(1<<PB0)	//select chip select display
#define CS_DISP_OFF 	PORTB |= (1<<PB0)	//deselect chip select display
#define DISP_DATA	PORTB |= (1<<PB2)	//set D/C to data
#define DISP_COMM	PORTB &= ~(1<<PB2)	//set D/C to command
#define DISP_RST_0	PORTB &= ~(1<<PB1)	//set Reset to 0
#define DISP_RST_1	PORTB |= (1<<PB1)	//set Reset to 1

/*
 * RES		PB1
 * CS_DISP	PB0
 * SCK		PB5
 * MISO		PB4
 * D/C		PB2
 * SDO		PD2
 * 
 * SDA		PC4
 * SCL		PC5
 * 
 * */

void SPI_MasterInit(void);
void SPI_MasterTransmit(char cData);
void send_data(char data);
void send_command(char command);

int main(void)
{
	DDRB 	|= (1<<PB0) | (1<<PB1) | (1<<PB2);//set CS_DISP and RES and D/C output
	PORTB	|= (1<<PB0) | (1<<PB1) | (1<<PB2);//set CS_DISP and RES and D/C high
	
	
	
	DDRB 	|= (1<<PB3);
	PORTB	|= (1<<PB3);
	
	DDRC |= (1<<PC5);
	PORTC |= (1<<PC5);
	PORTC &= ~(1<<PC5);
	
	
	
	//init SPI as master without interrupt
	SPI_MasterInit();
    // Enable Global Interrupts
    //sei();
		
	DISP_RST_1;
	_delay_ms(2000);
	DISP_RST_0;
	_delay_ms(2000);
	DISP_RST_1;
	_delay_ms(30000);
	PORTB &= ~(1<<PB2);


 	send_command(0xae);//--turn off oled panel
	send_command(0xd5);//--set display clock divide ratio/oscillator frequency
	send_command(0x80);//--set divide ratio
	send_command(0xa8);//--set multiplex ratio(1 to 64)
	send_command(0x3f);//--1/64 duty orig 3f
	send_command(0xd3);//-set display offset
	send_command(0x00);//-not offset
	send_command(0x8d);//--set Charge Pump enable/disable
	send_command(0x14);//--set(0x10) disable
	send_command(0x40);//--set start line address orig 40
	send_command(0xa6);//--set normal display
	send_command(0xa5);// Disable Entire Display On orig a4
	send_command(0xa1);//--set segment re-map 128 to 0
	send_command(0xC8);//--Set COM Output Scan Direction 64 to 0
	send_command(0xda);//--set com pins hardware configuration
	send_command(0x12);
	send_command(0x81);//--set contrast control register
	send_command(0x80);//orig 80
	send_command(0xd9);//--set pre-charge period
	send_command(0xf1);
	send_command(0xdb);//--set vcomh
	send_command(0x40);



	send_command(0xaf);//--turn on oled panel
	PORTB |= (1<<PB2);
		
	while(1)
	{ 
		
		send_data(0xa7);//invert display
		_delay_ms(2000);
		send_data(0xa6);//vormal display
		_delay_ms(2000);		
		
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
	CS_DISP_ON;
	SPI_MasterTransmit(data);
	CS_DISP_OFF;
}

void send_command(char command)
{
	DISP_COMM;
	CS_DISP_ON;
	SPI_MasterTransmit(command);
	CS_DISP_OFF;
}

