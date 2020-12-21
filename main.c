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
#include "grn_TWI.h"
#include "spi.h"
#include "grn_sht21.h"
#include "ssd1306.h"

#define BUTTON	(!(PIND & (1<<PD3))) && (entprell==0) //read button input
#define RELOAD_ENTPRELL 40
#define TOGGLEMAX 4
#define TEMPERATURE 0
#define HUMIDITY 1

uint16_t vor_komma(uint32_t value);
uint8_t nach_komma(uint32_t value);
int16_t temp=0;
uint16_t test=0;
int16_t hum=0;

//TIMER
ISR (TIMER1_COMPA_vect);
uint16_t vor_komma(uint32_t value);
uint8_t nach_komma(uint32_t value);


enum state {GREETER, ZERO, MEASURE};
uint8_t state;
volatile uint8_t ms10,ms100,sec,min, entprell;
volatile uint8_t screentoggle, toggle, toggle_alt;
char buffer[20]; // buffer to store string
char buff[20]; // buffer to store string
int main(void)
{
	DDRB 	|= (1<<PB0) | (1<<PB1) | (1<<PB2) | (1<<PB3) | (1<<PB5);//set CS_DISP and RES and D/C output
	PORTB	|= (1<<PB0) | (1<<PB1) | (1<<PB2) | (1<<PB3) | (1<<PB5);//set CS_DISP and RES and D/C high
	
	//TWI
	DDRC |= (1<<PC5) | (1<<PC4);	//set pins for SCL und SDA as output
	PORTC |= (1<<PC5) | (1<<PC4);	//set pins high
	PORTC &= ~(1<<PC5);
	DDRD |= (1<<PD1)|(1<<PD2);//set TX0 and SDO as output
	PORTD |= (1<<PD1);
	PORTD &= ~(1<<PD2);

	DDRD &= ~(1<<PD3) | (1<<PD0);	//Button and RX0 as input(red LED)
	PORTD |= (1<<PD3);	//activate Pullup
	
	//init SPI as master without interrupt
	SPI_MasterInit();
    //Timer 1 Configuration
	OCR1A = 1249;	//OCR1A = 0x3D08;==1sec
	
    TCCR1B |= (1 << WGM12);
    // Mode 4, CTC on OCR1A

    TIMSK1 |= (1 << OCIE1A);
    //Set interrupt on compare match

    TCCR1B |= (1 << CS11) | (1 << CS10);
    // set prescaler to 64 and start the timer

    sei();
    // enable interrupts
    
    ms10=0;
    ms100=0;
    sec=0;
    min=0;
    entprell=0;
    screentoggle=3;
    toggle=0;
    toggle_alt=toggle;
		
	Display_Init();
	Display_Clear();
	Set_Page_Address(0);
    Set_Column_Address(0);
    TWIInit();
	sht21_init();
	//DPS310_init(LOW);
   state = MEASURE;
	//sprintf(buffer,"sec=%d",sec);
	//Write_String(14,0,0,"test");
	/*
	 * 
	 * SHT21 0x80 / 0x81
	 * DPS310 0xee / 0xef
	 * 
	 * */
	while(1)
	{ 	
		switch(state)
		{
			case GREETER:	if(BUTTON)
							{
								state=ZERO;
								entprell=RELOAD_ENTPRELL;
								Write_String(14,0,0," Button ");
								Write_String(14,1,0,"   to   ");
								Write_String(14,2,0, "  ZERO  ");
							}
							if(toggle)
							{
									Write_String(14,0,0,"If found");
									Write_String(14,1,0,"please  ");
									Write_String(14,2,0, "contact ");
							}else
							{
									Write_String(14,0,0,"rgroener");
									Write_String(14,1,0,"@mailbox");
									Write_String(14,2,0, ".org    ");	
							}
							break;
			case ZERO:		if(BUTTON)
							{
								state=MEASURE;
								entprell=RELOAD_ENTPRELL;
								Write_String(14,0,0," Button ");
								Write_String(14,1,0,"   to   ");
								Write_String(14,2,0, "  ZERO  ");
							}
							break;
			case MEASURE:	temp = sht21_measure(0);
							sprintf(buffer,"%d.%02d*",vor_komma(temp), nach_komma(temp));
							Write_String(14,0,0,buffer);
							
							hum = sht21_measure(1);
							sprintf(buff,"%d.%d%%",vor_komma(hum), nach_komma(hum));
							Write_String(14,1,0,buff);
							/*
							TWIStart();
							test=TWIGetStatus();
							sprintf(buffer,"%02X",test);
							Write_String(14,1,0,buffer);
							while(1);*/
							
							break;
		}//End of switch(state)	
			
		
		//sprintf(buffer,"sec=%d",test);
		//Write_String(14,1,0,buffer);
	} //end while
}//end of main


ISR (TIMER1_COMPA_vect)
{
	
		ms10++;
		if(entprell)entprell--;
			
	if(ms10==10)	//100ms
	{
		ms10=0;
		ms100++;
	
		
	}
    if(ms100==10)	//sec
	{
		ms100=0;
		sec++;
		//change display screen in fixed time
		screentoggle++;
		if(screentoggle==TOGGLEMAX)
		{
			screentoggle=0;
			if(toggle==0)
			{
				toggle = 1;
			}else toggle =0;
		}
	}
	if(sec==60)	//Minute
	{
		sec=0;
		min++;
	}
}
uint16_t vor_komma(uint32_t value)
{
	return value/100;
	
}
uint8_t nach_komma(uint32_t value)
{
	uint8_t temp;
	temp = value/100;
	return value-(temp*100);
	
	
}
