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
#include "DPS310.h"
#include "EEPROM_64.h"

#define BUTTON	(!(PIND & (1<<PD3))) && (entprell==0) //read button input
#define TXLED_EIN PORTD &= ~(1<<PD1)
#define TXLED_AUS PORTD |= (1<<PD1)
#define RXLED_AUS PORTD &= ~(1<<PD0)
#define RXLED_EIN PORTD |= (1<<PD0)
#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 8UL)))-1)
#define RELOAD_ENTPRELL 40
//#define TOGGLEMAX 4
#define TEMPERATURE 0
#define HUMIDITY 1
#define UART_BAUD_RATE      9600 

uint16_t eeposition=0;
uint16_t eepos_max=0;
int16_t temperature=0;
uint32_t pres=0;
int32_t pressure=0;
uint16_t vor_komma(uint32_t value);
uint8_t nach_komma(uint32_t value);
int16_t temp=0;
uint16_t test=0;
uint16_t hum=0;
volatile uint8_t ms10,ms100,sec,min, entprell;
volatile uint8_t screentoggle, toggle, toggle_alt;
char buffer[20]; // buffer to store string
char buff[20]; // buffer to store string
enum state {LOGO, GREETER, ZERO, MEASURE, LOGGING};
uint8_t state;

int32_t zero_alt=0;
int32_t altitude=0;
uint8_t log_flag=0;
int32_t diff_alt=0;

//TIMER
ISR (TIMER1_COMPA_vect);
uint8_t togtime;
//UART
void uart_send_char(char c);
void uart_send_string(volatile char *s);
void Display_Logo(void);
ISR(USART0_RX_vect);
ISR(USART0_RX_vect)
{
	char received_byte;
	received_byte = UDR0;
	UDR0 = received_byte;//Echo Byte

}//end of USART_rx 

int32_t calc_altitude(int32_t zpres, int32_t apres)
{
	// Add these to the top of your program
	const float p0 = 101725;     // Pressure at sea level (Pa)
	float altitude;

  // Add this into loop(), after you've calculated the pressure
  altitude = (float)44330 * (1 - pow(((float) apres/p0), 0.190295));
  
  return altitude;
}

 

int main(void)
{
	DDRB 	|= (1<<PB0) | (1<<PB1) | (1<<PB2) | (1<<PB3) | (1<<PB5);//set CS_DISP and RES and D/C output
	PORTB	|= (1<<PB0) | (1<<PB1) | (1<<PB2) | (1<<PB3) | (1<<PB5);//set CS_DISP and RES and D/C high
	
	//TWI
	DDRC |= (1<<PC5) | (1<<PC4);	//set pins for SCL und SDA as output
	PORTC |= (1<<PC5) | (1<<PC4);	//set pins high
	PORTC &= ~(1<<PC5);
	
	DDRD |= (1<<PD1)| (1<<PD0);//set TX0 and RX as output
	TXLED_AUS;
	RXLED_AUS;

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
    
    //UART0
    UCSR0B = (1<<RXEN0) | (1<<TXEN0) | (1<<RXCIE0);	//Turn on RX and TX circuits RXCIE0 enables Interrupt when byte received
	UCSR0C = (1<<UCSZ01) | (1<<UCSZ00);	//8-Bit Char size
	UBRR0H = (BAUD_PRESCALE >> 8);	//load upper 8-Bits of baud rate value into high byte of UBRR0H
	UBRR0L = BAUD_PRESCALE;			//load lower 8-Bits of Baud rate into low byte of UBRR0L

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
    togtime=0;
		
	Display_Init();
	Display_Clear();
	Set_Page_Address(0);
    Set_Column_Address(0);
    TWIInit();
    _delay_ms(50);
	sht21_init();
	DPS310_init(ULTRA);
	
	state = MEASURE;
	Display_Eeprom(2);
	while(1);
	while(1)
	{ 	
		switch(state)
		{
			case LOGO:		Display_Logo();
							break;
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
							
			case MEASURE:	pres=DPS310_get_pres();
							sprintf(buffer,"%ld",pres);
							Write_String(14,0,0,buffer);
							sprintf(buffer,"%ld",zero_alt);
							Write_String(14,1,0,buffer);
							
							altitude=calc_altitude(zero_alt, pres);
							sprintf(buffer,"%ld",altitude);
							Write_String(14,2,0,buffer);
							
							if(BUTTON)
							{
								entprell=RELOAD_ENTPRELL;
								Display_Clear();
								state=LOGGING;
							}
							break;
			case LOGGING:	
							break;
		}//End of switch(state)	
		
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
		screentoggle++;
		if(screentoggle==togtime)
		{
			screentoggle=0;
			if(toggle==0)
			{
				toggle = 1;
			}else toggle =0;
		}
		
	}
    if(ms100==10)	//sec
	{
		ms100=0;
		sec++;
		//change display screen in fixed time
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

void uart_send_char(char c)
{
	while((UCSR0A & (1<<UDRE0)) == 0){};
    UDR0 = c;
}
void uart_send_string(volatile char *s)
{
	while(*s != 0x00)
	{
		uart_send_char(*s);
		s++;
	}
}//end of send_string
void Display_Logo(void)
{
	//draw Peanut Logger Logo at fix position
	Display_Picture(40,48,woodstock);
	Write_String(8,0,6,"PL");
	Write_String(8,1,6,"EO");
	Write_String(8,2,6,"AG");
	Write_String(8,3,6,"NG");
	Write_String(8,4,6,"UE");
	Write_String(8,5,6,"TR");
}
