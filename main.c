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
#define TOGGLEMAX 4
#define TEMPERATURE 0
#define HUMIDITY 1

uint16_t eeposition=0;
int16_t temperature=0;
uint32_t pres=0;
uint16_t vor_komma(uint32_t value);
uint8_t nach_komma(uint32_t value);
int16_t temp=0;
uint16_t test=0;
uint16_t hum=0;

//TIMER
ISR (TIMER1_COMPA_vect);

//UART
void uart_send_char(char c);
void uart_send_string(volatile char *s);



enum state {GREETER, ZERO, MEASURE};
uint8_t state;
volatile uint8_t ms10,ms100,sec,min, entprell;
volatile uint8_t screentoggle, toggle, toggle_alt;
char buffer[20]; // buffer to store string
char buff[20]; // buffer to store string

/* 9600 baud / Geschwindikeit Uebertragung RS232 Schnittstelle*/
#define UART_BAUD_RATE      9600      
ISR(USART0_RX_vect)
{
	char received_byte;
	received_byte = UDR0;
	UDR0 = received_byte;//Echo Byte

}//end of USART_rx 

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
		
	Display_Init();
	Display_Clear();
	Set_Page_Address(0);
    Set_Column_Address(0);
    TWIInit();
    _delay_ms(50);
	sht21_init();
	DPS310_init(ULTRA);
	
	
   state = MEASURE;
	//sprintf(buffer,"sec=%d",sec);
	//Write_String(14,0,0,"test");
	/*
	 * 
	 * SHT21 	0x80 / 0x81
	 * DPS310 	0xee / 0xef
	 * 24LC64	0xA0 / 0xA1
	 * 
	 * */
	 
	//uart_send_string("Feuchtigkeit\tSHT21\tDPS310\n"); 
	//uart_send_string("%\tC\tC\n");
	/*
	uint16_t address=0xABCD;
	
	uint8_t add_high,add_low;
	add_high=address >> 8;
	add_low=(uint8_t) address;
	
	
	sprintf(buffer,"0x%2X",add_high);
	Write_String(14,1,0,buffer);
	
	sprintf(buffer,"0x%2X", add_low);
	Write_String(14,0,0,buffer);
	* 
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
			case MEASURE:	temperature=DPS310_get_temp();
							sprintf(buffer,"T=%d",temperature);
							Write_String(14,0,0,buffer);
							
							
							pres=DPS310_get_pres();
							sprintf(buffer,"%d.%d",vor_komma(pres),nach_komma(pres));
							Write_String(14,1,0,buffer);
							
							temp=sht21_measure(TEMPERATURE);
							sprintf(buffer,"T=%d",temp);
							Write_String(14,2,0,buffer);
			
			
			
			/*temp=sht21_measure(HUMIDITY);
							sprintf(buffer,"' %d.%d%%",vor_komma(temp),nach_komma(temp));
							Write_String(14,1,0,buffer);
							sprintf(buffer,"%d.%d\t",vor_komma(temp),nach_komma(temp));
							uart_send_string(buffer);
							
							temp=sht21_measure(TEMPERATURE);
							sprintf(buffer,") %d",temp);
							Write_String(14,0,0,buffer);
							sprintf(buffer,"%d.%d\t",vor_komma(temp),nach_komma(temp));
							uart_send_string(buffer);
			
							pres=DPS310_get_temp(1);
							sprintf(buffer,"%d",pres);
							Write_String(14,2,0,buffer);
							sprintf(buffer,"%d.%d\n",vor_komma(pres),nach_komma(pres));
							uart_send_string(buffer);
							
							
							//ext_ee_random_write_16(eeposition,temp);
							eeposition += 2;
							_delay_ms(1000);
							
			for(uint8_t x=0;x<255;x++)
							{
								TWIStart();
								TWIWrite(x);
								temp=TWIGetStatus();
								sprintf(buffer,"%2X",x);
								Write_String(14,0,0,buffer);
								TWIStop();
								
								if(temp==0x18)
								{
									sprintf(buffer,"%2X",x);
									Write_String(14,2,0,buffer);
									if(x==0xee)TXLED_EIN;
									if(x==0x80)RXLED_EIN;
								}
								
								_delay_ms(20);
				
							}*/
			
							/*temp = sht21_measure(0);
							
							hum = sht21_measure(1);
							sprintf(buff,"%d.%d%%",vor_komma(hum), nach_komma(hum));
							Write_String(14,1,0,buff);
							
							TWIStart();
							test=TWIGetStatus();
							sprintf(buffer,"%02X",test);
							Write_String(14,1,0,buffer);
							while(1);*/
							
							break;
		}//End of switch(state)	
			
		//uart_send_char('A');
		/*
		Yes, a single backslash in a C-string is an escape character for control codes.
		"\n" = newline
		"\r" = carriage return
		"\xhh" =  character with hexadecimal value hh.
		*/
		//_delay_ms(1000);
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
