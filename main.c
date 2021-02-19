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
#include <avr/eeprom.h>

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
#define BUTTONWAIT 20	//delay before analysing counts of button
#define QNH 99300
/* Available places to store 
 * 32 bit values in external
 * eeprom
 * 
 * 24LC64 => 64kBit => 8 KByte => 2000 values (4Byte per 32Bit value) 
 */
#define EXTEEPROMSIZE 2000 

#define DISPLAYOFF	0
#define DISPLAYON	1	
#define FLIGHTOFF	0
#define FLIGHTON		1
#define ON 1
#define OFF 0

uint16_t eepos=1;	//start at 1 to work with multiplication x4
uint16_t eepos_max=1;
int16_t temperature=0;
uint32_t pres=0;
uint32_t pres_old=0;
int32_t pressure=0;
int32_t vor_komma(int32_t value);
uint8_t nach_komma(uint32_t value);
int16_t temp=0;
uint32_t test=0;
uint16_t hum=0;
volatile uint8_t ms10,ms100,sec,min, entprell;
volatile uint8_t screentoggle, toggle, toggle_alt;
char buffer[20]; // buffer to store string
char buff[20]; // buffer to store string
enum state {LOGO, MEMORY, APPEND, ZERO, LOGGING,FDATA, DOWNLOAD, FULLMEMO};
uint8_t state;
int32_t zero_alt=0;
int32_t altitude=0;
uint8_t log_flag=0;
int32_t diff_alt=0;
int32_t diff_alt_old=0;
int32_t altitude_old=0;
int32_t max_alt=0;//maximum altitude reached in flight
uint8_t display=1;//1 = display on / 0 = display off
uint32_t flighttime=0;//total flight time
uint32_t climbtime=0;//flight time in climb
int32_t reso=0;

uint16_t logcounter=0;

//Logging frequency
// 	50 => 500ms
//	100 => 1s
uint16_t logintervall=100;

//button count
uint16_t buttonwait=0;
uint8_t buttoncount=0;
//QNH
//TIMER
ISR (TIMER1_COMPA_vect);
uint8_t togtime;
//append Data?
uint16_t EEMEM eememposition; //Max Position 65535
/* 	Max flights to be saved
	value at array position
	is last position of this flightnr
*/	
uint16_t EEMEM eeflightlast[50];
uint16_t flightnr=0; //number of flights logged 
uint8_t EEMEM eemax_flightnr;//stored number of last flight
uint8_t maxflightnr=0;

//UART
void uart_send_char(char c);
void uart_send_string(volatile char *s);
void Display_Logo(void);
ISR(USART0_RX_vect);

uint32_t xxx=0;
void disp_altitude(int32_t altitude);
void Display_Eeprom(uint32_t data, uint32_t max,uint16_t ti);
void init_system(void);
void init_var(void);

int main(void)
{
	init_system();
	init_var();
    TWIInit();
    _delay_ms(50);
	sht21_init();
	DPS310_init(ULTRA);
	state = LOGO;
	Display_Logo();
	
	while(1)
	{ 	
		switch(state)
		{
			case LOGO:		if(BUTTON)
							{
								entprell=RELOAD_ENTPRELL;
								buttoncount++;
								//start wait time for buttoncount
								//at first activation of button
								if(buttonwait==0)buttonwait=BUTTONWAIT;
							}
							//analyse button counter
							if((buttoncount!=0) && (buttonwait==0))
							{
								//button pressed once
								if(buttoncount==1) 
								{
									//entprell=RELOAD_ENTPRELL;
									state=MEMORY;
									//show memory usage to decide 
									//for next flight
									Display_Eeprom(eepos_max,EXTEEPROMSIZE,EXTEEPROMSIZE-eepos_max);
								}else 
								//button pressed twice
								if(buttoncount==2)
								{
									//entprell=RELOAD_ENTPRELL;
									state=DOWNLOAD;
									Display_Clear();
									sprintf(buffer,"Button");
									Write_String(14,0,0,buffer);
									sprintf(buffer,"to");
									Write_String(14,1,0,buffer);
									sprintf(buffer,"Download");
									Write_String(14,2,0,buffer);
								}
								//reset for next process
								buttoncount=0;
								buttonwait=0;
							}break;
			
			case MEMORY:	if(BUTTON)
							{
								entprell=RELOAD_ENTPRELL;
								state=APPEND;
								Display_Clear();
								Write_String(14,0,0,"Flight");
								Write_String(14,1,0,"1 Append");
								Write_String(14,2,0,"2 Reset");
							}break;
									
			case APPEND:	if(BUTTON)
							{
								entprell=RELOAD_ENTPRELL;
								buttoncount++;
								//start wait time for buttoncount
								//at first activation of button
								if(buttonwait==0)buttonwait=BUTTONWAIT;
							}
							//analyse button counter
							if((buttoncount!=0) && (buttonwait==0))
							{
								//Button pressed once
								//Append new Data to Memory
								if(buttoncount==1) 
								{
									state=ZERO;
									entprell=RELOAD_ENTPRELL;
									//Read saved End of Data Position
									//from Eprom
									eepos_max=eeprom_read_word(&eememposition);
									//set start of new data to 
									//end of old data
									eepos=eepos_max;
									//update flightnr addording Eeprom
									flightnr=eeprom_read_byte(&eemax_flightnr);
									flightnr++;
								}else 
								//Button pressed twice
								//reset Memory
								if(buttoncount==2)
								{
									state=ZERO;
									entprell=RELOAD_ENTPRELL;
									//reset old Memory 
									//set position to start
									eepos=1;
									eepos_max=1;
									flightnr=1;
									eeprom_update_byte(&eemax_flightnr,1);
									eeprom_update_word(&eememposition,eepos_max);
									//save endposition in eeprom
									//of last flight
									eeprom_update_word(&eeflightlast[0], 0);
								}
								Display_Clear();
								
								Write_String(14,0,0,"Takeoff");
								Write_String(14,1,0,"Flight");
								sprintf(buffer,"Nr: %d",flightnr);
								Write_String(14,2,0,buffer);
																
								/*
								sprintf(buffer,"pos %d",eepos);
								Write_String(14,0,0,buffer);
								sprintf(buffer,"max %d",eepos_max);
								Write_String(14,1,0,buffer);
								sprintf(buffer,"nr %d",flightnr);
								Write_String(14,2,0,buffer);*/
								/*
								Write_String(14,0,0," Button ");
								Write_String(14,1,0,"   to   ");
								Write_String(14,2,0, "  ZERO  ");
								*/
								//reset for next process
								buttoncount=0;
								buttonwait=0;
							}
							break;
										
			case ZERO:		//set current altitude as reference
							if(BUTTON)
							{
								state=LOGGING;
								entprell=RELOAD_ENTPRELL;
								pres=DPS310_get_pres();
								altitude=calcalt(pres,QNH);
								zero_alt=altitude;
								flighttime=0;
								climbtime=0;
								Display_Clear();
							}
							break;
			case LOGGING:	//measure presure
							pres=DPS310_get_pres();
							//calculate altitude
							altitude=calcalt(pres,QNH);
							//calculate altitude difference 							
							diff_alt=altitude-zero_alt;
							//set new max altitude if higher
							if(diff_alt>max_alt)
							{
								max_alt=diff_alt;
								climbtime=flighttime;
							}
							if(log_flag)
							{
								//reset and wait for next logg_flag
								log_flag=0;
								//write altitude to propper eeprom positon
								ext_ee_random_write_32(eepos,diff_alt);
								if(display)
								{
									Display_Clear();
									sprintf(buffer,"%d",eepos);
									Write_String(14,0,0,buffer);
									disp_altitude(ext_ee_random_read_32(eepos));
									//sprintf(buffer,"%ld",);
									Write_String(14,1,0,buffer);
									sprintf(buffer,"%lds",flighttime);
									Write_String(14,2,0,buffer);
								}
								eepos++;//increase position in eeprom
								eepos_max++;//move last pos in eeprom
							}//eof log_flag
							//button indicate landing
							//display data of last flight
							if(BUTTON)
							{
								entprell=RELOAD_ENTPRELL;
								state=LOGO;
								/*
								EEMEM eeflightlast[50];
								flightnr=0; 
								EEMEM eemax_flightnr;
								*/
								//Set now End of data position 
								//in Eproom
								eeprom_update_word(&eememposition,eepos_max);
								//save endposition in eeprom
								//of last flight
								eeprom_update_word(&eeflightlast[flightnr], eepos_max-1);
								
								eeprom_update_byte(&eemax_flightnr,flightnr);
															
								
								Display_Clear();
								//climb time
								sprintf(buffer,") %lds", climbtime);
								Write_String(14,0,0,buffer);
								//display formatted max altitude
								disp_altitude(max_alt);
								//glide time
								sprintf(buffer,"(% lds", flighttime-climbtime);
								Write_String(14,2,0,buffer);
							}
							/*if(log_flag==1)
							{
								Display_Clear();
								log_flag=0;
								ext_ee_random_write_32(eepos,diff_alt);
								sprintf(buffer,"%d",eepos);
								Write_String(14,0,0,buffer);
								sprintf(buffer,"%ld",ext_ee_random_read_32(eepos));
								Write_String(14,1,0,buffer);
								eepos++;
								eepos_max++;
							}
							* */
							
							//only print if one of the values
							//has changed
							/*
							if((pres_old!=pres)||(altitude!=altitude_old))
							{
								//save old values
								pres_old=pres;
								altitude_old=altitude;
								//clear display
								Display_Clear();
								//print pressure
								sprintf(buffer,"%ld.%d",vor_komma(pres),nach_komma(pres));
								Write_String(14,0,0,buffer);
								//print current altitude
								sprintf(buffer,"%ld.%dm",vor_komma(altitude),nach_komma(altitude));
								Write_String(14,1,0,buffer);
								//print altitude difference
								sprintf(buffer,"%ld.%dm",vor_komma(diff_alt),nach_komma(diff_alt));
								Write_String(14,2,0,buffer);
							}//eof if(pres_old...)
							//check if button is pressed																				
							*/
							
							/*if(BUTTON)
							{
								entprell=RELOAD_ENTPRELL;
								buttoncount++;
								//start wait time for buttoncount
								//at first activation of button
								if(buttonwait==0)buttonwait=BUTTONWAIT;
							}
							//analyse button counter
							if((buttoncount!=0) && (buttonwait==0))
							{
								//button pressed once
								if(buttoncount==1) 
								{
									Display_Clear();
									sprintf(buffer,"Button");
									Write_String(14,0,0,buffer);
									sprintf(buffer,"  for  ");
									Write_String(14,0,0,buffer);
									sprintf(buffer," Data ");
									Write_String(14,0,0,buffer);
									state=LOGGING;
								}else 
								//button pressed twice
								if(buttoncount==2)
								{
									sprintf(buffer,"Zwei");
									Write_String(14,2,0,buffer);
									
									
									
								}
								//reset for next process
								buttoncount=0;
								buttonwait=0;
							}*/
							if(eepos==EXTEEPROMSIZE)
							{
								state=FULLMEMO;
								Display_Clear();
								Write_String(14,0,0,"No");
								Write_String(14,1,0,"Memory");
								Write_String(14,2,0,"left");								
							}
							
							break;
			
			case FDATA:		if(BUTTON)
							{
								entprell=RELOAD_ENTPRELL;
								state=MEMORY;
								//show memory usage to decide 
								//for next flight
								Display_Eeprom(eepos_max,EXTEEPROMSIZE,EXTEEPROMSIZE-eepos_max);
							}break;
							
			case DOWNLOAD:	Display_Clear();
							//get data from eprom		
							eepos_max=eeprom_read_word(&eememposition);
							eepos=eepos_max;
							flightnr=eeprom_read_byte(&eemax_flightnr);
							
							/*	uncomment for debugging						
							sprintf(buffer,"eepos_max %d\n",eepos_max);
							uart_send_string(buffer);
							sprintf(buffer,"flightnr %d\n",flightnr);
							uart_send_string(buffer);
							//list data numbers for all flights
							for(uint16_t ggg=1;ggg<flightnr+1;ggg++)
							{
								sprintf(buffer,"last flight %d\n",eeprom_read_word(&eeflightlast[ggg])-eeprom_read_word(&eeflightlast[ggg-1]));
								uart_send_string(buffer);
							}
							uart_send_string("\n\n");
							uart_send_string("\n\n");
							//send raw data list
							for(uint16_t ggg=1;ggg<eepos_max+1+1;ggg++)
							{
								sprintf(buffer,"%ld\n",ext_ee_random_read_32(ggg));
								uart_send_string(buffer);
							}
							//
							*/
							
							//***********************************
							//***********************************
							for(uint8_t flugnummer=1;flugnummer<flightnr+1;flugnummer++)
							{
								uart_send_string("m\t");
							}
							uart_send_string("\n");
							for(uint8_t flugnummer=1;flugnummer<flightnr+1;flugnummer++)
							{
								sprintf(buffer,"Flight%d\t",flugnummer);
								uart_send_string(buffer);
							}
							uart_send_string("\n\n");
							
							//find flight with the most data stored
							uint16_t maxdata=0;
							for(uint16_t ggg=1;ggg<flightnr+1;ggg++)
							{
								if(maxdata<eeprom_read_word(&eeflightlast[ggg])-eeprom_read_word(&eeflightlast[ggg-1]))maxdata=eeprom_read_word(&eeflightlast[ggg])-eeprom_read_word(&eeflightlast[ggg-1]);
								
							}
							sprintf(buffer,"maxdata %d\n",maxdata);
							uart_send_string(buffer);
							
							uart_send_string("\n\n");
													
							for(uint8_t datanummer=1;datanummer<maxdata;datanummer++)
							{
								for(uint8_t flugnummer=1;flugnummer<flightnr+1;flugnummer++)
								{
									if(datanummer>(eeprom_read_word(&eeflightlast[flugnummer])-eeprom_read_word(&eeflightlast[flugnummer-1])))
									{
										//no more values for this flight
										uart_send_string("\t");
									}else
									{
										sprintf(buffer,"%ld\t",ext_ee_random_read_32(datanummer+eeprom_read_word(&eeflightlast[flugnummer-1])));
										uart_send_string(buffer);
									}
								}
								uart_send_string("\n");
							}
							state=LOGO;
							Display_Logo();
							
							break;
							
			case FULLMEMO:	if(BUTTON)
							{
								entprell=RELOAD_ENTPRELL;
								state=LOGO;
								//Set now End of data position 
								//in Eproom
								eeprom_update_word(&eememposition,eepos_max);
								//save endposition in eeprom
								//of last flight
								eeprom_update_word(&eeflightlast[flightnr], eepos_max);
								//set nr of next flight
								flightnr++;
								eeprom_update_byte(&eemax_flightnr,flightnr);
								Display_Clear();
								Display_Logo();
							}break;
		}//End of switch(state)	
	} //end while
}//end of main


ISR (TIMER1_COMPA_vect)
{
	ms10++;
	logcounter++;
	if(logintervall==logcounter)
	{
		if(state==LOGGING)
		{
			log_flag=1;//save value to log
			flighttime++;
		}	
		logcounter=0;
	}
	
	if(entprell)entprell--;
	if(ms10==10)	//100ms
	{
		ms10=0;
		ms100++;
		/*if button count process runs
		 * decrease wait variable till zero*/
		if((buttoncount!=0) && (buttonwait!=0))buttonwait--;
		
		if(xxx<951)xxx++;
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
		//set log_flag to 1 every second (logging intervall)
		//flight function will set it back to zero
		
		sec++;
		//change display screen in fixed time
	}
	if(sec==60)	//Minute
	{
		sec=0;
		min++;
	}
}
int32_t vor_komma(int32_t value)
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
	Display_Picture(0,0,40,48,woodstock);
	Write_String(8,0,6,"PL");
	Write_String(8,1,6,"EO");
	Write_String(8,2,6,"AG");
	Write_String(8,3,6,"NG");
	Write_String(8,4,6,"UE");
	Write_String(8,5,6,"TR");
}
void Display_Eeprom(uint32_t data, uint32_t max,uint16_t ti)
{
	Display_Clear();
	Eeprom(data,max,0);
	Display_Picture(0,12,64,5,barscale);
	Write_String(8,3,0,"Runtime:");
	sprintf(buffer,"%d sec",ti);
	Write_String(8,5,0, buffer);
}
void disp_altitude(int32_t altitude)
{
	
		//max altitude
		sprintf(buffer,"' %ld.%dm", vor_komma(altitude),nach_komma(altitude));
		Write_String(14,1,0,buffer);
	
}//eof disp_altitude
ISR(USART0_RX_vect)
{
	char received_byte;
	received_byte = UDR0;
	UDR0 = received_byte;//Echo Byte

}//end of USART_rx 
void init_system(void)
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
}
void init_var(void)
{
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
	eepos_max=eeprom_read_word(&eememposition);
}



/*
 * Analyse number of button clicks
 * 
if(BUTTON)
{
	entprell=RELOAD_ENTPRELL;
	buttoncount++;
	//start wait time for buttoncount
	//at first activation of button
	if(buttonwait==0)buttonwait=BUTTONWAIT;
}
//analyse button counter
if((buttoncount!=0) && (buttonwait==0))
{
	//button pressed once
	if(buttoncount==1) 
	{
		sprintf(buffer,"Eins");
		Write_String(14,2,0,buffer);
	}else 
	//button pressed twice
	if(buttoncount==2)
	{
		sprintf(buffer,"Zwei");
		Write_String(14,2,0,buffer);
	}
	//reset for next process
	buttoncount=0;
	buttonwait=0;
}
* 
***********************
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
*/
