/*
 * Altitude logger for rubber powered models
 * 
 * grn Mai2022
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


#define READYLED_EIN	PORTD |=(1<<PD6)	//READYLED on
#define READYLED_AUS	PORTD &= ~(1<<PD6)	//READYLED off
#define BUTTON	(!(PIND & (1<<PD3))) && (entprell==0) //read button input
#define TXLED_EIN PORTD &= ~(1<<PD1)
#define TXLED_AUS PORTD |= (1<<PD1)
#define RXLED_AUS PORTD &= ~(1<<PD0)
#define RXLED_EIN PORTD |= (1<<PD0)
#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL)))-1)
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
 * 24LC64 	=> 64kBit 	=> 8 KByte 		=> (8k/4)2000 values (4Byte per 32Bit value) 
 * 24LC1026 => 1MBit	=>	1'026'000/8	=>	/4	=> 32'000 values
 */
#define EXTEEPROMSIZE 32000 //32000 for 1M eeprom / for 64k eeprom use 2000 

#define DISPLAYOFF	0
#define DISPLAYON	1	
#define FLIGHTOFF	0
#define FLIGHTON		1
#define ON 1
#define OFF 0
#define STARTDETECT 100	//detection altitude for start flag [cm]

uint16_t eepos=1;	//start at 1 to work with multiplication x4
int16_t eepos_max=1;
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
enum state {LOGO, MEMORY, APPEND, ZERO, LOGGING,FDATA, DOWNLOAD, FULLMEMO, CONT_DATA};
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
int8_t startdetect=0;
int8_t landdetect=0;
int8_t autoland=0; //automatic detect  landing?

uint8_t led_flash_time=0;
/*Logging frequency     64k		1M
 	10	=>	100ms  	=>	~3 min	~53 min
	20	=>	200ms	=>	~6 min	~1h46min
	25	=>	250m	=>	~8 min	~2h55min
 	50 	=> 	500ms	=>	~16 min	~4h30min
	100 => 	1s		=>	~33 min	~9h

change logintervall value according 
Table above*/
uint16_t logintervall=50;
uint16_t logintervall_counter=0;

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
uint16_t EEMEM eelast_intervall;
uint16_t intervall_faktor=1;
uint8_t maxflightnr=0;

uint8_t testvar=0;//for testing purpose
volatile uint8_t contdataticker=0;//ticks when to send uart data

uint16_t act_data_counter=0;//logcounter to show log numbers during logging

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
void reset_timer(void);

int main(void)
{
	
	init_system();
	init_var();
    TWIInit();
    _delay_ms(50);
	//sht21_init();
	DPS310_init(ULTRA);
		
	//Normal start case for logger
	//state = LOGO;
	//Display_Logo();
	
	//start case for sending continuous data
	//via UART
	state = CONT_DATA;
	Display_Clear();
	Write_String(14,0,0,"sending");
	Write_String(14,1,0,"live data");
	Write_String(14,2,0,"via UART");
	
	
#define DEBUB 0

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
									eeprom_update_word(&eememposition,logintervall);
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
										
			case ZERO:		//inidcate that ready to for take off
							READYLED_EIN;
							//set current altitude as reference
							if(BUTTON)
							{
								state=LOGGING;
								entprell=RELOAD_ENTPRELL;
								pres=DPS310_get_pres();
								altitude=calcalt(pres,QNH);
								zero_alt=altitude;
								reset_timer();
								flighttime=0;
								climbtime=0;
								startdetect=0;
								landdetect=0;
								Display_Clear();
								log_flag=1;//save value to log
								led_flash_time=10;//refill timer
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
								act_data_counter++;
								READYLED_EIN;
								//reset and wait for next logg_flag
								log_flag=0;
								//write altitude to propper eeprom positon
								ext_ee_random_write_32(eepos,diff_alt);
								if(display)
								{
									/*	no need to write to display
										if altitude has not changed
										since last time*/
									//if(altitude!=altitude_old)
									//{
										Display_Clear();
										sprintf(buffer,"%d",act_data_counter);
										Write_String(16,0,0,buffer);
										sprintf(buffer,"%ldm",vor_komma(ext_ee_random_read_32(eepos)));
										Write_String(16,2,0,buffer);
										altitude_old=altitude;//update altitude_old
									//}
								}//eof display
								eepos++;//increase position in eeprom
								eepos_max++;//move last pos in eeprom
								//if detection altitude is reached
								//set flag for start detection
								if(diff_alt>STARTDETECT)startdetect=1;
								//if autoland is enabled and altidute 
								//has fallen below Zero
								//landing has been detected and
								//logging will stop
								if(autoland && (diff_alt<0)&&startdetect)landdetect=1;
							}//eof log_flag
							//button indicate landing
							//display data of last flight
							if((BUTTON )|| landdetect)
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
								sprintf(buffer,"%d",act_data_counter);
								Write_String(16,0,0,buffer);
								Write_String(16,1,0,"STOP");
								sprintf(buffer,"%ldm",vor_komma(max_alt));
								Write_String(16,2,0,buffer);
								altitude_old=altitude;//update altitude_old
															
								/*
								Display_Clear();
								//climb time
								sprintf(buffer,") %lds", climbtime);
								Write_String(14,0,0,buffer);
								//display formatted max altitude
								disp_altitude(max_alt);
								//glide time
								sprintf(buffer,"(% lds", flighttime-climbtime);
								Write_String(14,2,0,buffer);
								* */
							}
						
							if(eepos>EXTEEPROMSIZE-2)
							{
								state=FULLMEMO;
								Display_Clear();
								Write_String(14,0,0,"Stopped");
								Write_String(14,1,0,"No Memo");
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
							logintervall=eeprom_read_word(&eelast_intervall);
							/* calculation of factor to print index
							 * as part of seconds*/
							switch(logintervall)
							{
								case 100:	//Intervall was 1 sec
											intervall_faktor=1;
											break;
								case 50:	//Intervall was 0.5 sec
											intervall_faktor=2;
											break;
								case 25:	//Intervall was 0.25 sec
											intervall_faktor=4;
											break;
								case 20:	//Intervall was 0.2 sec
											intervall_faktor=5;
											break;
								case 10:	//Intervall was 0.1 sec
											intervall_faktor=10;
											break;
							}
							
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
							uart_send_string("sec\t");
							for(uint8_t flugnummer=1;flugnummer<flightnr+1;flugnummer++)
							{
								uart_send_string("m\t");
							}
							uart_send_string("\nTime\t");
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
							sprintf(buffer,"maxdata %d\n",eepos_max);
							uart_send_string(buffer);
							uart_send_string("\n\n");
							for(uint32_t datanummer=1;datanummer<maxdata;datanummer++)
							{
								/* print index dependent on logintervall
								 * output in parts of seconds*/
								 
								if(logintervall==100)
								{								
									sprintf(buffer,"%ld\t",datanummer);
								}else
								{
									sprintf(buffer,"%ld.%d\t",\
									vor_komma(((uint32_t)datanummer*100)/intervall_faktor),\
									nach_komma(((uint32_t)datanummer*100)/intervall_faktor));
								}
								uart_send_string(buffer);
								for(uint8_t flugnummer=1;flugnummer<flightnr+1;flugnummer++)
								{
									if(datanummer>(eeprom_read_word(&eeflightlast[flugnummer])\
									-eeprom_read_word(&eeflightlast[flugnummer-1])))
									{
										//no more values for this flight
										uart_send_string("\t");
									}else
									{
										sprintf(buffer,"%ld.%d\t",\
										vor_komma(ext_ee_random_read_32(datanummer+eeprom_read_word(&eeflightlast[flugnummer-1]))),\
										nach_komma(ext_ee_random_read_32(datanummer+eeprom_read_word(&eeflightlast[flugnummer-1]))));
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
			case CONT_DATA: /*
								this state is only used to measure 
								* presure and temp data and send 
								* it via UART. Case not used in 
								* peanut logger mode.
							*/if(contdataticker >= 10)
							{
								pres=DPS310_get_pres();
								temp=DPS310_get_temp();
								
								sprintf(buffer,"%ld\t%d",pres,temp);
								uart_send_string(buffer);
								uart_send_string("\n\r");
								testvar++;
							}
							break;
		}//End of switch(state)	
	} //end while
}//end of main


ISR (TIMER1_COMPA_vect)
{
	ms10++;
	logintervall_counter++;
	if(led_flash_time!=0)
	{
		led_flash_time--;
	}else READYLED_AUS;
		
	if((state==LOGGING)&&(logintervall_counter==logintervall))
	{
		logintervall_counter=0;//reset counter
		log_flag=1;//save value to log
		led_flash_time=10;//refill timer
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
		contdataticker++;
		
		
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
		flighttime++;
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
	//Display_Eeprom(eepos_max,EXTEEPROMSIZE,EXTEEPROMSIZE-eepos_max);
	Display_Clear();
	Eeprom(data,max,0);
	Display_Picture(0,12,64,5,barscale);
	Write_String(8,3,0,"Runtime:");
	sprintf(buffer,"%ld",max);
	Write_String(8,5,0, buffer);
}
void disp_altitude(int32_t altitude)
{
	
		//max altitude
		sprintf(buffer,"%ld.%d", vor_komma(altitude),nach_komma(altitude));
		Write_String(16,1,0,buffer);
		
	
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
	
	DDRD |= (1<<PD6);//PD6 set output (READYLED)
	DDRD |= (1<<PD1);//set TX0 as output
	TXLED_AUS;
	RXLED_AUS;
	READYLED_AUS;

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
	reset_timer();
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
	if(eepos_max == -1)eepos_max=1;//gets rid of frame error
	eeprom_update_word(&eelast_intervall,logintervall);
}

void reset_timer(void)
{
	//reset all variables in Timer to zero
	ms10=0;
    ms100=0;
    sec=0;
    min=0;	
    logintervall_counter=0;
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
/*if(DEBUG)
							{
								// Sensor Test
								// Check if Sensorsoldering was 
								// succesful
								//measure presure
								pres=DPS310_get_pres();
								if(pres != pres_old)
								{				
									Display_Clear();			
									sprintf(buffer,"Pr %ld",pres);
									Write_String(14,0,0,buffer);
								}
								pres_old=pres;
								
								
							}//End of DEBUG 
							*/ 
