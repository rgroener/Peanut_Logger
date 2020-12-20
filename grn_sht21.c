/*	Library File
 * 
 * 	Sensirion SHT21 Sensor (Temperature / Humidity)
 *
 * 	grn; Apr 15
 */
 #define F_CPU 8000000UL                 // set the CPU clock
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include "grn_TWI.h"
#include "grn_sht21.h"

#define SHT21_W 0x80				//SHT21 Adresse und schreiben 	0x80
#define SHT21_R 0x81				//SHT21 Adresse und lesen		0x81
#define SHT21_TEMP_HOLDMASTER 0xE3		//Tempreature hold master = on =>0b11100011
#define SHT21_HUM_HOLDMASTER 0xF3		//Humidity hold master = on => 0b11100101
#define SHT21_SOFTRESET 0xFE				//Softreset
#define POLYNOMINAL 0x131				//P(x) = x^8+x^5+x^4+1 = 0b100110001
#define T_HOLD 	0
#define RH_HOLD 	1

typedef float ft;						//Float VAriable zur Umrechnung vor Rueckgabe
uint8_t sht21_init(void)
{
	/*	
	 * Softreset Sensor
	 */
	TWIStart();
	if(TWIGetStatus() != 0x08)return 1; 
	TWIWrite(SHT21_W);
	if(TWIGetStatus() != 0x18)return 2;
	TWIWrite(SHT21_SOFTRESET);
	if(TWIGetStatus() != 0x28)return 3;
	TWIStop();
	_delay_ms(15);	//Startuptime after Reset <15ms (Datasheet)
	return 0;
}
int16_t sht21_measure(uint8_t measure_mode)
{
	/*
	 * 	Temperaturmessung / Überprüfung der Checksumme	
	 * 
	 * 	Rückgabewert: 	16 bit Temperaturwert x 100 
	 * 					(letzte 2 Stellen sind Nachkommastellen)
	 * 
	 * For float as return value, you have to change the following line in 
	 * the Makefile and add mat.h library since only a minimalistic sprintf library is included
	 * by default. To add the full float library add/change the following 
	 * line to the Makefile.
	 * 
	 * # Minimalistic printf version
		PRINTF_LIB_MIN = -Wl,-u,vfprintf -lprintf_min

		# Floating point printf version (requires MATH_LIB = -lm below)
		PRINTF_LIB_FLOAT = -Wl,-u,vfprintf -lprintf_flt


		PRINTF_LIB = $(PRINTF_LIB_FLOAT) 
	 * 
	 * 
	 * 	Checksum error: Rückgabewert = 99;
	 */ 
	 
	uint8_t raw[]={0,0,0};
	uint16_t messwert,i;
	int16_t rueckgabewert;
	uint8_t ret=0;
	
	messwert=0;
	rueckgabewert=0;
	/*
0x00	No errors
0x08	START condition transmitted
0x10	Repeated START condition transmitted
0x18	SLA+W transmitted; ACK received
0x20	SLA+W transmitted; NAK received
0x28	Data byte transmitted; ACK received
0x30	Data byte transmitted; NAK received
0x38	Arbitration lost in SLA; or NAK received
0x40	SLA+R transmitted; ACK received
0x48	SLA+R transmitted; NAK received
0x50	Data received; ACK has been returned
0x58	Data received; NAK has been returned
0xE0	Arbitration lost
0xE1	Arbitration lost in START
0xE2	Arbitration lost in STOP
0xE3	Arbitration lost in read ACK
0xE4	Arbitration lost in read NAK
0xE5	Arbitration lost in write
0xF8	Unknown error
0xFF	Illegal START or STOP condition
	*/
	TWIStart();		
	ret=TWIGetStatus();						//start TWI
	if(ret != 0x08)return ret; 	//start condition transmitted?
	TWIWrite(SHT21_W);			//send write adress
	if(TWIGetStatus() != 0x18)return 22;	//SLA+W transmitted?
	switch(measure_mode)
	{
		case T_HOLD:	TWIWrite(SHT21_TEMP_HOLDMASTER);break;	//Modus = Temperatur master hold = on
		case RH_HOLD:	TWIWrite(SHT21_HUM_HOLDMASTER);break;	//Modus = Temperatur master hold = on
	}
	if(TWIGetStatus() != 0x28)return 33;	//kontrolle ob erfolgreich sonst Abbruch mit Error Code
	TWIStart();								//restart TWI
	if(TWIGetStatus() != 0x10)return 44; 	//kontrolle ob erfolgreich sonst Abbruch mit Error Code
	TWIWrite(SHT21_R);						//Adresse und lesen
	if(TWIGetStatus() != 0x40)return 55;	//kontrolle ob erfolgreich sonst Abbruch mit Error Code
	
	DDRC &= ~(1<<PC5);					//set SCL as Input
	for(i=0;i<1000;i++)					//wait for timeout or
	{
		if(PINC &= ~(1<<PC5))break;		//wait for end of conversion (Master hold mode)
	}
	DDRC |= (1<<PC5);					//set SCL as Output again
	
	raw[0] = TWIReadACK();				//empfange MSB
	if(TWIGetStatus() != 0x50)return 66;	//kontrolle ob erfolgreich sonst Abbruch mit Error Code
	raw[1] = TWIReadACK();				//empfange LSB / durch NACK wird checksumme nicht empfangen
	if(TWIGetStatus() != 0x50)return 77;	//kontrolle ob erfolgreich sonst Abbruch mit Error Code
	raw[2] = TWIReadNACK();				//receive checksum
	TWIStop();							//close TWI
	
	messwert=(raw[0]<<8)|raw[1];						//8 bit werte zu 16bit Wert zusammensetzen
	messwert &= ~0x003;								//Loescht letzte 2 Bits (Status Bits)
	switch(measure_mode)
	{
		case T_HOLD:	rueckgabewert = 100*(-46.85 + 175.72/65536 * messwert);break;
		case RH_HOLD:	rueckgabewert = 100*(-6.0 + 125.0/65536 * messwert);break; // return relative humidity;break;	//Modus = Temperatur master hold = on
	}
	 
	if(sht21_checksum(raw,2,raw[2])) 	//check result with checksum
	{
		return 1;
	}else return rueckgabewert;	
}//End of SHT21_read_temp

uint8_t sht21_checksum(uint8_t data[], uint8_t no_of_bytes, uint8_t checksum) 
{
	uint8_t crc = 0;	
  	uint8_t byteCtr;

 	 //calculates 8-Bit checksum with given polynomial
  	for (byteCtr = 0; byteCtr < no_of_bytes; ++byteCtr)
 	 { crc ^= (data[byteCtr]);
 	   for (uint8_t bit = 8; bit > 0; --bit)
 	   { if (crc & 0x80) crc = (crc << 1) ^ POLYNOMINAL;
 	     else crc = (crc << 1);
 	   }
 	 }
 	 if (crc != checksum) return 1;
 	 else return 0;
}

