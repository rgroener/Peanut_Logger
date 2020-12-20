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
	uint16_t messwert;
	int16_t rueckgabewert;
	
	messwert=0;
	rueckgabewert=0;

		
	TWIStart();							//start TWI
	if(TWIGetStatus() != 0x08)return 11; //kontrolle ob erfolgreich sonst Abbruch mit Error Code
	TWIWrite(SHT21_W);					//Adresse und Schreiben
	if(TWIGetStatus() != 0x18)return 22;	//kontrolle ob erfolgreich sonst Abbruch mit Error Code
	switch(measure_mode)
	{
		case T_HOLD:	TWIWrite(SHT21_TEMP_HOLDMASTER);break;	//Modus = Temperatur master hold = on
		case RH_HOLD:	TWIWrite(SHT21_HUM_HOLDMASTER);break;	//Modus = Temperatur master hold = on
	}
	if(TWIGetStatus() != 0x28)return 33;	//kontrolle ob erfolgreich sonst Abbruch mit Error Code

	DDRC &= ~(1<<PC5);					//set SCL as Input
	while(!(PINC &= ~(1<<PC5)));		//wait to end conversion (Master hold mode)
	DDRC |= (1<<PC5);					//set SCL as Output

	TWIStart();							//restart TWI
	if(TWIGetStatus() != 0x10)return 44; //kontrolle ob erfolgreich sonst Abbruch mit Error Code
	TWIWrite(SHT21_R);					//Adresse und lesen
	if(TWIGetStatus() != 0x40)return 55;	//kontrolle ob erfolgreich sonst Abbruch mit Error Code
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

