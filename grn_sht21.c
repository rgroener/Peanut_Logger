/*	Library File
 * 
 * 	Sensirion SHT21 Sensor (Temperature / Humidity)
 *
 * 	grn; Apr 15
 * 
 * 	Bemerkung: 	- momentan nur positive Temparaturen
 * 				- noch nicht alle Funktionen von Sensor implementiert
 * 				- Messungen mit defaul Werten (14Bit Auflösung)
 * 
 */
#define F_CPU 8000000UL// set the CPU clock
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include "grn_TWI.h"
#include "grn_sht21.h"

#define SHT21_W 0x80				//SHT21 Adresse und schreiben 0b10000000
#define SHT21_R 0x81			//SHT21 Adresse und lesen 0b10000001
#define SHT21_TEMP_HOLDMASTER 0xE3//Tempreature hold master = on
#define SHT21_HUM_HOLDMASTER 0xE5	//Humidity hold master = on
#define SHT21_SOFTRESET 0xFE		//Softreset
#define POLYNOMINAL 0x131				//P(x) = x^8+x^5+x^4+1 = 0b100110001

#define TEMPERATURE 	0
#define HUMIDITY		1

typedef float ft;						//Float VAriable zur Umrechnung vor Rueckgabe

int16_t sht21_measure(uint8_t mode)
{
	uint8_t bit_l, bit_h, crc, checksum;
	int16_t messwert, rueckgabewert;
	bit_l=0;
	bit_h=0;
	checksum = 0;
	messwert=0;
	rueckgabewert=0;
	crc=0;
		
	TWIStart();							//start TWI
	if(TWIGetStatus() != 0x08)return 4; //kontrolle ob erfolgreich sonst Abbruch mit Error Code
	TWIWrite(SHT21_W);					//Adresse und Schreiben
	if(TWIGetStatus() != 0x18)return 5;	//kontrolle ob erfolgreich sonst Abbruch mit Error Code
	switch(mode)
	{
		case TEMPERATURE:	TWIWrite(SHT21_TEMP_HOLDMASTER);	//Modus = Temperatur master hold = on
							break;
		case HUMIDITY:		TWIWrite(SHT21_HUM_HOLDMASTER);			//Modus = Feuchtigkeitsmessung master hold = on
							break;
	}
		
	if(TWIGetStatus() != 0x28)return 6;	//kontrolle ob erfolgreich sonst Abbruch mit Error Code
	TWIStart();							//restart TWI
	if(TWIGetStatus() != 0x10)return 7; //kontrolle ob erfolgreich sonst Abbruch mit Error Code
	TWIWrite(SHT21_R);					//Adresse und lesen
	if(TWIGetStatus() != 0x40)return 8;	//kontrolle ob erfolgreich sonst Abbruch mit Error Code
		
	bit_h = TWIReadACK();				//empfange MSB
	if(TWIGetStatus() != 0x50)return 9;	//kontrolle ob erfolgreich sonst Abbruch mit Error Code
	bit_l = TWIReadACK();				//empfange LSB
	if(TWIGetStatus() != 0x50)return 10;//kontrolle ob erfolgreich sonst Abbruch mit Error Code
	checksum = TWIReadNACK();			//empfange Checksum
	if(TWIGetStatus() != 0x58)return 11;//kontrolle ob erfolgreich sonst Abbruch mit Error Code
	TWIStop();							//schliesse TWI
	
	/*
	 * 	Checksummenprüfung
	 */	
		crc ^= bit_h;
		for(uint8_t x=8; x>0;--x)
		{
			if(crc & 0x80)
			{
				crc = (crc<<1) ^ POLYNOMINAL;
			}else
			{
				crc=(crc<<1);
			}
		} 
		crc ^= bit_l;
		for(uint8_t x=8; x>0;--x)
		{
			if(crc & 0x80)
			{
				crc = (crc<<1) ^ POLYNOMINAL;
			}else
			{
				crc=(crc<<1);
			}
		} 
		if(crc!=checksum) return 99;
				
	 /*
	 * Berechnung Temperatur und Rückgabewert
	 */
		messwert=(bit_h<<8)|bit_l;						//8 bit werte zu 16bit Wert zusammensetzen
		messwert &= ~0x003;								//Loescht letzte 2 Bits (Status Bits)
		
		switch(mode)
		{
			case TEMPERATURE:	rueckgabewert = 100*(-46.85 + 175.72/65536 * messwert);	//Berechnung Tempreatur
								break;
			case HUMIDITY:		rueckgabewert = 100*(-6.0 +125.0/65536 * messwert);//Berechnung Feuchtigkeit
								break;
		}
		return rueckgabewert;	
}//end of sht21_measure()

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
