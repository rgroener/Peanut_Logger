/*
 * Library for ssd1306 graficcontroller
 * source: 
 */
#ifndef SSD1306_H
#define SSD1306_H
#endif
#define F_CPU 8000000UL// set the CPU clock
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <avr/io.h>
#include <stdio.h>                 
#include <avr/pgmspace.h>
#include "spi.h"
#include "grn_TWI.h"

extern const char font8[728];
extern const char font14[1274];

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

void Display_Init(void);
void send_data(char data);
void send_command(char command);
void Set_Page_Address(unsigned char add);
void Set_Column_Address(unsigned char add);
void Set_Contrast_Control_Register(unsigned char mod);
void Display_Picture(uint8_t with, uint8_t height,const unsigned char pic[]);
void Display_Clear(void);
void Write_Char(uint8_t fontsize, char n);
void Char_Position(uint8_t fontsize, uint8_t row, uint8_t pos);
void Write_Char(uint8_t fontsize, char n);
void Write_String(uint8_t fontsize, uint8_t row, uint8_t pos, const char str[]); 
