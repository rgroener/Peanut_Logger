#ifndef EEPROM_64_H
#define EEPROM_64_H
#endif
#define F_CPU 8000000UL                 // set the CPU clock
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include "grn_TWI.h"

#define EEPROM64_W 0xA0	//24LC64 Eeprom Write address 
#define EEPROM64_R 0xA1	//24LC64 Eeprom Read address 

uint8_t ext_ee_random_read(uint16_t address);
uint8_t ext_ee_current_read(uint16_t address);
uint8_t ext_ee_random_write(uint16_t address, uint8_t data);
uint8_t ext_ee_current_write(uint16_t address, uint8_t data);
