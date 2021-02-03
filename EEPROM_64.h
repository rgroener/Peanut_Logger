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

uint8_t ext_ee_random_read_8(uint16_t address);
uint8_t ext_ee_check_data_8(uint16_t address, uint8_t data);
uint16_t ext_ee_random_read_16(uint16_t address);
int32_t ext_ee_random_read_32(uint16_t eeposition);
uint8_t ext_ee_check_data_16(uint16_t address, uint16_t data);
uint8_t ext_ee_random_write_8(uint16_t address, uint8_t data);
uint8_t ext_ee_random_write_16(uint16_t address, uint16_t data);
uint8_t ext_ee_random_write_32(uint16_t eeposition, int32_t data);

