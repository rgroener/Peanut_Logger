#ifndef DPS310_H
#define DPS310_H
#endif
#define F_CPU 8000000UL                 // set the CPU clock
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include "grn_TWI.h"

#define DEBUG 1
#define DPS310_W 0xee
#define DPS310_R 0xef
#define PRS_B2	0x00
#define PRS_B1	0x01
#define PRS_B0	0x02
#define TMP_B2 	0x03
#define TMP_B1	0x04
#define TMP_B0	0x05
#define PRS_CFG	0x06
#define TMP_CFG	0x07
#define MEAS_CFG	0x08
#define CFG_REG		0x09
#define INT_STS		0x0A
#define FIFO_STS	0x0B
#define RESET		0x0C
#define PRODUCT_ID	0x0D
#define COEF_SRCE	0x28


#define COEF_RDY 1
#define SENSOR_RDY 2
#define TMP_RDY 3
#define PRS_RDY 4
#define PROD_ID 5

#define LOW 1
#define MID 2
#define HIGH 3
#define ULTRA 4

//Compensation Scale Factors (Oversampling)
#define Scal_1 524288 //sinlge
#define Scal_2 1572864
#define Scal_4 3670016
#define Scal_8 7864320
#define Scal_16 253952
#define Scal_32 516096
#define Scal_64 1040384
#define Scal_128 2088960

#define SENS_OP_STATUS 0x08

#define MODE_STBY	0x00
#define MODE_COMMAND_PRES 0x01
#define MODE_COMMAND_TEMP 0x02
#define MODE_COMMAND_PRES_AND_TEMP 0x03
#define MODE_BACKGROUND_PRES 0x05
#define MODE_BACKGROUND_TEMP 0x06
#define MODE_BACKGROUND_PRES_AND_TEMP 0x07

#define SENSOR_READY_CHECK (DPS310_read(MEAS_CFG) & (1<<6)) != 0
#define COEFF_READY_CHECK (DPS310_read(MEAS_CFG) & (1<<7)) != 0
#define TEMP_READY_CHECK (DPS310_read(MEAS_CFG) & (1<<5)) != 0
#define PRES_READY_CHECK (DPS310_read(MEAS_CFG) & (1<<4)) != 0


//prototypen DPS310
uint8_t DPS310_read(uint8_t reg);
uint8_t DPS310_write(uint8_t reg, uint8_t data);
int16_t DPS310_readCoeffs(void);
void DPS310_sreset(void);
void DPS310_init(uint8_t acc);
uint32_t DPS310_get_sc_temp(uint8_t oversampling);
long DPS310_get_temp(uint8_t oversampling);
double DPS310_get_pres(uint8_t t_ovrs, uint8_t p_ovrs);
long calcalt(double press, uint32_t pressealevel);
