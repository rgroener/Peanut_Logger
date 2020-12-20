/*	Header File
 * 
 * 	Sensirion SHT21 Sensor (Temperature / Humidity)
 *
 * 	grn; Apr 15
 */

uint8_t sht21_init(void);
uint16_t sht21_read_temp(void);
uint16_t sht21_read_hum(void);
