/*	Header File
 * 
 * 	Sensirion SHT21 Sensor (Temperature / Humidity)
 *
 * 	grn; Apr 15
 */

uint8_t sht21_init(void);
int16_t sht21_measure(uint8_t mode);

