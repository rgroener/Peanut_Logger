#include "EEPROM_64.h"
//last Byte at address 8191
uint8_t ext_ee_random_read_8(uint16_t address)
{
	uint8_t add_high,add_low;
	add_high=address >> 8;		//shift high-byte to variable
	add_low=(uint8_t) address;	//cast low-byte to variable
	uint8_t result=0;
		
	TWIStart();
	if(TWIGetStatus() != 0x08)return 1;
	TWIWrite(EEPROM64_W);				//send write address
	if(TWIGetStatus() != 0x18)return 2;
	TWIWrite(add_high);					//send high byte
	if(TWIGetStatus() != 0x28)return 3;
	TWIWrite(add_low);					//send low byte
	if(TWIGetStatus() != 0x28)return 4;
	TWIStart();							//resend start
	if(TWIGetStatus() != 0x10)return 5; //repetet Start sent?
	TWIWrite(EEPROM64_R);
	if(TWIGetStatus() != 0x40)return 6;
	result=TWIReadNACK();				//read byte
	TWIStop();
	return result;	
}//end of ext_ee_read()
uint16_t ext_ee_random_read_16(uint16_t address)
{
	uint8_t add_high,add_low;
	add_high=address >> 8;		//shift high-byte to variable
	add_low=(uint8_t) address;	//cast low-byte to variable
	uint16_t result=0;
	uint8_t res_low, res_high;
	res_low=0;
	res_high=0;	
		
	TWIStart();
	if(TWIGetStatus() != 0x08)return 7;
	TWIWrite(EEPROM64_W);				//send write address
	if(TWIGetStatus() != 0x18)return 8;
	TWIWrite(add_high);					//send high byte
	if(TWIGetStatus() != 0x28)return 9;
	TWIWrite(add_low);					//send low byte
	if(TWIGetStatus() != 0x28)return 10;
	TWIStart();							//resend start
	if(TWIGetStatus() != 0x10)return 11; //repetet Start sent?
	TWIWrite(EEPROM64_R);
	if(TWIGetStatus() != 0x40)return 13;
	res_high=TWIReadACK();				//read byte
	if(TWIGetStatus() != 0x50)return 15;
	res_low=TWIReadNACK();				//read byte
	TWIStop();
	
	result = ( res_high << 8) | ( 0x00ff & res_low);//build 16 variable from 2 bytes
	
	return result;	
}//end of ext_ee_read()
uint32_t ext_ee_random_read_24(uint16_t address)
{
	uint8_t add_high,add_low;
	add_high=address >> 8;		//shift high-byte to variable
	add_low=(uint8_t) address;	//cast low-byte to variable
	uint32_t result=0;
	uint8_t res_low, res_high,res_msb;
	res_low=0;
	res_high=0;	
	res_msb=0;
		
	TWIStart();
	if(TWIGetStatus() != 0x08)return 7;
	TWIWrite(EEPROM64_W);				//send write address
	if(TWIGetStatus() != 0x18)return 8;
	TWIWrite(add_high);					//send high byte
	if(TWIGetStatus() != 0x28)return 9;
	TWIWrite(add_low);					//send low byte
	if(TWIGetStatus() != 0x28)return 10;
	TWIStart();							//resend start
	if(TWIGetStatus() != 0x10)return 11; //repetet Start sent?
	TWIWrite(EEPROM64_R);
	if(TWIGetStatus() != 0x40)return 13;
	res_msb=TWIReadACK();
	if(TWIGetStatus() != 0x50)return 15;
	res_high=TWIReadACK();				//read byte
	if(TWIGetStatus() != 0x50)return 15;
	res_low=TWIReadNACK();				//read byte
	TWIStop();
	/*weight  = ram_tmp_l;
	weight |= (ram_tmp_m <<  8);
	weight |= ((uint32_t)ram_tmp_h << 16); 
	*/
	
	//build 24 variable from 3 bytes
	result = res_low;
	result |= ((uint32_t)res_high << 8);
	result |= ((uint32_t)res_msb << 16);
	result |= ((uint32_t)0x00 << 24);
	
	return result;	
}//end of ext_ee_read()
uint8_t ext_ee_check_data_8(uint16_t address, uint8_t data)
{
	//Check if data at current address is equal data to be written.
	//If so, no need to overwrite existing data
	if(data == ext_ee_random_read_8(address))return 1; //data exist
	return 0; //new data
}
uint8_t ext_ee_random_write_8(uint16_t address, uint8_t data)
{
	//write 8 Bit data at given address
	
		uint8_t add_high,add_low;
		add_high=address >> 8;		//shift high-byte to variable
		add_low=(uint8_t) address;	//cast low-byte to variable
			
		TWIStart();
		if(TWIGetStatus() != 0x08)return 22;
		TWIWrite(EEPROM64_W);				//send write address
		if(TWIGetStatus() != 0x18)return 23;
		TWIWrite(add_high);					//send high byte
		if(TWIGetStatus() != 0x28)return 34;
		TWIWrite(add_low);					//send low byte
		if(TWIGetStatus() != 0x28)return 35;
		TWIWrite(data);
		if(TWIGetStatus() != 0x28)return 56;//data received acknowledged
		TWIStop();
		_delay_ms(50);//give eeprom time to write
		return 0;
}
uint8_t ext_ee_check_data_16(uint16_t address, uint16_t data)
{
	//Check if data at current address is equal data to be written.
	//If so, no need to overwrite existing data
	if(data == ext_ee_random_read_16(address))return 1; //data exist
	return 0; //new data
}
uint8_t ext_ee_random_write_16(uint16_t address, uint16_t data)
{
	//Write 16Bit data at given address
	
	uint8_t add_high,add_low;
	add_high=address >> 8;		//shift high-byte to variable
	add_low=(uint8_t) address;	//cast low-byte to variable
	
	uint8_t data_high, data_low;
	data_high = data >> 8;
	data_low = (uint8_t) data;
		
	TWIStart();
	if(TWIGetStatus() != 0x08)return 22;
	TWIWrite(EEPROM64_W);				//send write address
	if(TWIGetStatus() != 0x18)return 55;
	TWIWrite(add_high);					//send high byte
	if(TWIGetStatus() != 0x28)return 66;
	TWIWrite(add_low);					//send low byte
	if(TWIGetStatus() != 0x28)return 77;
	TWIWrite(data_high);
	if(TWIGetStatus() != 0x28)return 88;//data received acknowledged
	TWIWrite(data_low);
	if(TWIGetStatus() != 0x28)return 99;//data received acknowledged
	TWIStop();
	_delay_ms(50);//give eeprom time to write
	return 0;
	
}
uint8_t ext_ee_random_write_24(uint16_t address, uint32_t data)
{
	//Write 16Bit data at given address
	
	uint8_t add_high,add_low;
	add_high=address >> 8;		//shift high-byte to variable
	add_low=(uint8_t) address;	//cast low-byte to variable
	
	uint8_t data_high, data_low, data_msb;
	data_msb =	data >> 16;
	data_high = data >> 8;
	data_low = (uint8_t) data;
		
	TWIStart();
	if(TWIGetStatus() != 0x08)return 22;
	TWIWrite(EEPROM64_W);				//send write address
	if(TWIGetStatus() != 0x18)return 55;
	TWIWrite(add_high);					//send high byte
	if(TWIGetStatus() != 0x28)return 66;
	TWIWrite(add_low);					//send low byte
	if(TWIGetStatus() != 0x28)return 77;
	TWIWrite(data_msb);
	if(TWIGetStatus() != 0x28)return 87;//data received acknowledged
	TWIWrite(data_high);
	if(TWIGetStatus() != 0x28)return 88;//data received acknowledged
	TWIWrite(data_low);
	if(TWIGetStatus() != 0x28)return 99;//data received acknowledged
	TWIStop();
	_delay_ms(100);//give eeprom time to write
	return 0;
	
}
/* status codes
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
