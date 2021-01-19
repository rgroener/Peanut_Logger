#include "EEPROM_64.h"


uint8_t ext_ee_random_read(uint16_t address)
{
	uint8_t add_high,add_low;
	add_high=address >> 8;		//shift high-byte to variable
	add_low=(uint8_t) address;	//cast low-byte to variable
	uint8_t result=0;
		
	TWIStart();
	if(TWIGetStatus() != 0x08)return 199;
	TWIWrite(EEPROM64_W);				//send write address
	if(TWIGetStatus() != 0x18)return 9;
	TWIWrite(add_high);					//send high byte
	if(TWIGetStatus() != 0x28)return 10;
	TWIWrite(add_low);					//send low byte
	if(TWIGetStatus() != 0x28)return 11;
	TWIStart();							//resend start
	if(TWIGetStatus() != 0x10)return 12; //repetet Start sent?
	TWIWrite(EEPROM64_R);
	if(TWIGetStatus() != 0x40)return 13;
	result=TWIReadNACK();				//read byte
	TWIStop();
	return result;	
}//end of ext_ee_read()

uint8_t ext_ee_random_write(uint16_t address, uint8_t data)
{
	uint8_t add_high,add_low;
	add_high=address >> 8;		//shift high-byte to variable
	add_low=(uint8_t) address;	//cast low-byte to variable
		
	TWIStart();
	if(TWIGetStatus() != 0x08)return 1;
	TWIWrite(EEPROM64_W);				//send write address
	if(TWIGetStatus() != 0x18)return 2;
	TWIWrite(add_high);					//send high byte
	if(TWIGetStatus() != 0x28)return 3;
	TWIWrite(add_low);					//send low byte
	if(TWIGetStatus() != 0x28)return 3;
	TWIWrite(data);
	if(TWIGetStatus() != 0x28)return 5;//data received acknowledged
	TWIStop();
	return 0;
}



