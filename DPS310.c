#include "DPS310.h"
//compensation coefficients
int16_t m_C0;
int16_t m_C1;
int32_t m_C00;
int32_t m_C10;
int16_t m_C01;
int16_t m_C11;
int16_t m_C20;
int16_t m_C21;
int16_t m_C30;
uint8_t meas=0;
uint8_t id=0;
uint8_t pres_ovs, temp_ovs;

void DPS310_init(uint8_t acc)
{
	uint8_t bit=0;
	
	while(bit==0)// go on if Sensor ready flag is set
	{
		if(COEFF_READY_CHECK)bit=1;
		DPS310_readCoeffs();
		
		switch(acc)
		{
			case LOW: 	DPS310_write(PRS_CFG, 0x00);//eight times low power
						DPS310_write(TMP_CFG, 0x80);// 1 measurement
						DPS310_write(CFG_REG, 0x00);//p bit shift off
						DPS310_write(MEAS_CFG, 0x07);//cont temp and pres mess
						temp_ovs = 1;
						pres_ovs = 1;
						break;
			case MID: 	DPS310_write(PRS_CFG, 0x14);//2 messungen / sek   16 fach ovs
						DPS310_write(TMP_CFG, 0x90);//externer sens  2 messung / sek  single ovs
						DPS310_write(CFG_REG, 0x04);//p bit shift on
						DPS310_write(MEAS_CFG, 0x07);//cont temp and pres mess
						temp_ovs = 1;
						pres_ovs = 16;
						break;
			case HIGH: 	DPS310_write(PRS_CFG, 0x26);//4 messungen / sek   64 fach ovs
						DPS310_write(TMP_CFG, 0xA0);//externer sens  4 messung / sek  single ovs
						DPS310_write(CFG_REG, 0x04);//p bit shift on 
						DPS310_write(MEAS_CFG, 0x07);//cont temp and pres mess
						temp_ovs = 1;
						pres_ovs = 64;
						break;
			case ULTRA:	DPS310_write(PRS_CFG, 0xF7);//4 messungen / sek   64 fach ovs
						DPS310_write(TMP_CFG, 0xD7);//externer sens  4 messung / sek  single ovs
						DPS310_write(CFG_REG, 0x0C);//p bit shift on 
						DPS310_write(MEAS_CFG, 0x07);//cont temp and pres mess
						temp_ovs = 128;
						pres_ovs = 128;
						break;
			
		}
		//Korrekturwerte für falsche Temperaturwerte (2-fach normaler Temp Wert)
		// Quelle: https://github.com/Infineon/DPS310-Pressure-Sensor
		
		DPS310_write(0x0E, 0xA5);
		DPS310_write(0x0F, 0x96);
		DPS310_write(0x62, 0x02);
		DPS310_write(0x0E, 0x00);
		DPS310_write(0x0F, 0x00);
	}
}
uint8_t DPS310_read(uint8_t reg)
{
		uint8_t result=0;
		
		TWIStart();
		if(TWIGetStatus() != 0x08)return 123;
		TWIWrite(DPS310_W);
		if(TWIGetStatus() != 0x18)return 2;
		TWIWrite(reg);
		if(TWIGetStatus() != 0x28)return 3;
		TWIStart();
		if(TWIGetStatus() != 0x10)return 4; //repetet Start sent?
		TWIWrite(DPS310_R);
		if(TWIGetStatus() != 0x40)return 5;
		result=TWIReadNACK();
		TWIStop();
		_delay_us(30);
	return result;	
//Daten zurueckgeben
}
uint8_t DPS310_write(uint8_t reg, uint8_t data)
{
		TWIStart();
		if(TWIGetStatus() != 0x08)return 11;
		TWIWrite(DPS310_W);
		if(TWIGetStatus() != 0x18)return 22;
		TWIWrite(reg);
		if(TWIGetStatus() != 0x28)return 33;
		TWIWrite(data);
		if(TWIGetStatus() != 0x28)return 44;
		TWIStop();
		
		_delay_us(30);
	return 0;	
	
	//Daten zurueckgeben
}

int16_t DPS310_readCoeffs(void)
{
	uint16_t buffer[19];//coeffizienten
	uint8_t coeff_start;
	coeff_start=0x10;
	uint8_t x=0;
	
	//coeffizienten einlesen und in buffer-Array speichern
	//Addressen 0x10 - 0x21
	for(x=0;x<18;x++)
	{
		buffer[x]=DPS310_read(coeff_start);
		_delay_ms(10);
		coeff_start++;
	}
	 
    m_C0=(((int)buffer[0]<<8)|buffer[1])>>4;
    m_C0=m_C0/2;
      
    m_C1 = (((uint32_t)buffer[1] & 0x0F) << 8) | (uint32_t)buffer[2];
	if(m_C1 & ((uint32_t)1 << 11))
	{
		m_C1 -= (uint32_t)1 << 12;
	}
      
    m_C00= ((((long)buffer[3]<<8)|buffer[4])<<8)|buffer[5];
    m_C00=(m_C00<<8)>>12;

    m_C10=((((long)buffer[5]<<8)|buffer[6])<<8)|buffer[7];
    m_C10=(m_C10<<12)>>12;

    m_C01=((int)buffer[8]<<8)|buffer[9];

    m_C11=((int)buffer[10]<<8)|buffer[11];

    m_C20=((int)buffer[12]<<8)|buffer[13];

    m_C21=((int)buffer[14]<<8)|buffer[15];

    m_C30=((int)buffer[16]<<8)|buffer[17];
       
    return 0;
}


void DPS310_sreset(void)
{
	// softreset of DPS310 sensor
	DPS310_write(0x0c, 0x99);
	_delay_ms(50);
}


uint32_t DPS310_get_sc_temp(uint8_t oversampling)
{
	long temp_raw=0;
	uint8_t buff[3] = {0};
	buff[0] = DPS310_read(TMP_B2);
	buff[1] = DPS310_read(TMP_B1);
	buff[2] = DPS310_read(TMP_B0);
			
	temp_raw=((((long)buff[0]<<8)|buff[1])<<8)|buff[2];
	temp_raw=(temp_raw<<8)>>8;
				
	return temp_raw; 
}

long DPS310_get_temp(uint8_t oversampling)
{
	long temp_raw=0;
	double temp_sc=0;
	double temp_comp=0;
	long scalfactor=0;
	
			
			temp_raw=DPS310_get_sc_temp(oversampling);
			
			switch(oversampling)
			{
				case 1:	scalfactor = 524288;break;
				case 2:	scalfactor = 1572864;break;
				case 4:	scalfactor = 3670016;break;
				case 8:	scalfactor = 7864320;break;
				case 16:	scalfactor = 253952;break;
				case 32:	scalfactor = 516096;break;
				case 64:	scalfactor = 1040384;break;
				case 128:	scalfactor = 2088960;break;
				
			}
			temp_sc = (float)temp_raw/scalfactor;
			temp_comp=m_C0+m_C1*temp_sc;
			
			
			return (long)temp_comp*10; //2505 entspricht 25,5 Grad
}

double DPS310_get_pres(uint8_t t_ovrs, uint8_t p_ovrs)
{
	long temp_raw;
	double temp_sc;
	uint8_t buff[3] = {0};
	long prs_raw;
	double prs_sc;
	double prs_comp;
	long scalfactor=0;
	
		buff[0] = DPS310_read(TMP_B2);
		buff[1] = DPS310_read(TMP_B1);
		buff[2] = DPS310_read(TMP_B0);
		
		temp_raw=((((long)buff[0]<<8)|buff[1])<<8)|buff[2];
		temp_raw=(temp_raw<<8)>>8;
		
		switch(t_ovrs)
			{
				case 1:	scalfactor = 524288;break;
				case 2:	scalfactor = 1572864;break;
				case 4:	scalfactor = 3670016;break;
				case 8:	scalfactor = 7864320;break;
				case 16:	scalfactor = 253952;break;
				case 32:	scalfactor = 516096;break;
				case 64:	scalfactor = 1040384;break;
				case 128:	scalfactor = 2088960;break;
				
			}
			temp_sc = (float)temp_raw/scalfactor;
		
		buff[0] = DPS310_read(PRS_B2);
		buff[1] = DPS310_read(PRS_B1);
		buff[2] = DPS310_read(PRS_B0);
		
		prs_raw=((((long)buff[0]<<8)|buff[1])<<8)|buff[2];
		prs_raw=(prs_raw<<8)>>8;
		
		switch(p_ovrs)
			{
				case 1:	scalfactor = 524288;break;
				case 2:	scalfactor = 1572864;break;
				case 4:	scalfactor = 3670016;break;
				case 8:	scalfactor = 7864320;break;
				case 16:	scalfactor = 253952;break;
				case 32:	scalfactor = 516096;break;
				case 64:	scalfactor = 1040384;break;
				case 128:	scalfactor = 2088960;break;
				
			}
		prs_sc = (float)prs_raw/scalfactor;
		prs_comp=m_C00+prs_sc*(m_C10+prs_sc*(m_C20+(prs_sc*m_C30)))+temp_sc*m_C01+temp_sc*prs_sc*(m_C11+(prs_sc*m_C21));
		return prs_comp; //2505 entspricht 25,5 Grad
}

long calcalt(double press, uint32_t pressealevel)
{
   return 100*(44330 * (1 - pow((double) press / pressealevel, 0.1902226)));
	//*100 um stellen von Komma nicht zu verlieren
}





