#include "SPL06-007.h"
#include "Wire.h"

nel loop chiamo un checkPressure per ogni barometro che svuota il FIFO e lo salva in 
2 oggetti con le ultime tre misure e il loro timestamp
la pressione viene calcolata sempre con la temperatura disponibile
a un intervallo definito leggermente piu lungo dell intervallo tra le misure, calcolo i valori delle 2 pressioni all intervallo precedente usando una regressione lineare tra i 2 punti che contengono il timestamp del intervallo precendente
controllare se i coefficienti cambiano o sono fissi nel tempo, nel primo caso leggerli alla richiesta della pressione compensata, nel secondo caso leggerli all init


void SPL_init(uint8_t spl_chip_address)
{
	// ---- Oversampling of >8x for temperature or pressuse requires FIFO operational mode which is not implemented ---
	// ---- Use rates of 8x or less until feature is implemented ---
	i2c_eeprom_write_uint8_t(spl_chip_address, 0X06, 0x03);	// Pressure 8x oversampling

	i2c_eeprom_write_uint8_t(spl_chip_address, 0X07, 0X83);	// Temperature 8x oversampling

	i2c_eeprom_write_uint8_t(spl_chip_address, 0X08, 0B0111);	// continuous temp and pressure measurement

	i2c_eeprom_write_uint8_t(spl_chip_address, 0X09, 0X00);	// FIFO Pressure measurement  
}

void SPL_init_precise(uint8_t spl_chip_address)
{
	// ---- Oversampling of >8x for temperature or pressuse requires FIFO operational mode which is not implemented ---
	// ---- Use rates of 8x or less until feature is implemented ---
	i2c_eeprom_write_uint8_t(spl_chip_address, 0X06, 0x36); //0x26 richiesto da datasheet

	i2c_eeprom_write_uint8_t(spl_chip_address, 0X07, 0XA0);

	i2c_eeprom_write_uint8_t(spl_chip_address, 0X08, 0B0111);	// continuous temp and pressure measurement

	i2c_eeprom_write_uint8_t(spl_chip_address, 0X09, 0B110);	// FIFO Pressure measurement  
}

uint8_t get_spl_id(uint8_t spl_chip_address)
{
	return i2c_eeprom_read_uint8_t(spl_chip_address, 0x0D);	
}

uint8_t get_spl_prs_cfg(uint8_t spl_chip_address)
{
	return i2c_eeprom_read_uint8_t(spl_chip_address, 0x06);
}

uint8_t get_spl_tmp_cfg(uint8_t spl_chip_address)
{
	return i2c_eeprom_read_uint8_t(spl_chip_address, 0x07);
}

uint8_t get_spl_meas_cfg(uint8_t spl_chip_address)
{
	return i2c_eeprom_read_uint8_t(spl_chip_address, 0x08);
}

uint8_t get_spl_cfg_reg(uint8_t spl_chip_address)
{
	return i2c_eeprom_read_uint8_t(spl_chip_address, 0x09);
}

uint8_t get_spl_int_sts(uint8_t spl_chip_address)
{
	return i2c_eeprom_read_uint8_t(spl_chip_address, 0x0A);
}

uint8_t get_spl_fifo_sts(uint8_t spl_chip_address)
{
	return i2c_eeprom_read_uint8_t(spl_chip_address, 0x0B);
}



double get_altitude(double pressure, double seaLevelhPa) {
	double altitude;

	altitude = 44330 * (1.0 - pow(pressure / seaLevelhPa, 0.1903));

	return altitude;
}

double get_altitude_f(double pressure, double seaLevelhPa)
{
	double altitude;

	altitude = 44330 * (1.0 - pow(pressure / seaLevelhPa, 0.1903));

	return altitude * 3.281;
}


double get_traw_sc(uint8_t spl_chip_address)
{
	int32_t traw = get_traw(spl_chip_address);
	return (double(traw)/get_temperature_scale_factor(spl_chip_address));
}


double get_temp_c(uint8_t spl_chip_address)
{
	int16_t c0,c1;
	c0 = get_c0(spl_chip_address);
	c1 = get_c1(spl_chip_address);
	double traw_sc = get_traw_sc(spl_chip_address);
	return (double(c0) * 0.5f) + (double(c1) * traw_sc);
}


double get_temp_f(uint8_t spl_chip_address)
{
	int16_t c0,c1;
	c0 = get_c0(spl_chip_address);
	c1 = get_c1(spl_chip_address);
	double traw_sc = get_traw_sc(spl_chip_address);
	return (((double(c0) * 0.5f) + (double(c1) * traw_sc)) * 9/5) + 32;
}


double get_temperature_scale_factor(uint8_t spl_chip_address)
{
  
  double k;

  uint8_t tmp_Byte;
  tmp_Byte = i2c_eeprom_read_uint8_t(spl_chip_address, 0X07); // MSB

  //Serial.print("tmp_Byte: ");
  //Serial.println(tmp_Byte);

  //tmp_Byte = tmp_Byte >> 4; //Focus on bits 6-4
  tmp_Byte = tmp_Byte & 0B00000111;
  //Serial.print("tmp_Byte: ");
  //Serial.println(tmp_Byte);

  switch (tmp_Byte) 
  {
    case 0B000:
      k = (double) 524288.0;
    break;

    case 0B001:
      k = (double) 1572864.0;
    break;

    case 0B010:
      k = (double) 3670016.0;
    break;

    case 0B011:
      k = (double) 7864320.0;
    break;

    case 0B100:
      k = (double) 253952.0;
    break;

    case 0B101:
      k = (double) 516096.0;
    break;

    case 0B110:
      k = (double) 1040384.0;
    break;

    case 0B111:
      k = (double) 2088960.0;
    break;
  }

  return k;
}


int32_t get_traw(uint8_t spl_chip_address)
{
  int32_t tmp;
  uint8_t tmp_MSB,tmp_LSB,tmp_XLSB;
  tmp_MSB = i2c_eeprom_read_uint8_t(spl_chip_address, 0X03); // MSB

  tmp_LSB = i2c_eeprom_read_uint8_t(spl_chip_address, 0X04); // LSB

  tmp_XLSB = i2c_eeprom_read_uint8_t(spl_chip_address, 0X05); // XLSB

  tmp = (tmp_MSB << 8) | tmp_LSB;
  tmp = (tmp << 8) | tmp_XLSB;


  if(tmp & (1 << 23))
    tmp = tmp | 0XFF000000; // Set left bits to one for 2's complement conversion of negitive number
  
  
  return tmp;
}

double get_praw_sc(uint8_t spl_chip_address)
{
	int32_t praw = get_praw(spl_chip_address);
	return (double(praw)/get_pressure_scale_factor(spl_chip_address));
}

double get_pcomp(uint8_t spl_chip_address)
{
	int32_t c00,c10;
	int16_t c01,c11,c20,c21,c30;
	c00 = get_c00(spl_chip_address);
	c10 = get_c10(spl_chip_address);
	c01 = get_c01(spl_chip_address);
	c11 = get_c11(spl_chip_address);
	c20 = get_c20(spl_chip_address);
	c21 = get_c21(spl_chip_address);
	c30 = get_c30(spl_chip_address);
	double traw_sc = get_traw_sc(spl_chip_address);
	double praw_sc = get_praw_sc(spl_chip_address);
	return double(c00) + praw_sc * (double(c10) + praw_sc * (double(c20) + praw_sc * double(c30))) + traw_sc * double(c01) + traw_sc * praw_sc * ( double(c11) + praw_sc * double(c21));
}

double get_pressure(uint8_t spl_chip_address)
{
	double pcomp = get_pcomp(spl_chip_address);
	return pcomp / 100; // convert to mb
}



double get_pressure_scale_factor(uint8_t spl_chip_address)
{
	double k;

	uint8_t tmp_Byte;
	tmp_Byte = i2c_eeprom_read_uint8_t(spl_chip_address, 0X06); // MSB

	tmp_Byte = tmp_Byte & 0B00000111; // Focus on 2-0 oversampling rate 


	switch (tmp_Byte) // oversampling rate
	{
		case 0B000:
			k = (double) 524288.0;
		break;

		case 0B001:
			k = (double) 1572864.0;
		break;

		case 0B010:
			k = (double) 3670016.0;
		break;

		case 0B011:
			k = (double) 7864320.0;
		break;

		case 0B100:
			k = (double) 253952.0;
		break;

		case 0B101:
			k = (double) 516096.0;
		break;

		case 0B110:
			k = (double) 1040384.0;
		break;

		case 0B111:
			k = (double) 2088960.0;
		break;
	}

	return k;
}


int32_t get_fifo_measure(uint8_t spl_chip_address)
{
  int32_t measure;
  uint8_t measure_MSB,measure_LSB,measure_XLSB;
  measure_MSB = i2c_eeprom_read_uint8_t(spl_chip_address, 0X00); // MSB


  measure_LSB = i2c_eeprom_read_uint8_t(spl_chip_address, 0X01); // LSB


  measure_XLSB = i2c_eeprom_read_uint8_t(spl_chip_address, 0X02); // XLSB

  
  measure = (measure_MSB << 8) | measure_LSB;
  measure = (measure << 8) | measure_XLSB;



  if(measure & (1 << 23))
    measure = measure | 0XFF000000; // Set left bits to one for 2's complement conversion of negitive number
  
  
  return measure;
}

bool isFifoAvailable(uint8_t spl_chip_address){
  return !(get_spl_fifo_sts(spl_chip_address) && 0B01);

}

void checkPressure(spl_chip_address){
  unit32_t measure = get_fifo_measure(uint8_t spl_chip_address);
  while(measure!=0x800000)){  
    if(measure && 0B01) {
      //save pressure in 3 items array

    }
    else{
      //save temperature
    }
    measure = get_fifo_measure(uint8_t spl_chip_address);
  }
}



int32_t get_praw(uint8_t spl_chip_address)
{
  int32_t tmp;
  uint8_t tmp_MSB,tmp_LSB,tmp_XLSB;
  tmp_MSB = i2c_eeprom_read_uint8_t(spl_chip_address, 0X00); // MSB


  tmp_LSB = i2c_eeprom_read_uint8_t(spl_chip_address, 0X01); // LSB


  tmp_XLSB = i2c_eeprom_read_uint8_t(spl_chip_address, 0X02); // XLSB

  
  tmp = (tmp_MSB << 8) | tmp_LSB;
  tmp = (tmp << 8) | tmp_XLSB;



  if(tmp & (1 << 23))
    tmp = tmp | 0XFF000000; // Set left bits to one for 2's complement conversion of negitive number
  
  
  return tmp;
}

int16_t get_c0(uint8_t spl_chip_address)
{
  int16_t tmp; 
  uint8_t tmp_MSB,tmp_LSB;
  
  tmp_MSB = i2c_eeprom_read_uint8_t(spl_chip_address, 0X10); 
  tmp_LSB = i2c_eeprom_read_uint8_t(spl_chip_address, 0X11); 



  tmp_LSB = tmp_LSB >> 4;


  tmp = (tmp_MSB << 4) | tmp_LSB;

  if(tmp & (1 << 11)) // Check for 2's complement negative number
    tmp = tmp | 0XF000; // Set left bits to one for 2's complement conversion of negitive number
  
  return tmp;
}


int16_t get_c1(uint8_t spl_chip_address)
{
  int16_t tmp; 
  uint8_t tmp_MSB,tmp_LSB;
  
  tmp_MSB = i2c_eeprom_read_uint8_t(spl_chip_address, 0X11); 
  tmp_LSB = i2c_eeprom_read_uint8_t(spl_chip_address, 0X12); 


  tmp_MSB = tmp_MSB & 0XF;


  tmp = (tmp_MSB << 8) | tmp_LSB;

  if(tmp & (1 << 11)) // Check for 2's complement negative number
    tmp = tmp | 0XF000; // Set left bits to one for 2's complement conversion of negitive number
  
  return tmp;
}

int32_t get_c00(uint8_t spl_chip_address)
{
  int32_t tmp; 
  uint8_t tmp_MSB,tmp_LSB,tmp_XLSB;
  
  tmp_MSB = i2c_eeprom_read_uint8_t(spl_chip_address, 0X13); 
  tmp_LSB = i2c_eeprom_read_uint8_t(spl_chip_address, 0X14); 
  tmp_XLSB = i2c_eeprom_read_uint8_t(spl_chip_address, 0X15);


  
  tmp_XLSB = tmp_XLSB >> 4;

   
  tmp = (tmp_MSB << 8) | tmp_LSB;
  tmp = (tmp << 4) | tmp_XLSB;

  tmp = (uint32_t)tmp_MSB << 12 | (uint32_t)tmp_LSB << 4 | (uint32_t)tmp_XLSB >> 4;

  if(tmp & (1 << 19))
    tmp = tmp | 0XFFF00000; // Set left bits to one for 2's complement conversion of negitive number
    

  return tmp;
}

int32_t get_c10(uint8_t spl_chip_address)
{
  int32_t tmp; 
  uint8_t tmp_MSB,tmp_LSB,tmp_XLSB;
  
  tmp_MSB = i2c_eeprom_read_uint8_t(spl_chip_address, 0X15); // 4 bits
  tmp_LSB = i2c_eeprom_read_uint8_t(spl_chip_address, 0X16); // 8 bits
  tmp_XLSB = i2c_eeprom_read_uint8_t(spl_chip_address, 0X17); // 8 bits


  tmp_MSB = tmp_MSB & 0b00001111;



  tmp = (tmp_MSB << 4) | tmp_LSB;
  tmp = (tmp << 8) | tmp_XLSB;



  tmp = (uint32_t)tmp_MSB << 16 | (uint32_t)tmp_LSB << 8 | (uint32_t)tmp_XLSB;

  if(tmp & (1 << 19))
    tmp = tmp | 0XFFF00000; // Set left bits to one for 2's complement conversion of negitive number

  return tmp;
}



int16_t get_c01(uint8_t spl_chip_address)
{
  int16_t tmp; 
  uint8_t tmp_MSB,tmp_LSB;
  
  tmp_MSB = i2c_eeprom_read_uint8_t(spl_chip_address, 0X18); 
  tmp_LSB = i2c_eeprom_read_uint8_t(spl_chip_address, 0X19); 

  tmp = (tmp_MSB << 8) | tmp_LSB;
  return tmp;
}

int16_t get_c11(uint8_t spl_chip_address)
{
  int16_t tmp; 
  uint8_t tmp_MSB,tmp_LSB;
  
  tmp_MSB = i2c_eeprom_read_uint8_t(spl_chip_address, 0X1A); 
  tmp_LSB = i2c_eeprom_read_uint8_t(spl_chip_address, 0X1B); 

  tmp = (tmp_MSB << 8) | tmp_LSB;
  return tmp;
}

int16_t get_c20(uint8_t spl_chip_address)
{
  int16_t tmp; 
  uint8_t tmp_MSB,tmp_LSB;
  
  tmp_MSB = i2c_eeprom_read_uint8_t(spl_chip_address, 0X1C); 
  tmp_LSB = i2c_eeprom_read_uint8_t(spl_chip_address, 0X1D); 


  tmp = (tmp_MSB << 8) | tmp_LSB;
  return tmp;
}

int16_t get_c21(uint8_t spl_chip_address)
{
  int16_t tmp; 
  uint8_t tmp_MSB,tmp_LSB;
  
  tmp_MSB = i2c_eeprom_read_uint8_t(spl_chip_address, 0X1E); 
  tmp_LSB = i2c_eeprom_read_uint8_t(spl_chip_address, 0X1F); 

  tmp = (tmp_MSB << 8) | tmp_LSB;
  return tmp;
}

int16_t get_c30(uint8_t spl_chip_address)
{
  int16_t tmp; 
  uint8_t tmp_MSB,tmp_LSB;
  
  tmp_MSB = i2c_eeprom_read_uint8_t(spl_chip_address, 0X20); 
  tmp_LSB = i2c_eeprom_read_uint8_t(spl_chip_address, 0X21); 

  tmp = (tmp_MSB << 8) | tmp_LSB;
  return tmp;
  Serial.print("tmp: ");
  Serial.println(tmp);
}

void i2c_eeprom_write_uint8_t( uint8_t deviceaddress, uint8_t eeaddress, uint8_t data ) 
{
    uint8_t rdata = data;
    delay(5); // Make sure to delay log enough for EEPROM I2C refresh time
    Wire.beginTransmission(deviceaddress);
    Wire.write((uint8_t)(eeaddress));
    Wire.write(rdata);
    Wire.endTransmission();
}



uint8_t i2c_eeprom_read_uint8_t(  uint8_t deviceaddress, uint8_t eeaddress ) 
{
    uint8_t rdata = 0xFF;
    Wire.beginTransmission(deviceaddress);
    Wire.write(eeaddress); 
    Wire.endTransmission(false); // false to not release the line
    
    Wire.requestFrom(deviceaddress,1);
    if (Wire.available()) rdata = Wire.read();
    return rdata;
}
