#include "SPL06-007.h"
#include "Wire.h"

int32_t front_pressure_raw;
int32_t front_temperature_raw;
int32_t middle_pressure_raw;
int32_t middle_temperature_raw;
uint8_t front_address = 0x76;
uint8_t middle_address = 0x77;
double front_pressure_scale_factor;
double middle_pressure_scale_factor;
double front_temperature_scale_factor;
double middle_temperature_scale_factor;
int32_t front_c00, front_c10;
int16_t front_c01, front_c11, front_c20, front_c21, front_c30;
int32_t middle_c00, middle_c10;
int16_t middle_c01, middle_c11, middle_c20, middle_c21, middle_c30;
int16_t middle_c0,middle_c1;
int16_t front_c0,front_c1;

void SPL_init_precise(uint8_t spl_chip_address)
{
	i2c_eeprom_write_uint8_t(spl_chip_address, 0X06, 0x36); //0x26 richiesto da datasheet

	i2c_eeprom_write_uint8_t(spl_chip_address, 0X07, 0XA0);

	i2c_eeprom_write_uint8_t(spl_chip_address, 0X08, 0B0111);	// continuous temp and pressure measurement

	i2c_eeprom_write_uint8_t(spl_chip_address, 0X09, 0B110);	// FIFO Pressure measurement
}

void SPL_init()
{
  SPL_init_precise(middle_address);
  SPL_init_precise(front_address);
  middle_pressure_scale_factor = get_pressure_scale_factor(middle_address);
  front_pressure_scale_factor = get_pressure_scale_factor(front_address);
  middle_temperature_scale_factor = get_pressure_scale_factor(middle_address);
  front_temperature_scale_factor = get_pressure_scale_factor(front_address);
  middle_c00 = get_c00(middle_address);
	middle_c10 = get_c10(middle_address);
	middle_c01 = get_c01(middle_address);
	middle_c11 = get_c11(middle_address);
	middle_c20 = get_c20(middle_address);
	middle_c21 = get_c21(middle_address);
	middle_c30 = get_c30(middle_address);
  front_c00 = get_c00(front_address);
	front_c10 = get_c10(front_address);
	front_c01 = get_c01(front_address);
	front_c11 = get_c11(front_address);
	front_c20 = get_c20(front_address);
	front_c21 = get_c21(front_address);
	front_c30 = get_c30(front_address);
  middle_c0 = get_c0(middle_address);
	middle_c1 = get_c1(middle_address);
  front_c0 = get_c0(front_address);
	front_c1 = get_c1(front_address);
}



double calculatePressure(int32_t c00, int32_t c10, int16_t c01, int16_t c11, int16_t c20, int16_t c21, int16_t c30, double temperature_scale_factor, double pressure_scale_factor,int32_t pressure_raw, int32_t temperature_raw){
  double traw_sc = (double(temperature_raw)/temperature_scale_factor);
	double praw_sc = (double(pressure_raw)/pressure_scale_factor);
	return double(c00) + praw_sc * (double(c10) + praw_sc * (double(c20) + praw_sc * double(c30))) + traw_sc * double(c01) + traw_sc * praw_sc * (double(c11) + praw_sc * double(c21));
}

double calculateTemperatureC(double temperature_scale_factor, int32_t temperature_raw, int16_t c0,int16_t c1)
{
  double traw_sc = (double(temperature_raw)/temperature_scale_factor);
	return (double(c0) * 0.5f) + (double(c1) * traw_sc);
}

double getFrontPressure(){
  return calculatePressure(front_c00, front_c10, front_c01, front_c11, front_c20, front_c21, front_c30, front_temperature_scale_factor, front_pressure_scale_factor, front_pressure_raw, front_temperature_raw);
}

double getMiddlePressure(){
  return calculatePressure(middle_c00, middle_c10, middle_c01, middle_c11, middle_c20, middle_c21, middle_c30, middle_temperature_scale_factor, middle_pressure_scale_factor, middle_pressure_raw, middle_temperature_raw);
}

double getFrontTemperature(){
  return calculateTemperatureC(front_temperature_scale_factor, front_temperature_raw,front_c0, front_c1);
}

double getMiddleTemperature(){
  return calculateTemperatureC(middle_temperature_scale_factor, middle_temperature_raw,middle_c0, middle_c1);
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

boolean pressureAvailable(){
  boolean pressureAvailable = false;
  int32_t middle_measure = get_fifo_measure(middle_address);
  while(middle_measure!=0x800000){  
    if(middle_measure && 0B01) {
      middle_pressure_raw = middle_measure;
    }
    else{
      middle_temperature_raw = middle_measure;
    }
    middle_measure = get_fifo_measure(middle_address);
  }
  int32_t front_measure = get_fifo_measure(middle_address);
  while(front_measure!=0x800000){  
    if(front_measure && 0B01) {
      front_pressure_raw = front_measure;
      pressureAvailable = true;
    }
    else{
      front_temperature_raw = front_measure;
    }
    front_measure = get_fifo_measure(front_address);
  }
  return pressureAvailable;
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
