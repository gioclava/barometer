#include "Arduino.h"
#include <Wire.h>

double getFrontPressure();

double getMiddlePressure();

double getFrontTemperature();

double getMiddleTemperature();

void SPL_init();

boolean pressureAvailable();


void SPL_init_precise(uint8_t spl_chip_address);
double calculatePressure(int32_t c00, int32_t c10, int16_t c01, int16_t c11, int16_t c20, int16_t c21, int16_t c30, double temperature_scale_factor, double pressure_scale_factor,int32_t pressure_raw, int32_t temperature_raw);
void i2c_eeprom_write_uint8_t( uint8_t deviceaddress, uint8_t eeaddress, uint8_t data );
uint8_t i2c_eeprom_read_uint8_t(  uint8_t deviceaddress, uint8_t eeaddress );

int32_t get_praw(uint8_t spl_chip_address);

int16_t get_c0(uint8_t spl_chip_address);

int16_t get_c1(uint8_t spl_chip_address);

int32_t get_c00(uint8_t spl_chip_address);

int32_t get_c10(uint8_t spl_chip_address);

int16_t get_c01(uint8_t spl_chip_address);

int16_t get_c11(uint8_t spl_chip_address);

int16_t get_c20(uint8_t spl_chip_address);

int16_t get_c21(uint8_t spl_chip_address);

int16_t get_c30(uint8_t spl_chip_address);

double get_pressure_scale_factor(uint8_t spl_chip_address);

int32_t get_traw(uint8_t spl_chip_address);

double get_temperature_scale_factor(uint8_t spl_chip_address);

uint8_t get_spl_meas_cfg(uint8_t spl_chip_address);