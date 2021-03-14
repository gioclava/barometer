#include "Arduino.h"

#define SPL_1 0x76
#define SPL_2 0x77

void SPL_init(uint8_t spl_chip_address);

uint8_t get_spl_id(uint8_t spl_chip_address);		// Get ID Register 		0x0D
uint8_t get_spl_prs_cfg(uint8_t spl_chip_address);	// Get PRS_CFG Register	0x06
uint8_t get_spl_tmp_cfg(uint8_t spl_chip_address);	// Get TMP_CFG Register	0x07
uint8_t get_spl_meas_cfg(uint8_t spl_chip_address);	// Get MEAS_CFG Register	0x08
uint8_t get_spl_cfg_reg(uint8_t spl_chip_address);	// Get CFG_REG Register	0x09
uint8_t get_spl_int_sts(uint8_t spl_chip_address);	// Get INT_STS Register	0x0A
uint8_t get_spl_fifo_sts(uint8_t spl_chip_address);	// Get FIFO_STS Register	0x0B

double get_altitude(double pressure, double seaLevelhPa);	// get altitude in meters
double get_altitude_f(double pressure, double seaLevelhPa);	// get altitude in feet

int32_t get_traw(uint8_t spl_chip_address);
double get_traw_sc(uint8_t spl_chip_address);
double get_temp_c(uint8_t spl_chip_address);
double get_temp_f(uint8_t spl_chip_address);
double get_temperature_scale_factor(uint8_t spl_chip_address);

int32_t get_praw(uint8_t spl_chip_address);
double get_praw_sc(uint8_t spl_chip_address);
double get_pcomp(uint8_t spl_chip_address);
double get_pressure_scale_factor(uint8_t spl_chip_address);
double get_pressure(uint8_t spl_chip_address);

int16_t get_c0(uint8_t spl_chip_address);
int16_t get_c1(uint8_t spl_chip_address);
int32_t get_c00(uint8_t spl_chip_address);
int32_t get_c10(uint8_t spl_chip_address);
int16_t get_c01(uint8_t spl_chip_address);
int16_t get_c11(uint8_t spl_chip_address);
int16_t get_c20(uint8_t spl_chip_address);
int16_t get_c21(uint8_t spl_chip_address);
int16_t get_c30(uint8_t spl_chip_address);

void i2c_eeprom_write_uint8_t(  uint8_t deviceaddress, uint8_t eeaddress, uint8_t data );
uint8_t i2c_eeprom_read_uint8_t(  uint8_t deviceaddress, uint8_t eeaddress );

