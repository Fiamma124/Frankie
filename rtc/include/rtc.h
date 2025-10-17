#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdbool.h>

#define DS1307_I2C_ADDR 0x68
#define AT24C32_I2C_ADDR 0x50


typedef struct {
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	uint8_t day_of_week;
	uint8_t day;
	uint8_t month;
	uint8_t year;
} ds1307_time_t;

void ds1307_init(i2c_inst_t *i2c);
bool ds1307_get_time(i2c_inst_t *i2c, ds1307_time_t *time);
bool ds1307_set_time(i2c_inst_t *i2c, const ds1307_time_t *time);
bool at24c32_read(i2c_inst_t *i2c, uint16_t addr, uint8_t *buf, size_t len);
bool at24c32_write_log(i2c_inst_t *i2c, uint16_t addr, const char* recorrido);
bool eeprom_erase(i2c_inst_t *i2c);
