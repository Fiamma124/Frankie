
#include "rtc.h"
#include <stdbool.h>

#define AT24C32_I2C_ADDR 0x50

static uint8_t bcd2dec(uint8_t val) { return ((val / 16 * 10) + (val % 16)); }
static uint8_t dec2bcd(uint8_t val) { return ((val / 10 * 16) + (val % 10)); }


// Inicializa el DS1307 (habilita el reloj si está parado)
void ds1307_init(i2c_inst_t *i2c) {
	uint8_t data[2];
	// Leer segundos
	uint8_t reg = 0x00;
	i2c_write_blocking(i2c, DS1307_I2C_ADDR, &reg, 1, true);
	i2c_read_blocking(i2c, DS1307_I2C_ADDR, data, 1, false);
	// Si el bit 7 está en 1, el reloj está parado. Lo bajamos a 0.
	if (data[0] & 0x80) {
		data[0] &= ~0x80;
		uint8_t set_data[2] = {0x00, data[0]};
		i2c_write_blocking(i2c, DS1307_I2C_ADDR, set_data, 2, false);
	}
}


// Lee la hora del DS1307
bool ds1307_get_time(i2c_inst_t *i2c, ds1307_time_t *time) {
	uint8_t reg = 0x00;
	uint8_t data[7];
	if (i2c_write_blocking(i2c, DS1307_I2C_ADDR, &reg, 1, true) != 1) return false;
	if (i2c_read_blocking(i2c, DS1307_I2C_ADDR, data, 7, false) != 7) return false;
	time->seconds = bcd2dec(data[0] & 0x7F);
	time->minutes = bcd2dec(data[1]);
	time->hours = bcd2dec(data[2] & 0x3F);
	time->day_of_week = bcd2dec(data[3]);
	time->day = bcd2dec(data[4]);
	time->month = bcd2dec(data[5]);
	time->year = bcd2dec(data[6]);
	return true;
}


// Escribe la hora en el DS1307
bool ds1307_set_time(i2c_inst_t *i2c, const ds1307_time_t *time) {
	uint8_t data[8];
	data[0] = 0x00; // Registro inicial
	data[1] = dec2bcd(time->seconds & 0x7F);
	data[2] = dec2bcd(time->minutes);
	data[3] = dec2bcd(time->hours);
	data[4] = dec2bcd(time->day_of_week);
	data[5] = dec2bcd(time->day);
	data[6] = dec2bcd(time->month);
	data[7] = dec2bcd(time->year);
	return i2c_write_blocking(i2c, DS1307_I2C_ADDR, data, 8, false) == 8;
}


// Lee datos de la EEPROM AT24C32
// addr: dirección de memoria (0-4095), buf: buffer destino, len: cantidad de bytes
bool at24c32_read(i2c_inst_t *i2c, uint16_t addr, uint8_t *buf, size_t len) {
	if (addr > 4095 || len == 0) return false;
	uint8_t reg[2];
	reg[0] = (addr >> 8) & 0xFF; // MSB
	reg[1] = addr & 0xFF;        // LSB
	// Escribir dirección de memoria
	if (i2c_write_blocking(i2c, AT24C32_I2C_ADDR, reg, 2, true) != 2) return false;
	// Leer datos
	if (i2c_read_blocking(i2c, AT24C32_I2C_ADDR, buf, len, false) != (int)len) return false;
	return true;
}