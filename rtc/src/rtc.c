#include "rtc.h"
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"

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


// Escribe una cadena en la EEPROM AT24C32 en la dirección indicada
// Formato: DDMMAA-HH:MM-RECORRIDO (ejemplo: 141025-13:45-1A)
// recorrido debe ser un string de 2 caracteres (ej: "1A")
// Usa la hora actual del DS1307

bool at24c32_write_log(i2c_inst_t *i2c, uint16_t addr, const char* recorrido) {
	ds1307_time_t t;
	if (!ds1307_get_time(i2c, &t)) return false;
	char str[32];
	int n = snprintf(str, sizeof(str), "%02d%02d%02d-%02d:%02d-%s", t.day, t.month, t.year, t.hours, t.minutes, recorrido);
	if (n < 0 || n >= (int)sizeof(str)) return false;
	uint8_t buf[32];
	memcpy(buf, str, n+1); // incluye el nulo
	uint8_t reg[2];
	reg[0] = (addr >> 8) & 0xFF;
	reg[1] = addr & 0xFF;
	uint8_t tx[34];
	tx[0] = reg[0];
	tx[1] = reg[1];
	memcpy(&tx[2], buf, n+1);
	int written = i2c_write_blocking(i2c, AT24C32_I2C_ADDR, tx, n+3, false);
	// Espera no bloqueante para escritura EEPROM (5ms)
	vTaskDelay(pdMS_TO_TICKS(5));
	return written == n+3;
}

// Escribe un string (con terminador nulo) en una dirección fija de la AT24C32.
// Limita a 47 caracteres útiles + '\0' para ajustarse a bloques de 48 si se usan en lectura.
// Nota: Esta función asume que la escritura no cruza más de una página (32B). Para 0x100 y 0x140
// (alineadas a página) y payloads < 32B, es suficiente.
bool at24c32_write_str(i2c_inst_t *i2c, uint16_t addr, const char *str) {
	if (!str) return false;
	size_t n = strlen(str);
	if (n > 47) n = 47; // truncar para reservar byte nulo si se usa bloque de 48B
	uint8_t tx[2 + 48];
	tx[0] = (addr >> 8) & 0xFF;
	tx[1] = addr & 0xFF;
	memcpy(&tx[2], str, n);
	tx[2 + n] = '\0';
	int written = i2c_write_blocking(i2c, AT24C32_I2C_ADDR, tx, 2 + (int)n + 1, false);
	vTaskDelay(pdMS_TO_TICKS(5));
	return written == (2 + (int)n + 1);
}

// Borra únicamente la región de LOG a partir de EEPROM_LOG_BASE (definida en task_bluetooth)
// Preserva la zona 0x000-EEPROM_LOG_BASE-1 para parámetros fijos.
// Si necesitas borrar TODO, crea otra función explícita (full erase) para evitar errores involuntarios.
bool eeprom_erase(i2c_inst_t *i2c) {
	const int TOTAL_SIZE = 4096;      // tamaño típico AT24C32 en bytes
	const int PAGE_SIZE = 32;         // escribir por páginas
    #ifndef EEPROM_LOG_BASE
    #define EEPROM_LOG_BASE 0x200
    #endif

	uint8_t buf[PAGE_SIZE];
	for (int i = 0; i < PAGE_SIZE; i++) buf[i] = 0xFF;

	for (int addr = EEPROM_LOG_BASE; addr < TOTAL_SIZE; addr += PAGE_SIZE) {
		uint8_t reg[2];
		reg[0] = (addr >> 8) & 0xFF;
		reg[1] = addr & 0xFF;
		uint8_t tx[PAGE_SIZE + 2];
		tx[0] = reg[0];
		tx[1] = reg[1];
		memcpy(&tx[2], buf, PAGE_SIZE);

		int written = i2c_write_blocking(i2c, AT24C32_I2C_ADDR, tx, PAGE_SIZE + 2, false);
		vTaskDelay(pdMS_TO_TICKS(5)); // espera interna

		if (written != PAGE_SIZE + 2) {
			printf("Error borrando EEPROM (zona log) en addr 0x%03X (escritos %d)\n", addr, written);
			return false;
		}
		if (((addr - EEPROM_LOG_BASE) % (PAGE_SIZE * 16)) == 0) { // cada 512 bytes dentro de la zona log
			printf("Borradas %d/%d bytes de zona log\n", addr - EEPROM_LOG_BASE, TOTAL_SIZE - EEPROM_LOG_BASE);
		}
	}
	printf("Zona de LOG EEPROM borrada (desde 0x%03X hasta fin).\n", EEPROM_LOG_BASE);
	return true;
}

// Borra la región [0, end_addr) de la AT24C32 escribiendo 0xFF por páginas.
// Útil para limpiar la zona reservada inicial (por ejemplo 0x000-0x1FF).
bool eeprom_erase_prefix(i2c_inst_t *i2c, uint16_t end_addr) {
	const int TOTAL_SIZE = 4096;
	const int PAGE_SIZE = 32;
	if (end_addr > TOTAL_SIZE) end_addr = TOTAL_SIZE;
	if (end_addr == 0) return true; // nada que borrar

	uint8_t buf[PAGE_SIZE];
	for (int i = 0; i < PAGE_SIZE; i++) buf[i] = 0xFF;

	for (int addr = 0; addr < end_addr; addr += PAGE_SIZE) {
		uint8_t reg[2];
		reg[0] = (addr >> 8) & 0xFF;
		reg[1] = addr & 0xFF;
		int to_write = (end_addr - addr >= PAGE_SIZE) ? PAGE_SIZE : (end_addr - addr);
		uint8_t tx[2 + PAGE_SIZE];
		tx[0] = reg[0];
		tx[1] = reg[1];
		memcpy(&tx[2], buf, to_write);

		int written = i2c_write_blocking(i2c, AT24C32_I2C_ADDR, tx, 2 + to_write, false);
		vTaskDelay(pdMS_TO_TICKS(5));
		if (written != 2 + to_write) {
			printf("Error borrando prefijo EEPROM en addr 0x%03X (escritos %d)\n", addr, written);
			return false;
		}
		if ((addr % (PAGE_SIZE * 16)) == 0) { // cada 512 bytes
			printf("Borradas %d/%d bytes (prefijo)\n", addr, end_addr);
		}
	}
	printf("Prefijo EEPROM borrado correctamente (0x000-0x%03X)\n", end_addr - 1);
	return true;
}