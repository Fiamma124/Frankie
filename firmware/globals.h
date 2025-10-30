#ifndef GLOBALS_H
#define GLOBALS_H

#include "FreeRTOS.h"
#include "queue.h"
#include "config.h"
#include <stdint.h>
#include <stddef.h>
// i2c types used by print_fecha_hora_rtc
#include "hardware/i2c.h"

// Queues (defined in firmware.c)
extern QueueHandle_t q_tacometro_left;
extern QueueHandle_t q_tacometro_right;
extern QueueHandle_t q_uart;
extern QueueHandle_t q_bluetooth_chars;
extern QueueHandle_t q_codigo;

// Prototypes for functions defined in firmware.c but used by tasks
void print_fecha_hora_rtc(i2c_inst_t *i2c);
void task_tune_left_twiddle(void *params);
void task_tune_right_twiddle(void *params);
void task_calibracion(void *params);

// EEPROM / RTC helpers defined in other modules
bool at24c32_read(i2c_inst_t *i2c, uint16_t addr, uint8_t *buf, size_t len);
bool at24c32_write_log(i2c_inst_t *i2c, uint16_t addr, const char *s);
bool eeprom_erase(i2c_inst_t *i2c);

#endif // GLOBALS_H
