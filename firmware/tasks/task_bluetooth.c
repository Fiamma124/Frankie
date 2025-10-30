#include <math.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "string.h"
#include "config.h"
#include "globals.h"
#include "motor.h"
#include "task_bluetooth.h"
#include "task_recto.h"
#include "task_curva.h"

void task_bluetooth(void *params) {
	char buffer[4] = {0}; // Donde se guardará el recorrido preseteado. Ej "1A#"
	int cod_num = 0;
	int index = 2;
	int cod_letter = 0 ;
	char c;

	static uint16_t eeprom_addr = 0;

	while (true) {
		if (xQueueReceive(q_bluetooth_chars, &c, portMAX_DELAY)) {
		   if (c != '#') {
				//
				if (c == '1' || c == '2' || c == '3') {
					buffer[0] = c;
					cod_num = 1;
				} else if (c == 'A' || c == 'B' || c == 'C'|| c == 'D') {
					buffer[1] = c;
					cod_letter = 1 ;
				} else if (c == 'R' || c == 'M' || c == 'E' || c == 'P' || c == 'N' || c == 'Y' || c == 'Q') {
					buffer[0] = c;
					index = 1;
				} else {
					buffer[index++] = c;
				}
			}

			if (c == '#') {
				buffer[index] = '\0';
				printf("Secuencia completa recibida: %s\n", buffer);


				// Si la cadena es R#
				if (index == 1 && buffer[0] == 'R') {
					print_fecha_hora_rtc(i2c1);
					at24c32_write_log(i2c1, eeprom_addr, "R#");
					eeprom_addr += 48; // tamaño máximo de registro
				}
				  if (index == 1 && buffer[0] == 'M') {
					// Imprimir menú de comandos Bluetooth
					printf("\n--- MENÚ DE COMANDOS BLUETOOTH ---\n");
					printf("R#  - Registrar fecha y hora RTC en EEPROM\n");
					printf("L#  - Mostrar registros de log en EEPROM\n");
					printf("E#  - Borrar toda la EEPROM\n");
					printf("N#  - Ejecutar autotuning PID (Twiddle)\n");
					printf("Y#  - Leer PID óptimos guardados\n");
					printf("Q#  - Calibración de las ruedas\n");
					printf("P#  - Guardar comando personalizado\n");
					printf("[1-3][A-D]# - Ejecutar recorrido (ej: 1A#, 2B#)\n");
					printf("------------------------------------\n");
				}

				// Si la cadena es L#
				else if (index == 1 && buffer[0] == 'L') {
					// Imprimir todos los registros de log almacenados en la EEPROM
					const int registro_size = 48;
					uint8_t mem_buf[registro_size];
					uint16_t addr = 0;
					int registro_num = 0;
					printf("\n--- LOG EEPROM ---\n");
					while (addr + registro_size <= 4096) {
						if (!at24c32_read(i2c1, addr, mem_buf, registro_size)) {
							printf("Error leyendo EEPROM en registro %d\n", registro_num);
							break;
						}
						// Si el primer byte es 0xFF o 0x00, consideramos que no hay más registros válidos
						if (mem_buf[0] == 0xFF || mem_buf[0] == 0x00) {
							break;
						}
						// Aseguramos nulo al final por si falta
						mem_buf[registro_size-1] = '\0';
						// Imprime la cadena hasta el primer nulo
						printf("%d: %s\n", registro_num+1, mem_buf);
						addr += registro_size;
						registro_num++;
					}
					if (registro_num == 0) {
						printf("(Sin registros)\n");
					}
					printf("--- FIN LOG ---\n");
					at24c32_write_log(i2c1, eeprom_addr, "M#");
					eeprom_addr += 48;
				}
				// Si la cadena es E# -> borrar EEPROM
				else if (index == 1 && buffer[0] == 'E') {
					printf("Comando E# recibido: borrando EEPROM...\n");
					bool ok = eeprom_erase(i2c1);
					if (ok) {
						printf("Borrado completado, registrando evento E#\n");
						at24c32_write_log(i2c1, eeprom_addr, "E#");
						eeprom_addr += 48;
					} else {
						printf("Borrado fallido: no se registrará el evento E# en EEPROM\n");
					}
				}
				// Si la cadena es N#
				if (index == 1 && buffer[0] == 'N') {
					buffer[2] = '\0';
					printf("Ingresado: %s\n", buffer);
					//index = 2;
					at24c32_write_log(i2c1, eeprom_addr, buffer);
					eeprom_addr += 48;
					// Tareas de autotuning PID (ejecutar solo cuando quieras calibrar)
					xTaskCreate(task_tune_left_twiddle, "TuneLeft", 2 * configMINIMAL_STACK_SIZE, NULL, 6, NULL);
					xTaskCreate(task_tune_right_twiddle, "TuneRight", 2 * configMINIMAL_STACK_SIZE, NULL, 6, NULL);
					memset(buffer, 0, sizeof(buffer));
					cod_num = 0; 
					cod_letter = 0;
				}
				// Si la cadena es Y#
				if (index == 1 && buffer[0] == 'Y') {
					// Leer PID guardados en EEPROM (LEFT)
					char pidl_buf[48] = {0};
					float Kp_left = 0, Ki_left = 0, Kd_left = 0;
					if (at24c32_read(i2c1, 0x100, (uint8_t*)pidl_buf, sizeof(pidl_buf))) {
						if (strncmp(pidl_buf, "PIDL:", 5) == 0) {
							sscanf(pidl_buf+5, "%f,%f,%f", &Kp_left, &Ki_left, &Kd_left);
							printf("[EEPROM] PID LEFT: Kp=%.3f, Ki=%.3f, Kd=%.3f\n", Kp_left, Ki_left, Kd_left);
						} else {
							printf("[EEPROM] PID LEFT: No guardado\n");
						}
					} else {
						printf("[EEPROM] PID LEFT: Error de lectura\n");
					}
					// Leer PID guardados en EEPROM (RIGHT)
					char pidr_buf[48] = {0};
					float Kp_right = 0, Ki_right = 0, Kd_right = 0;
					if (at24c32_read(i2c1, 0x140, (uint8_t*)pidr_buf, sizeof(pidr_buf))) {
						if (strncmp(pidr_buf, "PIDR:", 5) == 0) {
							sscanf(pidr_buf+5, "%f,%f,%f", &Kp_right, &Ki_right, &Kd_right);
							printf("[EEPROM] PID RIGHT: Kp=%.3f, Ki=%.3f, Kd=%.3f\n", Kp_right, Ki_right, Kd_right);
						} else {
							printf("[EEPROM] PID RIGHT: No guardado\n");
						}
					} else {
						printf("[EEPROM] PID RIGHT: Error de lectura\n");
					}
				}
				// Si la cadena es P#
				if (index == 1 && buffer[0] == 'P') {
					buffer[2] = '\0';
					printf("Ingresado: %s\n", buffer);
					//index = 2;
					at24c32_write_log(i2c1, eeprom_addr, buffer);
					eeprom_addr += 48;
					memset(buffer, 0, sizeof(buffer));
					cod_num = 0; 
					cod_letter = 0;
				}
				if (index == 1 && buffer[0] == 'Q') {
					buffer[2] = '\0';
					printf("Ingresado: %s\n", buffer);
					//index = 2;
					at24c32_write_log(i2c1, eeprom_addr, buffer);
					eeprom_addr += 48;
					xTaskCreate(task_calibracion, "Calib", 3 * configMINIMAL_STACK_SIZE, NULL, 3, NULL);
					memset(buffer, 0, sizeof(buffer));
					cod_num = 0; 
					cod_letter = 0;
				}
				// Verifica formato de comando normal
				else if (cod_num == 1 && cod_letter==1) {
					buffer[2] = '\0';
					printf("Ingresado: %s\n", buffer);
					//index = 2;
					at24c32_write_log(i2c1, eeprom_addr, buffer);
					eeprom_addr += 48;
					xQueueSend(q_uart , buffer , portMAX_DELAY );
					if (buffer[1] == 'D') {
						xTaskCreate(task_recto, "Recto", 2 * configMINIMAL_STACK_SIZE, NULL, 3, NULL);
					}
					else if (buffer[1] == 'A' || buffer[1] == 'B' || buffer[1] == 'C') {
						xTaskCreate(task_curva, "Curva", 2 * configMINIMAL_STACK_SIZE, NULL, 3, NULL);
					}
					memset(buffer, 0, sizeof(buffer));
					cod_num = 0; 
					cod_letter = 0;
				}
				// Solo imprime 'Formato inválido' si no es R# ni M#
				else if (!(index == 1 && (buffer[0] == 'R' || buffer[0] == 'M' || buffer[0] == 'E' || buffer[0] == 'N'))) {
					//printf("Formato inválido\n");
				}

				// Reinicia el buffer y el índice después de procesar la cadena
				index = 0;
				memset(buffer, 0, sizeof(buffer));
			}

			// Si el buffer se llena sin recibir '#', lo reinicia para evitar desbordes
			if (index >= 4) {
				index = 0;
				memset(buffer, 0, sizeof(buffer));
				printf("Buffer reiniciado por desborde\n");
			}
		}
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}

