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

static void sanitize_line(const char *in, char *out, size_t out_sz) {
    // Quita espacios/quotes al inicio/fin, elimina quotes internos, colapsa espacios, y mayusculiza
    const char *p = in;
    // saltar espacios/quotes iniciales
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n' || *p == '"' || *p == '\'') p++;
    size_t o = 0;
    int prev_space = 1;
    for (; *p && o + 1 < out_sz; ++p) {
        char c = *p;
        if (c == '#') break; // por si acaso
        if (c == '"' || c == '\'') continue; // descartar quotes
        if (c == '\r' || c == '\n') break;
        int is_space = (c == ' ' || c == '\t');
        if (is_space) {
            if (prev_space) continue; // colapsar múltiples
            out[o++] = ' ';
            prev_space = 1;
        } else {
            // mayúsculas universales
            if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');
            out[o++] = c;
            prev_space = 0;
        }
    }
    // retirar espacio final si quedó
    if (o > 0 && out[o-1] == ' ') o--;
    out[o] = '\0';
}

void task_bluetooth(void *params) {
    char buffer[MAX_INPUT + 1] = {0}; // Línea completa hasta '#'
    int index = 0;
    char c;

    static uint16_t eeprom_addr = 0;

    while (true) {
        if (xQueueReceive(q_bluetooth_chars, &c, portMAX_DELAY)) {
            if (c != '#') {
                if (index < MAX_INPUT) {
                    buffer[index++] = c;
                } else {
                    // Overflow: reseteamos para evitar desbordes
                    index = 0;
                    memset(buffer, 0, sizeof(buffer));
                    printf("Buffer reiniciado por desborde (MAX_INPUT=%d)\n", MAX_INPUT);
                }
            }

            if (c == '#') {
                buffer[index] = '\0';
                char line[MAX_INPUT + 1] = {0};
                sanitize_line(buffer, line, sizeof(line));
                // Si la línea quedó vacía (solo CR/LF/espacios/quotes), ignorar silenciosamente
                if (line[0] == '\0') {
                    index = 0;
                    memset(buffer, 0, sizeof(buffer));
                    continue;
                }
                printf("Secuencia completa recibida: %s\n", line);

                // 1) Comandos de una sola letra (compatibilidad)
                if (strlen(line) == 1 && line[0] == 'R') {
                    print_fecha_hora_rtc(i2c1);
                    at24c32_write_log(i2c1, eeprom_addr, "R#");
                    eeprom_addr += 48;
                }
                else if (strlen(line) == 1 && line[0] == 'M') {
                    // Menú
                    printf("\n--- MENÚ DE COMANDOS BLUETOOTH ---\n");
                    printf("R#  - Registrar fecha y hora RTC en EEPROM\n");
                    printf("L#  - Mostrar registros de log en EEPROM\n");
                    printf("E#  - Borrar toda la EEPROM\n");
                    printf("N#  - Ejecutar autotuning PID (Twiddle)\n");
                    printf("Y#  - Leer PID óptimos guardados\n");
                    printf("Q#  - Calibración de las ruedas\n");
                    printf("P#  - Guardar comando personalizado\n");
                    printf("[1-3][A-D]# - Ejecutar recorrido (ej: 1A#, 2B#)\n");
                    printf("TUNE kpL kiL kdL kpR kiR kdR v t# - Sintonía rápida y corrida\n");
                    printf("TUNE_L kpL kiL kdL# | TUNE_R kpR kiR kdR#\n");
                    printf("RUN v t# - Ejecutar recto con setpoint v (m/s) por t (s)\n");
                    printf("------------------------------------\n");
                }
                else if (strlen(line) == 1 && line[0] == 'L') {
                    // Log EEPROM
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
                        if (mem_buf[0] == 0xFF || mem_buf[0] == 0x00) {
                            break;
                        }
                        mem_buf[registro_size-1] = '\0';
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
                else if (strlen(line) == 1 && line[0] == 'E') {
                    // Borrar EEPROM
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
                else if (strlen(line) == 1 && line[0] == 'N') {
                    // Letra libre
                    at24c32_write_log(i2c1, eeprom_addr, "N#");
                    eeprom_addr += 48;
                }
                else if (strlen(line) == 1 && line[0] == 'Y') {
                    // Leer PID guardados
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
                // 2) Comandos de recorrido corto [1-3][A-D]
                else if (strlen(line) == 2 && (line[0] >= '1' && line[0] <= '3') &&
                         (line[1] == 'A' || line[1] == 'B' || line[1] == 'C' || line[1] == 'D')) {
                    at24c32_write_log(i2c1, eeprom_addr, line);
                    eeprom_addr += 48;
                    // reenviar a q_uart para tareas que lo necesiten
                    xQueueSend(q_uart, line, portMAX_DELAY);
                    if (line[1] == 'D') {
                        xTaskCreate(task_recto, "Recto", 2 * configMINIMAL_STACK_SIZE, NULL, 3, NULL);
                    } else {
                        xTaskCreate(task_curva, "Curva", 2 * configMINIMAL_STACK_SIZE, NULL, 3, NULL);
                    }
                }
                // 3) NUEVOS: TUNE, TUNE_L, TUNE_R, RUN (texto largo)
                else if (strncmp(line, "TUNE ", 5) == 0 || strncmp(line, "TUNE_L ", 7) == 0 ||
                         strncmp(line, "TUNE_R ", 7) == 0 || strncmp(line, "RUN ", 4) == 0) {
                    // Asegurar que la tarea recto exista para ejecutar la prueba (singleton)
                    if (h_task_recto == NULL) {
                        xTaskCreate(task_recto, "Recto", 4 * configMINIMAL_STACK_SIZE, NULL, 3, &h_task_recto);
                        // pequeña espera para que arranque y pueda recibir
                        vTaskDelay(pdMS_TO_TICKS(10));
                    }
                    // Enviar línea completa a q_uart para que task_recto la parsee
                    if (xQueueSend(q_uart, line, pdMS_TO_TICKS(500)) == pdTRUE) {
                        printf("OK CMD -> q_uart: %s\n", line);
                    } else {
                        printf("ERROR: q_uart ocupada\n");
                    }
                }
                else {
                    // Comando desconocido o formato inválido
                    printf("Comando no reconocido: '%s'\n", line);
                }

                // Reset de línea
                index = 0;
                memset(buffer, 0, sizeof(buffer));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

