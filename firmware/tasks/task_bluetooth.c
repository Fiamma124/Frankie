#include <math.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "string.h"
#include "config.h"
#include "globals.h"
#include "motor.h"
#include "task_bluetooth.h"
#include "task_recto.h"
#include "task_curva.h"

// Tamaños y límites EEPROM (AT24C32 = 4096 bytes)
#define LOG_RECORD_SIZE 48
#define EEPROM_TOTAL_BYTES 4096
// Base del área de LOG (reservamos 0x000-0x1FF para metadatos / parámetros fijos)
#define EEPROM_LOG_BASE 0x200

// Dirección actual de escritura de logs (persistida en RAM tras escanear al inicio)
static uint16_t eeprom_addr = 0;
static SemaphoreHandle_t xI2CMutex = NULL; // para serializar I2C si hiciera falta

// Declaración adelantada: función para borrar prefijo hasta EEPROM_LOG_BASE
bool eeprom_erase_prefix(i2c_inst_t *i2c, uint16_t end_addr);

// Helper: imprimir líneas del menú sin inundar USB
static inline void bt_menu_println(const char *s) {
    printf("%s\n", s);
    vTaskDelay(pdMS_TO_TICKS(2));
}

// Escanea la EEPROM en bloques de LOG_RECORD_SIZE hasta encontrar un bloque vacío (0xFF o 0x00 en primer byte)
// Devuelve la dirección del próximo bloque libre. Si está llena, devuelve 0xFFFF.
static uint16_t eeprom_find_next_log_addr(i2c_inst_t *i2c) {
    uint8_t buf[LOG_RECORD_SIZE];
    for (uint16_t addr = EEPROM_LOG_BASE; addr + LOG_RECORD_SIZE <= EEPROM_TOTAL_BYTES; addr += LOG_RECORD_SIZE) {
        if (!at24c32_read(i2c, addr, buf, LOG_RECORD_SIZE)) {
            printf("[EEPROM] Error leyendo en 0x%03X, usando esa posición para siguiente log\n", addr);
            return addr;
        }
        if (buf[0] == 0xFF || buf[0] == 0x00) {
            return addr; // bloque libre
        }
    }
    return 0xFFFF; // lleno
}

// (Eliminado) Lectura en trozos: el usuario prefiere usar at24c32_read directamente.

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

    // Inicializar mutex I2C (si otros módulos lo usan, convendría mover a init global)
    if (xI2CMutex == NULL) {
        xI2CMutex = xSemaphoreCreateMutex();
    }

    // Inicializar puntero de logs una sola vez
    if (eeprom_addr == 0) {
        uint16_t next = eeprom_find_next_log_addr(i2c1);
        if (next == 0xFFFF) {
            printf("[EEPROM] Log lleno. Reiniciando a base 0x%03X (modo circular).\n", EEPROM_LOG_BASE);
            eeprom_addr = EEPROM_LOG_BASE; // circular
        } else {
            eeprom_addr = next;
            printf("[EEPROM] Próximo log en dirección 0x%03X\n", eeprom_addr);
        }
    }

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

                if (strlen(line) == 1 && line[0] == 'T') {
                    print_fecha_hora_rtc(i2c1);
                    at24c32_write_log(i2c1, eeprom_addr, "T#");
                    eeprom_addr += LOG_RECORD_SIZE;
                    if (eeprom_addr + LOG_RECORD_SIZE > EEPROM_TOTAL_BYTES) eeprom_addr = EEPROM_LOG_BASE;
                }
                else if (strlen(line) == 1 && line[0] == 'M') {
                    // Menú (ASCII puro y pausado para evitar floods en USB)
                    bt_menu_println("");
                    bt_menu_println("--- MENU DE COMANDOS BLUETOOTH ---");
                    bt_menu_println("T#  - Mostrar fecha y hora RTC en EEPROM");
                    bt_menu_println("L#  - Mostrar registros de LOG en EEPROM");
                    bt_menu_println("E#  - Borrar el LOG de la EEPROM");
                    bt_menu_println("N#  - Cargar PID desde EEPROM (no inicia)");
                    bt_menu_println("Y#  - Leer PID optimos guardados");
                    bt_menu_println("TUNE kpL kiL kdL kpR kiR kdR# (no inicia)");
                    bt_menu_println("TUNE_L kpL kiL kdL# | TUNE_R kpR kiR kdR# (no inicia)");
                    bt_menu_println("RUN v t# - Iniciar recto con setpoint v (m/s) por t (s)");
                    bt_menu_println("------------------------------------");
                }
                else if (strlen(line) == 1 && line[0] == 'L') { //Muestra en pantalla el Log EEPROM
                    const int registro_size = LOG_RECORD_SIZE;
                    uint8_t mem_buf[registro_size];
                    uint16_t addr = EEPROM_LOG_BASE; // listar desde la base de LOG
                    int registro_num = 0;
                    printf("\n--- LOG EEPROM (desde 0x%03X) ---\n", EEPROM_LOG_BASE);
                    const int soft_cap = 128; // límite suave para evitar floods
                    while (addr + registro_size <= EEPROM_TOTAL_BYTES) {
                        if (!at24c32_read(i2c1, addr, mem_buf, registro_size)) {
                            printf("Error leyendo EEPROM en registro %d\n", registro_num);
                            break;
                        }
                        if (mem_buf[0] == 0xFF || mem_buf[0] == 0x00) {
                            break;
                        }
                        mem_buf[registro_size-1] = '\0';
                        printf("%d (0x%03X): %s\n", registro_num+1, addr, mem_buf);
                        addr += registro_size;
                        registro_num++;
                        if ((registro_num % 8) == 0) {
                            // Pausa corta para no saturar USB
                            vTaskDelay(pdMS_TO_TICKS(2));
                        }
                        if (registro_num >= soft_cap) {
                            printf("... (truncado a %d entradas, use E# para limpiar o divida en consultas)\n", soft_cap);
                            break;
                        }
                    }
                    if (registro_num == 0) {
                        printf("(Sin registros)\n");
                    }
                    // Mostrar porcentaje de uso del área de LOG
                    {
                        const int total_log_bytes = EEPROM_TOTAL_BYTES - EEPROM_LOG_BASE;
                        const int used_bytes = registro_num * registro_size;
                        int pct = 0;
                        if (total_log_bytes > 0) pct = (used_bytes * 100) / total_log_bytes;
                        printf("Uso de LOG: %d%% (%d/%d bytes)\n", pct, used_bytes, total_log_bytes);
                    }
                    printf("--- FIN LOG ---\n");
                        at24c32_write_log(i2c1, eeprom_addr, "L#");
                        eeprom_addr += LOG_RECORD_SIZE;
                        if (eeprom_addr + LOG_RECORD_SIZE > EEPROM_TOTAL_BYTES) eeprom_addr = EEPROM_LOG_BASE;
                }
                else if (strlen(line) == 1 && line[0] == 'E') {// Borra el log de la EEPROM desde la base
                    printf("Comando E# recibido: borrando LOG de la EEPROM...\n");
                    bool ok = eeprom_erase(i2c1); 
                    if (ok) {
                        printf("Borrado completado, registrando evento E#\n");
                        eeprom_addr = EEPROM_LOG_BASE; // reset puntero base
                        at24c32_write_log(i2c1, eeprom_addr, "E#");
                        eeprom_addr += LOG_RECORD_SIZE;
                        if (eeprom_addr + LOG_RECORD_SIZE > EEPROM_TOTAL_BYTES) eeprom_addr = EEPROM_LOG_BASE;
                    } else {
                        printf("Borrado fallido: no se registrará el evento E# en EEPROM\n");
                    }
                }
                else if (strlen(line) == 1 && line[0] == 'N') { // Tunear con PID desde EEPROM
                    printf("Comando N# recibido: cargando PID de 0x100 y 0x140 y enviando TUNE (no inicia)...\n");
                    char pidl_buf[32] = {0};
                    char pidr_buf[32] = {0};
                    float Kp_left = 0, Ki_left = 0, Kd_left = 0;
                    float Kp_right = 0, Ki_right = 0, Kd_right = 0;

                    bool okL = at24c32_read(i2c1, 0x100, (uint8_t*)pidl_buf, 32);
                    bool okR = at24c32_read(i2c1, 0x140, (uint8_t*)pidr_buf, 32);
                    pidl_buf[31] = '\0';
                    pidr_buf[31] = '\0';

                    if (!okL || !okR) {
                        printf("[EEPROM] Error leyendo PID (L=%d, R=%d)\n", okL, okR);
                    } else {
                        bool haveL = (strncmp(pidl_buf, "PIDL:", 5) == 0) && (sscanf(pidl_buf+5, "%f,%f,%f", &Kp_left, &Ki_left, &Kd_left) == 3);
                        bool haveR = (strncmp(pidr_buf, "PIDR:", 5) == 0) && (sscanf(pidr_buf+5, "%f,%f,%f", &Kp_right, &Ki_right, &Kd_right) == 3);
                        if (haveL && haveR) {
                            // Asegurar que task_recto exista
                            if (h_task_recto == NULL) {
                                xTaskCreate(task_recto, "Recto", 4 * configMINIMAL_STACK_SIZE, NULL, 3, &h_task_recto);
                                vTaskDelay(pdMS_TO_TICKS(10));
                            }
                            char tune_cmd[128];
                            // Enviamos solo PID. RUN es el único que inicia la marcha.
                            snprintf(tune_cmd, sizeof(tune_cmd), "TUNE %.3f %.3f %.3f %.3f %.3f %.3f#",
                                     Kp_left, Ki_left, Kd_left, Kp_right, Ki_right, Kd_right);
                            if (xQueueSend(q_uart, tune_cmd, pdMS_TO_TICKS(500)) == pdTRUE) {
                                printf("ENVIADO %s\n", tune_cmd);
                            } else {
                                printf("ERROR_COLA\n");
                            }
                            // Log del evento
                            at24c32_write_log(i2c1, eeprom_addr, "N#");
                            eeprom_addr += LOG_RECORD_SIZE;
                            if (eeprom_addr + LOG_RECORD_SIZE > EEPROM_TOTAL_BYTES) eeprom_addr = EEPROM_LOG_BASE;
                        } else {
                            if (!haveL) printf("[EEPROM] PID LEFT: No válido ('%s')\n", pidl_buf);
                            if (!haveR) printf("[EEPROM] PID RIGHT: No válido ('%s')\n", pidr_buf);
                        }
                    }
                }
                else if (strlen(line) == 1 && line[0] == 'Y') {
                    // Leer PID guardados usando solo at24c32_read, con mutex para I2C
                    printf("Comando Y# recibido: leyendo PID en 0x100 y 0x140...\n");
                    char pidl_buf[32] = {0};
                    char pidr_buf[32] = {0};
                    float Kp_left = 0, Ki_left = 0, Kd_left = 0;
                    float Kp_right = 0, Ki_right = 0, Kd_right = 0;
                    printf("Leyendo PID guardados en EEPROM...\n");
                    // if (xI2CMutex) xSemaphoreTake(xI2CMutex, portMAX_DELAY);
                    bool okL = at24c32_read(i2c1, 0x100, (uint8_t*)pidl_buf, 32); // una página
                    bool okR = at24c32_read(i2c1, 0x140, (uint8_t*)pidr_buf, 32);
                    // if (xI2CMutex) xSemaphoreGive(xI2CMutex);
                    printf("Resultados de lectura:\n");
                    pidl_buf[31] = '\0';
                    pidr_buf[31] = '\0';

                    if (!okL) {
                        printf("[EEPROM] PID LEFT: Error de lectura\n");
                    } else if (strncmp(pidl_buf, "PIDL:", 5) == 0) {
                        if (sscanf(pidl_buf+5, "%f,%f,%f", &Kp_left, &Ki_left, &Kd_left) == 3)
                            printf("[EEPROM] PID LEFT: Kp=%.3f, Ki=%.3f, Kd=%.3f\n", Kp_left, Ki_left, Kd_left);
                        else
                            printf("[EEPROM] PID LEFT: Formato inválido ('%s')\n", pidl_buf);
                    } else {
                        printf("[EEPROM] PID LEFT: No guardado\n");
                    }

                    if (!okR) {
                        printf("[EEPROM] PID RIGHT: Error de lectura\n");
                    } else if (strncmp(pidr_buf, "PIDR:", 5) == 0) {
                        if (sscanf(pidr_buf+5, "%f,%f,%f", &Kp_right, &Ki_right, &Kd_right) == 3)
                            printf("[EEPROM] PID RIGHT: Kp=%.3f, Ki=%.3f, Kd=%.3f\n", Kp_right, Ki_right, Kd_right);
                        else
                            printf("[EEPROM] PID RIGHT: Formato inválido ('%s')\n", pidr_buf);
                    } else {
                        printf("[EEPROM] PID RIGHT: No guardado\n");
                    }
                }
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
                        printf("ENVIADO %s\n", line);
                    } else {
                        printf("ERROR_COLA\n");
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

