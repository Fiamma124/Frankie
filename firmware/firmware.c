#include <math.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "hardware/pwm.h"
#include "string.h"
#include "lcd.h"
#include "hardware/i2c.h"
#include "rtc.h"


// MACROS
// Eleccion de GPIO para SDA y SCL para el RTC
#define SDA_RTC    2
#define SCL_RTC    3
// Pines para control de dirección y PWM
#define MOTOR_M2_IN1 18
#define MOTOR_M2_IN2 19
#define MOTOR_M1_IN1 20
#define MOTOR_M1_IN2 21
#define MOTOR_RIGHT_PWM 22  // GPIO con soporte PWM
#define MOTOR_LEFT_PWM 26  // GPIO con soporte PWM
//Pines para los tacometros
#define IN_PIN_TACOMETRO_LEFT 28
#define IN_PIN_TACOMETRO_RIGHT 27
// Configuración de la frecuencia PWM (Hz)
#define PWM_FREQ 20000
// Cambiar velocidad (0 a 255)
#define MOTOR_LEFT      1
#define MOTOR_RIGHT     2
#define PI 3.14159265f
//Modos 
//SEGUN EL MODO, PONEMOS LA VELOCIDAD
#define VEL_01  97.0f
#define VEL_02  76.5f
#define VEL_03  57.9f
#define CURVA_01 0.75f
#define CURVA_02 0.5f
#define CURVA_03 0.25f
#define RECTA 0.0f
// Muestras para el filtro de media movil
#define N_PROM 5  // (0.5 s si el periodo es 100 ms)


//Constantes del auto
#define ANCHO 0.134f // Mitad del ancho de las ruedas
#define R_WHEEL 0.0325f // Radio de las ruedas

//Configuración del teclado matricial
#define FILAS 4
#define COLUMNAS 4
#define MAX_INPUT 4

//Defino lo necesario para hacer andar la UART
#define UART_ID uart0
#define UART_TX_PIN 16
#define UART_RX_PIN 17
#define BAUD_RATE 115200

//#define PWMBASE 245    
// Prototipo para evitar error de declaración
void task_recto(void *params);
void task_curva(void *params);
void task_tune_left_twiddle(void *params);
void task_tune_right_twiddle(void *params);
void task_calibracion(void *params);


//Defino mi cola
//QueueHandle_t q_matrix ;
QueueHandle_t q_tacometro_left ;
QueueHandle_t q_tacometro_right ;
QueueHandle_t q_uart ;
QueueHandle_t q_vel_impuesta_left ;
QueueHandle_t q_vel_impuesta_right ;
QueueHandle_t q_bluetooth_chars;
QueueHandle_t q_codigo;

// ===== Calibración de ruedas: zona muerta (d_dead) y ganancia k =====
// Requiere: q_tacometro_left, q_tacometro_right (vel en m/s cada 100 ms)
//           motor_set_direction(), motor_set_speed() ya existentes
// Umbrales ajustables:
#define SWEEP_STEP_DUTY          8      // paso de duty
#define SWEEP_SETTLE_MS          400    // tiempo para estabilizar en cada duty
#define SWEEP_AVG_SAMPLES        5      // muestras de 100 ms cada una (asumimos tacómetro a 100 ms)
#define DEAD_V_THRESHOLD         0.04f  // m/s a partir del cual consideramos "empezó a moverse"
#define REG_MARGIN_DUTY          12     // margen por encima de d_dead para la regresión

typedef struct {
    float d_dead;   // en niveles de PWM (0..255)
    float k;        // [m/s por nivel de PWM] sobre (duty - d_dead)
} calib_result_t;


typedef struct {
    float v_inner;
    float v_outer;
} VelData_t;

void on_uart_rx(void);

void configurar_gpio() {
    //INICIALIZACION DE LOS TACOMETROS
    gpio_init(IN_PIN_TACOMETRO_LEFT);
    gpio_set_dir(IN_PIN_TACOMETRO_LEFT, GPIO_IN);
    gpio_pull_down(IN_PIN_TACOMETRO_LEFT);

    gpio_init(IN_PIN_TACOMETRO_RIGHT);
    gpio_set_dir(IN_PIN_TACOMETRO_RIGHT, GPIO_IN);
    gpio_pull_down(IN_PIN_TACOMETRO_RIGHT);

    
    //INICIALIZACION DEL MOTOR
    //motor_init();
    //motor_set_direction(true);  

    //CONFIUGRACION DEL DISPLAY
    i2c_init(i2c1, 100000);
    //Seteo la funcion
    gpio_set_function(SDA_RTC, GPIO_FUNC_I2C);
    gpio_set_function(SCL_RTC, GPIO_FUNC_I2C);
    // Habilito pull-ups
    gpio_pull_up(SDA_RTC);
    gpio_pull_up(SCL_RTC);
  
}

void init_uart() {
    // Inicialización UART
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART); 
    stdio_uart_init_full(uart0, BAUD_RATE, UART_TX_PIN, UART_RX_PIN);

    // Configurar interrupciones UART
    irq_set_exclusive_handler(UART0_IRQ, on_uart_rx);
    irq_set_enabled(UART0_IRQ, true);
    uart_set_irq_enables(UART_ID, true, false);

    // Cola para caracteres entrantes
    
}

volatile static char rx_index = 0;
volatile static char rx_buffer[MAX_INPUT];

void on_uart_rx() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    while (uart_is_readable(UART_ID)) {
        char c = uart_getc(UART_ID);
        xQueueSendFromISR(q_bluetooth_chars, &c, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}


void motor_init() {
    float clock = 125000000 ;
    float divider = clock / (PWM_FREQ * (255+1));
    // Dirección
    gpio_init(MOTOR_M1_IN1);
    gpio_set_dir(MOTOR_M1_IN1, GPIO_OUT);
    gpio_put(MOTOR_M1_IN1, 0);

    gpio_init(MOTOR_M1_IN2);
    gpio_set_dir(MOTOR_M1_IN2, GPIO_OUT);
    gpio_put(MOTOR_M1_IN2, 0);

    gpio_init(MOTOR_M2_IN1);
    gpio_set_dir(MOTOR_M2_IN1, GPIO_OUT);
    gpio_put(MOTOR_M2_IN1, 0);

    gpio_init(MOTOR_M2_IN2);
    gpio_set_dir(MOTOR_M2_IN2, GPIO_OUT);
    gpio_put(MOTOR_M2_IN2, 0);


    // PWM Motor 1
    gpio_set_function(MOTOR_LEFT_PWM, GPIO_FUNC_PWM);
    uint slice1 = pwm_gpio_to_slice_num(MOTOR_LEFT_PWM);
    pwm_set_clkdiv(slice1, divider);
    pwm_set_wrap(slice1, 255);
    pwm_set_chan_level(slice1, pwm_gpio_to_channel(MOTOR_LEFT_PWM), 0);
    pwm_set_enabled(slice1, true);

    // PWM Motor 2
    gpio_set_function(MOTOR_RIGHT_PWM, GPIO_FUNC_PWM);
    uint slice2 = pwm_gpio_to_slice_num(MOTOR_RIGHT_PWM);
    pwm_set_clkdiv(slice2, divider);
    pwm_set_wrap(slice2, 255);
    pwm_set_chan_level(slice2, pwm_gpio_to_channel(MOTOR_RIGHT_PWM), 0);
    pwm_set_enabled(slice2, true);
}

// Cambiar dirección del motor
void motor_set_direction(uint8_t motor, bool forward) {
    if (motor == MOTOR_LEFT) {
        gpio_put(MOTOR_M1_IN1, forward ? 1 : 0);
        gpio_put(MOTOR_M1_IN2, forward ? 0 : 1);
    } else if (motor == MOTOR_RIGHT) {
        gpio_put(MOTOR_M2_IN1, forward ? 1 : 0);
        gpio_put(MOTOR_M2_IN2, forward ? 0 : 1);
    }
}

// Cambiar velocidad (0 a 255)
void motor_set_speed(uint8_t motor, uint16_t speed) {
    if (motor == MOTOR_LEFT) {
        uint slice1 = pwm_gpio_to_slice_num(MOTOR_LEFT_PWM);
        pwm_set_chan_level(slice1, pwm_gpio_to_channel(MOTOR_LEFT_PWM), speed);
    }
    else if (motor == MOTOR_RIGHT) {
        uint slice2 = pwm_gpio_to_slice_num(MOTOR_RIGHT_PWM);
        pwm_set_chan_level(slice2, pwm_gpio_to_channel(MOTOR_RIGHT_PWM), speed);
    }
}


// Imprime la fecha y hora en formato DDMMAA-HH:MM usando el RTC
void print_fecha_hora_rtc(i2c_inst_t *i2c) {
    ds1307_time_t t;
    if (ds1307_get_time(i2c, &t)) {
        // Formato: DDMMAA-HH:MM
        printf("%02d%02d%02d-%02d:%02d\n", t.day, t.month, t.year, t.hours, t.minutes);
    } else {
        printf("Error leyendo RTC\n");
    }
}

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
void task_tacometro_right(void *params) {
    int counter = 0;
    float vueltas = 0;
    int estado_anterior = gpio_get(IN_PIN_TACOMETRO_RIGHT);
    char str[16];
    float vel_derecha = 0;

    // buffers y estado del filtro (privados de ESTA tarea)
    static float buf[N_PROM] = {0};
    static float sum = 0.0f;
    static int   idx = 0;

    TickType_t start_tick = xTaskGetTickCount();  // tiempo de referencia
    const TickType_t periodo = pdMS_TO_TICKS(100);  // 100 ms
    const float T = ((float)periodo) / configTICK_RATE_HZ;

    while (true) {
        int estado_actual = gpio_get(IN_PIN_TACOMETRO_RIGHT);
        // Detectar cambio de estado (flanco de subida)
        if (estado_actual != estado_anterior) {
            estado_anterior = estado_actual;

            if (estado_actual == 1) {
                counter++;
            }
        }

        // ¿Pasó 1 segundo?
        if ((xTaskGetTickCount() - start_tick) >= periodo) {
            vueltas = counter / 20.0f;
            vel_derecha = vueltas * 2 * PI * R_WHEEL / T; // m/s

            // media móvil incremental
            sum += vel_derecha - buf[idx];
            buf[idx] = vel_derecha;
            idx = (idx + 1) % N_PROM;
            const float v_filtrada = sum / (float)N_PROM;

            xQueueOverwrite(q_tacometro_right , &v_filtrada );
            //xQueuePeek(q_tacometro_right, &vel_derecha, 0);
            //printf("Vel Der: %.3f [m/s], counter: %d, vueltas: %.2f\n", vel_derecha, counter, vueltas);
            counter = 0;
            vueltas = 0;
            start_tick = xTaskGetTickCount();  // reinicio del conteo
        }

        vTaskDelay(pdMS_TO_TICKS(1));  // respiro para evitar uso 100% CPU
    }
}

void task_tacometro_left(void *params) {
    int counter = 0;
    float vueltas = 0;
    int estado_anterior = gpio_get(IN_PIN_TACOMETRO_LEFT);
    char str[16];
    float vel_izquierda = 0;

    // buffers y estado del filtro (privados de ESTA tarea)
    static float buf[N_PROM] = {0};
    static float sum = 0.0f;
    static int   idx = 0;

    TickType_t start_tick = xTaskGetTickCount();  // tiempo de referencia
    const TickType_t periodo = pdMS_TO_TICKS(100);  // 100 ms
    const float T = ((float)periodo) / configTICK_RATE_HZ;

    while (true) {
        int estado_actual = gpio_get(IN_PIN_TACOMETRO_LEFT);
        // Detectar cambio de estado (flanco de subida)
        if (estado_actual != estado_anterior) {
            estado_anterior = estado_actual;

            if (estado_actual == 1) {
                counter++;
            }
        }

        // ¿Pasó 1 segundo?
        if ((xTaskGetTickCount() - start_tick) >= periodo) {
            vueltas = counter / 20.0f;
            vel_izquierda = vueltas * 2 * PI * R_WHEEL / T; // m/s

            // media móvil incremental
            sum += vel_izquierda - buf[idx];
            buf[idx] = vel_izquierda;
            idx = (idx + 1) % N_PROM;
            const float v_filtrada = sum / (float)N_PROM;
            xQueueOverwrite(q_tacometro_left , &v_filtrada );
            //xQueuePeek(q_tacometro_left, &vel_izquierda, 0);
            //printf("Vel Izq: %.3f [m/s], counter: %d, vueltas: %.2f\n", vel_izquierda, counter, vueltas);
            counter = 0;
            vueltas = 0;
            start_tick = xTaskGetTickCount();  // reinicio del conteo
        }

        vTaskDelay(pdMS_TO_TICKS(1));  // respiro para evitar uso 100% CPU
    }
}

// Lectura de velocidad desde la cola correspondiente
static bool leer_velocidad_rueda(uint8_t rueda, float *v, TickType_t timeout_ticks) {
    if (rueda == MOTOR_LEFT) {
        return xQueueReceive(q_tacometro_left, v, timeout_ticks);
    } else {
        return xQueueReceive(q_tacometro_right, v, timeout_ticks);
    }
}

// Sweep por rueda y sentido. Devuelve d_dead y k (por referencia).
static void sweep_rueda(uint8_t rueda, bool forward, calib_result_t *out) {
    // Buffers para recolectar datos (duty, v)
    float duty_buf[256 / SWEEP_STEP_DUTY + 2];
    float v_buf[256 / SWEEP_STEP_DUTY + 2];
    int    n = 0;

    // Configurar sentido
    motor_set_direction(rueda, forward);

    // Barrido de duty
    for (int duty = 0; duty <= 255; duty += SWEEP_STEP_DUTY) {
        motor_set_speed(rueda, duty);
        vTaskDelay(pdMS_TO_TICKS(SWEEP_SETTLE_MS));  // estabilizar

        // Promediar SWEEP_AVG_SAMPLES muestras del tacómetro (asumimos 100 ms cada una)
        float v_sum = 0.0f;
        int   got   = 0;
        for (int i = 0; i < SWEEP_AVG_SAMPLES; i++) {
            float v = 0.0f;
            if (leer_velocidad_rueda(rueda, &v, pdMS_TO_TICKS(150))) {
                v_sum += v;
                got++;
            } else {
                // Si no llegó a tiempo, esperá un poco y seguí intentando
                vTaskDelay(pdMS_TO_TICKS(20));
            }
        }
        float v_avg = (got > 0) ? (v_sum / (float)got) : 0.0f;

        // Guardar muestra
        duty_buf[n] = (float)duty;
        v_buf[n]    = v_avg;
        n++;

        // Log CSV en vivo (podés comentar si molesta)
        printf("CAL,%s,%s,%d,%.5f\n",
               (rueda == MOTOR_LEFT) ? "L" : "R",
               forward ? "FWD" : "REV",
               duty, v_avg);
    }

    // Detener la rueda
    motor_set_speed(rueda, 0);

    // 1) Encontrar d_dead: primer duty con v >= DEAD_V_THRESHOLD
    float d_dead = 255.0f;
    for (int i = 0; i < n; i++) {
        if (v_buf[i] >= DEAD_V_THRESHOLD) {
            d_dead = duty_buf[i];
            break;
        }
    }

    // 2) Calcular k con regresión lineal sobre puntos por encima de (d_dead + REG_MARGIN_DUTY)
    float x_sum = 0.0f, y_sum = 0.0f, xx_sum = 0.0f, xy_sum = 0.0f;
    int   m     = 0;
    for (int i = 0; i < n; i++) {
        float duty = duty_buf[i];
        float v    = v_buf[i];
        if (duty >= (d_dead + REG_MARGIN_DUTY) && v > 0.0f) {
            float x = duty - d_dead;   // centramos en d_dead
            float y = v;               // velocidad en m/s
            x_sum  += x;
            y_sum  += y;
            xx_sum += x * x;
            xy_sum += x * y;
            m++;
        }
    }

    float k = 0.0f;
    if (m >= 2) {
        float denom = (m * xx_sum - x_sum * x_sum);
        if (denom != 0.0f) {
            k = (m * xy_sum - x_sum * y_sum) / denom;  // pendiente de la recta v = k*(duty - d_dead) + b
        }
    }

    // Fallback si no hubo puntos útiles o denominador ~ 0: estimación dos-puntos
    if (k <= 0.0f) {
        // Buscar dos puntos distantes por encima de d_dead
        int i1 = -1, i2 = -1;
        for (int i = 0; i < n; i++) {
            if (duty_buf[i] >= (d_dead + REG_MARGIN_DUTY) && v_buf[i] > 0.0f) { i1 = i; break; }
        }
        for (int i = n - 1; i >= 0; i--) {
            if (duty_buf[i] >= (d_dead + REG_MARGIN_DUTY) && v_buf[i] > 0.0f) { i2 = i; break; }
        }
        if (i1 >= 0 && i2 >= 0 && i2 > i1) {
            float x1 = duty_buf[i1] - d_dead;
            float x2 = duty_buf[i2] - d_dead;
            float y1 = v_buf[i1];
            float y2 = v_buf[i2];
            float dx = (x2 - x1);
            if (dx != 0.0f) k = (y2 - y1) / dx;
        }
    }

    // Si nunca se movió, d_dead=255 y k=0
    if (d_dead >= 255.0f) {
        d_dead = 255.0f;
        k = 0.0f;
    }

    // Salida
    if (out) { out->d_dead = d_dead; out->k = k; }

    // Log final legible
    printf("[CAL-RES] rueda=%s sentido=%s  d_dead=%.1f  k=%.6f (m/s por nivel)\n",
           (rueda == MOTOR_LEFT) ? "LEFT" : "RIGHT",
           forward ? "FWD" : "REV",
           d_dead, k);
}

// ====== Wrapper de tarea para calibrar ambas ruedas en avance ======
void task_calibracion(void *params) {
    calib_result_t L = {0}, R = {0};

    // Asumimos que tus tacómetros ya actualizan velocidad cada 100 ms.
    // Hacemos sweep de Izquierda y Derecha, en avance.
    sweep_rueda(MOTOR_LEFT,  true, &L);
    sweep_rueda(MOTOR_RIGHT, true, &R);

    printf("\n=== RESULTADOS CALIBRACIÓN (AVANCE) ===\n");
    printf("LEFT : d_dead=%.1f | k=%.6f (m/s por nivel)\n",  L.d_dead, L.k);
    printf("RIGHT: d_dead=%.1f | k=%.6f (m/s por nivel)\n",  R.d_dead, R.k);
    printf("CSV arriba (prefijo CAL, por duty)\n");

    // Podés guardar estos valores en variables globales o EEPROM si querés persistirlos.
    // Luego, en tu lazo de control, usarás:
    // u_ff_left  = L.d_dead  + v_ref / L.k;
    // u_ff_right = R.d_dead  + v_ref / R.k;

    // Terminar tarea si es one-shot
    vTaskDelete(NULL);
}



// ===== Recta con Feedforward + PI (dt = 0.1 s, duración 5 s) =====
void task_recto(void *params) {
    // --- Ganancias medidas (tu calibración) ---
    const float d_dead_L = 160.0f;
    const float k_L      = 0.05f;   // [m/s por nivel]
    const float d_dead_R = 176.0f;
    const float k_R      = 0.058f;  // [m/s por nivel]

    // --- Setpoints por comando (igual que antes) ---
    float setpoint_left  = 0.2f;   // m/s
    float setpoint_right = 0.2f;   // m/s
    char cmd_buffer[MAX_INPUT + 1] = {0};

    if (xQueueReceive(q_uart, &cmd_buffer, 0)) {
        switch (cmd_buffer[0]) {
            case '1': setpoint_left = setpoint_right = 0.2f;  break;
            case '2': setpoint_left = setpoint_right = 0.6f;  break;
            case '3': setpoint_left = setpoint_right = 1.0f;  break;
            default:  setpoint_left = setpoint_right = 0.2f;  break;
        }
    }

    // --- Controladores (PI por rueda) ---
    float Kp_left  = 90.0f, Ki_left  = 50.0f, Kd_left  = 0.0f;
    float Kp_right = 75.0f, Ki_right = 40.0f, Kd_right = 0.0f;

    float error_left = 0, error_right = 0;
    float error_left_prev = 0, error_right_prev = 0;
    float integral_left = 0, integral_right = 0;

    // --- Temporización: 100 ms coherente con tacómetro ---
    const TickType_t periodo = pdMS_TO_TICKS(100);
    const float dt = 0.1f;
    TickType_t last_wake = xTaskGetTickCount();

    // --- Slew-rate limit opcional ---
    const float SLEW_PER_STEP = 1000.0f;
    float u_left_prev = 0.0f, u_right_prev = 0.0f;

    // --- Direcciones: ambas hacia adelante físico ---
    motor_set_direction(MOTOR_LEFT,  true);
    motor_set_direction(MOTOR_RIGHT, true);

    // --- Tiempo total de prueba (8 s) ---
    const float total_time_s = 8.0f;
    const int total_cycles = (int)(total_time_s / dt);

    // --- Contador para printf cada 0.5 s ---
    int print_counter = 0;

    // --- Loop principal (duración limitada) ---
    for (int cycle = 0; cycle < total_cycles; cycle++) {
        // 1) Leer velocidades medidas
        float velocidad_left = 0.0f, velocidad_right = 0.0f;
        xQueuePeek(q_tacometro_left,  &velocidad_left,  pdMS_TO_TICKS(50));
        xQueuePeek(q_tacometro_right, &velocidad_right, pdMS_TO_TICKS(50));

        // 2) Feedforward
        float u_ff_L = d_dead_L + (setpoint_left  / k_L);
        float u_ff_R = d_dead_R + (setpoint_right / k_R);
        if (u_ff_L > 255) u_ff_L = 255; if (u_ff_L < 0) u_ff_L = 0;
        if (u_ff_R > 255) u_ff_R = 255; if (u_ff_R < 0) u_ff_R = 0;

        // 3) PI
        error_left  = setpoint_left  - velocidad_left;
        error_right = setpoint_right - velocidad_right;

        integral_left  += error_left  * dt;
        integral_right += error_right * dt;

        float derivada_left  = (error_left  - error_left_prev)  / dt;
        float derivada_right = (error_right - error_right_prev) / dt;

        float pid_left  = Kp_left  * error_left  + Ki_left  * integral_left  + Kd_left  * derivada_left;
        float pid_right = Kp_right * error_right + Ki_right * integral_right + Kd_right * derivada_right;

        // 4) Salida total
        float u_left  = u_ff_L + pid_left;
        float u_right = u_ff_R + pid_right;

        // 5) Saturación + anti-windup
        if (u_left > 255)  { u_left = 255;  integral_left  -= error_left  * dt; }
        if (u_left < 0)    { u_left = 0;    integral_left  -= error_left  * dt; }
        if (u_right > 255) { u_right = 255; integral_right -= error_right * dt; }
        if (u_right < 0)   { u_right = 0;   integral_right -= error_right * dt; }

        // 6) Slew-rate
        float duL = u_left  - u_left_prev;
        float duR = u_right - u_right_prev;
        if (duL >  SLEW_PER_STEP) u_left  = u_left_prev  + SLEW_PER_STEP;
        if (duL < -SLEW_PER_STEP) u_left  = u_left_prev  - SLEW_PER_STEP;
        if (duR >  SLEW_PER_STEP) u_right = u_right_prev + SLEW_PER_STEP;
        if (duR < -SLEW_PER_STEP) u_right = u_right_prev - SLEW_PER_STEP;
        u_left_prev  = u_left;
        u_right_prev = u_right;

        // 7) Aplicar PWM
        motor_set_speed(MOTOR_LEFT,  (uint16_t)u_left);
        motor_set_speed(MOTOR_RIGHT, (uint16_t)u_right);

        // 8) Log cada 0.5 s
        print_counter++;
        if (print_counter >= 5) { // cada 5 ciclos (0.5 s)
            print_counter = 0;
            printf("t=%.1f  vL=%.3f  vR=%.3f  |  uFF_L=%.1f  uFF_R=%.1f  |  uL=%.1f  uR=%.1f  |  eL=%.3f  eR=%.3f\n",
                   cycle * dt,
                   velocidad_left, velocidad_right,
                   u_ff_L, u_ff_R,
                   u_left, u_right,
                   error_left, error_right);
        }

        // 9) Actualizar errores y esperar siguiente ciclo
        error_left_prev  = error_left;
        error_right_prev = error_right;
        vTaskDelayUntil(&last_wake, periodo);
    }

    // --- Fin de prueba: detener motores ---
    motor_set_speed(MOTOR_LEFT,  0);
    motor_set_speed(MOTOR_RIGHT, 0);
    printf("\n--- Fin de prueba (8 s). Motores detenidos ---\n");

    // --- Eliminar la tarea ---
    vTaskDelete(NULL);
}

void task_curva(void *params) {
    const uint32_t tiempo_ms = 5000; // 5 segundos
    printf("Falta implementar curva\n");
    vTaskDelay(pdMS_TO_TICKS(tiempo_ms));
    vTaskDelete(NULL);
}

// Twiddle autotune para el motor izquierdo
void task_tune_left_twiddle(void *params) {
    float p[3] = {304.4f, 124.48f, 24.37f}; // Kp, Ki, Kd iniciales (mejor medidos)
    float dp[3] = {30.44f, 12.448f, 2.437f}; // ~10% de cada parámetro
    float best_error = 1e9;
    float setpoints[] = {0.2f, 0.6f, 1.0f};
    int num_setpoints = sizeof(setpoints)/sizeof(float);
    const float periodo = 0.20f; // segundos (alineado con tacómetro 200ms)
    const int ciclos = 24; // más ciclos para mejor estadística
    const int warmup = 8; // más warm-up para estabilizar
    const float threshold = 0.1f; // umbral más estricto para detener Twiddle
    const float anti_windup_limit = 10.0f; // mayor margen para integral
    const float alpha_vel = 0.3f; // filtro EMA para velocidad

    TickType_t last_wake;
    int max_iter = 80; // cantidad máxima de iteraciones
    int iter = 0;
    while ((dp[0] + dp[1] + dp[2] > threshold) && (iter < max_iter)) {
        printf("[Twiddle_L] Iteración num %d: dp_sum=%.4f, Kp=%.3f, Ki=%.3f, Kd=%.3f, best_error=%.3f\n", iter, dp[0]+dp[1]+dp[2], p[0], p[1], p[2], best_error);
    for (int i = 0; i < 3; i++) {
            float old_pi = p[i];
            p[i] += dp[i];
            printf("[Twiddle_L] Probar parámetro %d: p=%.3f, dp=%.4f\n", i, p[i], dp[i]);
            float error = 0;
            for (int s = 0; s < num_setpoints; s++) {
                float setpoint = setpoints[s];
                int pwmbase = (setpoint == 0.6f) ? 210 : (setpoint == 1.0f) ? 240 : 180;
                float error_prev = 0, integral = 0, velocidad_left = 0, v_filt = 0, v_filt_prev = 0;
                // Warm-up: estabiliza el sistema antes de medir error
                for (int t = 0; t < warmup; t++) {
                    xQueuePeek(q_tacometro_left, &velocidad_left, pdMS_TO_TICKS((int)(periodo*1000)));
                    // Filtrado EMA de la velocidad medida y derivada sobre medición
                    v_filt = alpha_vel * velocidad_left + (1 - alpha_vel) * v_filt_prev;
                    float e = setpoint - v_filt;
                    float derivada = -(v_filt - v_filt_prev) / periodo;
                    // Integración condicional (anti-windup)
                    float integral_candidate = integral + e * periodo;
                    if (integral_candidate > anti_windup_limit) integral_candidate = anti_windup_limit;
                    if (integral_candidate < -anti_windup_limit) integral_candidate = -anti_windup_limit;
                    float pid_candidate = p[0]*e + p[1]*integral_candidate + p[2]*derivada;
                    int pwm_candidate = pwmbase + (int)pid_candidate;
                    int pwm = pwm_candidate;
                    if (pwm > 255) pwm = 255;
                    if (pwm < 0) pwm = 0;
                    // Si saturó y el error empuja más a saturación, no integramos
                    if (!((pwm == 255 && e > 0) || (pwm == 0 && e < 0))) {
                        integral = integral_candidate;
                    }
                    motor_set_speed(MOTOR_LEFT, pwm);
                    error_prev = e;
                    v_filt_prev = v_filt;
                    last_wake = xTaskGetTickCount();
                    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS((int)(periodo*1000)));
                }
                // Medición de error robusta (RMSE)
                float sq_error = 0;
                for (int t = 0; t < ciclos; t++) {
                    xQueuePeek(q_tacometro_left, &velocidad_left, pdMS_TO_TICKS((int)(periodo*1000)));
                    v_filt = alpha_vel * velocidad_left + (1 - alpha_vel) * v_filt_prev;
                    float e = setpoint - v_filt;
                    float derivada = -(v_filt - v_filt_prev) / periodo;
                    float integral_candidate = integral + e * periodo;
                    if (integral_candidate > anti_windup_limit) integral_candidate = anti_windup_limit;
                    if (integral_candidate < -anti_windup_limit) integral_candidate = -anti_windup_limit;
                    float pid_candidate = p[0]*e + p[1]*integral_candidate + p[2]*derivada;
                    int pwm_candidate = pwmbase + (int)pid_candidate;
                    int pwm = pwm_candidate;
                    if (pwm > 255) pwm = 255;
                    if (pwm < 0) pwm = 0;
                    if (!((pwm == 255 && e > 0) || (pwm == 0 && e < 0))) {
                        integral = integral_candidate;
                    }
                    motor_set_speed(MOTOR_LEFT, pwm);
                    if (t >= (ciclos/2)) { // enfocar en régimen permanente
                        sq_error += e*e;
                    }
                    error_prev = e;
                    v_filt_prev = v_filt;
                    last_wake = xTaskGetTickCount();
                    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS((int)(periodo*1000)));
                }
                error += sqrtf(sq_error / ciclos); // RMSE
                motor_set_speed(MOTOR_LEFT, 0);
                vTaskDelay(pdMS_TO_TICKS(300));
            }
            if (error < best_error) {
                best_error = error;
                dp[i] *= 1.05f;
                printf("[Twiddle_L] ¡Mejora! Nuevo best_error=%.4f, dp[%d]=%.4f\n", best_error, i, dp[i]);
            } else {
                p[i] = old_pi - dp[i];
                error = 0;
                for (int s = 0; s < num_setpoints; s++) {
                    float setpoint = setpoints[s];
                    int pwmbase = (setpoint == 0.6f) ? 210 : (setpoint == 1.0f) ? 240 : 180;
                    float error_prev = 0, integral = 0, velocidad_left = 0, v_filt = 0, v_filt_prev = 0;
                    for (int t = 0; t < warmup; t++) {
                        xQueuePeek(q_tacometro_left, &velocidad_left, pdMS_TO_TICKS((int)(periodo*1000)));
                        v_filt = alpha_vel * velocidad_left + (1 - alpha_vel) * v_filt_prev;
                        float e = setpoint - v_filt;
                        float derivada = -(v_filt - v_filt_prev) / periodo;
                        float integral_candidate = integral + e * periodo;
                        if (integral_candidate > anti_windup_limit) integral_candidate = anti_windup_limit;
                        if (integral_candidate < -anti_windup_limit) integral_candidate = -anti_windup_limit;
                        float pid_candidate = p[0]*e + p[1]*integral_candidate + p[2]*derivada;
                        int pwm_candidate = pwmbase + (int)pid_candidate;
                        int pwm = pwm_candidate;
                        if (pwm > 255) pwm = 255;
                        if (pwm < 0) pwm = 0;
                        if (!((pwm == 255 && e > 0) || (pwm == 0 && e < 0))) {
                            integral = integral_candidate;
                        }
                        motor_set_speed(MOTOR_LEFT, pwm);
                        error_prev = e;
                        v_filt_prev = v_filt;
                        last_wake = xTaskGetTickCount();
                        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS((int)(periodo*1000)));
                    }
                    float sq_error = 0;
                    for (int t = 0; t < ciclos; t++) {
                        xQueuePeek(q_tacometro_left, &velocidad_left, pdMS_TO_TICKS((int)(periodo*1000)));
                        v_filt = alpha_vel * velocidad_left + (1 - alpha_vel) * v_filt_prev;
                        float e = setpoint - v_filt;
                        float derivada = -(v_filt - v_filt_prev) / periodo;
                        float integral_candidate = integral + e * periodo;
                        if (integral_candidate > anti_windup_limit) integral_candidate = anti_windup_limit;
                        if (integral_candidate < -anti_windup_limit) integral_candidate = -anti_windup_limit;
                        float pid_candidate = p[0]*e + p[1]*integral_candidate + p[2]*derivada;
                        int pwm_candidate = pwmbase + (int)pid_candidate;
                        int pwm = pwm_candidate;
                        if (pwm > 255) pwm = 255;
                        if (pwm < 0) pwm = 0;
                        if (!((pwm == 255 && e > 0) || (pwm == 0 && e < 0))) {
                            integral = integral_candidate;
                        }
                        motor_set_speed(MOTOR_LEFT, pwm);
                        if (t >= (ciclos/2)) {
                            sq_error += e*e;
                        }
                        error_prev = e;
                        v_filt_prev = v_filt;
                        last_wake = xTaskGetTickCount();
                        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS((int)(periodo*1000)));
                    }
                    error += sqrtf(sq_error / ciclos);
                    motor_set_speed(MOTOR_LEFT, 0);
                    vTaskDelay(pdMS_TO_TICKS(300));
                }
                if (error < best_error) {
                    best_error = error;
                    dp[i] *= 1.05f;
                    printf("[Twiddle_L] ¡Mejora! Nuevo best_error=%.4f, dp[%d]=%.4f\n", best_error, i, dp[i]);
                } else {
                    p[i] = old_pi;
                    dp[i] *= 0.95f;
                    printf("[Twiddle_L] Sin mejora, restaurando p[%d]=%.3f, reduciendo dp[%d]=%.4f\n", i, p[i], i, dp[i]);
                }
            }
        }
        iter++;
    }
    printf("Twiddle PID Left: Kp=%.3f, Ki=%.3f, Kd=%.3f, error=%.3f\n", p[0], p[1], p[2], best_error);
    vTaskDelete(NULL);
}


// Twiddle autotune para el motor derecho
void task_tune_right_twiddle(void *params) {
    float p[3] = {234.0f, 92.64f, 23.0f}; // Kp, Ki, Kd iniciales (mejor medidos)
    float dp[3] = {23.4f, 9.264f, 2.3f}; // ~10% de cada parámetro
    float best_error = 1e9;
    float setpoints[] = {0.2f, 0.6f, 1.0f};
    int num_setpoints = sizeof(setpoints)/sizeof(float);
    const float periodo = 0.20f; // alineado con tacómetro
    const int ciclos = 24;
    const int warmup = 8;
    const float threshold = 0.1f; // umbral más estricto para detener Twiddle
    const float anti_windup_limit = 10.0f;
    const float alpha_vel = 0.3f; // filtro EMA para velocidad

    TickType_t last_wake;
    int max_iter = 80; // cantidad máxima de iteraciones
    int iter = 0;
    while ((dp[0] + dp[1] + dp[2] > threshold) && (iter < max_iter)) {
    printf("[Twiddle_R] Iteración num %d: dp_sum=%.4f, Kp=%.3f, Ki=%.3f, Kd=%.3f, best_error=%.3f\n", iter, dp[0]+dp[1]+dp[2], p[0], p[1], p[2], best_error);
    for (int i = 0; i < 3; i++) {
            float old_pi = p[i];
            p[i] += dp[i];
            float error = 0;
            for (int s = 0; s < num_setpoints; s++) {
                float setpoint = setpoints[s];
                int pwmbase = (setpoint == 0.6f) ? 210 : (setpoint == 1.0f) ? 240 : 180;
                float error_prev = 0, integral = 0, velocidad_right = 0, v_filt = 0, v_filt_prev = 0;
                for (int t = 0; t < warmup; t++) {
                    xQueuePeek(q_tacometro_right, &velocidad_right, pdMS_TO_TICKS((int)(periodo*1000)));
                    v_filt = alpha_vel * velocidad_right + (1 - alpha_vel) * v_filt_prev;
                    float e = setpoint - v_filt;
                    float derivada = -(v_filt - v_filt_prev) / periodo;
                    float integral_candidate = integral + e * periodo;
                    if (integral_candidate > anti_windup_limit) integral_candidate = anti_windup_limit;
                    if (integral_candidate < -anti_windup_limit) integral_candidate = -anti_windup_limit;
                    float pid_candidate = p[0]*e + p[1]*integral_candidate + p[2]*derivada;
                    int pwm_candidate = pwmbase + (int)pid_candidate;
                    int pwm = pwm_candidate;
                    if (pwm > 255) pwm = 255;
                    if (pwm < 0) pwm = 0;
                    if (!((pwm == 255 && e > 0) || (pwm == 0 && e < 0))) {
                        integral = integral_candidate;
                    }
                    motor_set_speed(MOTOR_RIGHT, pwm);
                    error_prev = e;
                    v_filt_prev = v_filt;
                    last_wake = xTaskGetTickCount();
                    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS((int)(periodo*1000)));
                }
                float sq_error = 0;
                for (int t = 0; t < ciclos; t++) {
                    xQueuePeek(q_tacometro_right, &velocidad_right, pdMS_TO_TICKS((int)(periodo*1000)));
                    v_filt = alpha_vel * velocidad_right + (1 - alpha_vel) * v_filt_prev;
                    float e = setpoint - v_filt;
                    float derivada = -(v_filt - v_filt_prev) / periodo;
                    float integral_candidate = integral + e * periodo;
                    if (integral_candidate > anti_windup_limit) integral_candidate = anti_windup_limit;
                    if (integral_candidate < -anti_windup_limit) integral_candidate = -anti_windup_limit;
                    float pid_candidate = p[0]*e + p[1]*integral_candidate + p[2]*derivada;
                    int pwm_candidate = pwmbase + (int)pid_candidate;
                    int pwm = pwm_candidate;
                    if (pwm > 255) pwm = 255;
                    if (pwm < 0) pwm = 0;
                    if (!((pwm == 255 && e > 0) || (pwm == 0 && e < 0))) {
                        integral = integral_candidate;
                    }
                    motor_set_speed(MOTOR_RIGHT, pwm);
                    if (t >= (ciclos/2)) {
                        sq_error += e*e;
                    }
                    error_prev = e;
                    v_filt_prev = v_filt;
                    last_wake = xTaskGetTickCount();
                    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS((int)(periodo*1000)));
                }
                error += sqrtf(sq_error / ciclos);
                motor_set_speed(MOTOR_RIGHT, 0);
                vTaskDelay(pdMS_TO_TICKS(300));
            }
            printf("[Twiddle_R] Error obtenido: %.4f (parámetro %d)\n", error, i);
            if (error < best_error) {
                best_error = error;
                dp[i] *= 1.05f;
                printf("[Twiddle_R] ¡Mejora! Nuevo best_error=%.4f, dp[%d]=%.4f\n", best_error, i, dp[i]);
            } else {
                p[i] = old_pi - dp[i];
                error = 0;
                for (int s = 0; s < num_setpoints; s++) {
                    float setpoint = setpoints[s];
                    int pwmbase = (setpoint == 0.6f) ? 210 : (setpoint == 1.0f) ? 240 : 180;
                    float error_prev = 0, integral = 0, velocidad_right = 0, v_filt = 0, v_filt_prev = 0;
                    for (int t = 0; t < warmup; t++) {
                        xQueuePeek(q_tacometro_right, &velocidad_right, pdMS_TO_TICKS((int)(periodo*1000)));
                        v_filt = alpha_vel * velocidad_right + (1 - alpha_vel) * v_filt_prev;
                        float e = setpoint - v_filt;
                        float derivada = -(v_filt - v_filt_prev) / periodo;
                        float integral_candidate = integral + e * periodo;
                        if (integral_candidate > anti_windup_limit) integral_candidate = anti_windup_limit;
                        if (integral_candidate < -anti_windup_limit) integral_candidate = -anti_windup_limit;
                        float pid_candidate = p[0]*e + p[1]*integral_candidate + p[2]*derivada;
                        int pwm_candidate = pwmbase + (int)pid_candidate;
                        int pwm = pwm_candidate;
                        if (pwm > 255) pwm = 255;
                        if (pwm < 0) pwm = 0;
                        if (!((pwm == 255 && e > 0) || (pwm == 0 && e < 0))) {
                            integral = integral_candidate;
                        }
                        motor_set_speed(MOTOR_RIGHT, pwm);
                        error_prev = e;
                        v_filt_prev = v_filt;
                        last_wake = xTaskGetTickCount();
                        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS((int)(periodo*1000)));
                    }
                    float sq_error = 0;
                    for (int t = 0; t < ciclos; t++) {
                        xQueuePeek(q_tacometro_right, &velocidad_right, pdMS_TO_TICKS((int)(periodo*1000)));
                        v_filt = alpha_vel * velocidad_right + (1 - alpha_vel) * v_filt_prev;
                        float e = setpoint - v_filt;
                        float derivada = -(v_filt - v_filt_prev) / periodo;
                        float integral_candidate = integral + e * periodo;
                        if (integral_candidate > anti_windup_limit) integral_candidate = anti_windup_limit;
                        if (integral_candidate < -anti_windup_limit) integral_candidate = -anti_windup_limit;
                        float pid_candidate = p[0]*e + p[1]*integral_candidate + p[2]*derivada;
                        int pwm_candidate = pwmbase + (int)pid_candidate;
                        int pwm = pwm_candidate;
                        if (pwm > 255) pwm = 255;
                        if (pwm < 0) pwm = 0;
                        if (!((pwm == 255 && e > 0) || (pwm == 0 && e < 0))) {
                            integral = integral_candidate;
                        }
                        motor_set_speed(MOTOR_RIGHT, pwm);
                        if (t >= (ciclos/2)) {
                            sq_error += e*e;
                        }
                        error_prev = e;
                        v_filt_prev = v_filt;
                        last_wake = xTaskGetTickCount();
                        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS((int)(periodo*1000)));
                    }
                    error += sqrtf(sq_error / ciclos);
                    motor_set_speed(MOTOR_RIGHT, 0);
                    vTaskDelay(pdMS_TO_TICKS(300));
                }
                if (error < best_error) {
                    best_error = error;
                    dp[i] *= 1.05f;
                    printf("[Twiddle_R] ¡Mejora! Nuevo best_error=%.4f, dp[%d]=%.4f\n", best_error, i, dp[i]);
                } else {
                    p[i] = old_pi;
                    dp[i] *= 0.95f;
                    printf("[Twiddle_R] Sin mejora, restaurando p[%d]=%.3f, reduciendo dp[%d]=%.4f\n", i, p[i], i, dp[i]);
                }
            }
        }
        
        iter++;
    }
    printf("Twiddle PID Right: Kp=%.3f, Ki=%.3f, Kd=%.3f, error=%.3f\n", p[0], p[1], p[2], best_error);
    vTaskDelete(NULL);
}



int main()
{
    stdio_init_all();
    configurar_gpio();
    init_uart();
    motor_init();
    motor_set_direction(MOTOR_LEFT,true);  // Sentido "hacia adelante"
    motor_set_direction(MOTOR_RIGHT,true);

    // Leer PID guardados en EEPROM (LEFT)
    char pidl_buf[48] = {0};
    float Kp_left = 6.0f, Ki_left = 0.0f, Kd_left = 0.0f;
    if (at24c32_read(i2c1, 0x100, (uint8_t*)pidl_buf, sizeof(pidl_buf))) {
        if (strncmp(pidl_buf, "PIDL:", 5) == 0) {
            sscanf(pidl_buf+5, "%f,%f,%f", &Kp_left, &Ki_left, &Kd_left);
            printf("[EEPROM] PID LEFT: Kp=%.3f, Ki=%.3f, Kd=%.3f\n", Kp_left, Ki_left, Kd_left);
        }
    }
    // Leer PID guardados en EEPROM (RIGHT)
    char pidr_buf[48] = {0};
    float Kp_right = 4.0f, Ki_right = 0.0f, Kd_right = 0.0f;
    if (at24c32_read(i2c1, 0x140, (uint8_t*)pidr_buf, sizeof(pidr_buf))) {
        if (strncmp(pidr_buf, "PIDR:", 5) == 0) {
            sscanf(pidr_buf+5, "%f,%f,%f", &Kp_right, &Ki_right, &Kd_right);
            printf("[EEPROM] PID RIGHT: Kp=%.3f, Ki=%.3f, Kd=%.3f\n", Kp_right, Ki_right, Kd_right);
        }
    }

    //INICIALIZACION DE LAS COLAS 
    q_codigo                = xQueueCreate(100 , sizeof(char[MAX_INPUT + 1])  );

    q_tacometro_left        = xQueueCreate(1 , sizeof(float) );
    q_tacometro_right       = xQueueCreate(1 , sizeof(float) );
    q_uart                  = xQueueCreate(1 , sizeof(char[MAX_INPUT + 1])  );
    q_bluetooth_chars       = xQueueCreate(MAX_INPUT, sizeof(char));

    //INICIALIZACION DE LAS TAREAS 
    //xTaskCreate(task_matrix     , "Matrix"      , 2 * configMINIMAL_STACK_SIZE  , NULL, 4, NULL);
    
    //xTaskCreate(task_lcd, "LCD",  configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    //xTaskCreate(task_PID_left       , "PID Left"        , 2 * configMINIMAL_STACK_SIZE , NULL, 2, NULL);
    //xTaskCreate(task_PID_right      , "PID Right"       , 2 * configMINIMAL_STACK_SIZE , NULL, 2, NULL);
    //xTaskCreate(task_motor          , "Motor"           , configMINIMAL_STACK_SIZE *3  , NULL, 4, NULL);
    xTaskCreate(task_tacometro_left , "Tacometro Left"  , 2 * configMINIMAL_STACK_SIZE , NULL, 2, NULL);
    xTaskCreate(task_tacometro_right, "Tacometro Right" , 2 * configMINIMAL_STACK_SIZE , NULL, 2, NULL);
    xTaskCreate(task_bluetooth      , "Bluetooth"       , 2 * configMINIMAL_STACK_SIZE , NULL, 5, NULL);
    // xTaskCreate(task_recto         , "Recto"          , configMINIMAL_STACK_SIZE , NULL, 3, NULL);

    vTaskStartScheduler();
    while (true) {
        // ...existing code...
    }
}


