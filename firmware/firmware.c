#include <stdio.h>
#include "pico/stdlib.h"
#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "hardware/pwm.h"
#include "string.h"
#include "lcd.h"
<<<<<<< HEAD
#include "hardware/i2c.h"
#include "rtc.h"
=======
#include "semphr.h"
>>>>>>> 2d86562 (Coordinacion de tareas funcionando BIEN POSTA FINAL FINAL AHORA SI)


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
#define PWM_FREQ 15000
// Cambiar velocidad (0 a 255)
#define MOTOR_LEFT      1
#define MOTOR_RIGHT     2
//Contastante de conversion de velocidad a PWN
#define PENDIENTE   1.2045f
#define ORDENADA    127.43f
//const float  PENDIENTE = 72.36 ;
//const float ORDENADA = 141.38 ;
#define PI 3.14f
//Modos 
//SEGUN EL MODO, PONEMOS LA VELOCIDAD
#define VEL_01  97.0f
#define VEL_02  76.5f
#define VEL_03  57.9f
#define CURVA_01 1.5f
#define CURVA_02 1.0f
#define CURVA_03 0.5f
#define RECTA 0.0f

//Cosntantes del auto
#define RADIUS 0.134f //radio entre el centro de las ruedas en m
#define R_WHEEL 0.0325f
//Configuración del teclado matricial
#define FILAS 4
#define COLUMNAS 4
#define MAX_INPUT 4

//Defino lo necesario para hacer andar la UART
#define UART_ID uart0
#define UART_TX_PIN 16
#define UART_RX_PIN 17
#define BAUD_RATE 115200

#define PID_ACTIVADO 0
#define PID_DESACTIVADO 0


//Defino mi cola
//QueueHandle_t q_matrix ;
QueueHandle_t q_tacometro_left ;
QueueHandle_t q_tacometro_right ;
QueueHandle_t q_uart ;
QueueHandle_t q_vel_impuesta_left ;
QueueHandle_t q_vel_impuesta_right ;
QueueHandle_t q_bluetooth_chars;
QueueHandle_t q_codigo;

SemaphoreHandle_t xsemPID_left_run ;
SemaphoreHandle_t xsemPID_right_run ;
SemaphoreHandle_t xsemPID_left_stop ;
SemaphoreHandle_t xsemPID_right_stop ;

typedef struct {
    float v_inner;
    float v_outer;
} VelData_t;

<<<<<<< HEAD
QueueHandle_t q_codigo;
QueueHandle_t q_vel_deseada;
=======
const uint FILA_PINS[FILAS] = {6, 7, 8, 9};
const uint COLUMNA_PINS[COLUMNAS] = {10, 11, 12, 13};
const char teclas[FILAS][COLUMNAS] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
};


//TaskHandle_t handlePID_left ;
//TaskHandle_t handlePID_right ;


void on_uart_rx(void);
>>>>>>> 2d86562 (Coordinacion de tareas funcionando BIEN POSTA FINAL FINAL AHORA SI)

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
    //Inicializo el display
    //lcd_init(i2c1, 0x27 );
    //lcd_clear();
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

<<<<<<< HEAD
=======
        vTaskDelay(pdMS_TO_TICKS(30));  // estabilización rápida

        for (int c = 0; c < COLUMNAS; c++) {
            if (gpio_get(COLUMNA_PINS[c]) == 0) {
                return teclas[f][c];
            }
        }
    }
    return 0;
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


>>>>>>> 2d86562 (Coordinacion de tareas funcionando BIEN POSTA FINAL FINAL AHORA SI)
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


void task_motor(void *params) {
    motor_init();
    motor_set_direction(MOTOR_LEFT,true);  // Sentido "hacia adelante"
    motor_set_direction(MOTOR_RIGHT,true);
    uint16_t speed_left = 255;
    uint16_t speed_right = 251;
    unsigned int sentido = 0;

    char buffer[MAX_INPUT + 1] = {0};
    float linear_speed = 0;
    float radius = 0;

    VelData_t diff_vel ;
    uint16_t vel_pwm_inner = 0 ;
    uint16_t vel_pwm_outer = 0 ;
    float aux_vel = 0 ;

    int set_time ;
    TickType_t start_tick = xTaskGetTickCount();  // tiempo de referencia
    TickType_t aux_tick = 0 ;

    motor_set_speed(MOTOR_LEFT,0);
    motor_set_speed(MOTOR_RIGHT,0);

    while (1) {
        xQueueReceive(q_uart, &buffer, portMAX_DELAY );
            
        if      (buffer[0]=='1'){linear_speed=VEL_01;}
        else if (buffer[0]=='2'){linear_speed=VEL_02;}
        else if (buffer[0]=='3'){linear_speed=VEL_03;}
            
        if      (buffer[1]=='A'){radius=CURVA_01;}
        else if (buffer[1]=='B'){radius=CURVA_02;}
        else if (buffer[1]=='C'){radius=CURVA_03;}
        else if (buffer[1]=='D'){radius=RECTA;}


        if (buffer[1] == 'A' || buffer[1] == 'B' || buffer[1] == 'C' ){
            //PASA DE ACTIVACIONES A VUELTAS CON (lineer_speed/21)
            //el 2*PI lo pasa a velocidad angular y el R_WHEEL a velocidad lineal
            start_tick = xTaskGetTickCount() ;
            aux_tick = start_tick ;
            printf("Ticks INICIALES: %u\n", (unsigned int)start_tick);
            
            aux_vel = (linear_speed / 21) * 2 * PI * R_WHEEL;
            set_time =  1000 * 2 * PI * radius / aux_vel;

            diff_vel.v_outer     = linear_speed * (1 + RADIUS / ( 2 * radius ) ) ;
            diff_vel.v_inner    = linear_speed * (1 - RADIUS / ( 2 * radius ) ) ;

            vel_pwm_inner = diff_vel.v_inner * PENDIENTE + ORDENADA;
            vel_pwm_outer = diff_vel.v_outer * PENDIENTE + ORDENADA;

            printf("Velocidad: %.2f | Radio: %.2f | Inner: %.2f | Outer: %.2f | Time: %d \n",
                linear_speed, radius , diff_vel.v_outer , diff_vel.v_inner , set_time);

            xQueueOverwrite(q_vel_impuesta_left , &vel_pwm_outer );
            xQueueOverwrite(q_vel_impuesta_right , &vel_pwm_outer  );

            vTaskDelay(pdMS_TO_TICKS(set_time/4 ));

            // %%%%%%%%%%%%%%%%%%%% PRIMER SET %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            xQueueOverwrite(q_vel_impuesta_left , &vel_pwm_outer  );
            xQueueOverwrite(q_vel_impuesta_right , &vel_pwm_inner  ); 

            start_tick = xTaskGetTickCount() - aux_tick ;
            printf("Ticks PRIMER: %u\n", (unsigned int)start_tick);

            vTaskDelay(pdMS_TO_TICKS(set_time/4 ));

            // %%%%%%%%%%%%%%%%%%%% SEGUNDO SET %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            xQueueOverwrite(q_vel_impuesta_left , &vel_pwm_inner  );
            xQueueOverwrite(q_vel_impuesta_right , &vel_pwm_outer  );

            start_tick = xTaskGetTickCount() - aux_tick ;
            printf("Ticks SEGUNDO: %u\n", (unsigned int)start_tick);

            vTaskDelay(pdMS_TO_TICKS(set_time/2));
            
            // %%%%%%%%%%%%%%%%%%%% TERCER SET %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            xQueueOverwrite(q_vel_impuesta_left , &vel_pwm_outer  );
            xQueueOverwrite(q_vel_impuesta_right , &vel_pwm_inner );

            start_tick = xTaskGetTickCount() - aux_tick ;
            printf("Ticks TERCER: %u\n", (unsigned int)start_tick);

            vTaskDelay(pdMS_TO_TICKS(set_time/4));

            // %%%%%%%%%%%%%%%%%%%% CUARTO SET %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        // TRAYECTORIA RECTA
        }else if (buffer[1]=='D'){
            diff_vel.v_inner     = linear_speed  ;
            diff_vel.v_outer     = linear_speed  ;

            vel_pwm_inner = diff_vel.v_inner * PENDIENTE + ORDENADA;
            vel_pwm_outer = diff_vel.v_outer * PENDIENTE + ORDENADA;

            xQueueOverwrite(q_vel_impuesta_left , &vel_pwm_outer  );
            xQueueOverwrite(q_vel_impuesta_right , &vel_pwm_inner  );

            vTaskDelay(pdMS_TO_TICKS(5000));
        }
            

        printf("Velocidad: %.2f | Radio: %.2f | Inner: %.2f | Outer: %.2f | Time: %d \n",
            linear_speed, radius , diff_vel.v_outer , diff_vel.v_inner , set_time);

        printf("vel_pwm_inner = %hu, vel_pwm_outer = %hu\n", vel_pwm_inner, vel_pwm_outer); 

        xQueueOverwrite(q_vel_impuesta_left ,  0 );
        xQueueOverwrite(q_vel_impuesta_right , 0  );

        memset(buffer, 0, sizeof(buffer));
        vTaskDelay(pdMS_TO_TICKS(1000));  // Reemplazo de vTaskDelay
        
    }
}
void task_PID_right(void *params) {
    uint16_t setpoint_raw = 0;
    float setpoint = 0.0;
    float medida = 0.0;
    int medida_aux = 0;

    float error = 0, error_anterior = 0;
    float integral = 0;
    float salida = 0;
    uint32_t cmd = PID_DESACTIVADO;  // recibirá las notificaciones

    // Constantes iniciales (ajustables)
    const float dt = 0.05; // 50 ms -> 20 Hz
    const float Kp = 0.75;    // suficiente para superar el umbral de 200
    const float Ki = 0.01;   // pequeña integración para que alcance el setpoint sin oscilar
    const float Kd = 0.05;       // derivativa (pequeña para suavizar)
    const float velocidad = 0.5; 

    float derivada = 0;
    const float PWM_MIN = 140.0;  // mínimo necesario para que el motor arranque

    while (1) {
        // Recibir nueva referencia
        //if (xQueueReceive(q_vel_impuesta_right, &setpoint_raw, 0) == pdTRUE) {
        //xTaskNotifyWait(0, 0, &cmd, portMAX_DELAY); 
        //if (cmd == PID_ACTIVADO){

            //printf ("PID DERECHO ACTIVADO\n");

                xQueuePeek(q_vel_impuesta_right, &setpoint_raw, 0);
                xQueueReceive(q_tacometro_right, &medida_aux, 0);
                //if (setpoint_raw != 0 ){
                setpoint = (float)setpoint_raw;
                
                if (setpoint_raw != 0 ){
                    medida = medida_aux * PENDIENTE + ORDENADA;
                    
                }else if (setpoint_raw == 0 ){
                    medida = 0 ;
                }
                    error = setpoint - medida;
                    integral += error;   // integración con tiempo
                    derivada = (error - error_anterior) ;
                    salida = setpoint + (Kp * error + Ki * integral + Kd * derivada)    *   velocidad;
                    //printf("RIGHT MOTOR \n Setpoint: %.2f | Medida: %.2f | Error: %.2f | Salida: %.2f| Integral: %.2f | Derivada: %.2f \n",
                        //setpoint, medida, error, salida,integral,derivada);

                    // Saturación de PWM
                    if (salida > 0 && salida < PWM_MIN) salida = PWM_MIN;
                    if (salida > 255) salida = 255;
                    
                    uint16_t salida_aux = (uint16_t)salida; 
                    motor_set_speed(MOTOR_RIGHT,salida);
                    //xQueueSend(q_vel_impuesta, &salida_aux, portMAX_DELAY);

                    error_anterior = error;

                //} else if  (setpoint_raw == 0 ){
                    //motor_set_speed(MOTOR_RIGHT,0);
                //}
            
        //}
        vTaskDelay(pdMS_TO_TICKS(250));  // 50 ms
    
    }
}


void task_PID_left(void *params) {
    uint16_t setpoint_raw = 0;
    float setpoint = 0.0;
    float medida = 0.0;
    int medida_aux = 0;

    float error = 0, error_anterior = 0;
    float integral = 0;
    float salida = 0;

    // Constantes iniciales (ajustables)
    const float dt = 0.05; // 50 ms -> 20 Hz
    const float Kp = 0.75;    // suficiente para superar el umbral de 200
    const float Ki = 0.01;   // pequeña integración para que alcance el setpoint sin oscilar
    const float Kd = 0.05;       // derivativa (pequeña para suavizar)
    const float velocidad = 0.5; 

    float derivada = 0;
    const float PWM_MIN = 140.0;  // mínimo necesario para que el motor arranque

    while (1) {

        // Recibir nueva referencia
        
            xQueuePeek(q_vel_impuesta_left, &setpoint_raw, 0) ;
            setpoint = (float)setpoint_raw;
            xQueueReceive(q_tacometro_left, &medida_aux, portMAX_DELAY);
            if (setpoint_raw != 0 ){
                medida = medida_aux * PENDIENTE + ORDENADA;
                
            }else if (setpoint_raw == 0 ){
                medida = 0 ;
            }
                error = setpoint - medida;
                integral += error;   // integración con tiempo
                derivada = (error - error_anterior) ;
                salida = setpoint + (Kp * error + Ki * integral + Kd * derivada)    *   velocidad;
                //printf("LEFT MOTOR \n Setpoint: %.2f | Medida: %.2f | Error: %.2f | Salida: %.2f| Integral: %.2f | Derivada: %.2f \n",
                    //setpoint, medida, error, salida,integral,derivada);

                // Saturación de PWM
                if (salida > 0 && salida < PWM_MIN) salida = PWM_MIN;
                if (salida > 255) salida = 255;
                
                uint16_t salida_aux = (uint16_t)salida; 
                motor_set_speed(MOTOR_LEFT,salida);
                //xQueueSend(q_vel_impuesta, &salida_aux, portMAX_DELAY);

                error_anterior = error;
    
            
        vTaskDelay(pdMS_TO_TICKS(250));  // 50 ms
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

<<<<<<< HEAD
=======

>>>>>>> 2d86562 (Coordinacion de tareas funcionando BIEN POSTA FINAL FINAL AHORA SI)
void task_bluetooth(void *params) {
    char buffer[4] = {0}; // Donde se guardará el recorrido preseteado. Ej "1A#"
    int cod_num = 0;
    int index = 2;
    int cod_letter = 0 ;
    char c;

    while (true) {
<<<<<<< HEAD
        if (uart_is_readable(UART_ID)) {
            char c = uart_getc(UART_ID);

            if (c != '#') {
=======
        if (xQueueReceive(q_bluetooth_chars, &c, portMAX_DELAY)) {
           if (c != '#') {
                //
>>>>>>> 2d86562 (Coordinacion de tareas funcionando BIEN POSTA FINAL FINAL AHORA SI)
                if (c == '1' || c == '2' || c == '3') {
                    buffer[0] = c;
                    cod_num = 1;
                } else if (c == 'A' || c == 'B' || c == 'C'|| c == 'D') {
                    buffer[1] = c;
                    cod_letter = 1 ;
                } else if (c == 'R' || c == 'M') {
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
                }
                // Si la cadena es M#
                else if (index == 1 && buffer[0] == 'M') {
                    uint8_t mem_buf[32];
                    if (at24c32_read(i2c1, 0, mem_buf, sizeof(mem_buf))) {
                        printf("\nEEPROM[0-31]:");
                        for (int i = 0; i < 32; ++i) {
                            if (mem_buf[i] == '\0') break;
                            char c = (mem_buf[i] >= 32 && mem_buf[i] <= 126) ? mem_buf[i] : '.';
                            putchar(c);
                        }
                        printf("\n");
                    } else {
                        printf("Error leyendo EEPROM\n");
                    }
                }
                // Verifica formato de comando normal
                else if (cod_num == 1 && cod_letter==1) {
                    buffer[2] = '\0';
                    printf("Ingresado: %s\n", buffer);
                    //index = 2;
                    xQueueSend(q_uart , buffer , portMAX_DELAY );
                    memset(buffer, 0, sizeof(buffer));
                    cod_num = 0; 
                    cod_letter = 0;
                }
                // Solo imprime 'Formato inválido' si no es R# ni M#
                else if (!(index == 1 && (buffer[0] == 'R' || buffer[0] == 'M'))) {
                    printf("Formato inválido\n");
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

    TickType_t start_tick = xTaskGetTickCount();  // tiempo de referencia
    const TickType_t periodo = pdMS_TO_TICKS(1000);  // 1 segundo



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
            vueltas = counter / 21.0f;
            //printf("Activaciones por segundo DER: %d\n", counter);
            //printf("Vueltas por segundo: %.2f\n", vueltas);
            xQueueOverwrite(q_tacometro_right , &counter  );
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

    TickType_t start_tick = xTaskGetTickCount();  // tiempo de referencia
    const TickType_t periodo = pdMS_TO_TICKS(1000);  // 1 segundo

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
            vueltas = counter / 21.0f;
            //printf("Activaciones por segundo IZQ: %d\n", counter);
            //printf("Vueltas por segundo: %.2f\n", vueltas);
            xQueueOverwrite(q_tacometro_left , &counter );
            counter = 0;
            vueltas = 0;
            start_tick = xTaskGetTickCount();  // reinicio del conteo
        }

        vTaskDelay(pdMS_TO_TICKS(1));  // respiro para evitar uso 100% CPU
    }
}

int main()
{
    stdio_init_all();
    configurar_gpio();
    init_uart();

    //INICIALIZACION DE LAS COLAS 
    q_codigo                = xQueueCreate(100 , sizeof(char[MAX_INPUT + 1])  );
    q_vel_impuesta_right    = xQueueCreate(1 , sizeof(uint16_t)  );
    q_vel_impuesta_left     = xQueueCreate(1 , sizeof(uint16_t)  );
    q_tacometro_left        = xQueueCreate(1 , sizeof(int) );
    q_tacometro_right       = xQueueCreate(1 , sizeof(int) );
    q_uart                  = xQueueCreate(1 , sizeof(char[MAX_INPUT + 1])  );
    q_bluetooth_chars       = xQueueCreate(MAX_INPUT, sizeof(char));

    xsemPID_left_run =  xSemaphoreCreateBinary();
    xsemPID_right_run =  xSemaphoreCreateBinary();
    xsemPID_left_stop =  xSemaphoreCreateBinary();
    xsemPID_right_stop =  xSemaphoreCreateBinary();

    //INICIALIZACION DE LAS TAREAS 
    //xTaskCreate(task_matrix     , "Matrix"      , 2 * configMINIMAL_STACK_SIZE  , NULL, 4, NULL);
<<<<<<< HEAD
   // xTaskCreate(task_motor      , "Motor"       , configMINIMAL_STACK_SIZE *3   , NULL, 3, NULL);
    //xTaskCreate(task_lcd, "LCD",  configMINIMAL_STACK_SIZE, NULL, 1, NULL);
   // xTaskCreate(Tacometro, "Tacometro", 2 * configMINIMAL_STACK_SIZE , NULL, 2, NULL);
    xTaskCreate(task_bluetooth , "Bluetooth"   , 2 * configMINIMAL_STACK_SIZE  , NULL, 5, NULL);
=======
    
    //xTaskCreate(task_lcd, "LCD",  configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(task_PID_left       , "PID Left"        , 2 * configMINIMAL_STACK_SIZE , NULL, 2, NULL);
    xTaskCreate(task_PID_right      , "PID Right"       , 2 * configMINIMAL_STACK_SIZE , NULL, 2, NULL);
    xTaskCreate(task_motor          , "Motor"           , configMINIMAL_STACK_SIZE *3  , NULL, 4, NULL);
    xTaskCreate(task_tacometro_left , "Tacometro Left"  , 2 * configMINIMAL_STACK_SIZE , NULL, 2, NULL);
    xTaskCreate(task_tacometro_right, "Tacometro Right" , 2 * configMINIMAL_STACK_SIZE , NULL, 2, NULL);
    xTaskCreate(task_bluetooth      , "Bluetooth"       , 2 * configMINIMAL_STACK_SIZE , NULL, 5, NULL);
>>>>>>> 2d86562 (Coordinacion de tareas funcionando BIEN POSTA FINAL FINAL AHORA SI)

    vTaskStartScheduler();
    while (true) {
        
    }
}


