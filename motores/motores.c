#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "hardware/pwm.h"
#include "string.h"

// Pines para control de dirección y PWM
#define MOTOR_M2_IN1 18
#define MOTOR_M2_IN2 19
#define MOTOR_M1_IN1 20
#define MOTOR_M1_IN2 21
#define MOTOR_M2_PWM 22  // GPIO con soporte PWM
#define MOTOR_M1_PWM 26  // GPIO con soporte PWM
//MACROS PARA EL TACOMETRO
#define IN_PIN_TACOMETRO 28
const float  PENDIENTE = 2.29 ;
const float ORDENADA = 174.79 ;
#define VENTANA_MS 100
// Configuración de la frecuencia PWM (Hz)
#define PWM_FREQ 15000
//MACROS DEL TECLADO
#define FILAS 4
#define COLUMNAS 4
#define MAX_INPUT 32

const uint FILA_PINS[FILAS] = {6, 7, 8, 9};
const uint COLUMNA_PINS[COLUMNAS] = {10, 11, 12, 13};
const char teclas[FILAS][COLUMNAS] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
};

QueueHandle_t q_vel_deseada;
QueueHandle_t q_vel_medida;
QueueHandle_t q_vel_impuesta;

void configurar_gpio() {
    //INICIALIZACION DEL TACOMETRO
    gpio_init(IN_PIN_TACOMETRO);
    gpio_set_dir(IN_PIN_TACOMETRO, GPIO_IN);
    gpio_pull_down(IN_PIN_TACOMETRO);

    //INICIALIZACION DEL TECLADO MATRICIAL
    for (int i = 0; i < FILAS; i++) {
        gpio_init(FILA_PINS[i]);
        gpio_set_dir(FILA_PINS[i], GPIO_OUT);
        gpio_put(FILA_PINS[i], 1);
    }
    for (int i = 0; i < COLUMNAS; i++) {
        gpio_init(COLUMNA_PINS[i]);
        gpio_set_dir(COLUMNA_PINS[i], GPIO_IN);
        gpio_pull_up(COLUMNA_PINS[i]);
    }
    printf("Listo para leer el teclado 4x4...\n");

    //INICIALIZACION DEL MOTOR
    //motor_init();
    //motor_set_direction(true);  
}

char escanear_teclado() {
    for (int f = 0; f < FILAS; f++) {
        for (int i = 0; i < FILAS; i++)
            gpio_put(FILA_PINS[i], i == f ? 0 : 1);

        vTaskDelay(pdMS_TO_TICKS(5));  // estabilización rápida

        for (int c = 0; c < COLUMNAS; c++) {
            if (gpio_get(COLUMNA_PINS[c]) == 0) {
                return teclas[f][c];
            }
        }
    }
    return 0;
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
    gpio_set_function(MOTOR_M1_PWM, GPIO_FUNC_PWM);
    uint slice1 = pwm_gpio_to_slice_num(MOTOR_M1_PWM);
    pwm_set_clkdiv(slice1, divider);
    pwm_set_wrap(slice1, 255);
    pwm_set_chan_level(slice1, pwm_gpio_to_channel(MOTOR_M1_PWM), 0);
    pwm_set_enabled(slice1, true);

    // PWM Motor 2
    gpio_set_function(MOTOR_M2_PWM, GPIO_FUNC_PWM);
    uint slice2 = pwm_gpio_to_slice_num(MOTOR_M2_PWM);
    pwm_set_clkdiv(slice2, divider);
    pwm_set_wrap(slice2, 255);
    pwm_set_chan_level(slice2, pwm_gpio_to_channel(MOTOR_M2_PWM), 0);
    pwm_set_enabled(slice2, true);
}

// Cambiar dirección del motor
void motor_set_direction(uint8_t motor, bool forward) {
    if (motor == 1) {
        gpio_put(MOTOR_M1_IN1, forward ? 1 : 0);
        gpio_put(MOTOR_M1_IN2, forward ? 0 : 1);
    } else if (motor == 2) {
        gpio_put(MOTOR_M2_IN1, forward ? 1 : 0);
        gpio_put(MOTOR_M2_IN2, forward ? 0 : 1);
    }
}

// Cambiar velocidad (0 a 255)
void motor_set_speed(uint8_t speed) {
    uint slice1 = pwm_gpio_to_slice_num(MOTOR_M1_PWM);
    uint slice2 = pwm_gpio_to_slice_num(MOTOR_M2_PWM);

    pwm_set_chan_level(slice1, pwm_gpio_to_channel(MOTOR_M1_PWM), speed);
    pwm_set_chan_level(slice2, pwm_gpio_to_channel(MOTOR_M2_PWM), speed);
}



void task_motor(void *params) {
    motor_init();
    motor_set_direction(1,false);  // Sentido "hacia adelante"
    motor_set_direction(2,false);
    uint16_t speed = 0;
    //unsigned int sentido = 0;

    while (1) {
        //motor_set_speed(speed);
        
        if (xQueueReceive(q_vel_impuesta, &speed, portMAX_DELAY)) {

            printf("Nueva velocidad recibida: %u\n", speed);

            motor_set_speed(speed);
            vTaskDelay(pdMS_TO_TICKS(250));
        }
            
           //speed = 0;
           //vTaskDelay(pdMS_TO_TICKS(100));
        
    }
}

void task_matrix(void *params) {
    char buffer[MAX_INPUT + 1] = {0};
    int index = 0;
    char tecla_anterior = 0;
    uint16_t speed = 0;
    long temp = 0 ;

    while (true) {
        char tecla_actual = escanear_teclado();

        if (tecla_actual != 0 && tecla_actual != tecla_anterior) {
            printf("Tecla presionada: %c\n", tecla_actual);

            if (index < MAX_INPUT && tecla_actual != '#') {
                buffer[index++] = tecla_actual;
                
            }
            if (tecla_actual == '*'){
                index = 0;
                memset(buffer, 0, sizeof(buffer));
            }
            if (tecla_actual == '#') {
                buffer[index] = '\0';
                printf("Ingresado: %s\n", buffer);
                index = 0;
                // Convertir a uint16_t
                char *endptr;
                temp = strtol(buffer, &endptr, 10);
                if (endptr == buffer || temp < 0 || temp > 65535) {
                    printf("Valor invalido\n");
                    continue;
                }
                if (temp > 255) {
                    temp = 255;
                }
                
                speed = (uint16_t)temp;
                
            }
        }
        xQueueOverwrite(q_vel_deseada, &speed);
        tecla_anterior = tecla_actual;

        vTaskDelay(pdMS_TO_TICKS(50));  // evita múltiples lecturas rápidas
    }
}

void task_control(void *params) {
    uint16_t setpoint_raw = 0;
    float setpoint = 0.0;
    float medida = 0.0;

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
    const float PWM_MIN = 200.0;  // mínimo necesario para que el motor arranque

    while (1) {
        // Recibir nueva referencia
        if (xQueueReceive(q_vel_deseada, &setpoint_raw, 0) == pdPASS) {
            setpoint = (float)setpoint_raw;
        }
        if (xQueueReceive(q_vel_medida, &medida, portMAX_DELAY)) {

            error = setpoint - medida;
            integral += error;   // integración con tiempo
            derivada = (error - error_anterior) ;
            salida = setpoint + (Kp * error + Ki * integral + Kd * derivada)    *   velocidad;
            printf("Setpoint: %.2f | Medida: %.2f | Error: %.2f | Salida: %.2f| Integral: %.2f | Derivada: %.2f \n",
                setpoint, medida, error, salida,integral,derivada);

            // Saturación de PWM
            if (salida > 0 && salida < PWM_MIN) salida = PWM_MIN;
            if (salida > 255) salida = 255;
            
            uint16_t salida_aux = (uint16_t)salida; 
            xQueueSend(q_vel_impuesta, &salida_aux, portMAX_DELAY);

            error_anterior = error;
        }

        vTaskDelay(pdMS_TO_TICKS(50));  // 50 ms
    }
}



void task_tacometro(void *params) {
    int counter = 0;
    float vueltas = 0;
    int estado_anterior = gpio_get(IN_PIN_TACOMETRO);
    char str[16];
    float vel_pwm = 0;

    TickType_t start_tick = xTaskGetTickCount();  // tiempo de referencia
    const TickType_t periodo = pdMS_TO_TICKS(1000);  // 1 segundo

    while (true) {
        int estado_actual = gpio_get(IN_PIN_TACOMETRO);

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
            
            if (counter != 0){
                vel_pwm = counter * PENDIENTE + ORDENADA;
                //printf("VEL PWM MEDIDA: %.2f\n", vel_pwm);
                //printf("Vueltas por segundo: %.2f\n", vueltas);
                //printf("Velocidad PWM: %.2f\n", vel_pwm);
                //xQueueSend(q_tacometro , &vueltas , portMAX_DELAY );
                counter = 0;
                vueltas = 0;
                xQueueOverwrite(q_vel_medida , &vel_pwm  );
            }
            start_tick = xTaskGetTickCount();  // reinicio del conteo
            
        }

        vTaskDelay(pdMS_TO_TICKS(1));  // respiro para evitar uso 100% CPU
    }
}

int main() {
    stdio_init_all();
    configurar_gpio();
    
    //INICIALIZACION DE LAS COLAS 
    q_vel_deseada   = xQueueCreate(1    , sizeof(uint16_t)  );
    q_vel_medida    = xQueueCreate(100  , sizeof(float)     );
    q_vel_impuesta  = xQueueCreate(1    , sizeof(uint16_t)  );
    //INICIALIZACION DE LAS TAREAS
    xTaskCreate(task_matrix     , "Matrix"      , 4 * configMINIMAL_STACK_SIZE  , NULL, 2, NULL);
    xTaskCreate(task_motor      , "Motor"       , configMINIMAL_STACK_SIZE *2   , NULL, 1, NULL);
    xTaskCreate(task_tacometro  , "Tacometro"   , configMINIMAL_STACK_SIZE + 100, NULL, 2, NULL);
    xTaskCreate(task_control    , "Control"     , configMINIMAL_STACK_SIZE + 100, NULL, 3, NULL);

    vTaskStartScheduler();

    while (1) { } 
}
