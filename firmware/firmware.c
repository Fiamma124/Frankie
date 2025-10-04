#include <stdio.h>
#include "pico/stdlib.h"
#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "hardware/pwm.h"
#include "string.h"
#include "lcd.h"

// MACROS
// Eleccion de GPIO para SDA y SCL para el LCD
#define SDA_GPIO    14
#define SCL_GPIO    15
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
#define CURVA_01 0.3f
#define CURVA_02 0.4f
#define CURVA_03 0.5f
#define RECTA 0.0f

//Cosntantes del auto
#define RADIUS 0.134f //radio entre el centro de las ruedas en m
#define R_WHEEL 0.0325f
//Configuración del teclado matricial
#define FILAS 4
#define COLUMNAS 4
#define MAX_INPUT 4

//Defino mi cola
//QueueHandle_t q_matrix ;
QueueHandle_t q_tacometro ;

typedef struct {
    float v_right;
    float v_left;
} VelData_t;

const uint FILA_PINS[FILAS] = {6, 7, 8, 9};
const uint COLUMNA_PINS[COLUMNAS] = {10, 11, 12, 13};
const char teclas[FILAS][COLUMNAS] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
};

QueueHandle_t q_codigo;
QueueHandle_t q_vel_deseada;

void configurar_gpio() {
    //INICIALIZACION DE LOS TACOMETROS
    gpio_init(IN_PIN_TACOMETRO_LEFT);
    gpio_set_dir(IN_PIN_TACOMETRO_LEFT, GPIO_IN);
    gpio_pull_down(IN_PIN_TACOMETRO_LEFT);

    gpio_init(IN_PIN_TACOMETRO_RIGHT);
    gpio_set_dir(IN_PIN_TACOMETRO_RIGHT, GPIO_IN);
    gpio_pull_down(IN_PIN_TACOMETRO_RIGHT);

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

    //CONFIUGRACION DEL DISPLAY
    i2c_init(i2c1, 100000);
    //Seteo la funcion
    gpio_set_function(SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(SCL_GPIO, GPIO_FUNC_I2C);
    // Habilito pull-ups
    gpio_pull_up(SDA_GPIO);
    gpio_pull_up(SCL_GPIO);
    //Inicializo el display
    lcd_init(i2c1, 0x27 );
    lcd_clear();
}
char escanear_teclado() {
    for (int f = 0; f < FILAS; f++) {
        for (int i = 0; i < FILAS; i++)
            gpio_put(FILA_PINS[i], i == f ? 0 : 1);

        vTaskDelay(pdMS_TO_TICKS(30));  // estabilización rápida

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
    uint16_t vel_pwm_right = 0 ;
    uint16_t vel_pwm_left = 0 ;
    float aux_vel = 0 ;

    int set_time ;

    while (1) {
        motor_set_speed(MOTOR_LEFT,0);
        motor_set_speed(MOTOR_RIGHT,0);
        
        if (xQueueReceive(q_codigo, &buffer, portMAX_DELAY)) {
            printf("Buffer recibido: %s\n", buffer);
            if      (buffer[0]=='3'){linear_speed=VEL_01;}
            else if (buffer[0]=='6'){linear_speed=VEL_02;}
            else if (buffer[0]=='9'){linear_speed=VEL_03;}
            
            if      (buffer[1]=='A'){radius=CURVA_01;}
            else if (buffer[1]=='B'){radius=CURVA_02;}
            else if (buffer[1]=='C'){radius=CURVA_03;}
            else if (buffer[1]=='D'){radius=RECTA;}

            if (buffer[1] == 'A' || buffer[1] == 'B' || buffer[1] == 'C' ){
                //PASA DE ACTIVACIONES A VUELTAS CON (lineer_speed/21)
                //el 2*PI lo pasa a velocidad angular y el R_WHEEL a velocidad lineal
                aux_vel = (linear_speed / 21) * 2 * PI * R_WHEEL;
                set_time = 1000 * 2 * PI * radius / aux_vel;

                diff_vel.v_left     = linear_speed * (1 + RADIUS / ( 2 * radius ) ) ;
                diff_vel.v_right    = linear_speed * (1 - RADIUS / ( 2 * radius ) ) ;

                vel_pwm_right = diff_vel.v_right * PENDIENTE + ORDENADA;
                vel_pwm_left = diff_vel.v_left * PENDIENTE + ORDENADA;

                motor_set_speed(MOTOR_LEFT,linear_speed);
                motor_set_speed(MOTOR_RIGHT,linear_speed);

                vTaskDelay(pdMS_TO_TICKS(set_time/4 ));

                motor_set_speed(MOTOR_LEFT,vel_pwm_left);
                motor_set_speed(MOTOR_RIGHT,vel_pwm_right);

                vTaskDelay(pdMS_TO_TICKS(set_time/4 ));

                motor_set_speed(MOTOR_LEFT,vel_pwm_right);
                motor_set_speed(MOTOR_RIGHT,vel_pwm_left);

                vTaskDelay(pdMS_TO_TICKS(set_time/2));
                
                motor_set_speed(MOTOR_LEFT,vel_pwm_left);
                motor_set_speed(MOTOR_RIGHT,vel_pwm_right);

                vTaskDelay(pdMS_TO_TICKS(set_time/4));
            }else if (buffer[1]=='D'){
                diff_vel.v_left     = linear_speed  ;
                diff_vel.v_right    = linear_speed  ;

                vel_pwm_right = diff_vel.v_right * PENDIENTE + ORDENADA;
                vel_pwm_left = diff_vel.v_left * PENDIENTE + ORDENADA;

                motor_set_speed(MOTOR_LEFT,vel_pwm_left);
                motor_set_speed(MOTOR_RIGHT,vel_pwm_right);

                vTaskDelay(pdMS_TO_TICKS(5000));
            }
            

            printf("Velocidad: %.2f | Radio: %.2f | Left: %.2f | Right: %.2f | Time: %d \n",
                linear_speed, radius , diff_vel.v_left , diff_vel.v_right , set_time);

            printf("vel_pwm_right = %hu, vel_pwm_left = %hu\n", vel_pwm_right, vel_pwm_left); 

        }
            
           vTaskDelay(pdMS_TO_TICKS(100));  // Reemplazo de vTaskDelay
        
    }
}

void task_matrix(void *params) {
    char buffer[MAX_INPUT + 1] = {0};
    int index = 2;
    char tecla_anterior = 0;
    char tecla_actual = 0;
    int cod_num = 0;
    int cod_letter = 0 ;
    

    while (true) {
        tecla_actual = escanear_teclado();
        if (tecla_actual != 0 && tecla_actual != tecla_anterior) {
            printf("Tecla presionada: %c\n", tecla_actual);

            if (tecla_actual != '#') {
                //
                if (tecla_actual == '3' || tecla_actual == '6' || tecla_actual == '9') {
                    buffer[0] = tecla_actual;
                    cod_num = 1;
                }else if (tecla_actual == 'A' || tecla_actual == 'B' || tecla_actual == 'C'|| tecla_actual == 'D') {
                    buffer[1] = tecla_actual;
                    cod_letter = 1 ;
                }else {buffer[index++] = tecla_actual;}
            }
            
            if (tecla_actual == '*') {
                //index = 0;
                memset(buffer, 0, sizeof(buffer));
                printf("Buffer borrado");
            }

            if (tecla_actual == '#') {
                if (cod_num == 1 && cod_letter==1){
                    buffer[2] = '\0';
                    printf("Ingresado: %s\n", buffer);
                    //index = 2;
                    xQueueSend(q_codigo , buffer , 0 );
                    printf("Ya paso por la cola \n");
                    memset(buffer, 0, sizeof(buffer));
                    cod_num = 0; 
                    cod_letter = 0;
                }else {
                    printf("Error: faltan datos antes de enviar.\n");
                }
                
            }
        }
        tecla_anterior = tecla_actual;

        vTaskDelay(pdMS_TO_TICKS(100)); 
    }
}


/*
void task_lcd(void *params) {
    char str_matrix[17] = {0};
    char str_taco[17] = {0};
    char matrix_input[MAX_INPUT + 1] = {0};
    float taco_input = 0;
    while (true) {
        lcd_clear();
        lcd_set_cursor(0, 0);
        lcd_string("Hola mundo");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}*/
void Tacometro(void *params) {
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
            //printf("Activaciones por segundo: %d\n", counter);
            //printf("Vueltas por segundo: %.2f\n", vueltas);
            xQueueSend(q_tacometro , &vueltas , portMAX_DELAY );
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
    //INICIALIZACION DE LAS COLAS 
    q_codigo        = xQueueCreate(100 , sizeof(char[MAX_INPUT + 1])  );
    //q_vel_deseada   = xQueueCreate(10 , sizeof(VelData_t)            );
    q_tacometro = xQueueCreate(100 , sizeof(float) );

    //INICIALIZACION DE LAS TAREAS 
    xTaskCreate(task_matrix     , "Matrix"      , 2 * configMINIMAL_STACK_SIZE  , NULL, 4, NULL);
    xTaskCreate(task_motor      , "Motor"       , configMINIMAL_STACK_SIZE *3   , NULL, 3, NULL);
    //xTaskCreate(task_lcd, "LCD",  configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(Tacometro, "Tacometro", 2 * configMINIMAL_STACK_SIZE , NULL, 2, NULL);


    vTaskStartScheduler();
    while (true) {
        
    }
}
