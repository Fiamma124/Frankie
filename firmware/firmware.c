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
#ifndef AT24C32_WRITE_STR_DECL
#define AT24C32_WRITE_STR_DECL
// Declaración adelantada (rtc.h en otro módulo aún no la expone)
bool at24c32_write_str(i2c_inst_t *i2c, uint16_t addr, const char *str);
bool eeprom_erase_prefix(i2c_inst_t *i2c, uint16_t end_addr); // forward decl borrado prefijo
#endif

// Shared project configuration (pins, constants used across modules)
#include "config.h"

//#define PWMBASE 245    
// Task prototypes and shared headers
#include "tasks/task_recto.h"
#include "tasks/task_curva.h"
#include "tasks/task_bluetooth.h"
#include "tasks/task_tacometro_left.h"
#include "tasks/task_tacometro_right.h"
#include "globals.h"
#include "motor.h"



//Defino mi cola
//QueueHandle_t q_matrix ;
QueueHandle_t q_tacometro_left ;
QueueHandle_t q_tacometro_right ;
QueueHandle_t q_uart ;
QueueHandle_t q_vel_impuesta_left ;
QueueHandle_t q_vel_impuesta_right ;
QueueHandle_t q_bluetooth_chars;

// Task handles
TaskHandle_t h_task_recto = NULL;

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

    // Purgar cualquier basura que haya quedado en el FIFO RX antes de habilitar IRQ
    while (uart_is_readable(UART_ID)) {
        (void)uart_getc(UART_ID);
    }

    // Configurar interrupciones UART (solo después de que las colas estén listas en main)
    irq_set_exclusive_handler(UART0_IRQ, on_uart_rx);
    irq_set_enabled(UART0_IRQ, true);
    uart_set_irq_enables(UART_ID, true, false);

    // Cola para caracteres entrantes
    printf("READY\n");
}

volatile static char rx_index = 0;
volatile static char rx_buffer[MAX_INPUT];

void on_uart_rx() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    while (uart_is_readable(UART_ID)) {
        char c = uart_getc(UART_ID);
        if (q_bluetooth_chars != NULL) {
            xQueueSendFromISR(q_bluetooth_chars, &c, &xHigherPriorityTaskWoken);
        }
    }
    // Hacer un único yield al final del servicio de interrupción
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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

// Lectura de velocidad desde la cola correspondiente
static bool leer_velocidad_rueda(uint8_t rueda, float *v, TickType_t timeout_ticks) {
    if (rueda == MOTOR_LEFT) {
        return xQueueReceive(q_tacometro_left, v, timeout_ticks);
    } else {
        return xQueueReceive(q_tacometro_right, v, timeout_ticks);
    }
}

int main()
{
    stdio_init_all();
    // Dar tiempo a que USB CDC enumere antes de prints/I2C intensivo
    sleep_ms(800);
    configurar_gpio();
    motor_init();
    motor_set_direction(MOTOR_LEFT,true);  // Sentido "hacia adelante"
    motor_set_direction(MOTOR_RIGHT,true);

    // Direcciones fijas PID
    #define EEPROM_PIDL_ADDR 0x100
    #define EEPROM_PIDR_ADDR 0x140

    // Opcional: borrar prefijo (0x000-0x1FF) en el arranque. Desactivado por defecto para evitar bloqueos largos.
    #define ERASE_PREFIX_AT_BOOT 0
    #if ERASE_PREFIX_AT_BOOT
    eeprom_erase_prefix(i2c1, 0x200); // 0x200 = EEPROM_LOG_BASE
    #endif

    // Valores por defecto (factory) si no existen en EEPROM
    float Kp_left  = 90.0f, Ki_left  = 38.0f, Kd_left  = 0.20f;
    float Kp_right = 75.0f, Ki_right = 35.0f, Kd_right = 0.18f;

    // Buffers lectura
    char pidl_buf[48] = {0};
    char pidr_buf[48] = {0};

    bool have_left = false, have_right = false;
    if (at24c32_read(i2c1, EEPROM_PIDL_ADDR, (uint8_t*)pidl_buf, sizeof(pidl_buf))) {
        if (strncmp(pidl_buf, "PIDL:", 5) == 0) {
            if (sscanf(pidl_buf+5, "%f,%f,%f", &Kp_left, &Ki_left, &Kd_left) == 3) {
                have_left = true;
            }
        }
    }
    if (at24c32_read(i2c1, EEPROM_PIDR_ADDR, (uint8_t*)pidr_buf, sizeof(pidr_buf))) {
        if (strncmp(pidr_buf, "PIDR:", 5) == 0) {
            if (sscanf(pidr_buf+5, "%f,%f,%f", &Kp_right, &Ki_right, &Kd_right) == 3) {
                have_right = true;
            }
        }
    }

    if (!have_left) {
        // Persistir defaults si no existen
        char tmp[48];
        snprintf(tmp, sizeof(tmp), "PIDL:%.3f,%.3f,%.3f", Kp_left, Ki_left, Kd_left);
        at24c32_write_str(i2c1, EEPROM_PIDL_ADDR, tmp);
        printf("[EEPROM] PID LEFT (default escrito) Kp=%.3f Ki=%.3f Kd=%.3f\n", Kp_left, Ki_left, Kd_left);
    } else {
        printf("[EEPROM] PID LEFT: Kp=%.3f Ki=%.3f Kd=%.3f\n", Kp_left, Ki_left, Kd_left);
    }
    if (!have_right) {
        char tmp[48];
        snprintf(tmp, sizeof(tmp), "PIDR:%.3f,%.3f,%.3f", Kp_right, Ki_right, Kd_right);
        at24c32_write_str(i2c1, EEPROM_PIDR_ADDR, tmp);
        printf("[EEPROM] PID RIGHT (default escrito) Kp=%.3f Ki=%.3f Kd=%.3f\n", Kp_right, Ki_right, Kd_right);
    } else {
        printf("[EEPROM] PID RIGHT: Kp=%.3f Ki=%.3f Kd=%.3f\n", Kp_right, Ki_right, Kd_right);
    }

    //INICIALIZACION DE LAS COLAS 
    q_tacometro_left        = xQueueCreate(1 , sizeof(float) );
    q_tacometro_right       = xQueueCreate(1 , sizeof(float) );
    q_uart                  = xQueueCreate(1 , sizeof(char[MAX_INPUT + 1])  );
    q_bluetooth_chars       = xQueueCreate(MAX_INPUT, sizeof(char));

    // Inicializar UART (IRQ habilitadas recién ahora, con colas ya listas)
    init_uart();

    //INICIALIZACION DE LAS TAREAS 

    xTaskCreate(task_tacometro_left , "Tacometro Left"  , 2 * configMINIMAL_STACK_SIZE , NULL, 2, NULL);
    xTaskCreate(task_tacometro_right, "Tacometro Right" , 2 * configMINIMAL_STACK_SIZE , NULL, 2, NULL);
    // Aumentamos el stack de Bluetooth para evitar overflow con printf y buffers locales
    xTaskCreate(task_bluetooth      , "Bluetooth"       , 4 * configMINIMAL_STACK_SIZE , NULL, 5, NULL);
    
    vTaskStartScheduler();
    while (true) {
        printf("Error: se salió del scheduler!\n");
    }
}


