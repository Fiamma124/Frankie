// Shared configuration and board constants used across tasks and firmware
#ifndef CONFIG_H
#define CONFIG_H

#include <stdbool.h>

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

// Pines para los tacometros
#define IN_PIN_TACOMETRO_LEFT 28
#define IN_PIN_TACOMETRO_RIGHT 27

// Configuración de la frecuencia PWM (Hz)
#define PWM_FREQ 20000

// Identificadores de ruedas
#define MOTOR_LEFT      1
#define MOTOR_RIGHT     2

// Constantes físicas / de filtro
#define PI 3.14159265f
#define N_PROM 5  // (numero de muestras para promedio)
#define R_WHEEL 0.0325f // Radio de las ruedas (m)

// Keyboard / input
#define MAX_INPUT 4

// UART
#define UART_ID uart0
#define UART_TX_PIN 16
#define UART_RX_PIN 17
#define BAUD_RATE 115200

#endif // CONFIG_H
