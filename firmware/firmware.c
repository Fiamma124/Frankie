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

// task_bluetooth implementation moved to tasks/task_bluetooth.c
// task_tacometro_right moved to tasks/task_tacometro_right.c

// task_tacometro_left moved to tasks/task_tacometro_left.c

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

    // Podés guardar estos valores en variables globales o EEPROM si querés persistirlos.
    // Luego, en tu lazo de control, usarás:
    // u_ff_left  = L.d_dead  + v_ref / L.k;
    // u_ff_right = R.d_dead  + v_ref / R.k;

    // Terminar tarea si es one-shot
    vTaskDelete(NULL);
}



// task_recto implementation moved to tasks/task_recto.c


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


