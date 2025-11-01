// This file will contain the implementation for task_recto. The original
// implementation lived in firmware.c; it was moved here to keep tasks modular.

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
#include "task_recto.h"

void task_recto(void *params) {
    // Registrar handle para evitar múltiples instancias
    h_task_recto = xTaskGetCurrentTaskHandle();

    // --- Calibración medida (feedforward) ---
    const float d_dead_L = 158.0f;
    const float k_L      = 0.013f;   // [m/s por nivel]
    const float d_dead_R = 152.0f;
    const float k_R      = 0.013f;   // [m/s por nivel]

    // --- PID inicial (ajustables por UART con TUNE/TUNE_L/TUNE_R) ---
    float Kp_left  = 90.0f, Ki_left  = 38.0f, Kd_left  = 0.20f;
    float Kp_right = 75.0f, Ki_right = 35.0f, Kd_right = 0.18f;

    // --- Control diferencial (sin IMU) ---
    float Kp_diff = 0.90f;   // subí/bajá si corrige lento/oscila
    float Ki_diff = 0.35f;   // integra desbalance sostenido
    float integral_diff = 0.0f;

    // --- Setpoint y duración de prueba ---
    float v_set = 0.2f; // m/s
    float t_sec = 5.0f; // duración por defecto

    // --- Estado del controlador ---
    float error_left = 0, error_right = 0;
    float error_left_prev = 0, error_right_prev = 0;
    float integral_left = 0, integral_right = 0;

    // Velocidades (último valor válido + filtro simple)
    float velL = 0.0f, velR = 0.0f;          // último valor válido
    float velL_f = 0.0f, velR_f = 0.0f;      // filtradas (IIR)
    float velL_prev = 0.0f, velR_prev = 0.0f; // para derivada de medida

    // Tiempos
    const TickType_t periodo = pdMS_TO_TICKS(100); // 100 ms
    TickType_t last_wake = xTaskGetTickCount();
    TickType_t t_prev = last_wake; // para dt real

    // Slew-rate (pasos de PWM por iteración)
    const float SLEW_PER_STEP = 25.0f;
    float u_left_prev = 0.0f, u_right_prev = 0.0f;

    // Anti-windup por clamp
    const float I_MAX_L = 2.0f / fmaxf(1.0f, Ki_left);
    const float I_MAX_R = 2.0f / fmaxf(1.0f, Ki_right);

    // --- Direcciones hacia adelante ---
    motor_set_direction(MOTOR_LEFT,  true);
    motor_set_direction(MOTOR_RIGHT, true);

    // --- Control del ciclo de prueba ---
    bool start_test = false;
    bool test_active = false;
    TickType_t test_deadline = 0;

    // --- Buffer de comandos UART ---
    char cmd_buffer[MAX_INPUT + 1] = {0};
    char line[MAX_INPUT + 1] = {0};

    printf("[RECTO] Listo. Enviá: TUNE ...# | TUNE_L ...# | TUNE_R ...# | RUN v t#\n");

    for (;;) {
        // 0) Revisar si llegó un comando por UART (no bloqueante)
        if (xQueueReceive(q_uart, &cmd_buffer, 0) == pdTRUE) {
            // Sanitizar: quitar quotes/espacios y mayúsculas
            const char *p = cmd_buffer;
            while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n' || *p == '"' || *p == '\'') p++;
            size_t o = 0;
            int prev_space = 1;
            while (*p && o + 1 < sizeof(line)) {
                char c = *p++;
                if (c == '#') break;
                if (c == '"' || c == '\'') continue;
                if (c == '\r' || c == '\n') break;
                int is_space = (c == ' ' || c == '\t');
                if (is_space) {
                    if (prev_space) continue;
                    line[o++] = ' ';
                    prev_space = 1;
                } else {
                    if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');
                    line[o++] = c;
                    prev_space = 0;
                }
            }
            if (o > 0 && line[o-1] == ' ') o--;
            line[o] = '\0';

            // Parseo básico por prefijos
            if (strncmp(line, "TUNE ", 5) == 0) {
                float kpL, kiL, kdL, kpR, kiR, kdR, v, t;
                int n = sscanf(line + 5, "%f %f %f %f %f %f %f %f",
                               &kpL, &kiL, &kdL, &kpR, &kiR, &kdR, &v, &t);
                if (n == 8) {
                    Kp_left = kpL; Ki_left = kiL; Kd_left = kdL;
                    Kp_right = kpR; Ki_right = kiR; Kd_right = kdR;
                    v_set = v; t_sec = t;
                    printf("[RECTO] TUNE OK: L(%.3f,%.3f,%.3f) R(%.3f,%.3f,%.3f) v=%.3f t=%.1f\n",
                           Kp_left, Ki_left, Kd_left, Kp_right, Ki_right, Kd_right, v_set, t_sec);
                    start_test = true;
                } else {
                    printf("[RECTO] ERROR TUNE: se esperan 8 valores\n");
                }
            } else if (strncmp(line, "TUNE_L ", 7) == 0) {
                float kpL, kiL, kdL;
                int n = sscanf(line + 7, "%f %f %f", &kpL, &kiL, &kdL);
                if (n == 3) {
                    Kp_left = kpL; Ki_left = kiL; Kd_left = kdL;
                    printf("[RECTO] TUNE_L OK: L(%.3f,%.3f,%.3f)\n", Kp_left, Ki_left, Kd_left);
                    start_test = true;
                } else {
                    printf("[RECTO] ERROR TUNE_L: se esperan 3 valores\n");
                }
            } else if (strncmp(line, "TUNE_R ", 7) == 0) {
                float kpR, kiR, kdR;
                int n = sscanf(line + 7, "%f %f %f", &kpR, &kiR, &kdR);
                if (n == 3) {
                    Kp_right = kpR; Ki_right = kiR; Kd_right = kdR;
                    printf("[RECTO] TUNE_R OK: R(%.3f,%.3f,%.3f)\n", Kp_right, Ki_right, Kd_right);
                    start_test = true;
                } else {
                    printf("[RECTO] ERROR TUNE_R: se esperan 3 valores\n");
                }
            } else if (strncmp(line, "RUN ", 4) == 0) {
                float v, t;
                int n = sscanf(line + 4, "%f %f", &v, &t);
                if (n == 2) {
                    v_set = v; t_sec = t;
                    printf("[RECTO] RUN v=%.3f t=%.1f\n", v_set, t_sec);
                    start_test = true;
                } else {
                    printf("[RECTO] ERROR RUN: formato 'RUN v t'\n");
                }
            } else if (line[0] >= '1' && line[0] <= '3' && line[1] != '\0') {
                // Comandos cortos 1/2/3 para setpoints discretos
                switch (line[0]) {
                    case '1': v_set = 0.2f; break;
                    case '2': v_set = 0.4f; break;
                    case '3': v_set = 0.6f; break;
                    default:  v_set = 0.2f; break;
                }
                t_sec = 5.0f;
                start_test = true;
                printf("[RECTO] CMD corto -> v=%.3f t=%.1f\n", v_set, t_sec);
            } else {
                printf("[RECTO] Comando ignorado: %s\n", line);
            }
        }

        // 1) Iniciar prueba si corresponde
        if (start_test && !test_active) {
            // --- Reset fuerte de estado para repetibilidad ---
            error_left = error_right = 0.0f;
            error_left_prev = error_right_prev = 0.0f;
            integral_left = integral_right = 0.0f;

            // Dif (sin IMU)
            integral_diff = 0.0f;

            // Velocidades y filtros
            velL = velR = 0.0f;
            velL_f = velR_f = 0.0f;
            velL_prev = velR_prev = 0.0f;

            // Slew
            u_left_prev = 0.0f;
            u_right_prev = 0.0f;

            // Drenar colas de tacómetro
            float tmp;
            while (xQueueReceive(q_tacometro_left,  &tmp, 0) == pdTRUE) {}
            while (xQueueReceive(q_tacometro_right, &tmp, 0) == pdTRUE) {}

            // Temporización
            last_wake = xTaskGetTickCount();
            t_prev = last_wake;

            test_deadline = last_wake + pdMS_TO_TICKS((int)(t_sec * 1000.0f));
            test_active = true;
            start_test = false;
            printf("[RECTO] TEST START v=%.3f t=%.1f\n", v_set, t_sec);
        }

        // 2) Si hay prueba activa, ejecutar un paso de control cada 100 ms
        if (test_active) {
            // dt real (en segundos)
            TickType_t t_now = xTaskGetTickCount();
            float dt = (t_now - t_prev) * (portTICK_PERIOD_MS / 1000.0f);
            if (dt <= 0.0f) dt = 0.001f; // salvaguarda
            t_prev = t_now;

            // Lecturas de velocidad (no bloqueantes): conservar último válido
            float v_new;
            if (xQueueReceive(q_tacometro_left,  &v_new, 0) == pdTRUE) velL = v_new;
            if (xQueueReceive(q_tacometro_right, &v_new, 0) == pdTRUE) velR = v_new;

            // Filtro IIR suave para ruido (opcional)
            const float alpha = 0.08f; // 0..1 (más bajo = más filtro)
            velL_f = alpha * velL_f + (1.0f - alpha) * velL;
            velR_f = alpha * velR_f + (1.0f - alpha) * velR;

            // Feedforward
            float u_ff_L = d_dead_L + (v_set / k_L);
            float u_ff_R = d_dead_R + (v_set / k_R);
            if (u_ff_L > 255) u_ff_L = 255; if (u_ff_L < 0) u_ff_L = 0;
            if (u_ff_R > 255) u_ff_R = 255; if (u_ff_R < 0) u_ff_R = 0;

            // --- Control diferencial (sin IMU) ---
            float e_diff = velR_f - velL_f;     // + si derecha va más rápido
            integral_diff += e_diff * dt;
            // Limitar la integral diferencial para evitar sesgos grandes
            if (integral_diff > 0.5f) integral_diff = 0.5f;
            if (integral_diff < -0.5f) integral_diff = -0.5f;

            float bias = Kp_diff * e_diff + Ki_diff * integral_diff;

            // Setpoints individuales corregidos
            float vL_ref = v_set - 0.5f * bias;
            float vR_ref = v_set + 0.5f * bias;

            // --- PID por rueda ---
            // Error
            float eL = vL_ref - velL_f;
            float eR = vR_ref - velR_f;

            // Integrales con clamp
            integral_left  += eL * dt;
            integral_right += eR * dt;
            if (integral_left  > I_MAX_L) integral_left  = I_MAX_L;
            if (integral_left  < -I_MAX_L) integral_left = -I_MAX_L;
            if (integral_right > I_MAX_R) integral_right = I_MAX_R;
            if (integral_right < -I_MAX_R) integral_right = -I_MAX_R;

            // Derivada sobre la medida (reduce "setpoint kick")
            float d_meas_L = -(velL_f - velL_prev) / dt;
            float d_meas_R = -(velR_f - velR_prev) / dt;
            velL_prev = velL_f;
            velR_prev = velR_f;

            float pid_left  = Kp_left  * eL + Ki_left  * integral_left  + Kd_left  * d_meas_L;
            float pid_right = Kp_right * eR + Ki_right * integral_right + Kd_right * d_meas_R;

            // Salida total
            float u_left  = u_ff_L + pid_left;
            float u_right = u_ff_R + pid_right;

            // Saturación 0..255
            if (u_left  > 255.0f) u_left  = 255.0f;
            if (u_left  <   0.0f) u_left  =   0.0f;
            if (u_right > 255.0f) u_right = 255.0f;
            if (u_right <   0.0f) u_right =   0.0f;

            // Slew-rate (limitador de cambio por paso)
            float duL = u_left  - u_left_prev;
            float duR = u_right - u_right_prev;
            if (duL >  SLEW_PER_STEP) u_left  = u_left_prev  + SLEW_PER_STEP;
            if (duL < -SLEW_PER_STEP) u_left  = u_left_prev  - SLEW_PER_STEP;
            if (duR >  SLEW_PER_STEP) u_right = u_right_prev + SLEW_PER_STEP;
            if (duR < -SLEW_PER_STEP) u_right = u_right_prev - SLEW_PER_STEP;
            u_left_prev  = u_left;
            u_right_prev = u_right;

            // Aplicar PWM
            uint16_t dutyL = (uint16_t)lroundf(fminf(fmaxf(u_left,  0.0f), 255.0f));
            uint16_t dutyR = (uint16_t)lroundf(fminf(fmaxf(u_right, 0.0f), 255.0f));
            motor_set_speed(MOTOR_LEFT,  dutyL);
            motor_set_speed(MOTOR_RIGHT, dutyR);

            // Log cada 0.5 s aprox (5 ciclos de 100 ms)
            static int print_counter = 0;
            print_counter++;
            if (print_counter >= 5) {
                print_counter = 0;
                float t_run = (float)(xTaskGetTickCount() - (test_deadline - pdMS_TO_TICKS((int)(t_sec * 1000.0f)))) / 1000.0f;
                printf("t=%.1f  vL=%.3f  vR=%.3f  |  vLref=%.3f vRref=%.3f  |  uFF_L=%.1f  uFF_R=%.1f  |  uL=%.1f  uR=%.1f  |  eL=%.3f  eR=%.3f  | bias=%.3f\n",
                       t_run, velL_f, velR_f, vL_ref, vR_ref, u_ff_L, u_ff_R, u_left, u_right, eL, eR, bias);
            }

            // Siguiente ciclo con temporizador estable
            vTaskDelayUntil(&last_wake, periodo);

            // ¿Terminó la prueba?
            if (xTaskGetTickCount() >= test_deadline) {
                motor_set_speed(MOTOR_LEFT,  0);
                motor_set_speed(MOTOR_RIGHT, 0);
                test_active = false;
                printf("\n--- Fin de prueba (%.1f s). Motores detenidos ---\n", t_sec);
            }
        } else {
            // Espera pasiva breve para no quemar CPU
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}
