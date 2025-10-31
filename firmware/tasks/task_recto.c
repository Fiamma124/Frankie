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

	// --- PID inicial (se puede ajustar por UART) ---
	float Kp_left  = 90.0f, Ki_left  = 38.0f, Kd_left  = 0.2f;
	float Kp_right = 75.0f, Ki_right = 30.0f, Kd_right = 0.18f;

	// --- Setpoint y duración de prueba ---
	float v_set = 0.2f; // m/s
	float t_sec = 5.0f; // duración por defecto

	// --- Estado del controlador ---
	float error_left = 0, error_right = 0;
	float error_left_prev = 0, error_right_prev = 0;
	float integral_left = 0, integral_right = 0;
	const TickType_t periodo = pdMS_TO_TICKS(100); // 100 ms
	const float dt = 0.1f;
	TickType_t last_wake = xTaskGetTickCount();
	const float SLEW_PER_STEP = 1000.0f;
	float u_left_prev = 0.0f, u_right_prev = 0.0f;

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
			// Reusar lógica simple aquí
			// trim inicio
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
				int n = sscanf(line + 5, "%f %f %f %f %f %f %f %f", &kpL, &kiL, &kdL, &kpR, &kiR, &kdR, &v, &t);
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
				} else {
					printf("[RECTO] ERROR TUNE_L: se esperan 3 valores\n");
				}
			} else if (strncmp(line, "TUNE_R ", 7) == 0) {
				float kpR, kiR, kdR;
				int n = sscanf(line + 7, "%f %f %f", &kpR, &kiR, &kdR);
				if (n == 3) {
					Kp_right = kpR; Ki_right = kiR; Kd_right = kdR;
					printf("[RECTO] TUNE_R OK: R(%.3f,%.3f,%.3f)\n", Kp_right, Ki_right, Kd_right);
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
				// Compatibilidad con comandos 1/2/3 para setpoints discretos
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
			// Reset de estado
			error_left = error_right = 0.0f;
			error_left_prev = error_right_prev = 0.0f;
			integral_left = integral_right = 0.0f;
			u_left_prev = u_right_prev = 0.0f;

			last_wake = xTaskGetTickCount();
			test_deadline = last_wake + pdMS_TO_TICKS((int)(t_sec * 1000.0f));
			test_active = true;
			start_test = false;
			printf("[RECTO] TEST START v=%.3f t=%.1f\n", v_set, t_sec);
		}

		// 2) Si hay prueba activa, ejecutar un paso de control cada 100 ms
		if (test_active) {
			// Leer velocidades
			float velocidad_left = 0.0f, velocidad_right = 0.0f;
			xQueueReceive(q_tacometro_left,  &velocidad_left,  pdMS_TO_TICKS(50));
			xQueueReceive(q_tacometro_right, &velocidad_right, pdMS_TO_TICKS(50));

			// Feedforward
			float u_ff_L = d_dead_L + (v_set / k_L);
			float u_ff_R = d_dead_R + (v_set / k_R);
			if (u_ff_L > 255) u_ff_L = 255; if (u_ff_L < 0) u_ff_L = 0;
			if (u_ff_R > 255) u_ff_R = 255; if (u_ff_R < 0) u_ff_R = 0;

			// PID por rueda
			error_left  = v_set - velocidad_left;
			error_right = v_set - velocidad_right;
			integral_left  += error_left  * dt;
			integral_right += error_right * dt;
			float derivada_left  = (error_left  - error_left_prev)  / dt;
			float derivada_right = (error_right - error_right_prev) / dt;
			float pid_left  = Kp_left  * error_left  + Ki_left  * integral_left  + Kd_left  * derivada_left;
			float pid_right = Kp_right * error_right + Ki_right * integral_right + Kd_right * derivada_right;

			// Salida total
			float u_left  = u_ff_L + pid_left;
			float u_right = u_ff_R + pid_right;

			// Saturación + anti-windup
			if (u_left > 255)  { u_left = 255;  integral_left  -= error_left  * dt; }
			if (u_left < 0)    { u_left = 0;    integral_left  -= error_left  * dt; }
			if (u_right > 255) { u_right = 255; integral_right -= error_right * dt; }
			if (u_right < 0)   { u_right = 0;   integral_right -= error_right * dt; }

			// Slew-rate
			float duL = u_left  - u_left_prev;
			float duR = u_right - u_right_prev;
			if (duL >  SLEW_PER_STEP) u_left  = u_left_prev  + SLEW_PER_STEP;
			if (duL < -SLEW_PER_STEP) u_left  = u_left_prev  - SLEW_PER_STEP;
			if (duR >  SLEW_PER_STEP) u_right = u_right_prev + SLEW_PER_STEP;
			if (duR < -SLEW_PER_STEP) u_right = u_right_prev - SLEW_PER_STEP;
			u_left_prev  = u_left;
			u_right_prev = u_right;

			// Aplicar PWM
			uint16_t dutyL = (uint16_t)lroundf(fminf(fmaxf(u_left, 0.0f), 255.0f));
			uint16_t dutyR = (uint16_t)lroundf(fminf(fmaxf(u_right, 0.0f), 255.0f));
			motor_set_speed(MOTOR_LEFT,  dutyL);
			motor_set_speed(MOTOR_RIGHT, dutyR);

			// Log cada 0.5 s
			static int print_counter = 0;
			print_counter++;
			if (print_counter >= 5) {
				print_counter = 0;
				float t_run = (float)(xTaskGetTickCount() - (test_deadline - pdMS_TO_TICKS((int)(t_sec * 1000.0f)))) / 1000.0f;
				printf("t=%.1f  vL=%.3f  vR=%.3f  |  uFF_L=%.1f  uFF_R=%.1f  |  uL=%.1f  uR=%.1f  |  eL=%.3f  eR=%.3f\n",
					   t_run, velocidad_left, velocidad_right,
					   u_ff_L, u_ff_R, u_left, u_right, error_left, error_right);
			}

			// Siguiente ciclo
			error_left_prev  = error_left;
			error_right_prev = error_right;
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

