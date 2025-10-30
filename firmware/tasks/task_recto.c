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
	// --- Ganancias medidas (tu calibración) ---
	const float d_dead_L = 168.0f;
	const float k_L      = 0.013f;   // [m/s por nivel] En el piso es 0.05f
	const float d_dead_R = 176.0f;
	const float k_R      = 0.013f;  // [m/s por nivel] En el piso es 0.058f
	// --- Setpoints por comando (igual que antes) ---
	float setpoint_left  = 0.2f;   // m/s
	float setpoint_right = 0.2f;   // m/s
	char cmd_buffer[MAX_INPUT + 1] = {0};

	if (xQueueReceive(q_uart, &cmd_buffer, 0)) {
		switch (cmd_buffer[0]) {
			case '1': setpoint_left = setpoint_right = 0.2f;  break;
			case '2': setpoint_left = setpoint_right = 0.4f;  break;
			case '3': setpoint_left = setpoint_right = 0.6f;  break;
			default:  setpoint_left = setpoint_right = 0.2f;  break;
		}
	}

	// --- Controladores (PI por rueda) ---
	float Kp_left  = 90.0f, Ki_left  = 30.0f, Kd_left  = 0.2f;
	float Kp_right = 75.0f, Ki_right = 24.0f, Kd_right = 0.18f;

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
	//xQueueOverwrite(q_tacometro_left, &velocidad_left); // ensure default
	//xQueueOverwrite(q_tacometro_right, &velocidad_right);
	xQueueReceive(q_tacometro_left,  &velocidad_left,  pdMS_TO_TICKS(50));
	xQueueReceive(q_tacometro_right, &velocidad_right, pdMS_TO_TICKS(50));
	printf("Velocidades leídas: vL=%.3f m/s, vR=%.3f m/s\n", velocidad_left, velocidad_right);

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
	uint16_t dutyL = (uint16_t)lroundf(fminf(fmaxf(u_left, 0.0f), 255.0f));
	uint16_t dutyR = (uint16_t)lroundf(fminf(fmaxf(u_right, 0.0f), 255.0f));
	motor_set_speed(MOTOR_LEFT,  dutyL);
	motor_set_speed(MOTOR_RIGHT, dutyR);

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

