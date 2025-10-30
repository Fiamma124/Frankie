#include <math.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "config.h"
#include "globals.h"
#include "task_tacometro_left.h"

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

