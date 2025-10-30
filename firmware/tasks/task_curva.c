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


#include <math.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "hardware/pwm.h"
#include "string.h"
#include "hardware/i2c.h"
#include "config.h"

void task_curva(void *params) {
    const uint32_t tiempo_ms = 5000; // 5 segundos
    printf("Falta implementar curva\n");
    vTaskDelay(pdMS_TO_TICKS(tiempo_ms));
    vTaskDelete(NULL);
}