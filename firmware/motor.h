#ifndef MOTOR_H
#define MOTOR_H

#include <stdbool.h>
#include <stdint.h>

void motor_init(void);
void motor_set_direction(uint8_t motor, bool forward);
void motor_set_speed(uint8_t motor, uint16_t speed);

#endif // MOTOR_H
