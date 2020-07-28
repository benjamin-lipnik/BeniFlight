#ifndef MOTOR_INTERFACE_H
#define MOTOR_INTERFACE_H

#include "configuration.h"
#include <stdint.h>

// motorji od 1 do 4*
//power je standardna vrednost od 1000 (min), do 2000(max)
//vsakemu motorju se posebej nastavi moc, z motor_assign_power. Nastavitev pride v veljavnost po klicu motors_apply
//motors_stop ustavi motorje takoj! klic motors_apply ni potreben

uint8_t motors_init(void * param);
void motors_stop();
void motor_assign_power(uint8_t motor, uint16_t power);
void motors_apply();

uint16_t motor_get_power(uint8_t motor);

#endif
