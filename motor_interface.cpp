#include "motor_interface.h"
#include <Arduino.h>

uint8_t motor_pins[4] = {MOTOR_PIN_A, MOTOR_PIN_B, MOTOR_PIN_C, MOTOR_PIN_D};
volatile uint32 * timer_registers[4] = {&TIMER4_BASE->CCR1, &TIMER4_BASE->CCR2, &TIMER4_BASE->CCR3, &TIMER4_BASE->CCR4};
uint16_t motor_power_values[4] = {1000,1000,1000,1000};

void motors_init(void * param) {
    TIMER4_BASE->CR1 = TIMER_CR1_CEN | TIMER_CR1_ARPE;
    TIMER4_BASE->CR2 = 0;
    TIMER4_BASE->SMCR = 0;
    TIMER4_BASE->DIER = 0;
    TIMER4_BASE->EGR = 0;
    TIMER4_BASE->CCMR1 = (0b110 << 4) | TIMER_CCMR1_OC1PE |(0b110 << 12) | TIMER_CCMR1_OC2PE;
    TIMER4_BASE->CCMR2 = (0b110 << 4) | TIMER_CCMR2_OC3PE |(0b110 << 12) | TIMER_CCMR2_OC4PE;
    TIMER4_BASE->CCER = TIMER_CCER_CC1E | TIMER_CCER_CC2E | TIMER_CCER_CC3E | TIMER_CCER_CC4E;
    TIMER4_BASE->PSC = 71;
    TIMER4_BASE->ARR = 5000;
    TIMER4_BASE->DCR = 0;

    motors_apply();

    pinMode(motor_pins[0], PWM);
    pinMode(motor_pins[1], PWM);
    pinMode(motor_pins[2], PWM);
    pinMode(motor_pins[3], PWM);
}
void motors_stop() {
  TIMER4_BASE->CCR1 = 1000;
  TIMER4_BASE->CCR2 = 1000;
  TIMER4_BASE->CCR3 = 1000;
  TIMER4_BASE->CCR4 = 1000;
}
void motor_assign_power(uint8_t motor, uint16_t power) {
  if(power > 2100) //failsafe
    power = 1000;

  if(motor_power_values[motor] != power) {
    motor_power_values[motor] = power;
  }
}
void motors_apply() {
  for(uint8_t i = 0; i < 4; i++) {
    *timer_registers[i] = motor_power_values[i];
  }
}

uint16_t motor_get_power(uint8_t motor) {
  return motor_power_values[motor];
}
