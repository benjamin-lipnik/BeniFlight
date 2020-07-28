#include "failsafe.h"
#include "motor_interface.h"
#include "comm.h"
#include <Arduino.h>

static void blink(uint8_t pin)
{
  digitalWrite(pin, HIGH);
  delay(100);
  digitalWrite(pin, LOW);
  delay(100);
}

void error_handler() {
  motors_stop();
  blink(BLUE_LED_PIN);
  while(1);
}
void error_handler_id(uint8_t id) {
  motors_stop();
  while(1) {
    char err_str[20];
    sprintf(err_str, "Error_id: %u\n\r", id);
    print(err_str);

    blink(BLUE_LED_PIN);
  }
}
