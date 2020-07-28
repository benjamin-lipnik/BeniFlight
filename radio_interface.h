#ifndef RADIO_INTERFACE_H
#define RADIO_INTERFACE_H

#include "configuration.h"
#include <stdint.h>

//Radio_pkg vsebuje podate z daljinca v standardni obliki (1000 = min), (2000 = max)

typedef struct {

  uint16_t power;
  uint16_t roll;
  uint16_t pitch;
  uint16_t yaw;

  uint8_t buttons;

}Radio_pkg;

const Radio_pkg flushed_pkg = {1000,1500,1500,1500,0};

void radio_init(void * params);
Radio_pkg * radio_read();


#endif
