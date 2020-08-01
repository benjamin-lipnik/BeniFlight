#include "ibus.h"
#include <Arduino.h>
#include "configuration.h"

static uint8_t ibus_buffer[IBUS_BUFFER_SIZE];
static uint8_t ibus_buffer_index = 0;

uint8_t ibus_init(void * param) {
  Serial.begin(115200);
  return INIT_OK;
}

uint16_t * ibus_read() {

  uint8_t cycles = 0;
  while(Serial.available()) {
    if(cycles++ >= IBUS_MAX_CYCLES_TIMEOUT) {
      return NULL;
    }

    uint8_t data = Serial.read();

    //HEADER
    if(ibus_buffer_index == 0 && data != 0x20) {
      continue;
    }
    if (ibus_buffer_index == 1 && data != 0x40) {
      ibus_buffer_index = 0;
      continue;
    }
    //DATA
    if(ibus_buffer_index < IBUS_BUFFER_SIZE) {

      if(ibus_buffer_index < IBUS_HEADER_SIZE) { //DO NOT CHANGE ENDIANESS FOR HEADER
          ibus_buffer[ibus_buffer_index++] = data;
      }
      else {
        uint8_t endianess = 0;
        //int endianess = 2*(i % 2 == 0)-1; //endianess conversion
        ibus_buffer[ibus_buffer_index++ + endianess] = data;
      }

    }
    else { //CHECKSUM & PACKAGING
      ibus_buffer_index = 0;
      uint16_t * ibus_buffer_short = ((uint16_t *)ibus_buffer) + 1; //SKIP THE HEADER
      uint16_t checksum = ibus_buffer_short[IBUS_MAX_CHANNELS]; // CHECHSUM POS

      uint16_t calc_checksum = 0xffff;
      for(uint8_t i = 0; i < (IBUS_BUFFER_SIZE - IBUS_CHECKSUM_SIZE); i++) {
        calc_checksum -= ibus_buffer[i];
      }

      //printf("checksum: %04x, calc: %04x\n\r", checksum, calc_checksum);

      if(checksum != calc_checksum) {
        //print checksums do not match
        //printf("bad checksum\n\r");
        return NULL;
      }
      //printf("Rx ok.\n\r");
      return ibus_buffer_short;
    }
  }
}
