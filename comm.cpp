#include "comm.h"
#include <Arduino.h>

uint8_t print_init(void * params) {
#ifdef ENABLE_PRINTING

    Serial.begin(115200);

#endif
  return INIT_OK;
}

void print(char * str) {
#ifdef ENABLE_PRINTING
  if(str)
    Serial.print(str);

#endif
}

void println (char * str) {
  print(str);
  print((char *)"\n\r");
}


#ifdef ENABLE_PRINTING

static char serial_buffer[100];
uint8_t buffer_index = 0;

#endif

char * serial_reader() {

#ifndef ENABLE_PRINTING
  return NULL;
#endif

  while(Serial.available()) {
    char rx_data = Serial.read();
    if(rx_data == '\n' || rx_data == '\r' || rx_data == ';' || rx_data == '\0') {
      serial_buffer[buffer_index] = '\0';
      buffer_index = 0;
      return serial_buffer;
    }
    if(buffer_index >= 100) //buffer overflow
      buffer_index = 0;
    serial_buffer[buffer_index++] = rx_data;
  }
  return NULL;
}
