#include "comm.h"
#include <Arduino.h>

void print_init(void * params) {
#ifdef ENABLE_PRINTING

    Serial.begin(115200);

#endif
}

void print(char * str) {
#ifdef ENABLE_PRINTING

  Serial.print(str);

#endif
}

void println (char * str) {
  print(str);
  print((char *)"\n\r");
}
