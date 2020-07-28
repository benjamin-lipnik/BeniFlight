#ifndef COMM_H
#define COMM_H

#include <stdint.h>
#include "configuration.h"

uint8_t print_init(void * params);
void print (char * str);
void println(char * str);

char * serial_reader();

#endif
