#ifndef COMM_H
#define COMM_H

#include <stdint.h>
#include "configuration.h"

uint8_t print_init(void * params);
void print (char * string);
void println(char * string);

char * serial_reader();

extern char str[150]; //buffer za izpisovanje

#endif
