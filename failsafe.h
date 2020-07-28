#ifndef FAILSAFE_H
#define FAILSAFE_H

#include "configuration.h"
#include <stdint.h>

void error_handler();
void error_handler_id(uint8_t id);

#endif
