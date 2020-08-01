#ifndef IBUS_H
#define IBUS_H

#include <stdint.h>

#define IBUS_MAX_CHANNELS        14
#define IBUS_CHECKSUM_SIZE       2
#define IBUS_HEADER_SIZE         2
#define IBUS_CHANNEL_BUFFER_SIZE (2*IBUS_MAX_CHANNELS)
#define IBUS_BUFFER_SIZE         (IBUS_HEADER_SIZE + IBUS_CHANNEL_BUFFER_SIZE + IBUS_CHECKSUM_SIZE)

#define IBUS_MAX_CYCLES_TIMEOUT  20

uint8_t ibus_init(void * param);
uint16_t * ibus_read();

#endif
