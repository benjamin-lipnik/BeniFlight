#include "radio_interface.h"
#include <Arduino.h>

#if !defined USE_IBUS

#include <SPI.h>
#include "RF_RX.h"

typedef struct {

  uint8_t power;
  uint8_t roll;
  uint8_t pitch;
  uint8_t yaw;

  uint8_t buttons;

}Nrf_radio_pkg;

static uint8_t rx_address[] = { 1, 2, 3, 4, 0 };
static Radio_pkg radio_data;
static Nrf_radio_pkg rx_data;

uint8_t radio_init(void * params) {

  pinMode(CE_PIN, OUTPUT);
  pinMode(CSN_PIN, OUTPUT);

  SPI.setClockDivider(SPI_CLOCK_DIV4);
  SPI.begin();

  if(!initRadio(rx_address, BITRATE2MBPS, 100)) {
    //Error
    return INIT_ERROR;
  }
  return INIT_OK;
}
Radio_pkg * radio_read() {

  uint8_t has_data = hasData();

  if(!has_data)
    return NULL;

  while(has_data) {
    readData(&rx_data, sizeof(rx_data));
    has_data = hasData();
  }

  radio_data.power   = 1000 + 4 * rx_data.power;
  radio_data.roll    = 1000 + 4 * rx_data.roll;
  radio_data.pitch   = 1000 + 4 * rx_data.pitch;
  radio_data.yaw     = 1000 + 4 * rx_data.yaw;
  radio_data.buttons = rx_data.buttons;

  return &radio_data;
}


#else

#include "ibus.h"

static Radio_pkg radio_data;
//static uint8_t data_order[6] = {4,6,2,0,8,10};

uint8_t radio_init(void * params) {
  return ibus_init(params);
}

Radio_pkg * radio_read() {
  
  uint16_t * ibus_rx = ibus_read();
  if(!ibus_rx)
    return NULL;

  //CE ZMANJKA SIGNALA
  for(uint8_t i = 0; i < IBUS_MAX_CHANNELS; i++) {
    if(ibus_rx[i] > 2100)
      return NULL;
  }

  radio_data.power   = ibus_rx[4];
  radio_data.yaw     = ibus_rx[6];
  radio_data.pitch   = ibus_rx[2];
  radio_data.roll    = ibus_rx[0];
  radio_data.buttons = (ibus_rx[8] > 1200) << ARM_BIT;

  return &radio_data;
}


#endif
