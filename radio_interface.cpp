#include "radio_interface.h"
#include <Arduino.h>
#include <SPI.h>
#include "RF_RX.h"

typedef struct {

  uint8_t power;
  uint8_t roll;
  uint8_t pitch;
  uint8_t yaw;

  uint8_t buttons;

}Nrf_radio_pkg;

uint8_t rx_address[] = { 1, 2, 3, 4, 0 };
Radio_pkg radio_data;
Nrf_radio_pkg rx_data;

void radio_init(void * params) {

  pinMode(CE_PIN, OUTPUT);
  pinMode(CSN_PIN, OUTPUT);

  SPI.setClockDivider(SPI_CLOCK_DIV4);
  SPI.begin();

  if(!initRadio(rx_address, BITRATE2MBPS, 100)) {
    //Error
  }
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
