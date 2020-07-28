#include "RF_RX.h"
#include <Arduino.h>
#include <SPI.h>

/*INTEFACES*/
void SPIWrite (uint8_t data) {
  SPI.transfer(data);
	//HAL_SPI_Transmit(hspi, &data, 1, 100);
}
uint8_t SPIRead() {
    return SPI.transfer(0xff);
}
void set_GPIO_pin (uint8_t pin, uint8_t state) {
  digitalWrite(pin, state);
}
inline void delay_milliseconds(unsigned long milliseconds) {
	delay(milliseconds);
}

void preInit (void * spi_handle) {
	//hspi = (SPI_HandleTypeDef *)spi_handle;
  pinMode(CE_PIN, OUTPUT);
  pinMode(CSN_PIN, OUTPUT);
  
  SPI.setClockDivider(SPI_CLOCK_DIV4);
  SPI.begin();
}

/*COMMUNICATION*/

void SPIWriteBlock(void * data, uint8_t size) {
	for(uint8_t i = 0; i < size; i++)
    {
        SPIWrite(*((uint8_t *)data+i));
    }
}
void SPIReadBlock(void * buff, uint8_t size) {
    for(uint8_t i = 0; i < size; i++)
    {
        *(uint8_t *)((uint8_t *)buff+i) = SPIRead();
    }
}
void writeRegister(uint8_t reg, uint8_t data) {
    csn(0);
    SPIWrite(W_REGISTER | (REGISTER_MASK & reg));
    SPIWrite(data);
    csn(1);
}
void writeRegisterBlock(uint8_t reg, void * data, uint8_t size) {
    csn(0);
    SPIWrite(W_REGISTER | (REGISTER_MASK & reg));
    SPIWriteBlock(data, size);
    csn(1);
}
uint8_t readRegister(uint8_t reg) {
    csn(0);
    SPIWrite(R_REGISTER | (REGISTER_MASK & reg));
    uint8_t ret_data = SPIRead();
    csn(1);
    return ret_data;
}
void readRegisterBlock(uint8_t reg, void * buff, uint8_t size) {
    csn(0);
    SPIWrite(R_REGISTER | (REGISTER_MASK & reg));
    SPIReadBlock(buff, size);
    csn(1);
}

/*LIB*/

uint8_t startRx() {

    flushTx();
    resetStatus();

    ce(0); // Put radio into Standby-I mode in order to transition into RX mode.
    writeRegister(CONFIG, CONFIG_REG_SETTINGS_FOR_RX_MODE);
    ce(1);

    // Wait for the transition into RX mode.
    delay_milliseconds(POWERDOWN_TO_RXTX_MODE_MILLIS);

    return readRegister(CONFIG) == CONFIG_REG_SETTINGS_FOR_RX_MODE;
}
uint8_t initRadio (uint8_t * receive_address, uint8_t bitrate, uint8_t channel) {

    //CE IN CSN moreta bit output

    csn(1);

    delay_milliseconds(OFF_TO_POWERDOWN_MILLIS);

    setChannel(channel);

    writeRegister(RF_SETUP, bitrate | RF_TX_PWR_MAX);

    if(receive_address != NULL)
        writeRegisterBlock(RX_ADDR_P1, receive_address, 5);

    //Enable features like no ack
    writeRegister(DYNPD, _BV(DPL_P0) | _BV(DPL_P1));
    writeRegister(FEATURE, _BV(EN_DPL) | _BV(EN_ACK_PAY) | _BV(EN_DYN_ACK));

    flushRx();
    flushTx();
    resetStatus();

    return startRx();
}
uint8_t hasData() {
    return ((getStatus()>>1) & 0b111) != 0b111; //rx fifo not empty
}
void readData (void * data, uint8_t size) {
    if(!size)
        size = readRegister(R_RX_PL_WID); //auto length

    csn(0);
    SPIWrite(R_RX_PAYLOAD);
    SPIReadBlock(data, size);
    csn(1);

    uint8_t status_reg = getStatus();
    if(status_reg & _BV(RX_DR))
    {
        resetStatus();
    }
}
