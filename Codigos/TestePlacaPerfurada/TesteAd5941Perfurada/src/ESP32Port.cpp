#include "SPI.h"
#include "Arduino.h"
#include "ESP32Port.h"

#include "driver/spi_master.h"

volatile static uint8_t ucInterrupted = 0; // Flag to indicate interrupt occurred
spi_device_handle_t spi_handle;



/**
 * @brief Using SPI to transmit N bytes and return the received bytes.
 */
void AD5940_ReadWriteNBytes(unsigned char *pSendBuffer, unsigned char *pRecvBuff, unsigned long length) {

  SPI.beginTransaction(SPISettings(AD5940_SPI_SPEED, AD5940_SPI_BIT_ORDER, AD5940_SPI_MODE)); // Adjust baud rate and settings as necessary
  for (unsigned long i = 0; i < length; i++) {
    pRecvBuff[i] = SPI.transfer(pSendBuffer[i]);
  }
 SPI.endTransaction();

}

void AD5940_CsClr(void) {
  digitalWrite(AD5940_CS_PIN, LOW);
}

void AD5940_CsSet(void) {
  digitalWrite(AD5940_CS_PIN, HIGH);
}

void AD5940_RstSet(void) {
  digitalWrite(AD5940_RST_PIN, HIGH);
}

void AD5940_RstClr(void) {
  digitalWrite(AD5940_RST_PIN, LOW);
}

void AD5940_Delay10us(uint32_t time) {
  if (time == 0) return;
  delayMicroseconds(time * 10);
}

uint32_t AD5940_GetMCUIntFlag(void) {
  return ucInterrupted;
}

uint32_t AD5940_ClrMCUIntFlag(void) {
  ucInterrupted = 0;
  return 1;
}

uint32_t AD5940_MCUResourceInit(void *pCfg) {
  // Initialize the SPI pins
  Serial.println("Initializing MCU resources for AD5940...");
  pinMode(AD5940_CS_PIN, OUTPUT);
  pinMode(AD5940_RST_PIN, OUTPUT);
  pinMode(AD5940_GP0INT_PIN, INPUT_PULLUP); // External interrupt pin

  Serial.println("Starting SPI...");
  SPI.begin(AD5940_SCK_PIN, AD5940_MISO_PIN, AD5940_MOSI_PIN, AD5940_CS_PIN);

  // Reset and set initial values for CS and RST pins
  AD5940_CsSet();
  AD5940_RstSet();

  // Attach interrupt for external GPIO pin (AD5940_GP0INT_PIN)
  attachInterrupt(digitalPinToInterrupt(AD5940_GP1INT_PIN), AD5940_InterruptHandler, FALLING);

  return 0;
}

/**
 * @brief Interrupt handler for external interrupt
 */
void IRAM_ATTR AD5940_InterruptHandler() {
  //Serial.println("Interrupt occurred!");
  ucInterrupted = 1;
}
