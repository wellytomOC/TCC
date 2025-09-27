#ifdef __cplusplus
extern "C" {
#endif


#ifndef AD5940_ESP32_H
#define AD5940_ESP32_H

#include "SPI.h"
#include "Arduino.h"

// Pin definitions for ESP32
#define AD5940_SCK_PIN                     48
#define AD5940_MISO_PIN                    21
#define AD5940_MOSI_PIN                    47
#define AD5940_CS_PIN                      45
#define AD5940_RST_PIN                     9
#define AD5940_GP0INT_PIN                  12
#define AD5940_GP1INT_PIN                  11
#define AD5940_GP2INT_PIN                  10

// SPI settings for the AD5940 communication
#define AD5940_SPI_SPEED                   8000000  // Baudrate for SPI (can be adjusted)
#define AD5940_SPI_MODE                    SPI_MODE0
#define AD5940_SPI_BIT_ORDER               MSBFIRST


// Function prototypes
void AD5940_ReadWriteNBytes(unsigned char *pSendBuffer, unsigned char *pRecvBuff, unsigned long length);
void AD5940_CsClr(void);
void AD5940_CsSet(void);
void AD5940_RstSet(void);
void AD5940_RstClr(void);
void AD5940_Delay10us(uint32_t time);
uint32_t AD5940_GetMCUIntFlag(void);
/// @brief 
/// @param  
/// @return 
uint32_t AD5940_ClrMCUIntFlag(void);
uint32_t AD5940_MCUResourceInit(void *pCfg);
void IRAM_ATTR AD5940_InterruptHandler(void);

#endif  // AD5940_ESP32_H


#ifdef __cplusplus
}
#endif