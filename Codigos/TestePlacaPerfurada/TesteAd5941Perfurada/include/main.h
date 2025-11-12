#pragma once
#include "arduino.h"
#include "stdio.h"
#include "string.h"
#include "math.h"

#ifdef __cplusplus
extern "C" {
#endif
#include "ad5940.h"
#ifdef __cplusplus
}
#endif

#define LCD_CS_PIN 1
#define LCD_RST_PIN 2
#define LCD_RS_PIN 3
#define LCD_MOSI_PIN 4
#define LCD_MISO_PIN 5
#define LCD_SCK_PIN 6

#define TCH_CS_PIN 7
#define TCH_IRQ_PIN 8

#define SD_CS_PIN 15

#define LED1_PIN 16
#define LED2_PIN 17

#define JMP1_PIN 37
#define JMP2_PIN 38
#define JMP3_PIN 39

#define USB_DN_PIN 13
#define USB_DP_PIN 14



typedef struct Type_GlobalVariables{
    bool isLPDACConfigured;

    uint8_t TestCounter;
    bool sweep_done;
    bool sweep_ready;
}Type_GlobalVariables;

extern Type_GlobalVariables GVariables;






// Frequency and step limits
#define MIN_START_FREQ_HZ   10
#define MAX_STOP_FREQ_HZ    200000
#define MIN_STEPS           10
#define MAX_STEPS           1000


typedef struct
{
/* Common configurations for all kinds of Application. */
  BoolFlag bParaChanged;        /* Indicate to generate sequence again. It's auto cleared by AppBIAInit */
  uint32_t SeqStartAddr;        /* Initialaztion sequence start address in SRAM of AD5940  */
  uint32_t MaxSeqLen;           /* Limit the maximum sequence.   */
  uint32_t SeqStartAddrCal;     /* Measurement sequence start address in SRAM of AD5940 */
  uint32_t MaxSeqLenCal;
/* Application related parameters */ 
  //BoolFlag bBioElecBoard;     /* The code is same for BioElec board and AD5941Sens1 board. No changes are needed */
  BoolFlag ReDoRtiaCal;         /* Set this flag to bTRUE when there is need to do calibration. */
  float SysClkFreq;             /* The real frequency of system clock */
  float WuptClkFreq;            /* The clock frequency of Wakeup Timer in Hz. Typically it's 32kHz. Leave it here in case we calibrate clock in software method */
  float AdcClkFreq;             /* The real frequency of ADC clock */
  uint32_t FifoThresh;           /* FIFO threshold. Should be N*4 */   
  float BiaODR;                 /* in Hz. ODR decides the period of WakeupTimer who will trigger sequencer periodically. DFT number and sample frequency decides the maxim ODR. */
  int32_t NumOfData;            /* By default it's '-1'. If you want the engine stops after get NumofData, then set the value here. Otherwise, set it to '-1' which means never stop. */
  float SinFreq;                /* Frequency of excitation signal */
  float RcalVal;                /* Rcal value in Ohm */
  uint32_t PwrMod;              /* Control Chip power mode(LP/HP) */
  float DacVoltPP;              /* Final excitation voltage is DAC_VOLTpp*DAC_PGA*EXCIT_GAIN, DAC_PGA= 1 or 0.2, EXCIT_GAIN=2 or 0.25. DAC output voltage in mV peak to peak. Maximum value is 800mVpp. Peak to peak voltage  */
  uint32_t ExcitBufGain;        /* Select from  EXCITBUFGAIN_2, EXCITBUFGAIN_0P25 */     
  uint32_t HsDacGain;           /* Select from  HSDACGAIN_1, HSDACGAIN_0P2 */  
  uint32_t HsDacUpdateRate;     /* DAC update rate is SystemCLoock/Divider. The available value is 7 to 255. Set to 7 for better performance */
  uint32_t ADCPgaGain;          /* PGA Gain select from GNPGA_1, GNPGA_1_5, GNPGA_2, GNPGA_4, GNPGA_9 !!! We must ensure signal is in range of +-1.5V which is limited by ADC input stage */   
  uint8_t ADCSinc3Osr;          /* SINC3 OSR selection. ADCSINC3OSR_2, ADCSINC3OSR_4 */
  uint8_t ADCSinc2Osr;          /* SINC2 OSR selection. ADCSINC2OSR_22...ADCSINC2OSR_1333 */
  uint32_t HstiaRtiaSel;        /* Use internal RTIA, select from RTIA_INT_200, RTIA_INT_1K, RTIA_INT_5K, RTIA_INT_10K, RTIA_INT_20K, RTIA_INT_40K, RTIA_INT_80K, RTIA_INT_160K */
  uint32_t CtiaSel;             /* Select CTIA in pF unit from 0 to 31pF */

  uint32_t DftNum;              /* DFT number */
  uint32_t DftSrc;              /* DFT Source */
  BoolFlag HanWinEn;            /* Enable Hanning window */

  /* Sweep Function Control */
  SoftSweepCfg_Type SweepCfg;
/* Private variables for internal usage */
  float SweepCurrFreq;
  float SweepNextFreq;
  float RtiaCurrValue[2];                    /* Calibrated Rtia value of current frequency */
  float RtiaCalTable[MAX_STEPS][2];   /* Calibrated Rtia Value table */
  float FreqofData;                         /* The frequency of latest data sampled */
  BoolFlag BIAInited;                       /* If the program run firstly, generated sequence commands */
  SEQInfo_Type InitSeqInfo;
  SEQInfo_Type MeasureSeqInfo;
  BoolFlag StopRequired;          /* After FIFO is ready, stop the measurement sequence */
  uint32_t FifoDataCount;         /* Count how many times impedance have been measured */
  uint32_t MeasSeqCycleCount;     /* How long the measurement sequence will take */
  float MaxODR;                   /* Max ODR for sampling in this config */       
/* End */
}AppBIACfg_Type;



extern AppBIACfg_Type AppBIACfg;
