#include "Arduino.h"
#include "main.h"
#include <stdio.h>
#include "string.h"
#include "math.h"
#include "ImpedanceMeter.h"
#include "main.h"



//****************************************************************/
//                VARIABLES
//****************************************************************/

// nao sei
#define APPBUFF_SIZE 512
uint32_t AppBuff[APPBUFF_SIZE];

// task
TaskHandle_t TaskImpedanceMeterTask = NULL;

// Measurement configuration
enum IMPEDANCE_METER_STATE ImpedanceMeterState = IMPEDANCE_METER_STATE_IDLE;


//****************************************************************/
//                PROTOTYPES
//****************************************************************/
static int32_t AD5940PlatformCfg(void);
int32_t ShowResult(uint32_t *pData, uint32_t DataCount);
void ImpedanceMeterTask(void *pvParameters);
void InitImpedanceMeterCfg(AppBIACfg_Type *pCfg);
static void ImpedanceInterruptHandler(void);


//****************************************************************/
//               INITS AND TASKS
//****************************************************************/

void InitImpedanceMeter(void)
{
  // Inicializa os parametros
  InitImpedanceMeterCfg(&AppBIACfg);

   xTaskCreate(ImpedanceMeterTask,"ImpedanceMeter", 16384, NULL, 5, &TaskImpedanceMeterTask);
}


void ImpedanceMeterTask(void *pvParameters)
{
  static uint16_t MeasurementCounter = 0;

  while(1)
  {

    switch(ImpedanceMeterState)
    {
      case IMPEDANCE_METER_STATE_IDLE:
      {
        // aguarda comando para iniciar varredura
        break;
      }

      case IMPEDANCE_METER_STATE_INITSWEEP:
      {

        // init sweep
        AD5940PlatformCfg();
        delay(10);

        AppBIAInit(AppBuff, APPBUFF_SIZE); 
        AppBIACtrl(BIACTRL_START, 0); 

        // go to measuring state
        ImpedanceMeterState = IMPEDANCE_METER_STATE_MEASURING;
        break;
      }

      case IMPEDANCE_METER_STATE_MEASURING:
      {
        // check for interrupts
        if(AD5940_GetMCUIntFlag())
        {
          MeasurementCounter++;
          ImpedanceInterruptHandler();
        }

        // check if sweep is done
        if(MeasurementCounter == AppBIACfg.SweepCfg.SweepPoints)
        {
          MeasurementCounter = 0;
          printf("Sweep done. Shutdown AFE.\n");
          //AppBIACtrl(BIACTRL_STOPSYNC, 0);
          AppBIACtrl(BIACTRL_STOPNOW, 0);
          ImpedanceMeterState = IMPEDANCE_METER_STATE_IDLE;
        }

        break;
      }

      default:
        break;
    }

    delay(10);
  }
  vTaskDelete(TaskImpedanceMeterTask);
}



//****************************************************************/
//                FUNCTIONS
//****************************************************************/


void InitImpedanceMeterCfg(AppBIACfg_Type *pCfg)
{
   memset(pCfg, 0, sizeof(AppBIACfg_Type));

  pCfg->bParaChanged   = bFALSE;
  pCfg->SeqStartAddr   = 0;
  pCfg->MaxSeqLen      = 0;

  pCfg->SeqStartAddrCal = 0;
  pCfg->MaxSeqLenCal    = 0;

  pCfg->ReDoRtiaCal    = bFALSE;
  pCfg->SysClkFreq     = 16000000.0;
  pCfg->WuptClkFreq    = 32000.0;
  pCfg->AdcClkFreq     = 16000000.0;
  pCfg->BiaODR         = 2.0;        // 2.0 Hz
  pCfg->NumOfData      = -1;
  pCfg->RcalVal        = 1000.0;     // 1kΩ

  pCfg->PwrMod         = AFEPWR_LP;
  pCfg->HstiaRtiaSel   = HSTIARTIA_1K;
  pCfg->CtiaSel        = 16;
  pCfg->ExcitBufGain   = EXCITBUFGAIN_2;
  pCfg->HsDacGain      = HSDACGAIN_1;
  pCfg->HsDacUpdateRate= 7;
  pCfg->DacVoltPP      = 800.0;

  pCfg->SinFreq        = 1000.0;     // 1 kHz

  pCfg->ADCPgaGain     = ADCPGA_1P5;
  pCfg->ADCSinc3Osr    = ADCSINC3OSR_2;
  pCfg->ADCSinc2Osr    = ADCSINC2OSR_22;

  pCfg->DftNum         = DFTNUM_8192;
  pCfg->DftSrc         = DFTSRC_SINC3;
  pCfg->HanWinEn       = bTRUE;

  // Nested structure: SweepCfg
  pCfg->SweepCfg.SweepEn      = bTRUE;
  pCfg->SweepCfg.SweepStart   = 200.0;
  pCfg->SweepCfg.SweepStop    = 20000.0;
  pCfg->SweepCfg.SweepPoints  = 100;
  pCfg->SweepCfg.SweepLog     = bFALSE;
  pCfg->SweepCfg.SweepIndex   = 0;

  pCfg->FifoThresh       = 4;
  pCfg->BIAInited        = bFALSE;
  pCfg->StopRequired     = bFALSE;
  pCfg->MeasSeqCycleCount= 0;
}

int32_t ShowResult(uint32_t *pData, uint32_t DataCount)
{
  fImpPol_Type *pImp = (fImpPol_Type*)pData;
  

  /* Print results */
  printf("Freq:%.2f ", AppBIACfg.FreqofData);
  for(int i=0;i<DataCount;i++)
  {
    printf("RzMag: %f Ohm , RzPhase: %f \n",pImp[i].Magnitude,pImp[i].Phase*180/MATH_PI);
  }
  return 0;
}

/* Initialize AD5940 basic blocks like clock */
static int32_t AD5940PlatformCfg(void)
{
  CLKCfg_Type clk_cfg;
  FIFOCfg_Type fifo_cfg;
  AGPIOCfg_Type gpio_cfg;

  /* Use hardware reset */
  AD5940_HWReset();
  /* Platform configuration */
  AD5940_Initialize();
  /* Step1. Configure clock */
  clk_cfg.ADCClkDiv = ADCCLKDIV_1;
  clk_cfg.ADCCLkSrc = ADCCLKSRC_HFOSC;
  clk_cfg.SysClkDiv = SYSCLKDIV_1;
  clk_cfg.SysClkSrc = SYSCLKSRC_HFOSC;
  clk_cfg.HfOSC32MHzMode = bFALSE;
  clk_cfg.HFOSCEn = bTRUE;
  clk_cfg.HFXTALEn = bFALSE;
  clk_cfg.LFOSCEn = bTRUE;
  AD5940_CLKCfg(&clk_cfg);
  /* Step2. Configure FIFO and Sequencer*/
  fifo_cfg.FIFOEn = bFALSE;
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_4KB;                       /* 4kB for FIFO, The reset 2kB for sequencer */
  fifo_cfg.FIFOSrc = FIFOSRC_DFT;
  fifo_cfg.FIFOThresh = 4;//AppBIACfg.FifoThresh;        /* DFT result. One pair for RCAL, another for Rz. One DFT result have real part and imaginary part */
  AD5940_FIFOCfg(&fifo_cfg);                             /* Disable to reset FIFO. */
  fifo_cfg.FIFOEn = bTRUE;  
  AD5940_FIFOCfg(&fifo_cfg);                             /* Enable FIFO here */
  
  /* Step3. Interrupt controller */
  
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE);           /* Enable all interrupt in Interrupt Controller 1, so we can check INTC flags */
  AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH, bTRUE);   /* Interrupt Controller 0 will control GP0 to generate interrupt to MCU */
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  /* Step4: Reconfigure GPIO */
  gpio_cfg.FuncSet = GP6_SYNC|GP5_SYNC|GP4_SYNC|GP2_TRIG|GP1_SYNC|GP0_INT;
  gpio_cfg.InputEnSet = AGPIO_Pin2;
  gpio_cfg.OutputEnSet = AGPIO_Pin0|AGPIO_Pin1|AGPIO_Pin4|AGPIO_Pin5|AGPIO_Pin6;
  gpio_cfg.OutVal = 0;
  gpio_cfg.PullEnSet = 0;

  AD5940_AGPIOCfg(&gpio_cfg);
  AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);  /* Allow AFE to enter sleep mode. */
  return 0;
}

static void ImpedanceInterruptHandler(void)
{
  AD5940_ClrMCUIntFlag(); /* Clear this flag */
  uint32_t temp = APPBUFF_SIZE;

  AppBIAISR(AppBuff, &temp); /* Deal with it and provide a buffer to store data we got */
  ShowResult(AppBuff, temp); /* Show the results to UART */

}




//****************************************************************/
//                PUBLIC FUNCTIONS
//****************************************************************/

void StartImpedanceSweep(void)
{
  if(ImpedanceMeterState == IMPEDANCE_METER_STATE_IDLE)
  {
    ImpedanceMeterState = IMPEDANCE_METER_STATE_INITSWEEP;
  }
}

void StartSingleMeasurement(uint32_t freq)
{
  if(ImpedanceMeterState != IMPEDANCE_METER_STATE_IDLE)
  {
    return;
  }

  AppBIACfg.SweepCfg.SweepEn = bFALSE;
  AppBIACfg.SinFreq = (float)freq;
  AppBIACfg.SweepCfg.SweepStart = (float)freq;
  AppBIACfg.FreqofData = (float)freq;

  AppBIACfg.bParaChanged = bTRUE;
  AppBIACfg.ReDoRtiaCal = bTRUE;
  AppBIACfg.SweepCfg.SweepPoints = 3;

  ImpedanceMeterState = IMPEDANCE_METER_STATE_INITSWEEP;
}



//****************************************************************/
//                END OF FILE
//****************************************************************/