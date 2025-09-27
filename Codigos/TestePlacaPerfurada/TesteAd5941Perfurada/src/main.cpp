#include <Arduino.h>
#include "main.h"
#include "stdio.h"
#include "ESP32Port.h"
extern "C"{
  #include "ad5940.h"
  #include "Impedance.h"
}

void AD5940_Main(void);

//funcoes
void PinSetup(void){
  //LEDs
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);

  //jumpers
  pinMode(JMP1_PIN, INPUT_PULLUP);
  pinMode(JMP2_PIN, INPUT_PULLUP);
  pinMode(JMP3_PIN, INPUT_PULLUP);

  //USB - not working, use as debug pins
  pinMode(USB_DN_PIN, INPUT_PULLUP);
  pinMode(USB_DP_PIN, INPUT_PULLUP);

  //AD GPIOs
  pinMode(AD5940_GP0INT_PIN, INPUT_PULLUP);
  pinMode(AD5940_GP1INT_PIN, INPUT_PULLUP);
  pinMode(AD5940_GP2INT_PIN, INPUT_PULLUP);
}

void DEMO_Test_SPI(void)
{
  const int TOTAL_TRIALS = 1000;
  unsigned long temp, i = TOTAL_TRIALS, SuccessCount = 0, FailCount = 0;
  char *ToBePrinted;
  /**
   * Hardware reset can always put AD5940 to default state. 
   * We recommend to use hardware reset rather than software reset
   * because there are some situations that SPI won't work, for example, AD59840 is in hibernate mode, 
   * or AD5940 system clock is 32kHz that SPI bus clock should also be limited..
   * */
  AD5940_HWReset();
  /**
   * @note MUST call this function whenever there is reset happened. This function will put AD5940 to right state.
   *       The reset can be software reset or hardware reset or power up reset.
  */
  AD5940_Initialize();
  /**
   * Normal application code starts here.
  */
  /**
   * Read register test.
  */
  temp = AD5940_ReadReg(REG_AFECON_ADIID);
  printf("Read ADIID register, got: 0x%04lx\n", temp);
  if(temp != AD5940_ADIID){
    printf("Read register test failed. Ending test.\n" );
    return;
  }
  else
    printf("Read register test pass. Proceeding to write test...\n");
  /**
   * Write register test.
   * */
  srand(0x1234);
  while(i--)
  {
    //delay(1);
    static unsigned long count;
    static unsigned long data;
    /* Generate a 32bit random data */
    data = rand()&0xffff;
    data <<= 16;
    data |= rand()&0xffff;
    count ++;	/* Read write count */
    /**
     * Register CALDATLOCK is 32-bit width, it's readable and writable.
     * We use it to test SPI register access.
    */
    AD5940_WriteReg(REG_AFE_CALDATLOCK, data);
    temp = AD5940_ReadReg(REG_AFE_CALDATLOCK);
    if(temp != data){
      FailCount++;
      printf("[%ld/%d]Write register failed. Written: @0x%08lx.   Return: @0x%08lx.   Millis: %ld.\n", count, TOTAL_TRIALS, data, temp, millis());
    }
    else{
      SuccessCount++;
      printf("[%ld/%d]Write success. Written: @0x%08lx.   Return: @0x%08lx.   Millis: %ld.\n", count, TOTAL_TRIALS, data, temp, millis());
    }
  }

  Serial.println("SPI read/write test completed");
  printf("Success count: %ld.    Fail count: %ld.\n", SuccessCount, FailCount);
}


void setup() {

  //UART initialization
  Serial.begin(115200);

  //pin configurations
  PinSetup();
  
  // Imprime uma mensagem de boas-vindas
  Serial.println("Hello AD5940 - Build Time:");
  Serial.println(__TIME__);
  

  delay(3000);


  //test SPI communication with AD5940
  
  AD5940_MCUResourceInit(0);
  AD5940_Main();
  
}

void loop() {
  digitalWrite(LED1_PIN, HIGH);
  digitalWrite(LED2_PIN, LOW);
  delay(100);
  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, HIGH);
  delay(100);
}



/*!
 *****************************************************************************
 @file:    AD5940Main.c
 @author:  Neo Xu
 @brief:   Standard 4-wire or 2-wire impedance measurement example.
 -----------------------------------------------------------------------------

Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.
By using this software you agree to the terms of the associated
Analog Devices Software License Agreement.
 
*****************************************************************************/


/**
   User could configure following parameters
**/

#define APPBUFF_SIZE 512
uint32_t AppBuff[APPBUFF_SIZE];

int32_t ImpedanceShowResult(uint32_t *pData, uint32_t DataCount)
{
  float freq;

  fImpPol_Type *pImp = (fImpPol_Type*)pData;
  AppIMPCtrl(IMPCTRL_GETFREQ, &freq);

  printf("Freq:%.2f ", freq);
  /*Process data*/
  for(int i=0;i<DataCount;i++)
  {
    printf("RzMag: %f Ohm , RzPhase: %f \n",pImp[i].Magnitude,pImp[i].Phase*180/MATH_PI);
  }
  return 0;
}

static int32_t AD5940PlatformCfg(void)
{
  CLKCfg_Type clk_cfg;
  FIFOCfg_Type fifo_cfg;
  AGPIOCfg_Type gpio_cfg;

  /* Use hardware reset */
  AD5940_HWReset();
  AD5940_Initialize();
  /* Platform configuration */
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
  fifo_cfg.FIFOThresh = 4;//AppIMPCfg.FifoThresh;        /* DFT result. One pair for RCAL, another for Rz. One DFT result have real part and imaginary part */
  AD5940_FIFOCfg(&fifo_cfg);
  fifo_cfg.FIFOEn = bTRUE;
  AD5940_FIFOCfg(&fifo_cfg);
  
  /* Step3. Interrupt controller */
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE);   /* Enable all interrupt in INTC1, so we can check INTC flags */
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH, bTRUE); 
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  /* Step4: Reconfigure GPIO */
  gpio_cfg.FuncSet = GP0_INT|GP1_SLEEP|GP2_SYNC;
  gpio_cfg.InputEnSet = 0;
  gpio_cfg.OutputEnSet = AGPIO_Pin0|AGPIO_Pin1|AGPIO_Pin2;
  gpio_cfg.OutVal = 0;
  gpio_cfg.PullEnSet = 0;
  AD5940_AGPIOCfg(&gpio_cfg);
  AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);  /* Allow AFE to enter sleep mode. */
  return 0;
}

void AD5940ImpedanceStructInit(void)
{
  AppIMPCfg_Type *pImpedanceCfg;
  
  AppIMPGetCfg(&pImpedanceCfg);
  /* Step1: configure initialization sequence Info */
  pImpedanceCfg->SeqStartAddr = 0;
  pImpedanceCfg->MaxSeqLen = 512; /* @todo add checker in function */

  pImpedanceCfg->RcalVal = 10000.0;
  pImpedanceCfg->SinFreq = 60000.0;
  pImpedanceCfg->FifoThresh = 4;
	
	/* Set switch matrix to onboard(EVAL-AD5940ELECZ) dummy sensor. */
	/* Note the RCAL0 resistor is 10kOhm. */
	pImpedanceCfg->DswitchSel = SWD_CE0;
	pImpedanceCfg->PswitchSel = SWP_RE0;
	pImpedanceCfg->NswitchSel = SWN_SE0;
	pImpedanceCfg->TswitchSel = SWT_SE0LOAD;
	/* The dummy sensor is as low as 5kOhm. We need to make sure RTIA is small enough that HSTIA won't be saturated. */
	pImpedanceCfg->HstiaRtiaSel = HSTIARTIA_5K;	
	
	/* Configure the sweep function. */
	pImpedanceCfg->SweepCfg.SweepEn = bTRUE;
	pImpedanceCfg->SweepCfg.SweepStart = 2.0f;	/* Start from 1kHz */
	pImpedanceCfg->SweepCfg.SweepStop = 200e3f;		/* Stop at 100kHz */
	pImpedanceCfg->SweepCfg.SweepPoints = 101;		/* Points is 101 */
	pImpedanceCfg->SweepCfg.SweepLog = bTRUE;
	/* Configure Power Mode. Use HP mode if frequency is higher than 80kHz. */
	pImpedanceCfg->PwrMod = AFEPWR_HP;
	/* Configure filters if necessary */
	pImpedanceCfg->ADCSinc3Osr = ADCSINC3OSR_2;		/* Sample rate is 800kSPS/2 = 400kSPS */
  pImpedanceCfg->DftNum = DFTNUM_16384;
  pImpedanceCfg->DftSrc = DFTSRC_SINC3;
}

void AD5940_Main(void)
{
  uint32_t temp;  
  AD5940PlatformCfg();
  AD5940ImpedanceStructInit();
  
  AppIMPInit(AppBuff, APPBUFF_SIZE);    /* Initialize IMP application. Provide a buffer, which is used to store sequencer commands */
  AppIMPCtrl(IMPCTRL_START, 0);          /* Control IMP measurement to start. Second parameter has no meaning with this command. */
 
  while(1)
  {
    if(AD5940_GetMCUIntFlag())
    {
      AD5940_ClrMCUIntFlag();
      temp = APPBUFF_SIZE;
      AppIMPISR(AppBuff, &temp);
      ImpedanceShowResult(AppBuff, temp);
    }
  }
}

