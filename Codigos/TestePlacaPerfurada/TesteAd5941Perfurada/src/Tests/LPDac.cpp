#include <stdio.h>
#include "string.h"
#include "Arduino.h"
#include "ESP32Port.h"
#include "LPDac.h"

extern "C"{
    #include "ad5940.h"
}


static uint16_t dacValue = 2048; // Default to mid-scale (1.3V)

void ConfigureLpTIA(void){
  AFERefCfg_Type ref_cfg;
  LPLoopCfg_Type lp_cfg;

  /* Initialize everything to zero(false/OFF/PowerDown), only turn on what we need */
  AD5940_StructInit(&ref_cfg, sizeof(ref_cfg));
  ref_cfg.LpBandgapEn = bTRUE;                        /* Enable low power bandgap */
  ref_cfg.LpRefBufEn = bTRUE;                         /* Enable the low power reference buffer - 2.5V output */
  AD5940_REFCfgS(&ref_cfg);                           /* Call reference configuration function */

  AD5940_StructInit(&lp_cfg, sizeof(lp_cfg));         /* Reset everything to zero(OFF) */
  /* Configure what we need below */
  lp_cfg.LpDacCfg.LpdacSel = LPDAC0;                  /* Select LPDAC0. Note LPDAC1 is available on ADuCM355 */
  lp_cfg.LpDacCfg.DacData12Bit = dacValue;            /* Output midscale voltage (0.2V + 2.4V)/2 = 1.3V */
  lp_cfg.LpDacCfg.DacData6Bit = 0;                    /* 6Bit DAC data */
  lp_cfg.LpDacCfg.DataRst =bFALSE;                    /* Do not keep DATA registers at reset status */
  lp_cfg.LpDacCfg.LpDacSW = LPDACSW_VBIAS2LPPA|LPDACSW_VBIAS2PIN|LPDACSW_VZERO2LPTIA|LPDACSW_VZERO2PIN;
  lp_cfg.LpDacCfg.LpDacRef = LPDACREF_2P5;            /* Select internal 2.5V reference */
  lp_cfg.LpDacCfg.LpDacSrc = LPDACSRC_MMR;            /* The LPDAC data comes from MMR not WG in this case */
  lp_cfg.LpDacCfg.LpDacVbiasMux = LPDACVBIAS_12BIT;   /* Connect Vbias signal to 12Bit LPDAC output */
  lp_cfg.LpDacCfg.LpDacVzeroMux = LPDACVZERO_6BIT;    /* Connect Vzero signal to 6bit LPDAC output */
  lp_cfg.LpDacCfg.PowerEn = bTRUE;                    /* Power up LPDAC */
  lp_cfg.LpAmpCfg.LpAmpSel = LPAMP0;
  lp_cfg.LpAmpCfg.LpAmpPwrMod = LPAMPPWR_NORM;           /* Set low power amplifiers to normal power mode */
  lp_cfg.LpAmpCfg.LpPaPwrEn = bTRUE;                  /* Enable LP PA(potentialstat amplifier) power */
  lp_cfg.LpAmpCfg.LpTiaPwrEn = bTRUE;                /* Leave LPTIA power off */
  lp_cfg.LpAmpCfg.LpTiaSW = LPTIASW(12)|LPTIASW(13)|LPTIASW(2)|LPTIASW(10)|LPTIASW(5)|LPTIASW(9)|LPTIASW(7); /* Close these switches to make sure LP PA amplifier is closed loop */
  lp_cfg.LpAmpCfg.LpTiaRf = LPTIARF_SHORT;
  lp_cfg.LpAmpCfg.LpTiaRtia = LPTIARTIA_OPEN;
  lp_cfg.LpAmpCfg.LpTiaRload = LPTIARLOAD_SHORT;
  
  AD5940_LPLoopCfgS(&lp_cfg); 
}

void RequestVoltage(void){

    while(Serial.available() == 0) delay(100); // Wait for user input
    float voltage = Serial.parseFloat();

    if(voltage <= 0.2f || voltage >= 2.4f){
        printf("Invalid voltage. Please try again.\n");
        return;
    }

    // Map voltage to 12-bit DAC value
    dacValue = static_cast<uint16_t>(((voltage - 0.2f) / (2.4f - 0.2f)) * 4095);
    ConfigureLpTIA();
    printf("LPDAC set to %.2f V (DAC Value: %d)\n", voltage, dacValue);
}

void LPDac_Main(void)
{
    ConfigureLpTIA();

    printf("Type a voltage between 0.2V and 2.4V and press ENTER: ");

    while(1){
        RequestVoltage();
        delay(100);
    }
}