#include <stdio.h>
#include "string.h"
#include "Arduino.h"
#include "ESP32Port.h"
#include "HSDAC.h"

extern "C"{
    #include "ad5940.h"
}


int SIN_FREQ = 25000;     /* 25kHz */

#define SYS_CLOCK_HZ 16000000.0 /* System clock frequency */

AFERefCfg_Type aferef_cfg;
HSLoopCfg_Type HpLoopCfg;
void HSDAC_Main(void){

    /* Configure reference voltages and buffers */
    aferef_cfg.HpBandgapEn = bTRUE;
    aferef_cfg.Hp1V1BuffEn = bTRUE;
    aferef_cfg.Hp1V8BuffEn = bTRUE;
    aferef_cfg.Disc1V1Cap = bFALSE;
    aferef_cfg.Disc1V8Cap = bFALSE;
    aferef_cfg.Hp1V8ThemBuff = bFALSE;
    aferef_cfg.Hp1V8Ilimit = bFALSE;
    aferef_cfg.Lp1V1BuffEn = bFALSE;
    aferef_cfg.Lp1V8BuffEn = bFALSE;
    /* LP reference control */
    aferef_cfg.LpBandgapEn = bTRUE;
    aferef_cfg.LpRefBufEn = bTRUE;
    aferef_cfg.LpRefBoostEn = bFALSE;
    AD5940_REFCfgS(&aferef_cfg);	

    
    AD5940_StructInit(&HpLoopCfg, sizeof(HpLoopCfg));

    HpLoopCfg.HsDacCfg.ExcitBufGain = EXCITBUFGAIN_2;
    HpLoopCfg.HsDacCfg.HsDacGain = HSDACGAIN_1;
    HpLoopCfg.HsDacCfg.HsDacUpdateRate = 7;

    HpLoopCfg.HsTiaCfg.DiodeClose = bFALSE;
    HpLoopCfg.HsTiaCfg.HstiaBias = HSTIABIAS_1P1;
    HpLoopCfg.HsTiaCfg.HstiaCtia = 16; /* 16pF */
    HpLoopCfg.HsTiaCfg.HstiaDeRload = HSTIADERLOAD_OPEN;
    HpLoopCfg.HsTiaCfg.HstiaDeRtia = HSTIADERTIA_OPEN;
    HpLoopCfg.HsTiaCfg.HstiaRtiaSel = HSTIADERTIA_100;

    HpLoopCfg.SWMatCfg.Dswitch = SWD_CE0;
    HpLoopCfg.SWMatCfg.Pswitch = SWP_PL;
    HpLoopCfg.SWMatCfg.Nswitch = SWN_NL;
    HpLoopCfg.SWMatCfg.Tswitch = SWT_TRTIA|SWT_DE0;

    HpLoopCfg.WgCfg.WgType = WGTYPE_SIN;
    HpLoopCfg.WgCfg.GainCalEn = bFALSE;
    HpLoopCfg.WgCfg.OffsetCalEn = bFALSE;
    HpLoopCfg.WgCfg.SinCfg.SinFreqWord = AD5940_WGFreqWordCal(SIN_FREQ,SYS_CLOCK_HZ);
    HpLoopCfg.WgCfg.SinCfg.SinAmplitudeWord = 2047;
    HpLoopCfg.WgCfg.SinCfg.SinOffsetWord = 0;
    HpLoopCfg.WgCfg.SinCfg.SinPhaseWord = 0;
    AD5940_HSLoopCfgS(&HpLoopCfg);


    AD5940_AFECtrlS(AFECTRL_DACREFPWR, bTRUE);
    AD5940_AFECtrlS(AFECTRL_EXTBUFPWR|AFECTRL_INAMPPWR|AFECTRL_HSTIAPWR|AFECTRL_HSDACPWR, bTRUE);
    AD5940_AFECtrlS(AFECTRL_WG, bTRUE);

    AD5940_AFEPwrBW(AFEPWR_LP, AFEBW_250KHZ);

    while(1)
    {
        // Your code to test HSDAC functionality goes here
        delay(1000);
    }
}