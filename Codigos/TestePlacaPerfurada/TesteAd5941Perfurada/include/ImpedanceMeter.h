#pragma once
extern "C"{
  #include "ad5940.h"
  #include "BodyImpedance.h"
}

void ImpedanceSweep_Main(void);
void InitImpedanceMeter(void);


extern AppBIACfg_Type ImpedanceMeterCfg;