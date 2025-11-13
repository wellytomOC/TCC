#pragma once
extern "C"{
  #include "ad5940.h"
  #include "BodyImpedance.h"
}



enum IMPEDANCE_METER_STATE{
  IMPEDANCE_METER_STATE_IDLE = 0,
  IMPEDANCE_METER_STATE_INITSWEEP,
  IMPEDANCE_METER_STATE_INITSINGLE,
  IMPEDANCE_METER_STATE_MEASURING,
};


void InitImpedanceMeter(void);
void StartImpedanceSweep(void);
void StartSingleMeasurement(uint32_t freq);
void calculate_impedance_components(float mag, float phase_rad, float freq, float *R, float *X, float *L, float *C);