#pragma once
#include "uiManager.h"


// Frequency and step limits
#define MIN_START_FREQ_HZ   10
#define MAX_STOP_FREQ_HZ    200000
#define MIN_STEPS           10
#define MAX_STEPS           1000


// Lifecycle functions
lv_obj_t * screen_parameters_create(void);
void screen_parameters_update(void);
void screen_parameters_destroy(void);

// Expose screen descriptor
extern Screen_t Screen_Parameters;