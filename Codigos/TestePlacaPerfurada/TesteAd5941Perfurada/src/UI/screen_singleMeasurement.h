#pragma once
#include "uiManager.h"


// Change TEMPLATE to your screen name
lv_obj_t * screen_singleMeasurement_create(void);
void screen_singleMeasurement_update(void);
void screen_singleMeasurement_destroy(void);

// Expose screen descriptor
extern Screen_t Screen_singleMeasurement;