#pragma once
#include "uiManager.h"

lv_obj_t * screen_results_create(void);
void screen_results_update(void);
void screen_results_destroy(void);

// Expose a screen descriptor
extern Screen_t Screen_Results;
