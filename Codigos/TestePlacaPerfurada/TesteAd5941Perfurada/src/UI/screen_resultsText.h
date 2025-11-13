#pragma once
#include "uiManager.h"


// Change RESULTS_TEXT to your screen name
lv_obj_t * screen_resultsText_create(void);
void screen_resultsText_update(void);
void screen_resultsText_destroy(void);

// Expose screen descriptor
extern Screen_t Screen_resultsText;
