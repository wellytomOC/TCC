#pragma once
#include "uiManager.h"




// Lifecycle functions
lv_obj_t * screen_parameters_create(void);
void screen_parameters_update(void);
void screen_parameters_destroy(void);

// Expose screen descriptor
extern Screen_t Screen_Parameters;