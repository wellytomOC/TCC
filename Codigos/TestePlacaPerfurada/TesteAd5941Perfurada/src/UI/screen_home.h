#pragma once
#include "uiManager.h"

lv_obj_t * screen_home_create(void);
void screen_home_update(void);
void screen_home_destroy(void);

// Expose a screen descriptor
extern Screen_t Screen_Home;
