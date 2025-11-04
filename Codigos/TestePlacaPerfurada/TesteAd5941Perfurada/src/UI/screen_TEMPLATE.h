#pragma once
#include "uiManager.h"


// Change TEMPLATE to your screen name
lv_obj_t * screen_TEMPLATE_create(void);
void screen_TEMPLATE_update(void);
void screen_TEMPLATE_destroy(void);

// Expose screen descriptor
extern Screen_t Screen_TEMPLATE;