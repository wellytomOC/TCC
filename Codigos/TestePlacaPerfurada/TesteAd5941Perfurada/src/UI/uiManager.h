#pragma once
#include "lvgl.h"
#include "main.h"


typedef struct {
    lv_obj_t * (*create)(void);   // Called once when switching to the screen
    void (*update)(void);         // Called every UI loop iteration
    void (*destroy)(void);        // Called when leaving the screen
} Screen_t;

typedef enum {
    SCREEN_HOME,
    SCREEN_LOADING,
    SCREEN_RESULTS,
    SCREEN_PARAMETERS,
    SCREEN_SINGLE_MEASUREMENT,

    SCREEN_MAX
} ScreenID_t;

void ui_manager_set_screen(ScreenID_t id);
void ui_manager_update(void);

bool ui_manager_is_screen(ScreenID_t id);
ScreenID_t ui_manager_get_current_id(void);
