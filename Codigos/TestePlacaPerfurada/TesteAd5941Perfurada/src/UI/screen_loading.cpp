// screen_loading.cpp
#include "screen_loading.h"
#include "lvgl.h"

static lv_obj_t * label_status;

lv_obj_t * screen_loading_create(void)
{
    lv_obj_t * scr = lv_obj_create(NULL);

    label_status = lv_label_create(scr);
    lv_label_set_text(label_status, "Loading...");
    lv_obj_center(label_status);


    GVariables.TestCounter = 0; // Reset test counter on loading screen for demo purposes
    lv_disp_load_scr(scr);

    return scr;
}

void screen_loading_update(void)
{
    // Optionally animate or show progress
    if(GVariables.TestCounter > 5){
        lv_label_set_text(label_status, "Loading Complete!");
        ui_manager_set_screen(SCREEN_HOME);
    } else {
        lv_label_set_text_fmt(label_status, "Loading... %d%%", GVariables.TestCounter * 20);
    }
}

void screen_loading_destroy(void)
{
    if (label_status) {
        lv_obj_del(label_status);
        label_status = nullptr;
    }
}

Screen_t Screen_Loading = {
    .create = screen_loading_create,
    .update = screen_loading_update,
    .destroy = screen_loading_destroy
};
