// screen_home.cpp
#include "screen_home.h"
#include "uiManager.h"
#include "lvgl.h"
#include "main.h"

extern Type_GlobalVariables GVariables;

static lv_obj_t * label_counter;

lv_obj_t * screen_home_create(void)
{
    lv_obj_t * scr = lv_obj_create(NULL);

    lv_obj_t * label_title = lv_label_create(scr);
    lv_label_set_text(label_title, "Home Screen");
    lv_obj_align(label_title, LV_ALIGN_TOP_MID, 0, 10);

    label_counter = lv_label_create(scr);
    lv_label_set_text(label_counter, "Seconds: 0");
    lv_obj_center(label_counter);

    // Example button to go to calibration screen
    lv_obj_t * btn = lv_btn_create(scr);
    lv_obj_align(btn, LV_ALIGN_BOTTOM_MID, 0, -20);

    lv_obj_t * lbl = lv_label_create(btn);
    lv_label_set_text(lbl, "Go to Cal");
    lv_obj_center(lbl);

    lv_obj_add_event_cb(btn, [](lv_event_t * e) {
        ui_manager_set_screen(SCREEN_LOADING);
    }, LV_EVENT_CLICKED, NULL);

    lv_disp_load_scr(scr);

    return scr;
}

void screen_home_update(void)
{
    char buf[32];
    snprintf(buf, sizeof(buf), "Seconds: %lu", (unsigned long)GVariables.TestCounter);
    lv_label_set_text(label_counter, buf);
}

void screen_home_destroy(void)
{
    if (label_counter) {
        lv_obj_del(label_counter);
        label_counter = nullptr;
    }
}

// Screen descriptor
Screen_t Screen_Home = {
    .create = screen_home_create,
    .update = screen_home_update,
    .destroy = screen_home_destroy
};
