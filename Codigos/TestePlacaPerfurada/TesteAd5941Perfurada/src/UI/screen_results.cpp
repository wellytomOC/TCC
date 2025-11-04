// screen_home.cpp
#include "screen_results.h"
#include "lvgl.h"
#include "main.h"

extern Type_GlobalVariables GVariables;


lv_obj_t * screen_results_create(void)
{

    // Create a blank LVGL screen
    lv_obj_t * scr = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x202020), 0); // dark background

    // Example title
    // lv_obj_t * label_title = lv_label_create(scr);
    // lv_label_set_text(label_title, "Template Screen");
    // lv_obj_align(label_title, LV_ALIGN_TOP_MID, 0, 10);

    // Example label that will be updated
    // label_info = lv_label_create(scr);
    // lv_label_set_text(label_info, "Waiting...");
    // lv_obj_center(label_info);

    // Example back button to go to home
    // lv_obj_t * btn_back = lv_btn_create(scr);
    // lv_obj_align(btn_back, LV_ALIGN_BOTTOM_MID, 0, -20);
    // lv_obj_t * lbl_back = lv_label_create(btn_back);
    // lv_label_set_text(lbl_back, "Back");
    // lv_obj_center(lbl_back);
    // lv_obj_add_event_cb(btn_back, [](lv_event_t * e) {
    //     ui_manager_set_screen(SCREEN_HOME);
    // }, LV_EVENT_CLICKED, NULL);

    // Load the screen
    lv_disp_load_scr(scr);
    return scr;
}

void screen_results_update(void)
{
    // Update any dynamic content on the results screen
    // For example, display texts: lv_label_set_text(label_info, GVariables.TestResults);
}

void screen_results_destroy(void)
{
    //delete any global objects created for this screen
}

// Screen descriptor
Screen_t Screen_Results = {
    .create = screen_results_create,
    .update = screen_results_update,
    .destroy = screen_results_destroy
};
