// screen_home.cpp
#include "screen_home.h"
#include "uiManager.h"
#include "lvgl.h"
#include "main.h"
#include "DisplayControl.h"

extern Type_GlobalVariables GVariables;

static lv_obj_t * label_counter;

lv_obj_t *screen_home_create(void)
{
    lv_obj_t *scr = lv_obj_create(NULL);
    lv_obj_set_style_pad_all(scr, 10, 0);
    lv_obj_set_flex_flow(scr, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(scr, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    // --- Title label ---
    lv_obj_t *label_title = lv_label_create(scr);
    lv_label_set_text(label_title, "Home Screen");
    lv_obj_set_style_text_font(label_title, &lv_font_montserrat_20, 0);
    lv_obj_align(label_title, LV_ALIGN_TOP_MID, 0, 10);

    // --- Counter label ---
    label_counter = lv_label_create(scr);
    lv_label_set_text(label_counter, "Seconds: 0");
    lv_obj_align(label_counter, LV_ALIGN_CENTER, 0, -40);

    // --- Measurement buttons container ---
    lv_obj_t *cont_buttons = lv_obj_create(scr);
    lv_obj_set_size(cont_buttons, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(cont_buttons, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(cont_buttons, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(cont_buttons, 30, 0); // spacing between buttons
    lv_obj_clear_flag(cont_buttons, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_align(cont_buttons, LV_ALIGN_CENTER, 0, 40);

    // --- "Varredura" button ---
    lv_obj_t *btn_varredura = lv_btn_create(cont_buttons);
    lv_obj_set_size(btn_varredura, 150, 50);
    lv_obj_t *lbl_varredura = lv_label_create(btn_varredura);
    lv_label_set_text(lbl_varredura, "Varredura");
    lv_obj_center(lbl_varredura);
    lv_obj_add_event_cb(btn_varredura, [](lv_event_t *e) {
        ui_manager_set_screen(SCREEN_PARAMETERS);
    }, LV_EVENT_CLICKED, NULL);

    // --- "Um ponto" button ---
    lv_obj_t *btn_um_ponto = lv_btn_create(cont_buttons);
    lv_obj_set_size(btn_um_ponto, 150, 50);
    lv_obj_t *lbl_um_ponto = lv_label_create(btn_um_ponto);
    lv_label_set_text(lbl_um_ponto, "Um ponto");
    lv_obj_center(lbl_um_ponto);
    lv_obj_add_event_cb(btn_um_ponto, [](lv_event_t *e) {
        ui_manager_set_screen(SCREEN_SINGLE_MEASUREMENT);
    }, LV_EVENT_CLICKED, NULL);

    // --- Calibration button ---
    lv_obj_t *btn_calibrate = lv_btn_create(scr);
    lv_obj_set_size(btn_calibrate, 150, 50);
    lv_obj_align(btn_calibrate, LV_ALIGN_BOTTOM_MID, 0, -20);
    lv_obj_t *lbl_cal = lv_label_create(btn_calibrate);
    lv_label_set_text(lbl_cal, "Calibrate Touch");
    lv_obj_center(lbl_cal);

    lv_obj_add_event_cb(btn_calibrate, [](lv_event_t *e) {
        // Temporarily disable LVGL while calibrating
        lv_disp_t *disp = lv_disp_get_default();
        lv_disp_set_default(NULL);
        calibrateTouch();
        // Re-enable LVGL
        lv_disp_set_default(disp);
        ui_manager_set_screen(SCREEN_HOME);
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
