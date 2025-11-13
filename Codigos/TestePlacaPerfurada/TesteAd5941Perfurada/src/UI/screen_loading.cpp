// screen_loading.cpp
#include "screen_loading.h"
#include "lvgl.h"

static lv_obj_t * label_status;


static lv_obj_t * progress_bar;
static uint16_t progress_value = 0;

lv_obj_t * screen_loading_create(void)
{
    lv_obj_t * scr = lv_obj_create(NULL);

    label_status = lv_label_create(scr);
    lv_label_set_text(label_status, "running measurements...");
    lv_obj_center(label_status);

    progress_bar = lv_bar_create(scr);
    lv_obj_set_size(progress_bar, 200, 20);
    lv_obj_align(progress_bar, LV_ALIGN_BOTTOM_MID, 0, -30);
    lv_bar_set_range(progress_bar, AppBIACfg.SweepCfg.SweepStart, AppBIACfg.SweepCfg.SweepStop);
    lv_bar_set_value(progress_bar, AppBIACfg.SweepCfg.SweepStart, LV_ANIM_ON);

    lv_disp_load_scr(scr);

    return scr;
}

void screen_loading_update(void)
{
    // update progress bar according to current frequency
    lv_bar_set_value(progress_bar, (int32_t)AppBIACfg.SweepCurrFreq, LV_ANIM_ON);

    // go to results screen if done
    if (AppBIACfg.FreqofData >= AppBIACfg.SweepCfg.SweepStop) {
        ui_manager_set_screen(SCREEN_HOME);
    }

}

void screen_loading_destroy(void)
{
    if (label_status) {
        lv_obj_del(label_status);
        label_status = nullptr;
    }
    if(progress_bar) {
        lv_obj_del(progress_bar);
        progress_bar = nullptr;
    }
}

Screen_t Screen_Loading = {
    .create = screen_loading_create,
    .update = screen_loading_update,
    .destroy = screen_loading_destroy
};
