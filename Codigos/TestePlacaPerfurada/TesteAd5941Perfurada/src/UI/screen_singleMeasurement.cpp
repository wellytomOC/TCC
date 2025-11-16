#include "screen_singleMeasurement.h"
#include "lvgl.h"
#include "main.h"
#include "ImpedanceMeter.h"

// Optional external variable access
extern Type_GlobalVariables GVariables;


// UI objects
static lv_obj_t * label_value;
static lv_obj_t * result_box;
static lv_obj_t * label_result;
static lv_obj_t * slider_freq;
static lv_obj_t * label_freq_value;

lv_obj_t * screen_singleMeasurement_create(void)
{
    lv_obj_t * scr = lv_obj_create(NULL);

    /* ---------------------- Title ---------------------- */
    lv_obj_t * label_title = lv_label_create(scr);
    lv_label_set_text(label_title, "Medicao unica");
    lv_obj_align(label_title, LV_ALIGN_TOP_MID, 0, 10);

    /* ---------------------- Back Button ---------------------- */
    lv_obj_t *btn_back = lv_btn_create(scr);
    lv_obj_set_size(btn_back, 80, 40);
    lv_obj_align(btn_back, LV_ALIGN_TOP_LEFT, 10, 10);

    lv_obj_t *lbl_back = lv_label_create(btn_back);
    lv_label_set_text(lbl_back, LV_SYMBOL_LEFT " Voltar");
    lv_obj_center(lbl_back);

    lv_obj_add_event_cb(btn_back, [](lv_event_t *e) {
        ui_manager_set_screen(SCREEN_HOME);
        GVariables.sweep_done = false;
    }, LV_EVENT_CLICKED, NULL);

    /* ---------------------- Frequency slider ---------------------- */
    slider_freq = lv_slider_create(scr);
    lv_obj_set_width(slider_freq, 300);
    lv_slider_set_range(slider_freq, MIN_START_FREQ_HZ, MAX_STOP_FREQ_HZ);
    lv_obj_align(slider_freq, LV_ALIGN_CENTER, 0, -80);

    // Frequency label (shows numeric value)
    label_freq_value = lv_label_create(scr);
    lv_label_set_text_fmt(label_freq_value, "%lu Hz", (unsigned long)MIN_START_FREQ_HZ);
    lv_obj_align_to(label_freq_value, slider_freq, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);

    lv_obj_add_event_cb(slider_freq, [](lv_event_t * e) {
        int32_t val = lv_slider_get_value(slider_freq);
        lv_label_set_text_fmt(label_freq_value, "%lu Hz", (unsigned long)val);
    }, LV_EVENT_VALUE_CHANGED, NULL);

    /* ---------------------- Measure button ---------------------- */
    lv_obj_t * btn_measure = lv_btn_create(scr);
    lv_obj_set_size(btn_measure, 150, 40);
    lv_obj_align(btn_measure, LV_ALIGN_CENTER, 0, 20);

    lv_obj_t * lbl_measure = lv_label_create(btn_measure);
    lv_label_set_text(lbl_measure, "Medir");
    lv_obj_center(lbl_measure);

    lv_obj_add_event_cb(btn_measure, [](lv_event_t * e) {
        uint32_t freq = lv_slider_get_value(slider_freq);
        lv_label_set_text(label_result, "");
        StartSingleMeasurement(freq);
    }, LV_EVENT_CLICKED, NULL);

    /* ---------------------- Results box ---------------------- */
    result_box = lv_obj_create(scr);
    lv_obj_set_size(result_box, 280, 80);
    lv_obj_align(result_box, LV_ALIGN_BOTTOM_MID, 0, -20);
    lv_obj_set_style_pad_all(result_box, 10, 0);
    lv_obj_clear_flag(result_box, LV_OBJ_FLAG_SCROLLABLE);

    label_result = lv_label_create(result_box);
    lv_label_set_text(label_result, "");
    lv_obj_center(label_result);

    lv_disp_load_scr(scr);
    return scr;
}

void screen_singleMeasurement_update(void)
{
    if (!GVariables.sweep_done)
    {
        return;
    }

    GVariables.sweep_done = false; // reset flag
    char buf[64];
    static float R, X, L, C;
    calculate_impedance_components(GVariables.MagnitudeBuffer[1], GVariables.PhaseBuffer[1], AppBIACfg.FreqofData, &R, &X, &L, &C);

    snprintf(buf, sizeof(buf), "R: %.2e Ohm\nX: %.2e Ohm\nL: %.2e H\nC: %.2e F", (double)R, (double)X, (double)L, (double)C);
    printf("R: %.2e Ω , X: %.2e Ω , L: %.2e H , C: %.2e F\n", (double)R, (double)X, (double)L, (double)C);
    //snprintf(buf, sizeof(buf), "Modulo: %.2f Ω\nFase: %.2f°", (double)GVariables.MagnitudeBuffer[1],(double)GVariables.PhaseBuffer[1]);
    lv_label_set_text(label_result, buf);

}

void screen_singleMeasurement_destroy(void)
{
    // Nothing dynamic to free in this simple screen, but clear pointers if needed
    label_value = nullptr;
    result_box = nullptr;
    label_result = nullptr;
    slider_freq = nullptr;
    label_freq_value = nullptr;
}

// Expose the screen descriptor to the UI manager
Screen_t Screen_singleMeasurement = {
    .create = screen_singleMeasurement_create,
    .update = screen_singleMeasurement_update,
    .destroy = screen_singleMeasurement_destroy
};