#include "screen_parameters.h"
#include "lvgl.h"
#include "main.h"
#include "ImpedanceMeter.h"

extern Type_GlobalVariables GVariables;

// ==== Screen-local widgets ====
static lv_obj_t * ta_start_freq = nullptr;   // spinbox
static lv_obj_t * ta_end_freq   = nullptr;   // spinbox
static lv_obj_t * slider_steps  = nullptr;
static lv_obj_t * lbl_steps_val = nullptr;
static lv_obj_t * scr_param     = nullptr;

static lv_obj_t * sw_mode = nullptr;

// ==== Forward declaration ====
static void CheckParamChanged(float startVal, float endVal, int steps);

// --------------------------------------------------
// Slider callback for steps
// --------------------------------------------------
static void steps_slider_cb(lv_event_t * e)
{
    int v = lv_slider_get_value(slider_steps);

    char buf[16];
    snprintf(buf, sizeof(buf), "%d", v);
    lv_label_set_text(lbl_steps_val, buf);
}

// --------------------------------------------------
// Screen creation
// --------------------------------------------------
lv_obj_t * screen_parameters_create(void)
{
    scr_param = lv_obj_create(NULL);
    lv_obj_clear_flag(scr_param, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(scr_param, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(scr_param, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(scr_param, 2, 0);
    lv_obj_set_style_pad_row(scr_param, 4, 0);

    lv_obj_t * label_title = lv_label_create(scr_param);
    lv_label_set_text(label_title, "Measurement Parameters");
    lv_obj_set_style_text_font(label_title, &lv_font_montserrat_20, 0);


    // ============================================================
    // Start frequency
    // ============================================================
    lv_obj_t * cont_start = lv_obj_create(scr_param);
    lv_obj_clear_flag(cont_start, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scroll_dir(cont_start, LV_DIR_NONE);
    lv_obj_set_size(cont_start, 420, 50);
    lv_obj_set_flex_flow(cont_start, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(cont_start, LV_FLEX_ALIGN_SPACE_BETWEEN,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    lv_obj_t * lbl_start = lv_label_create(cont_start);
    lv_label_set_text(lbl_start, "Start:");

    // -- MINUS button (bigger)
    lv_obj_t * btn_start_minus = lv_btn_create(cont_start);
    lv_obj_set_size(btn_start_minus, 80, 35);
    lv_obj_t * lbl_start_minus = lv_label_create(btn_start_minus);
    lv_label_set_text(lbl_start_minus, "-");
    lv_obj_center(lbl_start_minus);
    lv_obj_set_style_text_font(lbl_start_minus,   &lv_font_montserrat_26, 0);

    // Spinbox
    ta_start_freq = lv_spinbox_create(cont_start);

    lv_obj_t * sb_text = lv_obj_get_child(ta_start_freq, 0);  // internal text area
    lv_obj_set_style_text_align(sb_text, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_clear_flag(sb_text, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_clear_flag(ta_start_freq, LV_OBJ_FLAG_SCROLLABLE);
    lv_spinbox_set_range(ta_start_freq, MIN_START_FREQ_HZ, MAX_STOP_FREQ_HZ);
    lv_spinbox_set_digit_format(ta_start_freq, 6, 0);
    lv_spinbox_set_step(ta_start_freq, 1);
    lv_spinbox_set_value(ta_start_freq, AppBIACfg.SweepCfg.SweepStart);
    lv_obj_set_size(ta_start_freq, 110, 35);
    lv_obj_set_style_text_font(ta_start_freq, &lv_font_montserrat_24, 0);

    // -- PLUS button (bigger)
    lv_obj_t * btn_start_plus = lv_btn_create(cont_start);
    lv_obj_set_size(btn_start_plus, 80, 35);
    lv_obj_t * lbl_start_plus = lv_label_create(btn_start_plus);
    lv_label_set_text(lbl_start_plus, "+");
    lv_obj_center(lbl_start_plus);
    lv_obj_set_style_text_font(lbl_start_plus,   &lv_font_montserrat_26, 0);

    lv_obj_t * lbl_start_unit = lv_label_create(cont_start);
    lv_label_set_text(lbl_start_unit, "Hz");

    // Events
    lv_obj_add_event_cb(btn_start_minus, [](lv_event_t * e) {
        lv_spinbox_decrement(ta_start_freq);
        int start = lv_spinbox_get_value(ta_start_freq);
        int end   = lv_spinbox_get_value(ta_end_freq);
        if(start > end) lv_spinbox_set_value(ta_end_freq, start);
    }, LV_EVENT_CLICKED, NULL);

    lv_obj_add_event_cb(btn_start_plus, [](lv_event_t * e) {
        lv_spinbox_increment(ta_start_freq);
        int start = lv_spinbox_get_value(ta_start_freq);
        int end   = lv_spinbox_get_value(ta_end_freq);
        if(start > end) lv_spinbox_set_value(ta_end_freq, start);
    }, LV_EVENT_CLICKED, NULL);



    // ============================================================
    // End frequency
    // ============================================================
    lv_obj_t * cont_end = lv_obj_create(scr_param);
    lv_obj_clear_flag(cont_end, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scroll_dir(cont_end, LV_DIR_NONE);
    lv_obj_set_size(cont_end, 420, 50);
    lv_obj_set_flex_flow(cont_end, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(cont_end, LV_FLEX_ALIGN_SPACE_BETWEEN,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    lv_obj_t * lbl_end = lv_label_create(cont_end);
    lv_label_set_text(lbl_end, "End:");

    // minus (big)
    lv_obj_t * btn_end_minus = lv_btn_create(cont_end);
    lv_obj_set_size(btn_end_minus, 80, 35);
    lv_obj_t * lbl_end_minus = lv_label_create(btn_end_minus);
    lv_label_set_text(lbl_end_minus, "-");
    lv_obj_center(lbl_end_minus);
    lv_obj_set_style_text_font(lbl_end_minus,   &lv_font_montserrat_26, 0);

    // Spinbox
    ta_end_freq = lv_spinbox_create(cont_end);

    lv_obj_t * sb_text2 = lv_obj_get_child(ta_end_freq, 0);
    lv_obj_set_style_text_align(sb_text2, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_clear_flag(sb_text2, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_clear_flag(ta_end_freq, LV_OBJ_FLAG_SCROLLABLE);
    lv_spinbox_set_range(ta_end_freq, MIN_START_FREQ_HZ, MAX_STOP_FREQ_HZ);
    lv_spinbox_set_digit_format(ta_end_freq, 6, 0);
    lv_spinbox_set_step(ta_end_freq, 1);
    lv_spinbox_set_value(ta_end_freq, AppBIACfg.SweepCfg.SweepStop);
    lv_obj_set_size(ta_end_freq, 110, 35);
    lv_obj_set_style_text_font(ta_end_freq,   &lv_font_montserrat_24, 0);

    // plus (big)
    lv_obj_t * btn_end_plus = lv_btn_create(cont_end);
    
    lv_obj_set_size(btn_end_plus, 80, 35);
    lv_obj_t * lbl_end_plus = lv_label_create(btn_end_plus);
    lv_label_set_text(lbl_end_plus, "+");
    lv_obj_center(lbl_end_plus);
    lv_obj_set_style_text_font(lbl_end_plus,   &lv_font_montserrat_26, 0);

    lv_obj_t * lbl_end_unit = lv_label_create(cont_end);
    lv_label_set_text(lbl_end_unit, "Hz");

    // events
    lv_obj_add_event_cb(btn_end_minus, [](lv_event_t * e) {
        lv_spinbox_decrement(ta_end_freq);

        int start = lv_spinbox_get_value(ta_start_freq);
        int end   = lv_spinbox_get_value(ta_end_freq);
        if(end < start) lv_spinbox_set_value(ta_start_freq, end);
    }, LV_EVENT_CLICKED, NULL);

    lv_obj_add_event_cb(btn_end_plus, [](lv_event_t * e) {
        lv_spinbox_increment(ta_end_freq);

        int start = lv_spinbox_get_value(ta_start_freq);
        int end   = lv_spinbox_get_value(ta_end_freq);
        if(end < start) lv_spinbox_set_value(ta_start_freq, end);
    }, LV_EVENT_CLICKED, NULL);



    // ============================================================
    // Steps (slider)
    // ============================================================
    lv_obj_t * cont_steps = lv_obj_create(scr_param);
    lv_obj_clear_flag(cont_steps, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scroll_dir(cont_steps, LV_DIR_NONE);
    lv_obj_set_size(cont_steps, 420, 50);
    lv_obj_set_flex_flow(cont_steps, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(cont_steps, LV_FLEX_ALIGN_SPACE_BETWEEN,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    lv_obj_t * lbl_steps = lv_label_create(cont_steps);
    lv_label_set_text(lbl_steps, "Steps:");

    slider_steps = lv_slider_create(cont_steps);
    lv_slider_set_range(slider_steps, MIN_STEPS, MAX_STEPS);
    lv_slider_set_value(slider_steps, AppBIACfg.SweepCfg.SweepPoints, LV_ANIM_OFF);
    lv_obj_set_size(slider_steps, 250, 20);
    lv_obj_add_event_cb(slider_steps, steps_slider_cb, LV_EVENT_VALUE_CHANGED, NULL);

    lbl_steps_val = lv_label_create(cont_steps);
    lv_label_set_text_fmt(lbl_steps_val, "%d", AppBIACfg.SweepCfg.SweepPoints);



    // ============================================================
    // Sweep Mode
    // ============================================================
    lv_obj_t * cont_mode = lv_obj_create(scr_param);
    lv_obj_clear_flag(cont_mode, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scroll_dir(cont_mode, LV_DIR_NONE);
    lv_obj_set_size(cont_mode, 420, 50);
    lv_obj_set_flex_flow(cont_mode, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(cont_mode, LV_FLEX_ALIGN_SPACE_BETWEEN,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    lv_obj_t * lbl_mode = lv_label_create(cont_mode);
    lv_label_set_text(lbl_mode, "Mode:");

    sw_mode = lv_switch_create(cont_mode);
    lv_obj_set_size(sw_mode, 80, 35);
    lv_obj_add_state(sw_mode, LV_STATE_CHECKED);


    lv_obj_t * lbl_mode_val = lv_label_create(cont_mode);
    lv_label_set_text(lbl_mode_val, "Log");

    lv_obj_add_event_cb(sw_mode, [](lv_event_t * e){
        lv_obj_t * sw = (lv_obj_t *)lv_event_get_target_obj(e);
        lv_obj_t * parent = lv_obj_get_parent(sw);
        lv_obj_t * lbl = lv_obj_get_child(parent, 2);

        if (lv_obj_has_state(sw, LV_STATE_CHECKED))
            lv_label_set_text(lbl, "Log");
        else
            lv_label_set_text(lbl, "Linear");
    }, LV_EVENT_VALUE_CHANGED, NULL);



    // ============================================================
    // Buttons (Back / Start)
    // ============================================================
    lv_obj_t * cont_buttons = lv_obj_create(scr_param);
    lv_obj_clear_flag(cont_buttons, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scroll_dir(cont_buttons, LV_DIR_NONE);
    lv_obj_set_size(cont_buttons, 420, 50);
    lv_obj_set_flex_flow(cont_buttons, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(cont_buttons, LV_FLEX_ALIGN_SPACE_AROUND,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    lv_obj_t * btn_back = lv_btn_create(cont_buttons);
    lv_obj_set_size(btn_back, 100, 40);
    lv_obj_t * lbl_back = lv_label_create(btn_back);
    lv_label_set_text(lbl_back, "Back");
    lv_obj_center(lbl_back);
    lv_obj_add_event_cb(btn_back, [](lv_event_t * e) {
        ui_manager_set_screen(SCREEN_HOME);
    }, LV_EVENT_CLICKED, NULL);

    lv_obj_t * btn_start = lv_btn_create(cont_buttons);
    lv_obj_set_size(btn_start, 100, 40);
    lv_obj_t * lbl_start_btn = lv_label_create(btn_start);
    lv_label_set_text(lbl_start_btn, "Start");
    lv_obj_center(lbl_start_btn);

    lv_obj_add_event_cb(btn_start, [](lv_event_t * e) {

        float startVal = lv_spinbox_get_value(ta_start_freq);
        float endVal   = lv_spinbox_get_value(ta_end_freq);
        int steps      = lv_slider_get_value(slider_steps);

        CheckParamChanged(startVal, endVal, steps);

        AppBIACfg.SweepCfg.SweepStart  = startVal;
        AppBIACfg.SweepCfg.SweepStop   = endVal;
        AppBIACfg.SweepCfg.SweepPoints = steps;
        AppBIACfg.SweepCfg.SweepEn     = bTRUE;
        AppBIACfg.SweepCfg.SweepLog    = lv_obj_has_state(sw_mode, LV_STATE_CHECKED) ? bTRUE : bFALSE;

        StartImpedanceSweep();
        ui_manager_set_screen(SCREEN_RESULTS_TEXT);

    }, LV_EVENT_CLICKED, NULL);


    lv_disp_load_scr(scr_param);
    return scr_param;
}


// --------------------------------------------------
static void CheckParamChanged(float startVal, float endVal, int steps)
{
    if (startVal != AppBIACfg.SweepCfg.SweepStart ||
        endVal   != AppBIACfg.SweepCfg.SweepStop  ||
        steps    != AppBIACfg.SweepCfg.SweepPoints)
    {
        AppBIACfg.bParaChanged = bTRUE;
        AppBIACfg.ReDoRtiaCal  = bTRUE;
    }
}

void screen_parameters_update(void)
{
}

void screen_parameters_destroy(void)
{
    ta_start_freq = ta_end_freq = nullptr;
    slider_steps  = lbl_steps_val = nullptr;
    sw_mode       = nullptr;
    scr_param     = nullptr;
}

Screen_t Screen_Parameters = {
    .create  = screen_parameters_create,
    .update  = screen_parameters_update,
    .destroy = screen_parameters_destroy
};
