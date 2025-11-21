#include "screen_parameters.h"
#include "lvgl.h"
#include "main.h"
#include "ImpedanceMeter.h"

extern Type_GlobalVariables GVariables;

// ==== Screen-local widgets ====
static lv_obj_t * ta_start_freq = nullptr;
static lv_obj_t * ta_end_freq   = nullptr;
static lv_obj_t * slider_steps  = nullptr;  
static lv_obj_t * lbl_steps_val = nullptr; 
static lv_obj_t * scr_param     = nullptr;

static lv_obj_t * input_panel   = nullptr;
static lv_obj_t * input_field   = nullptr;
static lv_obj_t * active_textarea = nullptr;
static lv_obj_t * sw_mode = nullptr; 


// ==== Forward declarations ====
static void input_panel_create(lv_obj_t * target);
static void input_panel_close(void);
static void keyboard_event_cb(lv_event_t * e);
static void CheckParamChanged(float startVal, float endVal, int steps);

// --------------------------------------------------
// Create a floating numeric keyboard + mirror input
// --------------------------------------------------
static void input_panel_create(lv_obj_t * target)
{
    if (input_panel) lv_obj_del(input_panel);

    active_textarea = target;

    input_panel = lv_obj_create(lv_layer_top());
    lv_obj_remove_style_all(input_panel);
    lv_obj_set_size(input_panel, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(input_panel, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(input_panel, 128, 0);
    lv_obj_clear_flag(input_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(input_panel, LV_OBJ_FLAG_CLICKABLE);

    input_field = lv_textarea_create(input_panel);
    lv_textarea_set_one_line(input_field, true);
    lv_obj_set_width(input_field, LV_PCT(95));
    lv_obj_set_height(input_field, 50);
    lv_obj_align(input_field, LV_ALIGN_TOP_MID, 0, 5);
    lv_textarea_set_text(input_field, "");
    lv_obj_set_style_text_font(input_field, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_align(input_field, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_add_flag(input_field, LV_OBJ_FLAG_CLICK_FOCUSABLE);

    lv_obj_t * keyboard = lv_keyboard_create(input_panel);
    lv_keyboard_set_mode(keyboard, LV_KEYBOARD_MODE_NUMBER);
    lv_keyboard_set_textarea(keyboard, input_field);
    lv_obj_add_event_cb(keyboard, keyboard_event_cb, LV_EVENT_ALL, target);

    lv_obj_set_size(keyboard, LV_PCT(100), LV_PCT(80));
    lv_obj_align(keyboard, LV_ALIGN_BOTTOM_MID, 0, 0);
}

// --------------------------------------------------
static void input_panel_close(void)
{
    if (input_panel) {
        lv_obj_del(input_panel);
        input_panel = nullptr;
        input_field = nullptr;
        active_textarea = nullptr;
    }
}

// --------------------------------------------------
static void keyboard_event_cb(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * target = (lv_obj_t *)lv_event_get_user_data(e);

    if (code == LV_EVENT_READY) {
        const char * txt = lv_textarea_get_text(input_field);
        int val = atoi(txt);

        if (target == ta_start_freq) {
            if (val < MIN_START_FREQ_HZ) val = MIN_START_FREQ_HZ;
            int endVal = atoi(lv_textarea_get_text(ta_end_freq));
            if (endVal > 0 && val > endVal) val = endVal;
        }
        else if (target == ta_end_freq) {
            if (val > MAX_STOP_FREQ_HZ) val = MAX_STOP_FREQ_HZ;
            int startVal = atoi(lv_textarea_get_text(ta_start_freq));
            if (startVal > 0 && val < startVal) val = startVal;
        }

        char buf[16];
        snprintf(buf, sizeof(buf), "%d", val);
        lv_textarea_set_text(target, buf);
        input_panel_close();
    }
    else if (code == LV_EVENT_CANCEL) {
        input_panel_close();
    }
}

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

    // --- Start frequency ---
    lv_obj_t * cont_start = lv_obj_create(scr_param);
    lv_obj_set_size(cont_start, 280, 50);
    lv_obj_set_flex_flow(cont_start, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(cont_start, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(cont_start, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t * lbl_start = lv_label_create(cont_start);
    lv_label_set_text(lbl_start, "Start:");

    ta_start_freq = lv_textarea_create(cont_start);
    lv_textarea_set_one_line(ta_start_freq, true);
    lv_textarea_set_text(ta_start_freq, String(AppBIACfg.SweepCfg.SweepStart).c_str());
    lv_obj_set_width(ta_start_freq, 100);

    lv_obj_t * lbl_start_unit = lv_label_create(cont_start);
    lv_label_set_text(lbl_start_unit, "Hz");

    // --- End frequency ---
    lv_obj_t * cont_end = lv_obj_create(scr_param);
    lv_obj_set_size(cont_end, 280, 50);
    lv_obj_set_flex_flow(cont_end, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(cont_end, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(cont_end, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t * lbl_end = lv_label_create(cont_end);
    lv_label_set_text(lbl_end, "End:");

    ta_end_freq = lv_textarea_create(cont_end);
    lv_textarea_set_one_line(ta_end_freq, true);
    lv_textarea_set_text(ta_end_freq, String(AppBIACfg.SweepCfg.SweepStop).c_str());
    lv_obj_set_width(ta_end_freq, 100);

    lv_obj_t * lbl_end_unit = lv_label_create(cont_end);
    lv_label_set_text(lbl_end_unit, "Hz");

    // --- Steps ---
    lv_obj_t * cont_steps = lv_obj_create(scr_param);
    lv_obj_set_size(cont_steps, 280, 50);
    lv_obj_set_flex_flow(cont_steps, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(cont_steps, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(cont_steps, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t * lbl_steps = lv_label_create(cont_steps);
    lv_label_set_text(lbl_steps, "Steps:");

    slider_steps = lv_slider_create(cont_steps);
    lv_slider_set_range(slider_steps, MIN_STEPS, MAX_STEPS);
    lv_slider_set_value(slider_steps, 10, LV_ANIM_OFF);
    lv_obj_set_width(slider_steps, 150);
    lv_obj_add_event_cb(slider_steps, steps_slider_cb, LV_EVENT_VALUE_CHANGED, NULL);

    lbl_steps_val = lv_label_create(cont_steps);
    lv_label_set_text(lbl_steps_val, "10");

    // --- Sweep Mode (Linear / Logarithmic) ---
    lv_obj_t * cont_mode = lv_obj_create(scr_param);
    lv_obj_set_size(cont_mode, 280, 50);
    lv_obj_set_flex_flow(cont_mode, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(cont_mode, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(cont_mode, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t * lbl_mode = lv_label_create(cont_mode);
    lv_label_set_text(lbl_mode, "Mode:");

    sw_mode = lv_switch_create(cont_mode);
    // default = Linear (OFF)
    lv_obj_add_state(sw_mode, LV_STATE_DEFAULT);

    // Label showing current mode
    lv_obj_t * lbl_mode_val = lv_label_create(cont_mode);
    lv_label_set_text(lbl_mode_val, "Linear");

    // Update mode label when user toggles
    lv_obj_add_event_cb(sw_mode, [](lv_event_t * e){
        lv_obj_t * sw = (lv_obj_t *)lv_event_get_target(e);
        lv_obj_t * parent = lv_obj_get_parent(sw);
        lv_obj_t * lbl = lv_obj_get_child(parent, 2); // label is 3rd element

        if (lv_obj_has_state(sw, LV_STATE_CHECKED))
            lv_label_set_text(lbl, "Log");
        else
            lv_label_set_text(lbl, "Linear");
    }, LV_EVENT_VALUE_CHANGED, NULL);




    // --- Buttons ---
    lv_obj_t * cont_buttons = lv_obj_create(scr_param);
    lv_obj_set_size(cont_buttons, 280, 50);
    lv_obj_set_flex_flow(cont_buttons, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(cont_buttons, LV_FLEX_ALIGN_SPACE_AROUND, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(cont_buttons, LV_OBJ_FLAG_SCROLLABLE);

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

        float startVal = atof(lv_textarea_get_text(ta_start_freq));
        float endVal   = atof(lv_textarea_get_text(ta_end_freq));
        int steps      = lv_slider_get_value(slider_steps);   // <<< CHANGED HERE

        CheckParamChanged(startVal, endVal, steps);

        AppBIACfg.SweepCfg.SweepStart  = startVal;
        AppBIACfg.SweepCfg.SweepStop   = endVal;
        AppBIACfg.SweepCfg.SweepPoints = steps;
        AppBIACfg.SweepCfg.SweepEn     = bTRUE;
        AppBIACfg.SweepCfg.SweepLog = lv_obj_has_state(sw_mode, LV_STATE_CHECKED) ? bTRUE : bFALSE;

        StartImpedanceSweep();
        ui_manager_set_screen(SCREEN_RESULTS_TEXT);

    }, LV_EVENT_CLICKED, NULL);

    // only start/end have keyboard
    auto open_input_cb = [](lv_event_t * e) {
        lv_obj_t * ta = (lv_obj_t *)lv_event_get_target(e);
        input_panel_create(ta);
    };
    lv_obj_add_event_cb(ta_start_freq, open_input_cb, LV_EVENT_FOCUSED, NULL);
    lv_obj_add_event_cb(ta_end_freq,   open_input_cb, LV_EVENT_FOCUSED, NULL);

    lv_disp_load_scr(scr_param);
    return scr_param;
}

static void CheckParamChanged(float startVal, float endVal, int steps)
{
    if (startVal != AppBIACfg.SweepCfg.SweepStart ||
        endVal   != AppBIACfg.SweepCfg.SweepStop  ||
        steps    != AppBIACfg.SweepCfg.SweepPoints)
    {
        AppBIACfg.bParaChanged = bTRUE;
        AppBIACfg.ReDoRtiaCal = bTRUE;
    }
}

void screen_parameters_update(void)
{
}

void screen_parameters_destroy(void)
{
    ta_start_freq = ta_end_freq = nullptr;
    slider_steps = lbl_steps_val = nullptr;
    sw_mode = nullptr;

    scr_param = nullptr;
    input_panel_close();
}

Screen_t Screen_Parameters = {
    .create = screen_parameters_create,
    .update = screen_parameters_update,
    .destroy = screen_parameters_destroy
};
