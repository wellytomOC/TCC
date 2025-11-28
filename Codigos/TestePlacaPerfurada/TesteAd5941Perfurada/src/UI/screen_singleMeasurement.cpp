#include "screen_singleMeasurement.h"
#include "lvgl.h"
#include "main.h"
#include "ImpedanceMeter.h"

// External global variables
extern Type_GlobalVariables GVariables;

// UI objects
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

    /* ---------------------- Back Button (BOTTOM LEFT) ---------------------- */
    lv_obj_t * btn_back = lv_btn_create(scr);
    lv_obj_set_size(btn_back, 90, 40);
    lv_obj_align(btn_back, LV_ALIGN_BOTTOM_LEFT, 10, -10);

    lv_obj_t * lbl_back = lv_label_create(btn_back);
    lv_label_set_text(lbl_back, LV_SYMBOL_LEFT " Voltar");
    lv_obj_center(lbl_back);

    lv_obj_add_event_cb(
        btn_back,
        [](lv_event_t * e)
        {
            ui_manager_set_screen(SCREEN_HOME);
            GVariables.sweep_done = false;
            StopImpedanceMeasurement();
        },
        LV_EVENT_CLICKED,
        NULL
    );


    /* ---------------------- Frequency Adjustment Buttons ---------------------- */

    static const int steps[6]  = { -10000, -1000, -100, 100, 1000, 10000 };
    static const char * labels[6] = { "-10k", "-1k", "-100", "+100", "+1k", "+10k" };

    // Increased separation between buttons
    static const int x_offsets[6] = { -180, -110, -40, 40, 110, 180 };

    for(int i = 0; i < 6; i++)
    {
        lv_obj_t * b = lv_btn_create(scr);
        lv_obj_set_size(b, 60, 40);
        lv_obj_align(b, LV_ALIGN_TOP_MID, x_offsets[i], 35);

        lv_obj_t * l = lv_label_create(b);
        lv_label_set_text(l, labels[i]);
        lv_obj_center(l);

        lv_obj_add_event_cb(
            b,
            [](lv_event_t * e)
            {
                int step = *(int *)lv_event_get_user_data(e);
                int32_t val = lv_slider_get_value(slider_freq);

                val += step;
                if(val < MIN_START_FREQ_HZ) val = MIN_START_FREQ_HZ;
                if(val > MAX_STOP_FREQ_HZ) val = MAX_STOP_FREQ_HZ;

                lv_slider_set_value(slider_freq, val, LV_ANIM_OFF);
                lv_label_set_text_fmt(label_freq_value, "%lu Hz", (unsigned long)val);
            },
            LV_EVENT_CLICKED,
            (void *)&steps[i]
        );
    }


    /* ---------------------- Frequency slider ---------------------- */
    slider_freq = lv_slider_create(scr);
    lv_obj_set_width(slider_freq, 300);
    lv_slider_set_range(slider_freq, MIN_START_FREQ_HZ, MAX_STOP_FREQ_HZ);

    // Move slider UP more
    lv_obj_align(slider_freq, LV_ALIGN_CENTER, 0, -60);

    // Frequency numeric label
    label_freq_value = lv_label_create(scr);
    lv_label_set_text_fmt(label_freq_value, "%lu Hz", (unsigned long)MIN_START_FREQ_HZ);
    lv_obj_align_to(label_freq_value, slider_freq, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);

    lv_obj_add_event_cb(
    slider_freq,
    [](lv_event_t * e)
    {
        int32_t val = lv_slider_get_value(slider_freq);

        // ---- Round to nearest 100 Hz ----
        val = (val + 50) / 100 * 100;   // arredonda para múltiplo de 100

        // Clamp nos limites
        if(val < MIN_START_FREQ_HZ) val = MIN_START_FREQ_HZ;
        if(val > MAX_STOP_FREQ_HZ) val = MAX_STOP_FREQ_HZ;

        // Force slider to snap to the rounded value
        lv_slider_set_value(slider_freq, val, LV_ANIM_OFF);

        // Update label
        lv_label_set_text_fmt(label_freq_value, "%lu Hz", (unsigned long)val);
    },
    LV_EVENT_VALUE_CHANGED,
    NULL
);


    /* ---------------------- Measure button ---------------------- */
    lv_obj_t * btn_measure = lv_btn_create(scr);
    lv_obj_set_size(btn_measure, 150, 40);

    // Move "Medir" button UP more
    lv_obj_align(btn_measure, LV_ALIGN_CENTER, 0, 10);

    lv_obj_t * lbl_measure = lv_label_create(btn_measure);
    lv_label_set_text(lbl_measure, "Medir");
    lv_obj_center(lbl_measure);

    lv_obj_add_event_cb(
        btn_measure,
        [](lv_event_t * e)
        {
            uint32_t freq = lv_slider_get_value(slider_freq);
            lv_label_set_text(label_result, "");
            StartSingleMeasurement(freq);
        },
        LV_EVENT_CLICKED,
        NULL
    );


    /* ---------------------- Results box ---------------------- */
    result_box = lv_obj_create(scr);

    // Make result box narrower
    lv_obj_set_size(result_box, 240, 80);

    // Move slightly UP (still above back button, now safe)
    lv_obj_align(result_box, LV_ALIGN_BOTTOM_MID, 0, -40);

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
    if(!GVariables.sweep_done)
        return;

    GVariables.sweep_done = false;

    char buf[64];
    float R, X, L, C;

    calculate_impedance_components(
        GVariables.MagnitudeBuffer[4],
        GVariables.PhaseBuffer[4],
        AppBIACfg.FreqofData,
        &R, &X, &L, &C
    );

    snprintf(
        buf, sizeof(buf),
        "R: %.2e Ohm\nX: %.2e Ohm\nL: %.2e H\nC: %.2e F",
        (double)R, (double)X, (double)L, (double)C
    );

    lv_label_set_text(label_result, buf);
}


void screen_singleMeasurement_destroy(void)
{
    result_box = NULL;
    label_result = NULL;
    slider_freq = NULL;
    label_freq_value = NULL;
}


Screen_t Screen_singleMeasurement =
{
    .create  = screen_singleMeasurement_create,
    .update  = screen_singleMeasurement_update,
    .destroy = screen_singleMeasurement_destroy
};
