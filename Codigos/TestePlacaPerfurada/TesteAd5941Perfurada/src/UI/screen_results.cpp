#include "screen_results.h"
#include "screen_parameters.h"
#include "lvgl.h"
#include "main.h"
#include <math.h>  // for log10f()

extern Type_GlobalVariables GVariables;

// --- Static elements ---
static lv_obj_t * scr_results = nullptr;
static lv_obj_t * chart = nullptr;
static lv_chart_series_t * ser_mag = nullptr;
static lv_chart_series_t * ser_phase = nullptr;


static int32_t freq_data[MAX_STEPS];
static int32_t mag_data[MAX_STEPS];
static int32_t phase_data[MAX_STEPS];
static int data_count = 0;

static uint32_t MagRangeMax = 100, MagRangeMin = 0;

// --------------------------------------------------
// Create the results screen
// --------------------------------------------------
lv_obj_t * screen_results_create(void)
{
    scr_results = lv_obj_create(NULL);
    lv_obj_clear_flag(scr_results, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(scr_results, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(scr_results, 10, 0);

    // --- Title ---
    lv_obj_t * label_title = lv_label_create(scr_results);
    lv_label_set_text(label_title, "Measurement Results");
    lv_obj_set_style_text_font(label_title, &lv_font_montserrat_20, 0);
    lv_obj_align(label_title, LV_ALIGN_TOP_MID, 0, 10);



    // --- Chart ---
    chart = lv_chart_create(scr_results);
    lv_obj_set_size(chart, 300, 200);
    lv_obj_center(chart);
    lv_chart_set_type(chart, LV_CHART_TYPE_LINE);

    // Set number of points
    lv_chart_set_point_count(chart, AppBIACfg.SweepCfg.SweepPoints);

    // Create two series: magnitude and phase
    ser_mag   = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_BLUE), LV_CHART_AXIS_PRIMARY_Y);
    ser_phase = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_RED),  LV_CHART_AXIS_SECONDARY_Y);

    lv_chart_set_ext_y_array(chart, ser_mag, mag_data);
    lv_chart_set_ext_y_array(chart, ser_phase, phase_data);

    // Configure Y axes
    lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, MagRangeMin, MagRangeMax);   // magnitude range
    lv_chart_set_range(chart, LV_CHART_AXIS_SECONDARY_Y, -180, 180); // phase range



    // --- Back button ---
    lv_obj_t * btn_back = lv_btn_create(scr_results);
    lv_obj_align(btn_back, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_t * lbl_back = lv_label_create(btn_back);
    lv_label_set_text(lbl_back, "Back");
    lv_obj_center(lbl_back);
    lv_obj_add_event_cb(btn_back, [](lv_event_t * e) {
        ui_manager_set_screen(SCREEN_HOME);
        GVariables.sweep_done = false;
    }, LV_EVENT_CLICKED, NULL);

    // Clear data arrays
    data_count = 0;
    memset(freq_data, 0, sizeof(freq_data));
    memset(mag_data, 0, sizeof(mag_data));
    memset(phase_data, 0, sizeof(phase_data));

    lv_disp_load_scr(scr_results);
    GVariables.sweep_ready = true;
    return scr_results;
}

// --------------------------------------------------
// Add new data point dynamically
// --------------------------------------------------
static bool chart_needs_refresh = false;
void screen_results_add_point(int freq, int mag, int phase)
{
    if (!chart) return;
    if (data_count >= MAX_STEPS) return;

    freq_data[data_count]  = freq;
    mag_data[data_count]   = mag;
    phase_data[data_count] = phase;
    data_count++;

    //Increase mag range if needed
    if (mag > MagRangeMax) {
        MagRangeMax = (uint32_t)(mag + mag/10); // add 10% headroom
        lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, MagRangeMin, MagRangeMax);
    }

    //Update chart
    chart_needs_refresh = true;
}

// --------------------------------------------------
// Update chart every loop
// --------------------------------------------------
void screen_results_update(void)
{
    if (chart_needs_refresh) {
        lv_chart_refresh(chart);
        chart_needs_refresh = false;
    }
}



// --------------------------------------------------
// Destroy screen
// --------------------------------------------------
void screen_results_destroy(void)
{
    scr_results = nullptr;
    chart = nullptr;
    ser_mag = nullptr;
    ser_phase = nullptr;
    data_count = 0;
    GVariables.sweep_ready = false;
}

// --------------------------------------------------
// Screen descriptor
// --------------------------------------------------
Screen_t Screen_Results = {
    .create = screen_results_create,
    .update = screen_results_update,
    .destroy = screen_results_destroy
};
