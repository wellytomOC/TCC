#include "screen_results.h"
#include "lvgl.h"
#include "main.h"
#include <math.h>  // for log10f()

extern Type_GlobalVariables GVariables;

// --- Static elements ---
static lv_obj_t * scr_results = nullptr;
static lv_obj_t * chart = nullptr;
static lv_chart_series_t * ser_mag = nullptr;
static lv_chart_series_t * ser_phase = nullptr;

#define MAX_POINTS 200  // max number of measurement points

static float freq_data[MAX_POINTS];
static float mag_data[MAX_POINTS];
static float phase_data[MAX_POINTS];
static int data_count = 0;

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

    // X axis = frequency (log scale approximation)
    lv_chart_set_point_count(chart, MAX_POINTS);

    // Create two series: magnitude and phase
    ser_mag   = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_BLUE), LV_CHART_AXIS_PRIMARY_Y);
    ser_phase = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_RED),  LV_CHART_AXIS_SECONDARY_Y);

    // Configure Y axes
    lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, 0, 100);   // magnitude range
    lv_chart_set_range(chart, LV_CHART_AXIS_SECONDARY_Y, -180, 180); // phase range

    // --- Back button ---
    lv_obj_t * btn_back = lv_btn_create(scr_results);
    lv_obj_align(btn_back, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_t * lbl_back = lv_label_create(btn_back);
    lv_label_set_text(lbl_back, "Back");
    lv_obj_center(lbl_back);
    lv_obj_add_event_cb(btn_back, [](lv_event_t * e) {
        ui_manager_set_screen(SCREEN_HOME);
    }, LV_EVENT_CLICKED, NULL);

    // Clear data arrays
    data_count = 0;
    memset(freq_data, 0, sizeof(freq_data));
    memset(mag_data, 0, sizeof(mag_data));
    memset(phase_data, 0, sizeof(phase_data));

    lv_disp_load_scr(scr_results);
    return scr_results;
}

// --------------------------------------------------
// Add new data point dynamically
// --------------------------------------------------
void screen_results_add_point(float freq, float mag, float phase)
{
    if (data_count >= MAX_POINTS) return;

    freq_data[data_count]  = freq;
    mag_data[data_count]   = mag;
    phase_data[data_count] = phase;
    data_count++;
}

// --------------------------------------------------
// Update chart every loop
// --------------------------------------------------
void screen_results_update(void)
{
    if (!chart || data_count == 0) return;

    // Use log10(freq) to map X axis spacing
    float log_f0 = log10f(freq_data[0]);
    float log_fN = log10f(freq_data[data_count - 1]);
    float range = log_fN - log_f0;
    if (range <= 0.0f) range = 1.0f;  // avoid div by zero

    // Prepare temporary arrays to fill chart
    static lv_coord_t mag_points[MAX_POINTS];
    static lv_coord_t phase_points[MAX_POINTS];

    // Initialize all points as "none"
    for (int i = 0; i < MAX_POINTS; i++) {
        mag_points[i] = LV_CHART_POINT_NONE;
        phase_points[i] = LV_CHART_POINT_NONE;
    }

    // Map each measurement to chart position
    for (int i = 0; i < data_count; i++) {
        float x = (log10f(freq_data[i]) - log_f0) / range * (MAX_POINTS - 1);
        int idx = (int)roundf(x);
        if (idx < 0) idx = 0;
        if (idx >= MAX_POINTS) idx = MAX_POINTS - 1;

        mag_points[idx] = (lv_coord_t)mag_data[i];
        phase_points[idx] = (lv_coord_t)phase_data[i];
    }

    // Push new data to LVGL chart (without shifting)
    lv_chart_set_ext_y_array(chart, ser_mag, mag_points);
    lv_chart_set_ext_y_array(chart, ser_phase, phase_points);

    lv_chart_refresh(chart);
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
}

// --------------------------------------------------
// Screen descriptor
// --------------------------------------------------
Screen_t Screen_Results = {
    .create = screen_results_create,
    .update = screen_results_update,
    .destroy = screen_results_destroy
};
