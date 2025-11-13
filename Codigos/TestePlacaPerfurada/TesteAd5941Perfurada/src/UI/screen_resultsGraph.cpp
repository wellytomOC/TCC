#include "screen_resultsGraph.h"
#include "lvgl.h"
#include "main.h"
#include "ImpedanceMeter.h"

static lv_obj_t * chart;               // Main chart
static lv_chart_series_t * series;     // Active data series
static lv_obj_t * btn_text;            // Button to go to text screen
static lv_obj_t * btns[4];             // Magnitude, Phase, L, C buttons
static uint8_t selected_index = 0;     // Which graph is active (0=M,1=P,2=L,3=C)

static const char *btn_labels[] = {"Magnitude", "Phase", "Inductance", "Capacitance"};

static void plot_selected_data(uint8_t type);
static void btn_select_event_cb(lv_event_t * e);

/* ---------------- Create ---------------- */
lv_obj_t * screen_resultsGraph_create(void)
{
    lv_obj_t * scr = lv_obj_create(NULL);

    /* ---------- Back button ---------- */
    lv_obj_t * btn_back = lv_btn_create(scr);
    lv_obj_set_size(btn_back, 80, 40);
    lv_obj_align(btn_back, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_t * lbl_back = lv_label_create(btn_back);
    lv_label_set_text(lbl_back, "Back");
    lv_obj_center(lbl_back);
    lv_obj_add_event_cb(btn_back, [](lv_event_t * e) {
        ui_manager_set_screen(SCREEN_HOME);
    }, LV_EVENT_CLICKED, NULL);

    /* ---------- Text button ---------- */
    btn_text = lv_btn_create(scr);
    lv_obj_set_size(btn_text, 80, 40);
    lv_obj_align(btn_text, LV_ALIGN_TOP_RIGHT, -10, 10);
    lv_obj_t * lbl_text = lv_label_create(btn_text);
    lv_label_set_text(lbl_text, "Text");
    lv_obj_center(lbl_text);
    lv_obj_add_event_cb(btn_text, [](lv_event_t * e) {
        ui_manager_set_screen(SCREEN_RESULTS_TEXT);
    }, LV_EVENT_CLICKED, NULL);

    /* ---------- Chart ---------- */
    chart = lv_chart_create(scr);
    lv_obj_set_size(chart, 300, 180);
    lv_obj_align(chart, LV_ALIGN_CENTER, 0, 0);
    lv_chart_set_type(chart, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(chart, GVariables.MeasurementCounter);
    lv_chart_set_div_line_count(chart, 4, 4);
    lv_chart_set_update_mode(chart, LV_CHART_UPDATE_MODE_CIRCULAR);
    series = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_BLUE), LV_CHART_AXIS_PRIMARY_Y);

    /* ---------- Bottom buttons ---------- */
    for (int i = 0; i < 4; i++) {
        btns[i] = lv_btn_create(scr);
        lv_obj_set_size(btns[i], 100, 40);
        lv_obj_align(btns[i], LV_ALIGN_BOTTOM_LEFT, 10 + i * 90, -10);

        lv_obj_t * lbl = lv_label_create(btns[i]);
        lv_label_set_text(lbl, btn_labels[i]);
        lv_obj_center(lbl);

        lv_obj_add_event_cb(btns[i], btn_select_event_cb, LV_EVENT_CLICKED, (void *)(uintptr_t)i);
    }

    // Default selected: Magnitude
    lv_obj_set_style_bg_color(btns[0], lv_palette_main(LV_PALETTE_BLUE), 0);
    for (int i = 1; i < 4; i++)
        lv_obj_set_style_bg_color(btns[i], lv_palette_main(LV_PALETTE_GREY), 0);

    // Initial plot
    plot_selected_data(0);

    lv_disp_load_scr(scr);
    return scr;
}

/* ---------------- Event callback ---------------- */
static void btn_select_event_cb(lv_event_t * e)
{
    lv_obj_t * btn = (lv_obj_t *)lv_event_get_target(e);

    int index = (int)(uintptr_t)lv_event_get_user_data(e);

    selected_index = index;

    // Update button colors
    for (int j = 0; j < 4; j++) {
        lv_obj_set_style_bg_color(btns[j],
            j == selected_index ? lv_palette_main(LV_PALETTE_BLUE)
                                : lv_palette_main(LV_PALETTE_GREY),
            0);
    }

    plot_selected_data(selected_index);
}

/* ---------------- Update ---------------- */
void screen_resultsGraph_update(void)
{
    // Optional: dynamically refresh if you want live plotting
}

/* ---------------- Destroy ---------------- */
void screen_resultsGraph_destroy(void)
{
    chart = nullptr;
    series = nullptr;
    btn_text = nullptr;
    for (int i = 0; i < 4; i++) btns[i] = nullptr;
    selected_index = 0;
}

/* ---------------- Plot Function ---------------- */
static void plot_selected_data(uint8_t type)
{
    if (GVariables.MeasurementCounter == 0) return;

    lv_chart_set_point_count(chart, GVariables.MeasurementCounter);

    for (uint32_t i = 0; i < GVariables.MeasurementCounter; i++)
    {
        float y_val = 0.0f;

        if (type == 0) { // Magnitude
            y_val = GVariables.MagnitudeBuffer[i];
        }
        else if (type == 1) { // Phase
            y_val = GVariables.PhaseBuffer[i];
        }
        else {
            float R, X, L, C;
            calculate_impedance_components(GVariables.MagnitudeBuffer[i],
                                           GVariables.PhaseBuffer[i],
                                           GVariables.FreqBuffer[i],
                                           &R, &X, &L, &C);
            if (type == 2) y_val = L;
            else if (type == 3) y_val = C;
        }

        lv_chart_set_value_by_id(chart, series, i, (lv_coord_t)y_val);
    }

    // Adjust Y-axis range dynamically
    switch (type)
    {
        case 0: lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, 0, 1e6); break;     // Mag
        case 1: lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, -180, 180); break;  // Phase
        case 2: lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, 0, 1e-3); break;    // L (H)
        case 3: lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, 0, 1e-6); break;    // C (F)
    }

    lv_chart_refresh(chart);
}

/* ---------------- Screen Descriptor ---------------- */
Screen_t Screen_resultsGraph = {
    .create = screen_resultsGraph_create,
    .update = screen_resultsGraph_update,
    .destroy = screen_resultsGraph_destroy
};
