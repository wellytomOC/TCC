#include "screen_resultsGraph.h"
#include "lvgl.h"
#include "main.h"
#include "ImpedanceMeter.h"
#include <float.h>

extern AppBIACfg_Type AppBIACfg;

static lv_obj_t * chart;
static lv_chart_series_t * series;
static lv_obj_t * btns[4];
static uint8_t selected_index = 0;

static lv_obj_t * scale_y;     // LEFT vertical scale
static lv_obj_t * scale_x;     // BOTTOM horizontal scale

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
        ui_manager_set_screen(SCREEN_RESULTS_TEXT);
    }, LV_EVENT_CLICKED, NULL);


    /* ---------- Main container for chart + scales ---------- */
    lv_obj_t * main_cont = lv_obj_create(scr);
    lv_obj_set_size(main_cont, 380, 260);
    lv_obj_align(main_cont, LV_ALIGN_TOP_RIGHT, 0, 0);
    lv_obj_set_flex_flow(main_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_clear_flag(main_cont, LV_OBJ_FLAG_SCROLLABLE);

    /* ---------- Chart + Y-scale row ---------- */
    lv_obj_t * row = lv_obj_create(main_cont);
    lv_obj_remove_style_all(row);
    lv_obj_set_size(row, lv_pct(100), lv_pct(90));
    lv_obj_align(row, LV_ALIGN_TOP_RIGHT, 50, 15);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

    /* ---------- LEFT Y-scale ---------- */
    scale_y = lv_scale_create(row);
    lv_scale_set_mode(scale_y, LV_SCALE_MODE_VERTICAL_LEFT);
    lv_obj_set_size(scale_y, 20, lv_pct(90));
    lv_obj_clear_flag(scale_y, LV_OBJ_FLAG_SCROLLABLE);

    /* ---------- Chart ---------- */
    chart = lv_chart_create(row);
    lv_obj_set_size(chart, 260, 180);
    lv_chart_set_type(chart, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(chart, GVariables.MeasurementCounter);
    series = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_BLUE), LV_CHART_AXIS_PRIMARY_Y);
    lv_obj_clear_flag(chart, LV_OBJ_FLAG_SCROLLABLE);

    /* ---------- BOTTOM X-scale ---------- */
    scale_x = lv_scale_create(main_cont);
    lv_scale_set_mode(scale_x, LV_SCALE_MODE_HORIZONTAL_BOTTOM);
    lv_obj_set_size(scale_x, lv_pct(100), 15);
    lv_obj_clear_flag(scale_x, LV_OBJ_FLAG_SCROLLABLE);

    /* ---------- Bottom buttons ---------- */
    for (int i = 0; i < 4; i++) {
        btns[i] = lv_btn_create(scr);
        lv_obj_set_size(btns[i], 100, 40);
        lv_obj_align(btns[i], LV_ALIGN_BOTTOM_LEFT, 10 + i * 115, -10);

        lv_obj_t * lbl = lv_label_create(btns[i]);
        lv_label_set_text(lbl, btn_labels[i]);
        lv_obj_center(lbl);

        lv_obj_add_event_cb(btns[i], btn_select_event_cb, LV_EVENT_CLICKED, (void *)(uintptr_t)i);
    }

    lv_obj_set_style_bg_color(btns[0], lv_palette_main(LV_PALETTE_BLUE), 0);
    for (int i = 1; i < 4; i++)
        lv_obj_set_style_bg_color(btns[i], lv_palette_main(LV_PALETTE_GREY), 0);

    plot_selected_data(0);

    lv_disp_load_scr(scr);
    return scr;
}

/* ---------------- Button event ---------------- */
static void btn_select_event_cb(lv_event_t * e)
{
    int index = (int)(uintptr_t)lv_event_get_user_data(e);
    selected_index = index;

    for (int j = 0; j < 4; j++) {
        lv_obj_set_style_bg_color(btns[j],
            j == selected_index ? lv_palette_main(LV_PALETTE_BLUE)
                                : lv_palette_main(LV_PALETTE_GREY),
            0);
    }

    plot_selected_data(selected_index);
}

/* ---------------- Plot Function ---------------- */
static void plot_selected_data(uint8_t type)
{
    if (GVariables.MeasurementCounter == 0) return;

    lv_coord_t *ser_y_points =
        (lv_coord_t *)lv_chart_get_series_y_array(chart, series);
    if (!ser_y_points) return;

    float min_val = FLT_MAX;
    float max_val = -FLT_MAX;

    /* ------------ Fill the Y data ------------- */
    for (uint32_t i = 0; i < GVariables.MeasurementCounter; ++i)
    {
        float v = 0;

        if (type == 0) {
            v = GVariables.MagnitudeBuffer[i];
        }
        else if (type == 1) {
            v = GVariables.PhaseBuffer[i] * 180.0f / M_PI;
        }
        else {
            float R, X, L, C;
            calculate_impedance_components(
                GVariables.MagnitudeBuffer[i],
                GVariables.PhaseBuffer[i],
                GVariables.FreqBuffer[i],
                &R, &X, &L, &C
            );

            if (type == 2)      v = L * 1e6f;     // µH
            else if (type == 3) v = C * 1e9f;     // nF
        }

        if (v < min_val) min_val = v;
        if (v > max_val) max_val = v;

        ser_y_points[i] = (lv_coord_t)roundf(v);
    }

    if (max_val == min_val) {
        max_val = min_val + 1.0f;
    }

    /* ----------- Y-axis limits ----------- */
    float y_min_f, y_max_f;

    if (type == 1) {     // Phase
        y_min_f = -180;
        y_max_f = 180;
    }
    else {               // Mag / L / C
        if (max_val < 0) max_val = 0;
        y_min_f = 0;
        y_max_f = max_val;
    }

    lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y,
                       (lv_coord_t)y_min_f, (lv_coord_t)y_max_f);

    /* ----------- Update LEFT scale ----------- */
    lv_scale_set_total_tick_count(scale_y, 6);
    lv_scale_set_major_tick_every(scale_y, 1);

    static char y_labels[6][20];
    static const char * y_texts[7];

    for (int i = 0; i < 6; i++) {
        float val = y_min_f + (y_max_f - y_min_f) * (i / 5.0f);

        if (type == 2)
            snprintf(y_labels[i], sizeof(y_labels[i]), "%.1f uH", val);
        else if (type == 3)
            snprintf(y_labels[i], sizeof(y_labels[i]), "%.1f nF", val);
        else
            snprintf(y_labels[i], sizeof(y_labels[i]), "%.1f", val);

        y_texts[i] = y_labels[i];
    }
    y_texts[6] = NULL;

    lv_scale_set_text_src(scale_y, y_texts);

    /* ----------- Update X-scale ----------- */
    uint32_t N = GVariables.MeasurementCounter;
    float f0 = AppBIACfg.SweepCfg.SweepStart;
    float f1 = AppBIACfg.SweepCfg.SweepStop;

    lv_scale_set_total_tick_count(scale_x, 6);
    lv_scale_set_major_tick_every(scale_x, 1);

    static char x_labels[6][16];
    static const char * x_texts[7];

    for (int i = 0; i < 6; i++)
    {
        float f;

        if (AppBIACfg.SweepCfg.SweepLog == 1)
        {
            float t = i / 5.0f;
            float logf0 = log10f(f0);
            float logf1 = log10f(f1);
            f = powf(10.0f, logf0 + t * (logf1 - logf0));
        }
        else
        {
            f = f0 + (f1 - f0) * (i / 5.0f);
        }

        if (f >= 1000)
            snprintf(x_labels[i], sizeof(x_labels[i]), "%.1fk", f / 1000.0f);
        else
            snprintf(x_labels[i], sizeof(x_labels[i]), "%.0f", f);

        x_texts[i] = x_labels[i];
    }
    x_texts[6] = NULL;

    lv_scale_set_text_src(scale_x, x_texts);

    lv_chart_refresh(chart);
}



/* ---------------- Destroy ---------------- */
void screen_resultsGraph_destroy(void)
{
    chart = nullptr;
    series = nullptr;
    scale_x = nullptr;
    scale_y = nullptr;

    for (int i = 0; i < 4; i++) btns[i] = nullptr;
    selected_index = 0;
}

/* ---------------- Screen Descriptor ---------------- */
Screen_t Screen_resultsGraph = {
    .create = screen_resultsGraph_create,
    .update = nullptr,
    .destroy = screen_resultsGraph_destroy
};
