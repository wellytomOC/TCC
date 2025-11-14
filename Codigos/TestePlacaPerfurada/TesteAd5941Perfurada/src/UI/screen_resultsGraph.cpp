#include "screen_resultsGraph.h"
#include "lvgl.h"
#include "main.h"
#include "ImpedanceMeter.h"
#include <float.h>

extern AppBIACfg_Type AppBIACfg;

/* --------------------------------------------------------- */
/*                    STATIC OBJECTS                         */
/* --------------------------------------------------------- */
static lv_obj_t * chart;
static lv_chart_series_t * series;
static lv_obj_t * btns[4];
static uint8_t selected_index = 0;

static lv_obj_t * scale_y;     // Left vertical scale
static lv_obj_t * scale_x;     // Bottom horizontal scale

static const char *btn_labels[] = {
    "Magnitude", "Phase", "Induc.(uH)", "Capacit.(nF)"
};

/* --------------------------------------------------------- */
static void plot_selected_data(uint8_t type);
static void btn_select_event_cb(lv_event_t * e);

/* --------------------------------------------------------- */
/*                     CREATE SCREEN                         */
/* --------------------------------------------------------- */
lv_obj_t * screen_resultsGraph_create(void)
{
    lv_obj_t * scr = lv_obj_create(NULL);

    /* Root container (vertical) */
    lv_obj_t * root = lv_obj_create(scr);
    lv_obj_remove_style_all(root);
    lv_obj_set_size(root, lv_pct(100), lv_pct(100));
    lv_obj_set_flex_flow(root, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_grow(root, 1);

    /* ---------------- Graph Area ---------------- */
    lv_obj_t * graph_area = lv_obj_create(root);
    lv_obj_remove_style_all(graph_area);
    lv_obj_set_width(graph_area, lv_pct(100));
    lv_obj_set_flex_grow(graph_area, 1);
    lv_obj_set_flex_flow(graph_area, LV_FLEX_FLOW_COLUMN);

    /* ▣  Row: Y-scale + Chart  */
    lv_obj_t * row = lv_obj_create(graph_area);
    lv_obj_remove_style_all(row);
    lv_obj_set_width(row, lv_pct(100));
    lv_obj_set_height(row, lv_pct(100));
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_grow(row, 1);

    /* --- Y SCALE (left side) --- */
    scale_y = lv_scale_create(row);
    lv_scale_set_mode(scale_y, LV_SCALE_MODE_VERTICAL_LEFT);
    lv_obj_set_width(scale_y, 48);  // More width to avoid clipping
    lv_obj_set_style_height(scale_y, LV_PCT(100), 0);
    lv_obj_set_style_text_font(scale_y, &lv_font_montserrat_10, 0);

    /* safe padding */
    lv_obj_set_style_pad_left(scale_y, 10, 0);
    lv_obj_set_style_pad_right(scale_y, 6, 0);
    lv_obj_set_style_pad_top(scale_y, 6, 0);
    lv_obj_set_style_pad_bottom(scale_y, 4, 0);

    /* --- CHART --- */
    chart = lv_chart_create(row);
    lv_obj_set_flex_grow(chart, 1);
    lv_chart_set_type(chart, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(chart, GVariables.MeasurementCounter);

    series = lv_chart_add_series(chart,
            lv_palette_main(LV_PALETTE_BLUE),
            LV_CHART_AXIS_PRIMARY_Y);

    lv_obj_set_style_pad_all(chart, 6, 0);

    /* --- X SCALE (bottom) --- */
    scale_x = lv_scale_create(graph_area);
    lv_scale_set_mode(scale_x, LV_SCALE_MODE_HORIZONTAL_BOTTOM);
    lv_obj_set_width(scale_x, lv_pct(98));
    lv_obj_set_height(scale_x, 28);
    lv_obj_set_style_margin_left(scale_x, 52, 0);
    lv_obj_set_style_text_font(scale_x, &lv_font_montserrat_10, 0);

    /* padding for clear labels */
    //lv_obj_set_style_pad_left(scale_x, , 0);
    lv_obj_set_style_pad_right(scale_x, 1, 0);
    lv_obj_set_style_pad_top(scale_x, 4, 0);
    lv_obj_set_style_pad_bottom(scale_x, 6, 0);

    /* ---------------- Button Bar ---------------- */
    lv_obj_t * btn_row = lv_obj_create(root);
    lv_obj_remove_style_all(btn_row);
    lv_obj_set_width(btn_row, lv_pct(100));
    lv_obj_set_height(btn_row, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(btn_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(btn_row,
                          LV_FLEX_ALIGN_SPACE_EVENLY,
                          LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);

    /* Back button */
    lv_obj_t * btn_back = lv_btn_create(btn_row);
    lv_obj_set_size(btn_back, lv_pct(20), 40);
    lv_obj_t * lbl_back = lv_label_create(btn_back);
    lv_label_set_text(lbl_back, "Back");
    lv_obj_center(lbl_back);
    lv_obj_add_event_cb(btn_back, [](lv_event_t * e) {
        ui_manager_set_screen(SCREEN_RESULTS_TEXT);
    }, LV_EVENT_CLICKED, NULL);

    /* Data type selector buttons */
    for (int i = 0; i < 4; i++) {
        btns[i] = lv_btn_create(btn_row);
        lv_obj_set_size(btns[i], lv_pct(20), 40);

        lv_obj_t * lbl = lv_label_create(btns[i]);
        lv_label_set_text(lbl, btn_labels[i]);
        lv_obj_center(lbl);

        lv_obj_add_event_cb(btns[i], btn_select_event_cb,
                            LV_EVENT_CLICKED, (void *)(uintptr_t)i);
    }

    /* Highlight initial button */
    for (int i = 0; i < 4; i++) {
        lv_obj_set_style_bg_color(btns[i],
            i == 0 ? lv_palette_main(LV_PALETTE_BLUE)
                   : lv_palette_main(LV_PALETTE_GREY),
            0);
    }

    /* Initial plot */
    plot_selected_data(0);

    lv_disp_load_scr(scr);
    return scr;
}

/* --------------------------------------------------------- */
/*                  BUTTON SELECTION                         */
/* --------------------------------------------------------- */
static void btn_select_event_cb(lv_event_t * e)
{
    int index = (int)(uintptr_t)lv_event_get_user_data(e);
    selected_index = index;

    for (int j = 0; j < 4; j++) {
        lv_obj_set_style_bg_color(btns[j],
            j == selected_index
                ? lv_palette_main(LV_PALETTE_BLUE)
                : lv_palette_main(LV_PALETTE_GREY),
            0);
    }

    plot_selected_data(selected_index);
}

/* --------------------------------------------------------- */
/*                       PLOTTING                            */
/* --------------------------------------------------------- */
static void plot_selected_data(uint8_t type)
{
    if (GVariables.MeasurementCounter == 0) return;

    lv_coord_t *ser_y_points =
        (lv_coord_t *)lv_chart_get_series_y_array(chart, series);

    if (!ser_y_points) return;

    float min_val = FLT_MAX;
    float max_val = -FLT_MAX;

    /* ---- Fill series ---- */
    for (uint32_t i = 0; i < GVariables.MeasurementCounter; i++)
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

            if (type == 2) v = L * 1e6f;   // µH
            else           v = C * 1e9f;   // nF
        }

        if (v < min_val) min_val = v;
        if (v > max_val) max_val = v;

        ser_y_points[i] = (lv_coord_t)roundf(v);
    }

    if (max_val == min_val)
        max_val = min_val + 1;

    /* ---- Y range ---- */
    float y_min, y_max;
    if (type == 1) {
        y_min = -180;
        y_max = 180;
    } else {
        y_min = 0;
        y_max = max_val;
    }

    lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y,
                       (lv_coord_t)y_min, (lv_coord_t)y_max);

    /* ---- Y labels ---- */
    static char y_labels[11][20];
    static const char *y_texts[12];

    for (int i = 0; i < 11; i++) {
        float val = y_min + (y_max - y_min) * (i / 10.0f);

        snprintf(y_labels[i], sizeof(y_labels[i]), "%.1f", val);

        y_texts[i] = y_labels[i];
    }
    y_texts[11] = NULL;

    lv_scale_set_total_tick_count(scale_y, 11);
    lv_scale_set_major_tick_every(scale_y, 1);
    lv_scale_set_text_src(scale_y, y_texts);

    /* ---- X labels ---- */
    static char x_labels[11][16];
    static const char *x_texts[12];

    float f0 = AppBIACfg.SweepCfg.SweepStart;
    float f1 = AppBIACfg.SweepCfg.SweepStop;

    for (int i = 0; i < 11; i++) {
        float f;

        if (AppBIACfg.SweepCfg.SweepLog == 1) {
            float t = i / 10.0f;
            f = powf(10.0f,
                     log10f(f0) + t * (log10f(f1) - log10f(f0)));
        } else {
            f = f0 + (f1 - f0) * (i / 10.0f);
        }

        if (f >= 1000)
            snprintf(x_labels[i], sizeof(x_labels[i]), "%.1fk", f / 1000.0f);
        else
            snprintf(x_labels[i], sizeof(x_labels[i]), "%.0f", f);

        x_texts[i] = x_labels[i];
    }
    x_texts[11] = NULL;
    lv_scale_set_total_tick_count(scale_x, 11);
    lv_scale_set_major_tick_every(scale_x, 1);
    lv_scale_set_text_src(scale_x, x_texts);

    lv_chart_refresh(chart);
}

/* --------------------------------------------------------- */
void screen_resultsGraph_destroy(void)
{
    chart = NULL;
    series = NULL;
    scale_x = NULL;
    scale_y = NULL;

    for (int i = 0; i < 4; i++)
        btns[i] = NULL;

    selected_index = 0;
}

/* --------------------------------------------------------- */
Screen_t Screen_resultsGraph = {
    .create = screen_resultsGraph_create,
    .update = NULL,
    .destroy = screen_resultsGraph_destroy
};
