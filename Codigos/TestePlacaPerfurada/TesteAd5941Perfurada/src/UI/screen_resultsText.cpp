#include "screen_resultsText.h"
#include "lvgl.h"
#include "main.h"
#include "ImpedanceMeter.h"

static lv_obj_t * table_results;
static lv_obj_t * progress_bar;
static lv_obj_t * btn_graph;

static uint32_t last_counter = 0;

/* Forward */
static void add_measurement_row(uint32_t index);


/* ============================================================
 *  CREATE SCREEN
 * ============================================================
 */
lv_obj_t * screen_resultsText_create(void)
{
    lv_obj_t * scr = lv_obj_create(NULL);

    /* ---------------- Back Button ---------------- */
    lv_obj_t * btn_back = lv_btn_create(scr);
    lv_obj_set_size(btn_back, 80, 40);
    lv_obj_align(btn_back, LV_ALIGN_BOTTOM_LEFT, 10, -5);
    lv_obj_t * lbl_back = lv_label_create(btn_back);
    lv_label_set_text(lbl_back, "Back");
    lv_obj_center(lbl_back);

    lv_obj_add_event_cb(btn_back, [](lv_event_t * e) {
        ui_manager_set_screen(SCREEN_HOME);
        GVariables.sweep_done = false;
    }, LV_EVENT_CLICKED, NULL);

    /* ---------------- Graph Button ---------------- */
    btn_graph = lv_btn_create(scr);
    lv_obj_set_size(btn_graph, 80, 40);
    lv_obj_align(btn_graph, LV_ALIGN_BOTTOM_RIGHT, -10, -5);

    lv_obj_t * lbl_graph = lv_label_create(btn_graph);
    lv_label_set_text(lbl_graph, "Graph");
    lv_obj_center(lbl_graph);

    lv_obj_add_event_cb(btn_graph, [](lv_event_t * e) {
        ui_manager_set_screen(SCREEN_RESULTSGRAPH);
    }, LV_EVENT_CLICKED, NULL);

    lv_obj_add_flag(btn_graph, LV_OBJ_FLAG_HIDDEN); // hidden until sweep finished

    /* ---------------- TABLE ---------------- */
    table_results = lv_table_create(scr);

    // smaller width for more data
    lv_obj_set_size(table_results, lv_pct(95), 250);
    lv_obj_align(table_results, LV_ALIGN_TOP_MID, 0, 10);

    lv_obj_set_style_border_width(table_results, 1, 0);
    lv_obj_set_style_text_font(table_results, &lv_font_montserrat_12, 0);

    lv_table_set_col_cnt(table_results, 4);
    lv_table_set_row_cnt(table_results, 1);

    // Tight column widths (adjust as needed)
    lv_table_set_col_width(table_results, 0, 100);   // Freq
    lv_table_set_col_width(table_results, 1, 100);   // R
    lv_table_set_col_width(table_results, 2, 100);   // X
    lv_table_set_col_width(table_results, 3, 100);   // L/C

    /* Reduce the height of each row */
    static lv_style_t style_cell;
    lv_style_init(&style_cell);
    lv_style_set_pad_top(&style_cell, 0);
    lv_style_set_pad_bottom(&style_cell, 0);
    lv_style_set_pad_left(&style_cell, 2);
    lv_style_set_pad_right(&style_cell, 2);

    lv_obj_add_style(table_results, &style_cell, LV_PART_ITEMS);

    lv_table_set_cell_value(table_results, 0, 0, "Freq (Hz)");
    lv_table_set_cell_value(table_results, 0, 1, "R (Ohm)");
    lv_table_set_cell_value(table_results, 0, 2, "X (Ohm)");
    lv_table_set_cell_value(table_results, 0, 3, "L/C (H/F)");

    /* ---------------- PROGRESS BAR ---------------- */
    progress_bar = lv_bar_create(scr);
    lv_obj_set_size(progress_bar, 200, 15);
    lv_obj_align(progress_bar, LV_ALIGN_BOTTOM_MID, 0, -20);

    lv_bar_set_range(progress_bar,
                     (int32_t)AppBIACfg.SweepCfg.SweepStart,
                     (int32_t)AppBIACfg.SweepCfg.SweepStop);

    lv_bar_set_value(progress_bar,
                     (int32_t)AppBIACfg.SweepCfg.SweepStart,
                     LV_ANIM_OFF);

    last_counter = 0;

    lv_disp_load_scr(scr);
    return scr;
}


/* ============================================================
 *  ADD ROW (CALLED WHEN NEW MEASUREMENT ARRIVES)
 * ============================================================
 */
static void add_measurement_row(uint32_t index)
{
    char bufFreq[32], bufR[48], bufX[48], bufLC[48];

    float R, X, L, C;
    float freq = GVariables.FreqBuffer[index];

    calculate_impedance_components(
        GVariables.MagnitudeBuffer[index],
        GVariables.PhaseBuffer[index],
        freq,
        &R, &X, &L, &C
    );

    snprintf(bufFreq, sizeof(bufFreq), "%.1f", (double)freq);
    snprintf(bufR,   sizeof(bufR),   "%.3e", (double)R);
    snprintf(bufX,   sizeof(bufX),   "%.3e", (double)X);

    if (X >= 0)
        snprintf(bufLC, sizeof(bufLC), "%.3e H", (double)L);
    else
        snprintf(bufLC, sizeof(bufLC), "%.3e F", (double)C);

    /* Insert row */
    uint32_t row = lv_table_get_row_cnt(table_results);
    lv_table_set_row_cnt(table_results, row + 1);

    lv_table_set_cell_value(table_results, row, 0, bufFreq);
    lv_table_set_cell_value(table_results, row, 1, bufR);
    lv_table_set_cell_value(table_results, row, 2, bufX);
    lv_table_set_cell_value(table_results, row, 3, bufLC);
}


/* ============================================================
 *  UPDATE SCREEN
 * ============================================================
 */
void screen_resultsText_update(void)
{
    /* Update progress bar */
    lv_bar_set_value(progress_bar, (int32_t)AppBIACfg.FreqofData, LV_ANIM_ON);

    /* Show graph button after sweep ends */
    if (GVariables.sweep_done)
        lv_obj_clear_flag(btn_graph, LV_OBJ_FLAG_HIDDEN);

    /* Add new rows */
    while (last_counter < GVariables.MeasurementCounter)
    {
        add_measurement_row(last_counter);
        last_counter++;
    }
}


/* ============================================================
 *  DESTROY
 * ============================================================
 */
void screen_resultsText_destroy(void)
{
    table_results = NULL;
    progress_bar = NULL;
    btn_graph = NULL;
    last_counter = 0;
}


Screen_t Screen_resultsText = {
    .create = screen_resultsText_create,
    .update = screen_resultsText_update,
    .destroy = screen_resultsText_destroy
};
