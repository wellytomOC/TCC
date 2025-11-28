#include "screen_resultsText.h"
#include "lvgl.h"
#include "main.h"
#include "ImpedanceMeter.h"

static lv_obj_t * table_results;
static lv_obj_t * progress_bar;
static lv_obj_t * btn_graph;
static lv_obj_t * btn_save;
static lv_obj_t * btn_next;
static lv_obj_t * btn_prev;
static lv_obj_t * lbl_page;

static uint32_t last_counter = 0;

// Pagination
#define ROWS_PER_PAGE 14
static uint32_t current_page = 0;
static uint32_t total_pages = 0;

/* Forward */
static void refresh_table(void);

/* ============================================================
 *  CREATE SCREEN
 * ============================================================ */
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
        StopImpedanceMeasurement();
    }, LV_EVENT_CLICKED, NULL);

    /* ---------------- Graph Button ---------------- */
    btn_graph = lv_btn_create(scr);
    lv_obj_set_size(btn_graph, 80, 40);
    lv_obj_align(btn_graph, LV_ALIGN_BOTTOM_RIGHT, -10, -5);
    lv_obj_t * lbl_graph = lv_label_create(btn_graph);
    lv_label_set_text(lbl_graph, "Graph");
    lv_obj_center(lbl_graph);
    lv_obj_add_flag(btn_graph, LV_OBJ_FLAG_HIDDEN);

    lv_obj_add_event_cb(btn_graph, [](lv_event_t * e) {
        ui_manager_set_screen(SCREEN_RESULTSGRAPH);
    }, LV_EVENT_CLICKED, NULL);

    /* ---------------- Save Button ---------------- */
    btn_save = lv_btn_create(scr);
    lv_obj_set_size(btn_save, 80, 40);
    lv_obj_align(btn_save, LV_ALIGN_BOTTOM_RIGHT, -100, -5);  // left of Graph
    lv_obj_t * lbl_save = lv_label_create(btn_save);
    lv_label_set_text(lbl_save, "Save");
    lv_obj_center(lbl_save);
    lv_obj_add_flag(btn_save, LV_OBJ_FLAG_HIDDEN); // hidden until sweep ends

    lv_obj_add_event_cb(btn_save, [](lv_event_t * e) {

        Serial.println("BEGIN_DATA");   // Start marker

        for (uint32_t i = 0; i < GVariables.MeasurementCounter; i++)
        {
            float freq = GVariables.FreqBuffer[i];
            float mag  = GVariables.MagnitudeBuffer[i];
            float ph   = GVariables.PhaseBuffer[i];

            // Send CSV: freq, magnitude, phase
            Serial.printf("%f,%f,%f\n", freq, mag, ph);
        }

        Serial.println("END_DATA");     // End marker

    }, LV_EVENT_CLICKED, NULL);
    

    /* ---------------- TABLE ---------------- */
    table_results = lv_table_create(scr);
    lv_obj_clear_flag(table_results, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_set_size(table_results, lv_pct(95), 230);
    lv_obj_align(table_results, LV_ALIGN_TOP_MID, 0, 10);

    lv_obj_set_style_border_width(table_results, 1, 0);
    lv_obj_set_style_text_font(table_results, &lv_font_montserrat_12, 0);

    lv_table_set_col_cnt(table_results, 4);
    lv_table_set_row_cnt(table_results, ROWS_PER_PAGE + 1);

    lv_table_set_col_width(table_results, 0, 100);
    lv_table_set_col_width(table_results, 1, 100);
    lv_table_set_col_width(table_results, 2, 100);
    lv_table_set_col_width(table_results, 3, 100);

    static lv_style_t style_cell;
    lv_style_init(&style_cell);
    lv_style_set_pad_top(&style_cell, 0);
    lv_style_set_pad_bottom(&style_cell, 0);
    lv_style_set_pad_left(&style_cell, 2);
    lv_style_set_pad_right(&style_cell, 2);
    lv_obj_add_style(table_results, &style_cell, LV_PART_ITEMS);

    // Header
    lv_table_set_cell_value(table_results, 0, 0, "Freq (Hz)");
    lv_table_set_cell_value(table_results, 0, 1, "R (Ohm)");
    lv_table_set_cell_value(table_results, 0, 2, "X (Ohm)");
    lv_table_set_cell_value(table_results, 0, 3, "L/C (H/F)");

    /* ---------------- Pagination Controls ---------------- */
    btn_prev = lv_btn_create(scr);
    lv_obj_set_size(btn_prev, 60, 40);
    lv_obj_align(btn_prev, LV_ALIGN_BOTTOM_MID, -80, -5);
    lv_obj_t * lbl_prev = lv_label_create(btn_prev);
    lv_label_set_text(lbl_prev, "<");
    lv_obj_center(lbl_prev);

    btn_next = lv_btn_create(scr);
    lv_obj_set_size(btn_next, 60, 40);
    lv_obj_align(btn_next, LV_ALIGN_BOTTOM_MID, 20, -5);
    lv_obj_t * lbl_next = lv_label_create(btn_next);
    lv_label_set_text(lbl_next, ">");
    lv_obj_center(lbl_next);

    lbl_page = lv_label_create(scr);
    lv_obj_align(lbl_page, LV_ALIGN_BOTTOM_MID, -30, -5);
    lv_label_set_text(lbl_page, "0/0");

    lv_obj_add_event_cb(btn_prev, [](lv_event_t * e){
        if(current_page > 0) {
            current_page--;
            refresh_table();
        }
    }, LV_EVENT_CLICKED, NULL);

    lv_obj_add_event_cb(btn_next, [](lv_event_t * e){
        if(current_page + 1 < total_pages) {
            current_page++;
            refresh_table();
        }
    }, LV_EVENT_CLICKED, NULL);

    /* ---------------- PROGRESS BAR ---------------- */
    progress_bar = lv_bar_create(scr);
    lv_obj_set_size(progress_bar, 200, 15);
    lv_obj_align(progress_bar, LV_ALIGN_BOTTOM_MID, 0, -50);

    lv_bar_set_range(progress_bar,
                     (int32_t)AppBIACfg.SweepCfg.SweepStart,
                     (int32_t)AppBIACfg.SweepCfg.SweepStop);

    lv_bar_set_value(progress_bar,
                     (int32_t)AppBIACfg.SweepCfg.SweepStart,
                     LV_ANIM_OFF);

    last_counter = GVariables.MeasurementCounter;
    current_page = 0;

    refresh_table();

    lv_disp_load_scr(scr);
    return scr;
}

/* ============================================================
 *  REFRESH TABLE
 * ============================================================ */
static void refresh_table(void)
{
    if(last_counter == 0) return;

    total_pages = (last_counter + ROWS_PER_PAGE - 1) / ROWS_PER_PAGE;
    if(current_page >= total_pages) current_page = total_pages - 1;

    uint32_t start_idx = current_page * ROWS_PER_PAGE;
    uint32_t end_idx = start_idx + ROWS_PER_PAGE;
    if(end_idx > last_counter) end_idx = last_counter;

    lv_table_set_row_cnt(table_results, ROWS_PER_PAGE + 1);

    for(uint32_t row = 1; row <= ROWS_PER_PAGE; row++)
        for(uint32_t col = 0; col < 4; col++)
            lv_table_set_cell_value(table_results, row, col, "");

    for(uint32_t row = 0; row < end_idx - start_idx; row++)
    {
        uint32_t idx = start_idx + row;
        float R, X, L, C;
        float freq = GVariables.FreqBuffer[idx];

        calculate_impedance_components(
            GVariables.MagnitudeBuffer[idx],
            GVariables.PhaseBuffer[idx],
            freq,
            &R, &X, &L, &C
        );

        char bufFreq[32], bufR[48], bufX[48], bufLC[48];
        snprintf(bufFreq, sizeof(bufFreq), "%.1f", (double)freq);
        snprintf(bufR,   sizeof(bufR),   "%.3e", (double)R);
        snprintf(bufX,   sizeof(bufX),   "%.3e", (double)X);

        if(X >= 0)
            snprintf(bufLC, sizeof(bufLC), "%.3e H", (double)L);
        else
            snprintf(bufLC, sizeof(bufLC), "%.3e F", (double)C);

        lv_table_set_cell_value(table_results, row + 1, 0, bufFreq);
        lv_table_set_cell_value(table_results, row + 1, 1, bufR);
        lv_table_set_cell_value(table_results, row + 1, 2, bufX);
        lv_table_set_cell_value(table_results, row + 1, 3, bufLC);
    }

    char page_text[32];
    snprintf(page_text, sizeof(page_text), "%d/%d", current_page + 1, total_pages);
    lv_label_set_text(lbl_page, page_text);
}

/* ============================================================
 *  UPDATE SCREEN
 * ============================================================ */
void screen_resultsText_update(void)
{
    lv_bar_set_value(progress_bar, (int32_t)AppBIACfg.FreqofData, LV_ANIM_ON);

    if(GVariables.sweep_done) {
        lv_obj_clear_flag(btn_graph, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(btn_save, LV_OBJ_FLAG_HIDDEN);
    }

    if(last_counter < GVariables.MeasurementCounter)
    {
        last_counter = GVariables.MeasurementCounter;
        refresh_table();
    }
}

/* ============================================================
 *  DESTROY
 * ============================================================ */
void screen_resultsText_destroy(void)
{
    table_results = NULL;
    progress_bar = NULL;
    btn_graph = NULL;
    btn_save = NULL;
    btn_next = NULL;
    btn_prev = NULL;
    lbl_page = NULL;

    last_counter = 0;
    current_page = 0;
    total_pages = 0;
}

/* ============================================================
 *  Screen struct
 * ============================================================ */
Screen_t Screen_resultsText = {
    .create = screen_resultsText_create,
    .update = screen_resultsText_update,
    .destroy = screen_resultsText_destroy
};
