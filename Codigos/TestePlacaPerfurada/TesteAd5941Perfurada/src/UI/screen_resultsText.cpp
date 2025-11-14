#include "screen_resultsText.h"
#include "lvgl.h"
#include "main.h"
#include "ImpedanceMeter.h"

static lv_obj_t * ta_results;     // Scrollable text area
static lv_obj_t * progress_bar;   // Loading bar
static lv_obj_t * btn_graph;      // "Graph" button
static uint32_t last_counter = 0; // Track last processed measurement

lv_obj_t * screen_resultsText_create(void)
{
    lv_obj_t * scr = lv_obj_create(NULL);

    /* ---------------- Back button ---------------- */
    lv_obj_t * btn_back = lv_btn_create(scr);
    lv_obj_set_size(btn_back, 80, 40);
    lv_obj_align(btn_back, LV_ALIGN_TOP_LEFT, 10, 10);

    lv_obj_t * lbl_back = lv_label_create(btn_back);
    lv_label_set_text(lbl_back, "Back");
    lv_obj_center(lbl_back);

    lv_obj_add_event_cb(btn_back, [](lv_event_t * e) {
        ui_manager_set_screen(SCREEN_HOME);
        GVariables.sweep_done = false;
    }, LV_EVENT_CLICKED, NULL);

    /* ---------------- Graph button (hidden by default) ---------------- */
    btn_graph = lv_btn_create(scr);
    lv_obj_set_size(btn_graph, 80, 40);
    lv_obj_align(btn_graph, LV_ALIGN_TOP_RIGHT, -10, 10);

    lv_obj_t * lbl_graph = lv_label_create(btn_graph);
    lv_label_set_text(lbl_graph, "Graph");
    lv_obj_center(lbl_graph);

    lv_obj_add_event_cb(btn_graph, [](lv_event_t * e) {
        ui_manager_set_screen(SCREEN_RESULTSGRAPH);
    }, LV_EVENT_CLICKED, NULL);

    lv_obj_add_flag(btn_graph, LV_OBJ_FLAG_HIDDEN); // start hidden

    /* ---------------- Text area ---------------- */
    ta_results = lv_textarea_create(scr);
    lv_obj_set_size(ta_results, lv_pct(95), 200);
    lv_obj_align(ta_results, LV_ALIGN_CENTER, 0, 0);
    lv_textarea_set_text(ta_results, "");       // start empty
    lv_obj_set_scroll_dir(ta_results, LV_DIR_VER);
    lv_textarea_set_cursor_click_pos(ta_results, false);
    lv_obj_set_style_border_width(ta_results, 1, 0);
    lv_obj_set_style_text_font(ta_results, &lv_font_montserrat_10, 0); // smaller text
    lv_obj_set_style_text_line_space(ta_results, 6, 0);

    /* ---------------- Progress bar ---------------- */
    progress_bar = lv_bar_create(scr);
    lv_obj_set_size(progress_bar, 200, 15);
    lv_obj_align(progress_bar, LV_ALIGN_BOTTOM_MID, 0, -20);
    lv_bar_set_range(progress_bar,
                     (int32_t)AppBIACfg.SweepCfg.SweepStart,
                     (int32_t)AppBIACfg.SweepCfg.SweepStop);
    lv_bar_set_value(progress_bar,
                     (int32_t)AppBIACfg.SweepCfg.SweepStart,
                     LV_ANIM_OFF);

    lv_disp_load_scr(scr);

    last_counter = 0;
    return scr;
}

void screen_resultsText_update(void)
{
    // Update progress bar based on current frequency
    lv_bar_set_value(progress_bar, (int32_t)AppBIACfg.FreqofData, LV_ANIM_ON);

    // Show "Graph" button only when sweep is done
    if (GVariables.sweep_done)
        lv_obj_clear_flag(btn_graph, LV_OBJ_FLAG_HIDDEN);
    else
        lv_obj_add_flag(btn_graph, LV_OBJ_FLAG_HIDDEN);

    // Check if new measurements were added
    if (GVariables.MeasurementCounter > last_counter)
    {
        char buf[160];
        for (uint32_t i = last_counter; i < GVariables.MeasurementCounter; i++)
        {
            float R, X, L, C;
            calculate_impedance_components(GVariables.MagnitudeBuffer[i],
                                           GVariables.PhaseBuffer[i],
                                           AppBIACfg.FreqofData,
                                           &R, &X, &L, &C);

            snprintf(buf, sizeof(buf),
                     "F: %.1f Hz | R: %.2e Ohm | X: %.2e Ohm | L: %.2e H | C: %.2e F\n",
                     (double)AppBIACfg.FreqofData, (double)R, (double)X, (double)L, (double)C);

            lv_textarea_add_text(ta_results, buf);
        }
        last_counter = GVariables.MeasurementCounter;

        // Auto-scroll to bottom
        lv_textarea_set_cursor_pos(ta_results, LV_TEXTAREA_CURSOR_LAST);
    }
}

void screen_resultsText_destroy(void)
{
    ta_results = nullptr;
    progress_bar = nullptr;
    btn_graph = nullptr;
    last_counter = 0;
}

Screen_t Screen_resultsText = {
    .create = screen_resultsText_create,
    .update = screen_resultsText_update,
    .destroy = screen_resultsText_destroy
};
