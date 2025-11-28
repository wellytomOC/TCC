#include "Arduino.h"
#include "main.h"
#include "DisplayControl.h"
#include "TFT_eSPI.h"
#include <Preferences.h>
#include <lvgl.h>
#include "UI/uiManager.h"






//****************************************************************/
//                VARIABLES
//****************************************************************/

extern Type_GlobalVariables GVariables;

TaskHandle_t TaskHandleDisplayTask = NULL;

//TFT_eSPI
TFT_eSPI tft = TFT_eSPI();


//Save calibration data in NVS
Preferences prefs;  // NVS flash storage
#define PREF_NAMESPACE "touch"
#define PREF_KEY_CAL "caldata"
#define PREF_KEY_FLAG "calok"



//LVGL
#define SCREEN_WIDTH  480
#define SCREEN_HEIGHT 320
static uint32_t my_tick_get_cb(void) { return millis(); }


static lv_color_t buf1[SCREEN_WIDTH * SCREEN_HEIGHT / 10] DMA_ATTR;




//****************************************************************/
//                PROTOTYPES
//****************************************************************/

void DisplayTask(void *pvParameters);

bool loadTouchCalibration(void);

void my_flush_cb(lv_display_t * display, const lv_area_t * area, uint8_t * px_map);
void my_input_read(lv_indev_t * indev, lv_indev_data_t * data);
static void lv_log_print_g_cb(lv_log_level_t level, const char *buf);


void lv_example_get_started_2(void);


//****************************************************************/
//               INITS AND TASKS
//****************************************************************/
void StartDisplayControl(void){
    //init display
    tft.init();
    tft.setRotation(1);
    
    //calibrate
    if (!loadTouchCalibration()) {
        calibrateTouch();
    }



    //LVGL
    lv_init();
    lv_tick_set_cb(my_tick_get_cb); //tick callback from millis()
    lv_log_register_print_cb(lv_log_print_g_cb); // Register print function for LVGL logs

    lv_display_t * display1 = lv_display_create(SCREEN_WIDTH, SCREEN_HEIGHT);
    lv_display_set_buffers(display1, buf1, NULL, sizeof(buf1), LV_DISPLAY_RENDER_MODE_PARTIAL);

    lv_display_set_flush_cb(display1, my_flush_cb); //flush callback

    lv_indev_t * indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, my_input_read);

    ui_manager_set_screen(SCREEN_HOME);


    xTaskCreate(DisplayTask,"DisplayTask", 20000, NULL, 5, &TaskHandleDisplayTask);


    //tests


    //lv_example_get_started_2();



}

void DisplayTask(void *pvParameters) {


    while(1){

        lv_timer_handler(); /* let the GUI do its work */
        ui_manager_update();
        delay(5);
    }
}




//****************************************************************/
//                FUNCTIONS
//****************************************************************/

//Calibration
void calibrateTouch(void) {
  uint16_t calData[5];
  uint8_t calDataOK = 0;

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextFont(2);
  tft.setCursor(20, 100);
  tft.println("Touch screen to calibrate");
  tft.println();
  tft.println("Follow the crosshair markers...");

  delay(1500);
  tft.calibrateTouch(calData, TFT_RED, TFT_BLACK, 25);

  //feedback screen
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(20, 100);
  tft.println("Calibration complete!");

  // Save calibration
  prefs.begin(PREF_NAMESPACE, false); // RW mode
  prefs.putBytes(PREF_KEY_CAL, calData, sizeof(calData));
  prefs.putBool(PREF_KEY_FLAG, true);
  prefs.end();

  tft.setTouch(calData);
  Serial.println("Calibration data saved & applied.");
  delay(1000);

  
  delay(1000);
}

bool loadTouchCalibration(void) {
  uint16_t calData[5];
  prefs.begin(PREF_NAMESPACE, true);  // read-only
  bool ok = prefs.getBool(PREF_KEY_FLAG, false);

  if (ok) {
    prefs.getBytes(PREF_KEY_CAL, calData, sizeof(calData));
    prefs.end();
    tft.setTouch(calData);
    Serial.println("Loaded touch calibration from flash.");
    return true;
  } else {
    prefs.end();
    Serial.println("No calibration data found in flash.");
    return false;
  }
}


//LVGL Callbacks
void my_flush_cb(lv_display_t * display, const lv_area_t * area, uint8_t * px_map)
{
    /* Copilot Version */
    // int32_t x, y;
    // for(y = area->y1; y <= area->y2; y++) {
    //     for(x = area->x1; x <= area->x2; x++) {
    //         uint16_t color = *((uint16_t *)px_map);
    //         tft.drawPixel(x, y, color);
    //         px_map += 2;
    //     }
    // }
    // lv_display_flush_ready(display);
    



    /* ChatGPT Version  */
    uint32_t w = area->x2 - area->x1 + 1;
    uint32_t h = area->y2 - area->y1 + 1;

    tft.startWrite();
    tft.setAddrWindow(area->x1, area->y1, w, h);
    tft.pushColors((uint16_t *)px_map, w * h, true);
    tft.endWrite();

    lv_display_flush_ready(display);
}

void my_input_read(lv_indev_t * indev, lv_indev_data_t * data)
{
    uint16_t x, y;
    bool touched = tft.getTouch(&x, &y);
    if(touched) {
        data->state = LV_INDEV_STATE_PR;
        data->point.x = x;
        data->point.y = y;
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
}

static void lv_log_print_g_cb(lv_log_level_t level, const char *buf)
{
    LV_UNUSED(level);
    Serial.write(buf);
}





//tests

static void btn_event_cb(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * btn = lv_event_get_target_obj(e);
    if(code == LV_EVENT_CLICKED) {
        static uint8_t cnt = 0;
        cnt++;

        /*Get the first child of the button which is the label and change its text*/
        lv_obj_t * label = lv_obj_get_child(btn, 0);
        lv_label_set_text_fmt(label, "Button: %d", cnt);
    }
}

void lv_example_get_started_2(void)
{
    lv_obj_t * btn = lv_button_create(lv_screen_active());     /*Add a button the current screen*/
    lv_obj_set_pos(btn, 10, 10);                            /*Set its position*/
    lv_obj_set_size(btn, 120, 50);                          /*Set its size*/
    lv_obj_add_event_cb(btn, btn_event_cb, LV_EVENT_ALL, NULL);           /*Assign a callback to the button*/

    lv_obj_t * label = lv_label_create(btn);          /*Add a label to the button*/
    lv_label_set_text(label, "Button");                     /*Set the labels text*/
    lv_obj_center(label);
}


