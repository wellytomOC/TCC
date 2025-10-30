#include "main.h"
#include "DisplayControl.h"
#include "TFT_eSPI.h"
//#include "DisplaySetup.h"


//****************************************************************/
//                VARIABLES
//****************************************************************/

extern Type_GlobalVariables GVariables;
TFT_eSPI tft = TFT_eSPI();
TaskHandle_t TaskHandleDisplayTask = NULL;


//****************************************************************/
//                PROTOTYPES
//****************************************************************/

void DisplayTask(void *pvParameters);


//****************************************************************/
//               INITS AND TASKS
//****************************************************************/
void StartDisplayControl(void){

    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(2);
    tft.println("Touch test ready!");


    xTaskCreate(DisplayTask,"DisplayTask", 8192, NULL, 5, &TaskHandleDisplayTask);

}

void DisplayTask(void *pvParameters) {
    uint16_t x, y;
    while(1){

        if (tft.getTouch(&x, &y)) {
            Serial.printf("Touch: x=%d, y=%d\n", x, y);
            tft.fillCircle(x, y, 3, TFT_GREEN);
        } 

    }
}




//****************************************************************/
//                FUNCTIONS
//****************************************************************/