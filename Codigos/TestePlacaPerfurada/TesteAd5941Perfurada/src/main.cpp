#include <Arduino.h>
#include "main.h"
#include "stdio.h"
#include "ESP32Port.h"
#include "DisplayControl.h"
#include "UI/uiManager.h"
#include "UI/screen_results.h"
//#include "DisplaySetup.h"
extern "C"{
  #include "ad5940.h"
  #include "BodyImpedance.h"
  #include "AD5940MainBodyImp.h"
}

#include "Tests/LPDac.h"
#include "Tests/HSDAC.h"
#include "Tests/ADC.h"




//****************************************************************/
//                VARIABLES
//****************************************************************/

Type_GlobalVariables GVariables;


//****************************************************************/
//                PROTOTYPES
//****************************************************************/
void ConfigureLpTIA(void);
void DEMO_Test_SPI(void);
void DebugLed(void *parameters);
void TestTask(void *parameters);
void HwResetAndClockConfig(void);



//****************************************************************/
//                BASIC FUNCTIONS
//****************************************************************/
TaskHandle_t TaskHandleDebugLed = NULL;
TaskHandle_t TaskHandleTestTask = NULL;

void PinSetup(void){
  //LEDs
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);

  //jumpers
  pinMode(JMP1_PIN, INPUT_PULLUP);
  pinMode(JMP2_PIN, INPUT_PULLUP);
  pinMode(JMP3_PIN, INPUT_PULLUP);

  //USB - not working, use as debug pins
  pinMode(USB_DN_PIN, INPUT_PULLUP);
  pinMode(USB_DP_PIN, INPUT_PULLUP);

  //AD GPIOs
  pinMode(AD5940_GP0INT_PIN, INPUT_PULLUP);
  pinMode(AD5940_GP1INT_PIN, INPUT_PULLUP);
  pinMode(AD5940_GP2INT_PIN, INPUT_PULLUP);
}

void setup() {

  //UART initialization
  Serial.begin(115200);

  //pin configurations
  PinSetup();
  
  
  // Imprime uma mensagem de boas-vindas
  Serial.println("Hello AD5940 - Build Time:");
  Serial.println(__TIME__);
  
  //Inicia a task para debugging
  xTaskCreate(DebugLed,"DebugLed", 2048, NULL, 5, &TaskHandleDebugLed);
  

  //Inicia o display
  StartDisplayControl();




  //Inicia os testes do AD5940
  delay(5000);
  AD5940_MCUResourceInit(0);

  // HwResetAndClockConfig();
  // Serial.println("Starting AD5940 Tests...");
  // xTaskCreate(TestTask,"TestTask", 8192, NULL, 5, &TaskHandleTestTask);

  // Serial.println("Starting LPDAC Test...");
  // LPDac_Main();

  //ImpedanceSweep_Main();


  //HSDAC_Main();
}




void loop() {
    GVariables.TestCounter++;

    if (GVariables.sweep_ready == true) {
        
        static int32_t step = 0;

        // Cache locally for readability
        int32_t f_start = (int32_t)GVariables.SweepParams.startFreq;
        int32_t f_end   = (int32_t)GVariables.SweepParams.endFreq;
        int32_t n_steps = (int32_t)GVariables.SweepParams.steps;

        if (!GVariables.sweep_done) {
            // --- Compute logarithmic frequency (still needs float math for log10/pow) ---
            // We’ll compute in float and then convert back to int32_t for freq
            float log_start = log10f((float)f_start);
            float log_end   = log10f((float)f_end);
            float log_step  = (log_end - log_start) / (float)(n_steps - 1);

            int32_t freq = (int32_t)powf(10.0f, log_start + step * log_step);

            // --- Predictable data ---
            // Magnitude rises linearly from 10 to 100
            int32_t measuredMagnitude = 10 + (90 * step) / (n_steps - 1);

            // Phase falls linearly from +90° to −90°
            int32_t measuredPhase = 90 - (180 * step) / (n_steps - 1);

            // Add point to chart (convert to float for chart function)
            screen_results_add_point(freq, measuredMagnitude, measuredPhase);

            // Increment step
            step++;

            // Stop when sweep is done
            if (step >= n_steps) {
                GVariables.sweep_done = true;
                Serial.println("Sweep complete!");
            }
        }
    }

    delay(250); // smooth updates
}






//****************************************************************/
//               TASKS
//****************************************************************/
void DebugLed(void *parameters)
{
  while (1)
  {
    digitalWrite(LED1_PIN, HIGH);
    digitalWrite(LED2_PIN, LOW);
    delay(100);
    digitalWrite(LED1_PIN, LOW);
    digitalWrite(LED2_PIN, HIGH);
    delay(100);
  }
  vTaskDelete(TaskHandleDebugLed);
}

void TestTask(void *parameters)
{
  
  ADC_Main();
  vTaskDelete(TaskHandleTestTask);
}







//****************************************************************/
//                Tests and Examples Code
//****************************************************************/

//Test SPI communication with AD5940
void DEMO_Test_SPI(void)
{
  const int TOTAL_TRIALS = 1000;
  unsigned long temp, i = TOTAL_TRIALS, SuccessCount = 0, FailCount = 0;
  char *ToBePrinted;
  /**
   * Hardware reset can always put AD5940 to default state. 
   * We recommend to use hardware reset rather than software reset
   * because there are some situations that SPI won't work, for example, AD59840 is in hibernate mode, 
   * or AD5940 system clock is 32kHz that SPI bus clock should also be limited..
   * */
  AD5940_HWReset();
  /**
   * @note MUST call this function whenever there is reset happened. This function will put AD5940 to right state.
   *       The reset can be software reset or hardware reset or power up reset.
  */
  AD5940_Initialize();
  /**
   * Normal application code starts here.
  */
  /**
   * Read register test.
  */
  temp = AD5940_ReadReg(REG_AFECON_ADIID);
  printf("Read ADIID register, got: 0x%04lx\n", temp);
  if(temp != AD5940_ADIID){
    printf("Read register test failed. Ending test.\n" );
    return;
  }
  else
    printf("Read register test pass. Proceeding to write test...\n");
  /**
   * Write register test.
   * */
  srand(0x1234);
  while(i--)
  {
    //delay(1);
    static unsigned long count;
    static unsigned long data;
    /* Generate a 32bit random data */
    data = rand()&0xffff;
    data <<= 16;
    data |= rand()&0xffff;
    count ++;	/* Read write count */
    /**
     * Register CALDATLOCK is 32-bit width, it's readable and writable.
     * We use it to test SPI register access.
    */
    AD5940_WriteReg(REG_AFE_CALDATLOCK, data);
    temp = AD5940_ReadReg(REG_AFE_CALDATLOCK);
    if(temp != data){
      FailCount++;
      printf("[%ld/%d]Write register failed. Written: @0x%08lx.   Return: @0x%08lx.   Millis: %ld.\n", count, TOTAL_TRIALS, data, temp, millis());
    }
    else{
      SuccessCount++;
      printf("[%ld/%d]Write success. Written: @0x%08lx.   Return: @0x%08lx.   Millis: %ld.\n", count, TOTAL_TRIALS, data, temp, millis());
    }
  }

  Serial.println("SPI read/write test completed");
  printf("Success count: %ld.    Fail count: %ld.\n", SuccessCount, FailCount);
}


//CLock config
void HwResetAndClockConfig(void)
{
  CLKCfg_Type clk_cfg;

  /* Use hardware reset */
  AD5940_HWReset();
  /* Platform configuration */
  AD5940_Initialize();


  /* Configure clock to use external source */
  clk_cfg.ADCClkDiv = ADCCLKDIV_1;
  clk_cfg.ADCCLkSrc = ADCCLKSRC_XTAL;
  clk_cfg.SysClkDiv = SYSCLKDIV_1;
  clk_cfg.SysClkSrc = SYSCLKSRC_XTAL;
  clk_cfg.HfOSC32MHzMode = bFALSE;
  clk_cfg.HFOSCEn = bFALSE;
  clk_cfg.HFXTALEn = bTRUE;
  clk_cfg.LFOSCEn = bTRUE;
  AD5940_CLKCfg(&clk_cfg);
}



