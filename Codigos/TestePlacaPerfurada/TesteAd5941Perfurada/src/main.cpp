#include <Arduino.h>
#include "main.h"
#include "stdio.h"
#include "ESP32Port.h"
#include "DisplayControl.h"
#include "ImpedanceMeter.h"
#include "UI/uiManager.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
//#include "DisplaySetup.h"
extern "C"{
  #include "ad5940.h"
  #include "BodyImpedance.h"
}

#include "Tests/LPDac.h"
#include "Tests/HSDAC.h"
#include "Tests/ADC.h"




//****************************************************************/
//                VARIABLES
//****************************************************************/

Type_GlobalVariables GVariables;
AppBIACfg_Type AppBIACfg;


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

void PrintMemoryReport()
{
  Serial.println();
  Serial.println(F("============================================================="));
  Serial.println(F("                   MEMORY USAGE REPORT                       "));
  Serial.println(F("-------------------------------------------------------------"));
  Serial.printf("%-25s | %-12s | %-12s | %-12s\n",
                "Region", "Total (bytes)", "Free (bytes)", "Used (%)");
  Serial.println(F("-------------------------------------------------------------"));

  // --- Internal RAM ---
  size_t total_internal = heap_caps_get_total_size(MALLOC_CAP_INTERNAL);
  size_t free_internal  = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  size_t used_internal  = total_internal - free_internal;

  // --- PSRAM ---
  size_t total_psram = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
  size_t free_psram  = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
  size_t used_psram  = total_psram - free_psram;

  // --- General-purpose 8-bit capable heap ---
  size_t total_8bit = heap_caps_get_total_size(MALLOC_CAP_8BIT);
  size_t free_8bit  = heap_caps_get_free_size(MALLOC_CAP_8BIT);
  size_t used_8bit  = total_8bit - free_8bit;

  // Print all
  Serial.printf("%-25s | %-12u | %-12u | %6.1f%%\n",
                "Internal RAM",
                (unsigned)total_internal,
                (unsigned)free_internal,
                total_internal ? (100.0 * used_internal / total_internal) : 0.0);

  Serial.printf("%-25s | %-12u | %-12u | %6.1f%%\n",
                "PSRAM",
                (unsigned)total_psram,
                (unsigned)free_psram,
                total_psram ? (100.0 * used_psram / total_psram) : 0.0);

  Serial.printf("%-25s | %-12u | %-12u | %6.1f%%\n",
                "8-bit capable heap",
                (unsigned)total_8bit,
                (unsigned)free_8bit,
                total_8bit ? (100.0 * used_8bit / total_8bit) : 0.0);

  Serial.println(F("-------------------------------------------------------------"));
  Serial.printf("Largest free block (internal): %u bytes\n",
                (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
  Serial.printf("Largest free block (SPIRAM):   %u bytes\n",
                (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));
  Serial.println(F("============================================================="));
  Serial.println();
}

void PrintLVGLMemoryReport()
{
  lv_mem_monitor_t mon;
  lv_mem_monitor(&mon); // Fill monitor structure with info

  Serial.println();
  Serial.println(F("============================================================="));
  Serial.println(F("                   LVGL MEMORY REPORT                         "));
  Serial.println(F("-------------------------------------------------------------"));
  Serial.printf("Total size:           %u bytes\n",  (unsigned)mon.total_size);
  Serial.printf("Free size:            %u bytes\n",  (unsigned)mon.free_size);
  Serial.printf("Used size:            %u bytes\n",  (unsigned)(mon.total_size - mon.free_size));
  Serial.printf("Used percentage:      %d %%\n",   mon.used_pct);
  Serial.printf("Fragmentation:        %d %%\n",   mon.frag_pct);
  Serial.printf("Largest free block:   %u bytes\n",  (unsigned)mon.free_biggest_size);
  Serial.println(F("============================================================="));
  Serial.println();
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


  //Inicia Impedance meter
  AD5940_MCUResourceInit(0);
  InitImpedanceMeter();



  //testes do AD5940

  // HwResetAndClockConfig();
  // Serial.println("Starting AD5940 Tests...");
  // xTaskCreate(TestTask,"TestTask", 8192, NULL, 5, &TaskHandleTestTask);
  // Serial.println("Starting LPDAC Test...");
  // LPDac_Main();
  //HSDAC_Main();

}




void loop() {
  
  //PrintMemoryReport();
  //PrintLVGLMemoryReport();
  

  delay(2000); // smooth updates
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



