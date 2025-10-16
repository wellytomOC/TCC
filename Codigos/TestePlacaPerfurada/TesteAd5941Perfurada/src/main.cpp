#include <Arduino.h>
#include "main.h"
#include "stdio.h"
#include "ESP32Port.h"
extern "C"{
  #include "ad5940.h"
  #include "BodyImpedance.h"
  #include "AD5940MainBodyImp.h"
}

//funcoes


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


void setup() {

  //UART initialization
  Serial.begin(115200);

  //pin configurations
  PinSetup();
  
  // Imprime uma mensagem de boas-vindas
  Serial.println("Hello AD5940 - Build Time:");
  Serial.println(__TIME__);
  

  delay(3000);


  AD5940_MCUResourceInit(0);
  AD5940_Main();
  
}

void loop() {
  digitalWrite(LED1_PIN, HIGH);
  digitalWrite(LED2_PIN, LOW);
  delay(100);
  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, HIGH);
  delay(100);
}

