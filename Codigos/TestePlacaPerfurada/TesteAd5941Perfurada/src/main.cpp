#include <Arduino.h>
#include "main.h"

//funcoes
void DEMO_Test_SPI(void)
{
  unsigned long temp, i;
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
  if(temp != AD5940_ADIID)
    printf("Read register test failed.\n" );
  else
    printf("Read register test pass\n");
  /**
   * Write register test.
   * */
  srand(0x1234);
  i =500;
  while(i--)
  {
    delay(10);
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
    if(temp != data)
      printf("Write register test failed @0x%08lx\n", data);
    if(!(count%1000))
      printf("Read/Write has been done %ld times, latest data is 0x%08lx\n", count, data);
    printf("Times remaining: %ld\n", i);
  }
  printf("SPI read/write test completed");
}


void setup() {

  //UART initialization
  Serial.begin(115200);

  
  // Imprime uma mensagem de boas-vindas
  Serial.println("Hello AD5940 - Build Time:");
  Serial.println(__TIME__);
  
  AD5940_MCUResourceInit(0);
  DEMO_Test_SPI();
  
}

void loop() {
  Serial.println("Hello World");
  delay(1000);
  Serial.println("Hello World2");
  delay(1000);
}
