#include "stm32f0xx_conf.h"
#include "3595-LCD-Driver.h"
#include "main.h"
#include "STM32-Debounce.h"

//Variables
static __IO uint32_t TimingDelay;

void _delay_ms(__IO uint32_t nTime)
{
  TimingDelay = nTime;

  while(TimingDelay != 0);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}


void SysTick_Handler(void) {
  static uint16_t tick = 0;
  static uint16_t ten_ms_tick = 0;

  switch (tick++) {
    case 1000:
      tick = 0;
      GPIOC->ODR ^= (1 << 8);
      break;
  }
  
  TimingDelay_Decrement();
  
  if (ten_ms_tick++ > 9) {
    ten_ms_tick = 0;
    debounce_interrupt_service();
  }
}

int main(void)
{
  SysTick_Config(SystemCoreClock/1000);
  
  SPI_Config();

  //Setup output for blinking blue LED  
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;   // enable the clock to GPIOC
  GPIOC->MODER |= (1<<16);
  
  LCD_init();
  
  
  unsigned char ballX = 0;
  //Make playing area
  Draw_Box(0,0,97,66,white);
  //Make 'ball'
  Draw_Box(ballX,32,ballX+2,34,blue);
  
  while(ballX++<(PAGE_SIZE-2)){
    _delay_ms(100);
    
    Draw_Box(ballX+2,32,ballX+2,34,blue);
    Draw_Box(ballX-1,32,ballX-1,34,white);
  }
  

  while(1)
  {
    if( get_key_press( 1<<KEY0 )) { 
      static unsigned char but_temp = 0;
      if (but_temp++) {
        but_temp = 0;
        Draw_Box(0,0,97,66,green);
      }
      else Draw_Box(10,10,19,19,white); 
      }
  }

}
