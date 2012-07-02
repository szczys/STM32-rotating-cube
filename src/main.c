#include "stm32f0xx_conf.h"
#include "3595-LCD-Driver.h"
#include "main.h"
#include "STM32-Debounce.h"
#include <math.h>

typedef struct {
  float x;
  float y;
  float z;	
} triPoint;

triPoint vertices[8];
triPoint lineVertices[8];

uint32_t angleX = 0;
uint32_t angleY = 0;
uint32_t angleZ = 0;

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

triPoint rotateX(triPoint startingPoint, uint16_t angle) {
  triPoint computedPoint;
  
  float rad = angle * 3.141592 / 180;
  float cosa = cos(rad);
  float sina = sin(rad);
  computedPoint.x = startingPoint.x;
  computedPoint.y = (startingPoint.y * cosa) - (startingPoint.z * sina);
  computedPoint.z = (startingPoint.y * sina) + (startingPoint.z * cosa);
  return computedPoint;
}

triPoint rotateY(triPoint startingPoint, uint16_t angle) {
  triPoint computedPoint;
  
  float rad = angle * 3.141592 / 180;
  float cosa = cos(rad);
  float sina = sin(rad);
  computedPoint.y = startingPoint.y;
  computedPoint.z = (startingPoint.z * cosa) - (startingPoint.x * sina);
  computedPoint.x = (startingPoint.z * sina) + (startingPoint.x * cosa);
  return computedPoint;
}

triPoint rotateZ(triPoint startingPoint, uint16_t angle) {
  triPoint computedPoint;
  
  float rad = angle * 3.141592 / 180;
  float cosa = cos(rad);
  float sina = sin(rad);
  computedPoint.z = startingPoint.z;
  computedPoint.x = (startingPoint.x * cosa) - (startingPoint.y * sina);
  computedPoint.y = (startingPoint.x * sina) + (startingPoint.y * cosa);
  return computedPoint;
}

triPoint projectPoint(triPoint startingPoint, uint8_t win_width, uint8_t win_height, uint8_t fov, uint8_t viewer_distance) {
  triPoint returnPoint;
  float factor = fov/(viewer_distance + startingPoint.z);
  returnPoint.x = startingPoint.x * factor + win_width/2;
  returnPoint.y = -startingPoint.y * factor + win_height/2;
  returnPoint.z = 1;
  return returnPoint;
}

//http://www.dreamincode.net/forums/topic/263402-how-do-i-draw-a-line-in-c-on-a-ppm-image-do-i-use-arrays-or-bresenha/page__view__findpost__p__1533961?s=a910238b04c308c7957b4170b2dc6b57
void drawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2) {
  float m = (float)(y2-y1)/(x2-x1);
  uint8_t rounded;
    
  if (fabs(m) < 1) {
    float y;
    uint8_t xH, xL, yL;
    if (x1 < x2) { xH = x2; xL = x1; yL = y1; } 
    else { xH = x1; xL = x2; yL = y2;}
    for (uint8_t x=xL; x<=xH; x++) {
      y = (x-xL)*m + yL;
      rounded = (uint8_t)(y + 0.5);
      Draw_Box(x,y,x,y,blue);
    }
  }
  
  else {
    float x;
    uint8_t yH, yL, xL;
    if (y1 < y2) { yH = y2; yL = y1; xL = x1; } 
    else { yH = y1; yL = y2; xL = x2;}
    for (uint8_t y=yL; y<=yH; y++) {
      x = (y-yL)/m + xL;
      rounded = (uint8_t)(x + 0.5);
      Draw_Box(rounded,y, rounded, y,blue);
    }
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
  
  //Fill the starting vertices
  vertices[0].x = -1;
  vertices[0].y = 1;
  vertices[0].z = -1;
  vertices[1].x = 1;
  vertices[1].y = 1;
  vertices[1].z = -1;  
  vertices[2].x = 1;
  vertices[2].y = -1;
  vertices[2].z = -1; 
  vertices[3].x = -1;
  vertices[3].y = -1;
  vertices[3].z = -1;
  vertices[4].x = -1;
  vertices[4].y = 1;
  vertices[4].z = 1;  
  vertices[5].x = 1;
  vertices[5].y = 1;
  vertices[5].z = 1;  
  vertices[6].x = 1;
  vertices[6].y = -1;
  vertices[6].z = 1;
  vertices[7].x = -1;
  vertices[7].y = -1;
  vertices[7].z = 1;
  

  triPoint calcPoint, newPoint;
  

  
  while(1) {

    Draw_Box(0,0,97,66,white);
  
    for (uint8_t i = 0; i<8; i++) {
      calcPoint = rotateZ(vertices[i],angleZ++);
      calcPoint = rotateY(calcPoint,angleY++);
      calcPoint = rotateX(calcPoint,angleX++);
      //calcPoint = vertices[i];
      newPoint = projectPoint(calcPoint, PAGE_SIZE, ROW_SIZE, 60, 4); 
      //Draw_Box((uint8_t)newPoint.x,(uint8_t)newPoint.y,(uint8_t)newPoint.x+2,newPoint.y+2,blue);
      lineVertices[i] = newPoint;
    }
    drawLine(lineVertices[0].x,lineVertices[0].y,lineVertices[1].x,lineVertices[1].y);
    drawLine(lineVertices[1].x,lineVertices[1].y,lineVertices[2].x,lineVertices[2].y);
    drawLine(lineVertices[2].x,lineVertices[2].y,lineVertices[3].x,lineVertices[3].y);
    drawLine(lineVertices[3].x,lineVertices[3].y,lineVertices[0].x,lineVertices[0].y);
    drawLine(lineVertices[0].x,lineVertices[0].y,lineVertices[4].x,lineVertices[4].y);
    drawLine(lineVertices[1].x,lineVertices[1].y,lineVertices[5].x,lineVertices[5].y);
    drawLine(lineVertices[2].x,lineVertices[2].y,lineVertices[6].x,lineVertices[6].y);
    drawLine(lineVertices[3].x,lineVertices[3].y,lineVertices[7].x,lineVertices[7].y);
    drawLine(lineVertices[4].x,lineVertices[4].y,lineVertices[5].x,lineVertices[5].y);
    drawLine(lineVertices[5].x,lineVertices[5].y,lineVertices[6].x,lineVertices[6].y);
    drawLine(lineVertices[6].x,lineVertices[6].y,lineVertices[7].x,lineVertices[7].y);
    drawLine(lineVertices[7].x,lineVertices[7].y,lineVertices[4].x,lineVertices[4].y);
    
    _delay_ms(100);    
  }

}
