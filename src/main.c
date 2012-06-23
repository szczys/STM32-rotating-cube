#include "stm32f0xx_conf.h"

//LCD Definitions
#define LCD_PORT GPIOC->ODR
#define LCD_DDR  DDRC
#define LCD_CLK (1<<0)
#define LCD_SIO (1<<1)
#define LCD_CS  (1<<2)
#define LCD_RST (1<<3)

#define page_size	97
#define row_size	66

#define red	0b00000111
#define yellow	0b00111111
#define green	0b00111100
#define cyan	0b11111000
#define blue	0b11000000
#define magenta	0b11000111
#define white	0b11111111
#define black	0b11111111

#define SPIx                             SPI1
#define SPIx_CLK                         RCC_APB2Periph_SPI1
#define SPIx_GPIO_PORT                   GPIOA
#define SPIx_GPIO_CLK                    RCC_AHBPeriph_GPIOA
#define SPIx_AF                          GPIO_AF_0
#define SPIx_SCK_PIN                     GPIO_Pin_5
#define SPIx_SCK_SOURCE                  GPIO_PinSource5
#define SPIx_MISO_PIN                    GPIO_Pin_6
#define SPIx_MISO_SOURCE                 GPIO_PinSource6
#define SPIx_MOSI_PIN                    GPIO_Pin_7
#define SPIx_MOSI_SOURCE                 GPIO_PinSource7
#define SPIx_NSS_PIN                     GPIO_Pin_4
#define SPIx_NSS_SOURCE                  GPIO_PinSource4



//Variables
unsigned char cursor_x = 0;	// Tracks cursor position (top-left corner)
unsigned char cursor_y = 0;	// Tracks cursor position (top-left corner)
static __IO uint32_t TimingDelay;
SPI_InitTypeDef SPI_InitStructure;


void LCD_init(void);
void LCD_Out(unsigned char Data, unsigned char isCmd);
void StripedScreen(void);
void Hello_World(void);
void _delay_ms(__IO uint32_t nTime);
void TimingDelay_Decrement(void);


void LCD_init(void)
{
  //LCD_DDR |= (LCD_CLK | LCD_SIO | LCD_CS | LCD_RST);

  LCD_PORT |= (LCD_CLK | LCD_SIO | LCD_CS);

  //Hardware Reset
  LCD_PORT &= ~LCD_RST;
  _delay_ms(1);
  LCD_PORT |= LCD_RST;
  _delay_ms(5);

  //Software Reset
  LCD_Out(0x01, 1);
  _delay_ms(10);

/*
  //Refresh set
  LCD_Out(0xB9, 1);
  LCD_Out(0x00, 0);
*/


  //Display Control
  LCD_Out(0xB6, 0);
  LCD_Out(128, 0);
  LCD_Out(128, 0);
  LCD_Out(129, 0);
  LCD_Out(84, 0);
  LCD_Out(69, 0);
  LCD_Out(82, 0);
  LCD_Out(67, 0);


/*
  //Temperature gradient set
  LCD_Out(0xB7, 1);
  for(char i=0; i<14; i++)  LCD_Out(0, 0);
*/

  //Booster Voltage On
  LCD_Out(0x03, 1);
  _delay_ms(50);  //NOTE: At least 40ms must pass between voltage on and display on.
		  //Other operations may be carried out as long as the display is off
		  //for this length of time.

/*
  //Test Mode
  LCD_Out(0x04, 1);
*/

/*
  // Power Control
  LCD_Out(0xBE, 1);
  LCD_Out(4, 0);
*/

  //Sleep Out
  LCD_Out(0x11, 1);

  //Display mode Normal
  LCD_Out(0x13, 1);

  //Display On
  LCD_Out(0x29, 1);

  //Set Color Lookup Table
  LCD_Out(0x2D, 1);		//Red and Green (3 bits each)
  char x, y;
  for(y = 0; y < 2; y++) {
	  for(x = 0; x <= 14; x+=2) {
		  LCD_Out(x, 0);
	  }
  }
  //Set Color Lookup Table	//Blue (2 bits)
  LCD_Out(0, 0);
  LCD_Out(4, 0);
  LCD_Out(9, 0);
  LCD_Out(14, 0);

  //Set Pixel format to 8-bit color codes
  LCD_Out(0x3A, 1);
  LCD_Out(0b00000010, 0);

//***************************************
//Initialization sequence from datasheet:

//Power to chip
//RES pin=low
//RES pin=high -- 5ms pause
//Software Reset
//5ms Pause
//INIESC
  //<Display Setup 1>
    //REFSET
    //Display Control
    //Gray Scale position set
    //Gamma Curve Set
    //Common Driver Output Select
  //<Power Supply Setup>
    //Power Control
    //Sleep Out
    //Voltage Control
    //Write Contrast
    //Temperature Gradient
    //Boost Voltage On
  //<Display Setup 2>
    //Inversion On
    //Partial Area
    //Vertical Scroll Definition
    //Vertical Scroll Start Address
  //<Display Setup 3>
    //Interface Pixel Format
    //Colour Set
    //Memory access control
    //Page Address Set
    //Column Address Set
    //Memory Write
  //Display On

//****************************************
}

//LCD_Out function comes from source code found here:
//http://hobbyelektronik.org/Elo/AVR/3510i/index.htm
//Unfortunately this is the only way I know to attribute
//this code to the writer.
void LCD_Out(unsigned char Data, unsigned char isCmd) 
{


	while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET) ;;
	
	if (isCmd) {
		SPI_I2S_SendData16(SPIx, (uint16_t)Data);
	
	}
	else SPI_I2S_SendData16(SPIx, ((uint16_t)Data | (1<<8)));

	
	//SPI_I2S_SendData16(SPIx, 0x01FF);
	
	/*
	if(isCmd) LCD_PORT |= LCD_CS;  
	LCD_PORT &= ~(LCD_CLK|LCD_CS);  //Clock and CS low

	LCD_PORT |= LCD_SIO;		//SData High
	if(isCmd) LCD_PORT &= ~LCD_SIO; //If it is a command, SData Low

	LCD_PORT |= LCD_CLK;		//Clock High

	for(char x=0; x<8; x++)	{
		LCD_PORT &= ~(LCD_SIO|LCD_CLK);		//Clock and SData low
		if(Data & 128) LCD_PORT |= LCD_SIO;  	// Mask MSB - SData high if it is a 1
		LCD_PORT |= LCD_CLK;			//Clock High
		Data=Data<<1;				//Shift bits 1 left (new MSB to be read)
		
	}
	*/
}

static void SPI_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable the SPI peripheral */
  RCC_APB2PeriphClockCmd(SPIx_CLK, ENABLE);
  /* Enable SCK, MOSI, MISO and NSS GPIO clocks */
  RCC_AHBPeriphClockCmd(SPIx_GPIO_CLK, ENABLE);


  /* SPI pin mappings */
/*
  GPIO_PinAFConfig(SPIx_GPIO_PORT, SPIx_SCK_SOURCE, SPIx_AF);
  GPIO_PinAFConfig(SPIx_GPIO_PORT, SPIx_MOSI_SOURCE, SPIx_AF);
  //GPIO_PinAFConfig(SPIx_GPIO_PORT, SPIx_MISO_SOURCE, SPIx_AF);
  GPIO_PinAFConfig(SPIx_GPIO_PORT, SPIx_NSS_SOURCE, SPIx_AF);
*/
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;

  /* SPI SCK pin configuration */
  GPIO_InitStructure.GPIO_Pin = SPIx_SCK_PIN;
  GPIO_Init(SPIx_GPIO_PORT, &GPIO_InitStructure);

  /* SPI  MOSI pin configuration */
  GPIO_InitStructure.GPIO_Pin =  SPIx_MOSI_PIN;
  GPIO_Init(SPIx_GPIO_PORT, &GPIO_InitStructure);

  /* SPI MISO pin configuration */
  //GPIO_InitStructure.GPIO_Pin = SPIx_MISO_PIN;
  //GPIO_Init(SPIx_MISO_GPIO_PORT, &GPIO_InitStructure);
  
  /* SPI NSS pin configuration */
  GPIO_InitStructure.GPIO_Pin = SPIx_NSS_PIN;
  GPIO_Init(SPIx_GPIO_PORT, &GPIO_InitStructure);
  
  /* SPI configuration -------------------------------------------------------*/
  SPI_I2S_DeInit(SPIx);
  SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_9b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  //SPI_InitStructure.SPI_CRCPolynomial = 7;
  
  SPI_Init(SPIx, &SPI_InitStructure);
  SPI_SSOutputCmd(SPIx, ENABLE);
  SPI_Cmd(SPIx, ENABLE);
  
  //1. Select the serial clock baud rate using the BR[2:0] bits (see Note Note:)
  
  //2. Set the CPOL and CPHA bits combination to define one of the four relationships
  //  between the data transfer and the serial clock (see Figure 259 and Note )
  //3. Select a transmission mode by configuring RXONLY, BIDIOE and BIDIMODE (see
  //  Note ).
  //4. Set the DS bit in order to select the data length for the transfer.
  //5. Configure the LSBFIRST bit to define the frame format (see Note ).
  //6. Set SSM, SSI and SSOE according to application needs. In master mode, the internal
  //  NSS signal must stay at a high level during the complete sequence (see
  // Section 26.3.4: Slave select (NSS) pin management on page 639). In slave mode, the
  //internal NSS signal must stay at a low level during the complete sequence (see Note ).
  //7. Set the FRF bit if the TI protocol is required (see Section 26.4.2: TI mode on page 649).
  //8. Set the NSSP bit if the NSS pulse mode between two data units is required. The CHPA
  //  bit must be set to 1 for this configuration (see Note ).
  //9. Set the FRXTH bit. The RXFIFO threshold must be aligned to the read access size for
  //  the SPIx_DR register.
  //10. Initialize LDMA_TX and LDMA_RX bits if DMA is used.
  //11. Set the CRC polynomial to “in” and set the CRCEN bit if CRC is needed.
  //12. Set the MSTR bit while the NSS internal signal is at a high level (see Note Note: and
  //Section 26.3.4: Slave select (NSS) pin management)
  //13. Enable the SPI by setting the SPE bit (see Note ).

}

void StripedScreen(void)
{
  unsigned char color_palate[] = {
	//BBGGGRRR
	0b00000111,	//Red
	0b00111111,	//Yellow
	0b00111100,	//Green
	0b11111000,	//Cyan
	0b11000000,	//Blue
	0b11000111,	//Magenta
	0b11111111,	//White
	0b00000111	//This should be 0x00(black) but for screen wrapping it was changed to Red
  };

  LCD_Out(0x13, 1);
  for (unsigned char i=0; i<8; i++)
  {
    LCD_Out(0x2A, 1);
    LCD_Out(0, 0);
    LCD_Out(97, 0);
    LCD_Out(0x2B, 1);
    LCD_Out(i*9, 0);
    LCD_Out((i*9)+8, 0);
    LCD_Out(0x2C, 1);
    for (int j=0; j<882; j++)
    {
      LCD_Out(color_palate[i], 0);
    }
  }
}

void Hello_World(void)
{
  //Binary representation of "Hello World"
  unsigned char Hello_World[5][5] = {
    { 0b10101110, 0b10001000, 0b01001010, 0b10010011, 0b00100110 },
    { 0b10101000, 0b10001000, 0b10101010, 0b10101010, 0b10100101 },
    { 0b11101100, 0b10001000, 0b10101010, 0b10101011, 0b00100101 },
    { 0b10101000, 0b10001000, 0b10101010, 0b10101010, 0b10100101 },
    { 0b10101110, 0b11101110, 0b01000101, 0b00010010, 0b10110110 }
  };

    LCD_Out(0x2A, 1);
    LCD_Out(8, 0);
    LCD_Out(87, 0);
    LCD_Out(0x2B, 1);
    LCD_Out(23, 0);
    LCD_Out(32, 0);
    LCD_Out(0x2C, 1);
    for (unsigned char i=0; i<5; i++) //Scan Rows
    {
      char h=2;
      while(h)
      {
	for (unsigned char k=0; k<5; k++) //Scan Columns
	{
	  for (char j=0; j<8; j++)
	  {
	    if (Hello_World[i][k] & 1<<(7-j))	//Should there be a letter pixel here?
	    {
	      LCD_Out(0x00, 0);			//yes - draw it in black
	      LCD_Out(0x00, 0);			
	    }
	    else 
	    {
	      LCD_Out(0xFF, 0);			//no - draw background in white
	      LCD_Out(0xFF, 0);
	    }
	  }
	}
	--h;
      }
    }
}

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

  switch (tick++) {
  	case 1000:
  		tick = 0;
  		GPIOC->ODR ^= (1 << 8);
  		break;
  }
  
  TimingDelay_Decrement();
}

int main(void)
{
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	SPI_Config();
	
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; 	// enable the clock to GPIOC
						//(RM0091 lists this as IOPCEN, not GPIOCEN)

	
	GPIOC->MODER = (1<<16) | (1<<6) | (1<<4) | (1<<2) | (1<<0);
	GPIOC->ODR |= LCD_CLK | LCD_SIO | LCD_CS | LCD_RST;
	

	SysTick_Config(SystemCoreClock/1000);
	
	GPIOC->ODR &= ~LCD_CS;
	_delay_ms(5);
	LCD_init();
	_delay_ms(5);
	GPIOC->ODR |= LCD_CS;
	
	StripedScreen();
	Hello_World();

/*
	LCD_Out(0x01,1);
	LCD_Out(0xFC,1);
	LCD_Out(0x00,0);
*/
	while(1);

}
