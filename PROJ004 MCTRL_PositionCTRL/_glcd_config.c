/* Includes ------------------------------------------------------------------*/
#include "_glcd_config.h"
#include "stdio.h"
/* Private typedef ------------------------------------------------------------*/
/* Private define --------------------------------------------------------------*/
/* Private macro ---------------------------------------------------------------*/
/* Private variables -----------------------------------------------------------*/
/************************************/
/* Font Code Size 5:Byte/Font Table */
/************************************/
const unsigned char tab_font[ ] = 
{
 0x00, 0x00, 0x00, 0x00, 0x00,   // sp  
 0x00, 0x00, 0x2f, 0x00, 0x00,   // !  
 0x00, 0x07, 0x00, 0x07, 0x00,   // "  
 0x14, 0x7f, 0x14, 0x7f, 0x14,   // #  
 0x24, 0x2a, 0x7f, 0x2a, 0x12,   // $  
 0x62, 0x64, 0x08, 0x13, 0x23,   // %  
 0x36, 0x49, 0x55, 0x22, 0x50,   // &  
 0x00, 0x05, 0x03, 0x00, 0x00,   // ’  
 0x00, 0x1c, 0x22, 0x41, 0x00,   // (  
 0x00, 0x41, 0x22, 0x1c, 0x00,   // )  
 0x14, 0x08, 0x3E, 0x08, 0x14,   // *  
 0x08, 0x08, 0x3E, 0x08, 0x08,   // +  
 0x00, 0x00, 0xA0, 0x60, 0x00,   // ,  
 0x08, 0x08, 0x08, 0x08, 0x08,   // -  
 0x00, 0x60, 0x60, 0x00, 0x00,   // .  
 0x20, 0x10, 0x08, 0x04, 0x02,   // /  
 0x3E, 0x51, 0x49, 0x45, 0x3E,   // 0  
 0x00, 0x42, 0x7F, 0x40, 0x00,   // 1  
 0x42, 0x61, 0x51, 0x49, 0x46,   // 2  
 0x21, 0x41, 0x45, 0x4B, 0x31,   // 3  
 0x18, 0x14, 0x12, 0x7F, 0x10,   // 4  
 0x27, 0x45, 0x45, 0x45, 0x39,   // 5  
 0x3C, 0x4A, 0x49, 0x49, 0x30,   // 6  
 0x01, 0x71, 0x09, 0x05, 0x03,   // 7  
 0x36, 0x49, 0x49, 0x49, 0x36,   // 8  
 0x06, 0x49, 0x49, 0x29, 0x1E,   // 9  
 0x00, 0x36, 0x36, 0x00, 0x00,   // :  
 0x00, 0x56, 0x36, 0x00, 0x00,   // ;  
 0x08, 0x14, 0x22, 0x41, 0x00,   // <  
 0x14, 0x14, 0x14, 0x14, 0x14,   // =  
 0x00, 0x41, 0x22, 0x14, 0x08,   // >  
 0x02, 0x01, 0x51, 0x09, 0x06,   // ?  
 0x32, 0x49, 0x59, 0x51, 0x3E,   // @  
 0x7C, 0x12, 0x11, 0x12, 0x7C,   // A  
 0x7F, 0x49, 0x49, 0x49, 0x36,   // B  
 0x3E, 0x41, 0x41, 0x41, 0x22,   // C  
 0x7F, 0x41, 0x41, 0x22, 0x1C,   // D  
 0x7F, 0x49, 0x49, 0x49, 0x41,   // E  
 0x7F, 0x09, 0x09, 0x09, 0x01,   // F  
 0x3E, 0x41, 0x49, 0x49, 0x7A,   // G  
 0x7F, 0x08, 0x08, 0x08, 0x7F,   // H  
 0x00, 0x41, 0x7F, 0x41, 0x00,   // I  
 0x20, 0x40, 0x41, 0x3F, 0x01,   // J  
 0x7F, 0x08, 0x14, 0x22, 0x41,   // K  
 0x7F, 0x40, 0x40, 0x40, 0x40,   // L  
 0x7F, 0x02, 0x0C, 0x02, 0x7F,   // M  
 0x7F, 0x04, 0x08, 0x10, 0x7F,   // N  
 0x3E, 0x41, 0x41, 0x41, 0x3E,   // O  
 0x7F, 0x09, 0x09, 0x09, 0x06,   // P  
 0x3E, 0x41, 0x51, 0x21, 0x5E,   // Q  
 0x7F, 0x09, 0x19, 0x29, 0x46,   // R  
 0x46, 0x49, 0x49, 0x49, 0x31,   // S  
 0x01, 0x01, 0x7F, 0x01, 0x01,   // T  
 0x3F, 0x40, 0x40, 0x40, 0x3F,   // U  
 0x1F, 0x20, 0x40, 0x20, 0x1F,   // V  
 0x3F, 0x40, 0x38, 0x40, 0x3F,   // W  
 0x63, 0x14, 0x08, 0x14, 0x63,   // X  
 0x07, 0x08, 0x70, 0x08, 0x07,   // Y  
 0x61, 0x51, 0x49, 0x45, 0x43,   // Z  
 0x00, 0x7F, 0x41, 0x41, 0x00,   // [  
 0x55, 0x2A, 0x55, 0x2A, 0x55,   // 55  
 0x00, 0x41, 0x41, 0x7F, 0x00,   // ]  
 0x04, 0x02, 0x01, 0x02, 0x04,   // ^  
 0x40, 0x40, 0x40, 0x40, 0x40,   // _  
 0x00, 0x01, 0x02, 0x04, 0x00,   // ’  
 0x20, 0x54, 0x54, 0x54, 0x78,   // a  
 0x7F, 0x48, 0x44, 0x44, 0x38,   // b  
 0x38, 0x44, 0x44, 0x44, 0x20,   // c  
 0x38, 0x44, 0x44, 0x48, 0x7F,   // d  
 0x38, 0x54, 0x54, 0x54, 0x18,   // e  
 0x08, 0x7E, 0x09, 0x01, 0x02,   // f  
 0x18, 0xA4, 0xA4, 0xA4, 0x7C,   // g  
 0x7F, 0x08, 0x04, 0x04, 0x78,   // h  
 0x00, 0x44, 0x7D, 0x40, 0x00,   // i  
 0x40, 0x80, 0x84, 0x7D, 0x00,   // j  
 0x7F, 0x10, 0x28, 0x44, 0x00,   // k  
 0x00, 0x41, 0x7F, 0x40, 0x00,   // l  
 0x7C, 0x04, 0x18, 0x04, 0x78,   // m  
 0x7C, 0x08, 0x04, 0x04, 0x78,   // n  
 0x38, 0x44, 0x44, 0x44, 0x38,   // o  
 0xFC, 0x24, 0x24, 0x24, 0x18,   // p  
 0x18, 0x24, 0x24, 0x18, 0xFC,   // q  
 0x7C, 0x08, 0x04, 0x04, 0x08,   // r  
 0x48, 0x54, 0x54, 0x54, 0x20,   // s  
 0x04, 0x3F, 0x44, 0x40, 0x20,   // t  
 0x3C, 0x40, 0x40, 0x20, 0x7C,   // u  
 0x1C, 0x20, 0x40, 0x20, 0x1C,   // v  
 0x3C, 0x40, 0x30, 0x40, 0x3C,   // w  
 0x44, 0x28, 0x10, 0x28, 0x44,   // x  
 0x1C, 0xA0, 0xA0, 0xA0, 0x7C,   // y  
 0x44, 0x64, 0x54, 0x4C, 0x44,   // z  
 0x00, 0x08, 0x36, 0x41, 0x00,   // {  
 0x00, 0x00, 0x7F, 0x00, 0x00,   // |  
 0x00, 0x41, 0x36, 0x08, 0x00,   // }  
 0x08, 0x10, 0x08, 0x04, 0x08    // ~  
};

/* Private function  -----------------------------------------------------------*/
/**
  *	@brief setup graphic lcd.
  */
void graphic_lcd_config(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	/* Configure IO connected to LCD-NOKIA5110 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_SCE, ENABLE);
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = LCD5110_SCE_PIN;
	GPIO_Init(LCD5110_SCE_PORT, &GPIO_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_RES, ENABLE);
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = LCD5110_RES_PIN;
	GPIO_Init(LCD5110_RES_PORT, &GPIO_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_DC, ENABLE);
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = LCD5110_DC_PIN;
	GPIO_Init(LCD5110_DC_PORT, &GPIO_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_LED, ENABLE);
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = LCD5110_LED_PIN;
	GPIO_Init(LCD5110_LED_PORT, &GPIO_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_SCLK, ENABLE);
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = LCD5110_SCLK_PIN;
	GPIO_Init(LCD5110_SCLK_PORT, &GPIO_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_SDIN, ENABLE);
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = LCD5110_SDIN_PIN;
	GPIO_Init(LCD5110_SDIN_PORT, &GPIO_InitStructure);
	
	/* Initial GPIO Signal Interface LCD Nokia-5110 */
	LCD5110_SCLK_LO();											// Standby SCLK
	LCD5110_RES_LO();												// Active Reset
	LCD5110_RES_HI();												// Normal Operation  
	LCD5110_DC_HI(); 												// D/C = High("1"=Data)
	LCD5110_LED_HI();												// LED = High(ON LED)
	LCD5110_SCE_LO();												// SCE = Low(Enable)

	/* Start Initial & Display Character to LCD */
	lcd_initial();                								// Initial LCD
	lcd_clear_screen();              								// Clear Screen Display
	
	lcd_gotoxy(0,0);											// Set Cursor = Line-1
    lcd_print_string("  ROBOT CLUB  ",14);						// Display LCD Line-1       
    lcd_print_string("  ENGINEERING ",14);						// Display LCD Line-2  
    lcd_print_string("    iNexus    ",14);						// Display LCD Line-3 
    lcd_print_string("Are you Ready?",14);						// Display LCD Line-4  
    lcd_print_string("We will got it",14);						// Display LCD Line-5 
    lcd_print_string("Go Go Go Go Go",14);						// Display LCD Line-6 
	
	LCD5110_LED_LO();												// LED = LO(OFF LED)	

    Delay_us(2000000);
	
}

/**
  *	@brief printf number.
  */
void lcd_put_num(unsigned int inum){
	unsigned int i;
	unsigned char j = 0;
	unsigned char keep[5];
	for( i=1000 ; i > 0 ; i/=10){
		if(inum%i <= inum){
			keep[j] = inum/i;
			// char output
			//printf("[%d]- %d  ",j,keep[j]);
			lcd_put_char(keep[j]+48);
			// remain numder = inum % i
			inum = inum%i;
			j++;
		}
	}
}

// *****************************************************************************
// WriteSpiData.c
// Writes 8-bit command to LCD display via SPI interface
// *****************************************************************************
void SPI_Send_Byte(unsigned char SPI_Data)					// Send Byte(Software SPI)
{
  unsigned int Bit = 0;										// Bit Counter

  LCD5110_SCLK_LO();										// Standby SCLK 
   
  for (Bit = 0; Bit < 8; Bit++)								// 8 Bit Write
  {
    LCD5110_SCLK_LO(); 										// Standby SCLK 

	if((SPI_Data&0x80)>>7) 
	{ 
	  LCD5110_SDIN_HI();  
	}
	else
	{ 
	  LCD5110_SDIN_LO();
	}

	LCD5110_SCLK_HI();										// Strobe Signal Bit(SDIN)

	SPI_Data <<= 1;	 										// Next Bit Data
  } 
  LCD5110_SCLK_LO();										// Standby SCLK 
}

/********************************/
/* Write Data or Command to LCD */
/* D/C = "0" = Write Command    */
/* D/C = "1" = Write Display    */
/********************************/ 
void lcd_write_data(unsigned char DataByte) 
{                
  LCD5110_DC_HI(); 												// Active DC = High("1"=Data)    
  SPI_Send_Byte(DataByte);
}    

/********************************/
/* Write Data or Command to LCD */
/* D/C = "0" = Write Command    */
/* D/C = "1" = Write Display    */
/********************************/ 
void lcd_write_command(unsigned char CommandByte) 
{                  
  LCD5110_DC_LO(); 												// Active DC = Low("0"=Command)  
  SPI_Send_Byte(CommandByte);
}    

/**************************/
/* Initial LCD Nokia-5110 */
/**************************/          
void lcd_initial(void)      
{   
  LCD5110_RES_LO();												// Active Reset
  LCD5110_RES_HI();												// Normal Operation

  lcd_write_command(32+1);       								// Function Set = Extend Instruction(00100+PD,V,H=00100+0,0,1)
  lcd_write_command(128+38);     								// Set VOP(1+VOP[6..0] = 1+0100110)
  lcd_write_command(4+3);            							// Temp Control(000001+TC1,TC0=000001+1,1)
  lcd_write_command(16+3);       								// Bias System(00010,BS2,BS1,BS0=00010,0,1,1)

  lcd_write_command(32+0);       								// Function Set = Basic Instruction(00100+PD,V,H = 00100+0,0,0)
  lcd_write_command(12);         								// Display Control = Normal Mode(00001D0E=00001100)
}   

/****************************/
/* Clear Screen Display LCD */
/****************************/
void lcd_clear_screen(void)       
{  
  unsigned int  i=0;   											// Memory Display(Byte) Counter
    
  lcd_write_command(128+0);       								// Set X Position = 0(0..83)
  lcd_write_command(64+0);         								// Set Y Position = 0(0..5)
  
  for(i=0;i<504;i++)   											// All Display RAM = 504 Byte  
  lcd_write_data(0);  											// Clear Screen Display
}     

/***************************/
/* Set Cursor X,Y Position */
/* X[0-83]: 84 Column Data */
/* Y[0-5] : 6 Row(48 Dot)  */
/***************************/
void lcd_gotoxy(unsigned char x,unsigned char y)  
{  
  lcd_write_command(128+x);  	 								// Set X Position(1+x6,x5,x4,x3,x2,x1,x0)
  lcd_write_command(64+y);  									// Set Y Position(01000+y2,y1,y0)
}  

/***************************/
/* Put Char to LCD Display */
/***************************/
void lcd_put_char(unsigned char character) 
{  
  unsigned char font_size_count = 0;  							// Font Size Counter
  unsigned int  font_data_index;  								// Font Data Pointer

  font_data_index = character-32;      							// Skip 0x00..0x1F Font Code
  font_data_index = font_data_index*5;  						// 5 Byte / Font       
  
  while(font_size_count<5)                          			// Get 5 Byte Font & Display on LCD
  {  													
    lcd_write_data(tab_font[font_data_index]);    				// Get Data of Font From Table & Write LCD
    font_size_count++;  										// Next Byte Counter
    font_data_index++;  										// Next	Byte Pointer
  }  
  lcd_write_data(0);											// 1 Pixel Dot Space
}    

/*******************************/
/* Print String to LCD Display */
/*******************************/
void lcd_print_string(unsigned char *string , unsigned char CharCount) 
{          
  unsigned char i=0;  											// Dummy Character Count

  while(i<CharCount)  
  {    
    lcd_put_char(string[i]);									// Print 1-Char to LCD
    i++;                           								// Next Character Print
  }  
}

/*******************************************************************************
* Function Name  : Delay
* Description    : Inserts a delay time.
* Input          : nCount: specifies the delay time length.
* Output         : None
* Return         : None
*******************************************************************************/
void Delay_us(vu32 nCount)
{
  for(; nCount != 0; nCount--);
}


////////////////////////////////////////////////////////////////////////////////////////////
