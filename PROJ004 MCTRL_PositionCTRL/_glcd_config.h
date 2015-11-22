/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

  // Define LCD-NOKIA5110 PinIO Interface Mask Bit 
  #define  LCD5110_SCE_PIN 		       GPIO_Pin_8			// SCE = PB[8]
  #define  LCD5110_SCE_PORT		       GPIOB
  #define  RCC_APB2Periph_GPIO_SCE     RCC_APB2Periph_GPIOB

  #define  LCD5110_RES_PIN		       GPIO_Pin_9			// RES = PB[9]
  #define  LCD5110_RES_PORT		       GPIOB
  #define  RCC_APB2Periph_GPIO_RES     RCC_APB2Periph_GPIOB

  #define  LCD5110_DC_PIN		       GPIO_Pin_10			// DC = PB[10]
  #define  LCD5110_DC_PORT	           GPIOB
  #define  RCC_APB2Periph_GPIO_DC      RCC_APB2Periph_GPIOB

  #define  LCD5110_SDIN_PIN	           GPIO_Pin_11			// SDIN = PB[11]
  #define  LCD5110_SDIN_PORT	       GPIOB
  #define  RCC_APB2Periph_GPIO_SDIN    RCC_APB2Periph_GPIOB

  #define  LCD5110_SCLK_PIN		       GPIO_Pin_12			// SCLK = PB[12]
  #define  LCD5110_SCLK_PORT	       GPIOB
  #define  RCC_APB2Periph_GPIO_SCLK    RCC_APB2Periph_GPIOB

  #define  LCD5110_LED_PIN	           GPIO_Pin_13			// LED = PB[13]
  #define  LCD5110_LED_PORT	           GPIOB
  #define  RCC_APB2Periph_GPIO_LED     RCC_APB2Periph_GPIOB

  #define  LCD5110_SCE_HI()    	       GPIO_WriteBit(LCD5110_SCE_PORT,LCD5110_SCE_PIN,Bit_SET)
  #define  LCD5110_SCE_LO()		       GPIO_WriteBit(LCD5110_SCE_PORT,LCD5110_SCE_PIN,Bit_RESET)

  #define  LCD5110_RES_HI()    	       GPIO_WriteBit(LCD5110_RES_PORT,LCD5110_RES_PIN,Bit_SET)  
  #define  LCD5110_RES_LO() 		   GPIO_WriteBit(LCD5110_RES_PORT,LCD5110_RES_PIN,Bit_RESET)

  #define  LCD5110_DC_HI() 	           GPIO_WriteBit(LCD5110_DC_PORT,LCD5110_DC_PIN,Bit_SET)
  #define  LCD5110_DC_LO() 	           GPIO_WriteBit(LCD5110_DC_PORT,LCD5110_DC_PIN,Bit_RESET)

  #define  LCD5110_SDIN_HI() 	       GPIO_WriteBit(LCD5110_SDIN_PORT,LCD5110_SDIN_PIN,Bit_SET)
  #define  LCD5110_SDIN_LO()	       GPIO_WriteBit(LCD5110_SDIN_PORT,LCD5110_SDIN_PIN,Bit_RESET)

  #define  LCD5110_SCLK_HI() 	       GPIO_WriteBit(LCD5110_SCLK_PORT,LCD5110_SCLK_PIN,Bit_SET)
  #define  LCD5110_SCLK_LO() 	       GPIO_WriteBit(LCD5110_SCLK_PORT,LCD5110_SCLK_PIN,Bit_RESET)
   
  #define  LCD5110_LED_HI() 	       GPIO_WriteBit(LCD5110_LED_PORT,LCD5110_LED_PIN,Bit_SET)
  #define  LCD5110_LED_LO()	           GPIO_WriteBit(LCD5110_LED_PORT,LCD5110_LED_PIN,Bit_RESET)
 
 /* Exported Function --------------------------------------------------------------------------*/
void graphic_lcd_config(void);
void SPI_Send_Byte(unsigned char SPI_Data);						// Send Byte(Software SPI)
void lcd_write_data(unsigned char DataByte);					// Write Data to LCD
void lcd_write_command(unsigned char CommandByte);				// Write Command to LCD
void lcd_initial(void);											// Initial LCD Nokia-5110
void lcd_clear_screen(void);									// Clear Screen Display
void lcd_fill_picture(void);									// Fill Picture Display
void lcd_gotoxy(unsigned char x,unsigned char y); 				// Set Cursor X(0..83),Y(0..5) 
void lcd_put_char(unsigned char character); 					// Put String(1 Char)
void lcd_print_string(unsigned char *string , 
                      unsigned char CharCount); 				// Print String to LCD
void Delay_us(vu32 nCount);
void lcd_put_num(unsigned int);
 
 
 

