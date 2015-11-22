/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "main.h"
#include "_mc_config.h"
#include "_glcd_config.h"
#include "_dsp_config.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/*      Drive Motor Control    */
// PA4 -> GPIO LEFT
// PA5 -> GPIO RIGHT
// PA6 -> PWM  LEFT			
// PA7 -> PWM  RIGHT
// PC3 -> INT0 LEFT
// PB1 -> INT1 LEFT
// PC4 -> INT0 RIGHT
// PD2 -> INT1 RIGHT
#define 	PWM_MotorLeftPin 	GPIO_Pin_6
#define 	PWM_MotorRightPin 	GPIO_Pin_7
#define 	MotorRightPin 		GPIO_Pin_5
#define 	MotorLeftPin 		GPIO_Pin_4
#define 	MotorPort 			GPIOA
#define		MaxSpeed			1000

/********* Panel I/O Interface *********/
#define 	BUTTON1				!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)
#define 	BUTTON2				!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8)
#define 	BUTTON3				!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15)
#define 	BUTTON4			   	!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_14)
#define 	LED4_On				GPIO_WriteBit(GPIOC, GPIO_Pin_8, Bit_RESET)
#define 	LED4_Off			GPIO_WriteBit(GPIOC, GPIO_Pin_8, Bit_SET)
#define 	LED3_On				GPIO_WriteBit(GPIOC, GPIO_Pin_9, Bit_RESET)
#define 	LED3_Off			GPIO_WriteBit(GPIOC, GPIO_Pin_9, Bit_SET)
#define 	LED2_On				GPIO_WriteBit(GPIOC, GPIO_Pin_10, Bit_RESET)
#define 	LED2_Off			GPIO_WriteBit(GPIOC, GPIO_Pin_10, Bit_SET)
#define 	LED1_On				GPIO_WriteBit(GPIOC, GPIO_Pin_11, Bit_RESET)
#define 	LED1_Off			GPIO_WriteBit(GPIOC, GPIO_Pin_11, Bit_SET)

/****************** MCU Configuration *******************/
static __IO uint32_t	TimingDelay;
unsigned char  UB;

void RCC_Configuration(void);
void GPIO_Configuration(void);
void UART1_Configuration(void);
void Delay(__IO uint32_t nTime);

/*********************** Track **************************/
uint16_t 	PWM_ML				= 20;
uint16_t	PWM_MR				= 20;

void encoder_configspeed(void);
void Foward(uint16_t MLs,uint16_t MRs);
void Backward(uint16_t MLs,uint16_t MRs);
void Turn_Right(uint16_t MLs,uint16_t MRs);
void Turn_Left(uint16_t MLs,uint16_t MRs);
void track(void);
/*********************** Sensor *************************/
void senser_showlcd(void);
void MoveToPoint(void);
int Read_Sensor(void);
int Read_Sensor2(void);

/******************* Encoder Special ********************/
int32_t 	enCountLeft			= 0;
int32_t 	enCountRight 		= 0;
int8_t 		leftDirec  			= 0;
int8_t 		rightDirec  		= 0;
int8_t 		enCountRightDirec 	= 0;
int  		SpeedLeft;
int		  	SpeedRight;

/****** PID SPEED CONTROLLER ******/
int16_t		setPointRight 		= 0;
int16_t		setPointLeft 		= 0;
int16_t 	pidR				=	0;
int16_t 	pidL				=	0;
/*float kpL = 3.04; float kiL = 0.50; float kdL = 10.50;*/
float 		kpL 				= 4;//5.04;
float 		kiL 				= 0.09;//0.90; 
float 		kdL 				= 30;//28.50;

float 		kpR 				= 20;
float 		kiR 				= 2; 
float 		kdR 				= 20;

float 		erR;
float 		erpR;
float 		erL;
float 		erpL;
extern 		uint8_t pidEnable 	= 0;

void run(int16_t,int16_t);

/****** PID POSITION CONTROLLER ******/
void turnaround(int16_t,int16_t);
void eTurnover(void);
void eTurnleft(void);
void eTurnright(void);

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
//int8_t	toggleEnable = 1;
  RCC_Configuration();
  GPIO_Configuration();
  UART1_Configuration();
  TIM_Configuration();
  graphic_lcd_config();
  printf("\n\n\riNexus Robot Contest \n\r");
  lcd_clear_screen();
  lcd_clear_screen() ;
  Delay(50);
  NVIC_Configuration();
  while(!BUTTON4);
  Delay(500);
  /* Infinite loop */
  while (1)
  {
  		printf("SENSOR %d \n\r",Read_Sensor());
		//MoveToPoint();
		run(150,150);
		if(BUTTON1){
			LED1_On;
			Delay(500);
			eTurnleft();
		}else{
			LED1_Off;
		}
		if(BUTTON2){
			LED2_On;
			Delay(500);
			eTurnright();
		}else{
			LED2_Off;
		}
		if(BUTTON3){
			LED3_On;
			Delay(500);
			eTurnover();
		}else{
			LED3_Off;
		}


		//Set_speed(300,300);
		//run(150,150);
		//printf("L %d , R %d\n\r",enCountLeft,enCountRight);
		//turnaround(2000,2000);
		//Delay(500);
		//turnaround(-1200,1200);
		//turnaround(-2000,-2000);
		//Delay(500);
		//turnaround(1200,-1200);
		//printf("L %d : %d : %.2f , R %d : %d : %.2f\n\r",SpeedLeft,pidL,erL,SpeedRight,pidR,erR);
		//_eLeft(0); _eRight(0);
  		/*Foward(99,99);
		lcd_gotoxy(0,0);
		lcd_print_string("RIGHT",5);
		lcd_gotoxy(5*7,0);
		lcd_put_num(SpeedRight);
		lcd_gotoxy(0,1);
		lcd_print_string("LEFT ",5);
		lcd_gotoxy(5*7,1);
		lcd_put_num(SpeedLeft);

		lcd_gotoxy(0,3);
		lcd_print_string("PIDR ",5);
		lcd_gotoxy(5*7,3);
		lcd_put_num((uint16_t)pidR);

		lcd_gotoxy(0,4);
		lcd_print_string("PIDL ",5);
		lcd_gotoxy(5*7,4);
		lcd_put_num((uint16_t)pidL);	
		*/
		//printf("R %4d PID %4d E %.4f\t L %4d PID %4d E %.4f\n\r",SpeedRight,pidR,erR,SpeedLeft,pidL,erL);
		//printf("L %4d PID %4d E %.4f P%.2f P%.2f D%.2f\n\r",SpeedLeft,pidL,erL,kpL,kiL,kdL);
		/*UB = USART_ReceiveData(USART1);
		if(UB == '1'){
			kpL += 0.01;
		}
		else if(UB == '2'){
			kiL += 0.01;
		}
		else if(UB == '3'){
			kdL += 0.01;
		}
		else if(UB == '4'){
			kpR += 0.01;
		}
		else if(UB == '5'){
			kiR += 0.01;
		}
		else if(UB == '6'){
			kdR += 0.01;
		}
		*/
		//printf("PID %dLeft%d    %dRight%d\n\r",leftDirec,enCountLeft,enCountRightDirec,enCountRight);
		//track();
		//senser_showlcd();
  	  	//Foward(1,1)  ;
  }
}
/**
  *		@brief Sensor to LCD.
  *
  */
void senser_showlcd(void)
{
int B[5],F[5],a[3];
		B[0]=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11);
		B[1]=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_12);
		B[2]=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_6);
		B[3]=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7);
		B[4]=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14);
lcd_gotoxy(0,0);
lcd_put_char(B[0]+48);
lcd_gotoxy(1*8,0);
lcd_put_char(B[1]+48);
lcd_gotoxy(2*8,0);
lcd_put_char(B[2]+48);
lcd_gotoxy(3*8,0);
lcd_put_char(B[3]+48);
lcd_gotoxy(4*8,0);
lcd_put_char(B[4]+48);


		F[0]=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15);
		F[1]=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_13);
		F[2]=GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_5);
		F[3]=GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_6);
		F[4]=GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_7);

lcd_gotoxy(0*8,1);
lcd_put_char(F[0]+48);
lcd_gotoxy(1*8,1);
lcd_put_char(F[1]+48);
lcd_gotoxy(2*8,1);
lcd_put_char(F[2]+48);
lcd_gotoxy(3*8,1);
lcd_put_char(F[3]+48);
lcd_gotoxy(4*8,1);
lcd_put_char(F[4]+48);

a[2]=(B[0]*1)+(B[1]*2)+(B[2]*4)+(B[3]*8)+(B[4]*16);
a[0]=a[2]%10;

a[1]=a[2]/10;

lcd_gotoxy(3*8,2);
lcd_put_char((a[1])+48);
lcd_gotoxy(4*8,2);
lcd_put_char((a[0])+48);
//lcd_clear_screen() ; 
}
void track(void)
{
int B[5],a;		
		B[0]=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11);
		B[1]=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_12);
		B[2]=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_6);
		B[3]=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7);
		B[4]=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14);
a=(B[0]*1)+(B[1]*2)+(B[2]*4)+(B[3]*8)+(B[4]*16);
 
 if(a>9)
Foward(50,50);

else if(a<9)
Foward(50,50);

else 
Foward(50,50);


}
/**
  *		@brief Sensor.
  *
  */
int Read_Sensor(void)
{
int B[5],a;		
		B[0]=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11);
		B[1]=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_12);
		B[2]=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_6);
		B[3]=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7);
		B[4]=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14);
a=(B[0]*1)+(B[1]*2)+(B[2]*4)+(B[3]*8)+(B[4]*16);

return a;

}
int Read_Sensor2(void)
{
int B[5],a;		
		B[0]=GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_5);
		B[1]=GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_6);
		B[2]=GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_7);
		B[3]=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15);
		B[4]=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_13);
a=(B[0]*1)+(B[1]*2)+(B[2]*4)+(B[3]*8)+(B[4]*16);
										   //PC5,PC6,PC7			 //PB15		 //PA13
return a;

}
void MoveToPoint(void)
{
	 	 int s;
		 s=Read_Sensor();
		 pidEnable = 0;
 		/*pidEnable = 0;
		if((s<9)&&(s>7))
		{
			
			Set_speed(40,80);		 
		}
		
		else if((s>9)&&(s<11))
		{
			
			Set_speed(80,40); 	 
		}
		else if(s==9)
		{
			Set_speed(80,40); 		
		}	
		else{
			Set_speed(0,0);
		}
		Delay(30);	*/
		if(s > 14){
			//run(0,0);
			Set_speed(0,0);
			Delay(5000);
			//Set_speed(250,600);
		}
		/*// 14 to 10 ::: Error to Left
		else if((s > 9)&&(s <= 14)){
			//run(100,100 - s*7);
			pidEnable = 0;
			Set_speed(80,300 - s*21);		
		}
		// 4 to 8   ::: Error to Right
		else if((s >= 4)&&(s < 9)){
			//run(s*12,100);
			Set_speed(s*10,300);
		}*/
		// 11 to 10 ::: Error to Left
		else if((s > 9)&&(s <= 11)){
			pidEnable = 0;
			Set_speed(200,600 - s*50);		
		}
		// 14 to 12 ::: Error to Left
		else if((s >= 12)&&(s <= 14)){
			pidEnable = 0;
			Set_speed(200,600 - s*52);		
		}		
		// 6 to 8   ::: Error to Right
		else if((s >= 6)&&(s < 9)){
			Set_speed(s*25,600);
		}
		// 4 to 5   ::: Error to Right
		else if((s >= 4)&&(s <= 5)){
			Set_speed(s*10,600);
		}

		else if(s == 9){
			Set_speed(340,600);
		}
		else{
			//Set_speed(0,0);
			Set_speed(340,600);			
		}
								
}
/*---------------------------------- eSpecial ----------------------------------------*/
void eRun(void){
	turnaround(950,950);
}
void eTurnover(void){
	turnaround(-950,950);
}
void eTurnleft(void){
	turnaround(-450,450);
}
void eTurnright(void){
	turnaround(450,-450);
}
void turnaround(int16_t swL,int16_t swR){
// 9 5 0
// Left
int16_t erPosL = 0;
int16_t erpPosL = 0;
int16_t	IntPosL = 0;
int16_t setPositionL = 0;
int16_t pidPosL = 0;
float	kpPosL	= 1.2000;
float	kiPosL	= 0.0001;
float	kdPosL	= 1;
// Left
int16_t erPosR = 0;
int16_t erpPosR = 0;
int16_t	IntPosR = 0;
int16_t setPositionR = 0;
int16_t pidPosR = 0;
float	kpPosR	= 1.2000;
float	kiPosR	= 0.0001;
float	kdPosR	= 1;		
	pidEnable = 0;
	enCountLeft = 0;
	enCountRight = 0;

	setPositionL = swL * 4;
	setPositionR = swR * 4;
	
	// Overflow
	TimingDelay = 1000;

	do{	
		erPosL =  setPositionL - enCountLeft;
		IntPosL += kiPosL*erPosL;
		pidPosL = (kpPosL*erPosL) + IntPosL +(kdPosL * (erPosL - erpPosL));
		erpPosL = erPosL;

		erPosR =  setPositionR - enCountRight;
		IntPosR += kiPosR*erPosR;
		pidPosR = (kpPosR*erPosR) + IntPosR +(kdPosR * (erPosR - erpPosR));
		erpPosR = erPosR;

		if(pidPosL > 400)
			pidPosL = 400;
		else if(pidPosL < -400)
		pidPosL = -400;

		if(pidPosR > 400)
			pidPosR = 400;
		else if(pidPosR < -400)
		pidPosR = -400;

		_eLeft(pidPosL);
		_eRight(pidPosR);

		printf("%d : %d , %d : %d\n\r",erPosL,pidPosL,erPosR,pidPosR);
	}while((!(((erPosL >= -15)&&(erPosL <= 15)) && ((erPosR >= -15)&&(erPosR <= 15))))&&(!(TimingDelay <= 0)));
	enCountLeft = 0;
	enCountRight = 0;
	_eRight(0);
	_eLeft(0);	
}

void run(int16_t setLeft,int16_t setRight){
	pidEnable = 1;
	setPointLeft = setLeft;
	setPointRight = setRight;	
}
/*------------------------------------ The End ------------------------------------------*/
void Foward(uint16_t MLs,uint16_t MRs)
{
	pidEnable = 0;
	GPIO_WriteBit(MotorPort,MotorRightPin,Bit_SET);
	GPIO_WriteBit(MotorPort,MotorLeftPin,Bit_RESET);

	Set_speed((MLs)*200-1,(100-MRs)*200-1);	
   		//setPointRight	= 	MRs;
		//setPointLeft	=	MLs;
}
void Backward(uint16_t MLs,uint16_t MRs)
{
	pidEnable = 0;
	GPIO_WriteBit(MotorPort,MotorRightPin,Bit_RESET);
	GPIO_WriteBit(MotorPort,MotorLeftPin,Bit_SET);

	PWM_ML=(100-MLs)*200;
	PWM_MR=(MRs)*200-1;
	encoder_configspeed();
}
void Turn_Left(uint16_t MLs,uint16_t MRs)
{
	pidEnable = 0;
	GPIO_WriteBit(MotorPort,MotorRightPin,Bit_RESET);
	GPIO_WriteBit(MotorPort,MotorLeftPin,Bit_RESET);

	PWM_MR=(MLs)*200-1;
	PWM_ML=(MRs)*200-1;
	encoder_configspeed();


}
void Turn_Right(uint16_t MLs,uint16_t MRs)
{
	pidEnable = 0;
	GPIO_WriteBit(MotorPort,MotorRightPin,Bit_SET);
	GPIO_WriteBit(MotorPort,MotorLeftPin,Bit_SET);

	PWM_MR=(100-MLs)*200;
	PWM_ML=(100-MRs)*200;
	encoder_configspeed();


}
void encoder_configspeed()
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

  	/* -----------------------------------------------------------------------
    TIM3 Configuration: generate 1 PWM signals with 1 different duty cycles:
    TIM3CLK = 1 MHz, Prescaler = 143, TIM3 counter clock = 1 MHz
	Normal Clock for APB1(Low Speed Bus) = SystemClock/72
    TIM3 ARR Register = 19999 => TIM3 Frequency = TIM3 counter clock/(ARR + 1)
    TIM3 Frequency = 25 Hz.
    TIM3 Channel3 duty cycle = (TIM3_CCR1/ TIM3_ARR)* 100 = 50%
	----------------------------------------------------------------------- */

  	/* Time base configuration */
  	TIM_TimeBaseStructure.TIM_Period = 20000 - 1;
  	TIM_TimeBaseStructure.TIM_Prescaler = (144/8) - 1;
  	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  	/* PWM1 Mode configuration: Channel1 For MotorLeft */
  	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  	TIM_OCInitStructure.TIM_Pulse = PWM_ML;
  	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  	TIM_OC1Init(TIM3, &TIM_OCInitStructure);

  	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

  	/* PWM1 Mode configuration: Channel2 For MotorRiht */
  	TIM_OCInitStructure.TIM_Pulse = PWM_MR;
  	TIM_OC2Init(TIM3, &TIM_OCInitStructure);

  	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM3, ENABLE);

	/* TIM3 enable */
  	TIM_Cmd(TIM3, ENABLE);

}

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
void Delay(__IO uint32_t nTime)
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

/*******************************************************************************
* Function Name  : RCC_Configuration
* Description    : Configures the different system clocks.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_Configuration(void)
{
  ErrorStatus HSEStartUpStatus;
  /* RCC system reset(for debug purpose) */
  RCC_DeInit();

  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);

  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if(HSEStartUpStatus == SUCCESS)
  {
    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

    /* Flash 2 wait state */
    FLASH_SetLatency(FLASH_Latency_2);
 
    /* HCLK = SYSCLK */
    RCC_HCLKConfig(RCC_SYSCLK_Div1); 
  
    /* PCLK2 = HCLK */
    RCC_PCLK2Config(RCC_HCLK_Div1); 

    /* PCLK1 = HCLK/2 */
    RCC_PCLK1Config(RCC_HCLK_Div2);
    
    /* ADC 14MHz Max => 72MHz => 12MHz*/
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);

    /* PLLCLK = 8MHz * 9 = 72 MHz */
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

    /* Enable PLL */ 
    RCC_PLLCmd(ENABLE);

    /* Wait till PLL is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }

    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    /* Wait till PLL is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x08)
    {
    }
  }
	/* Enable RCC */	

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6 | RCC_APB1Periph_TIM3, ENABLE);	

  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA 
  						| RCC_APB2Periph_GPIOB  
						| RCC_APB2Periph_GPIOC
						| RCC_APB2Periph_GPIOF , ENABLE);
    
  /* Enable UART1 clocks */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

  /* Setup SysTick Timer for 1 msec interrupts  */
  if (SysTick_Config(SystemCoreClock / 1000))
  { 
    /* Capture error */ 
    while (1);
  } 
}

/*******************************************************************************
* Function Name  : GPIO_Configuration
* Description    : Configures the different GPIO ports.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void GPIO_Configuration(void)
{
  	GPIO_InitTypeDef GPIO_InitStructure;
 	EXTI_InitTypeDef EXTI_InitStructure;

  	/* Configure USART1 Rx (PA.10) as input floating */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  	/* Configure USART1 Tx (PA.09) as alternate function push-pull */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  	/* Motor PWM */
  	// PA6 -> PWM  LEFT		
  	// PA7 -> PWM  RIGHT
  	/* Motor PWM */
  	GPIO_InitStructure.GPIO_Pin = PWM_MotorLeftPin|PWM_MotorRightPin;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  	GPIO_Init(MotorPort, &GPIO_InitStructure);
  	/*Motor Left/Right*/
  	GPIO_InitStructure.GPIO_Pin = MotorLeftPin|MotorRightPin;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  	GPIO_Init(MotorPort, &GPIO_InitStructure);

	// PA4 -> GPIO LEFT
	// PA5 -> GPIO RIGHT
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4	
								| GPIO_Pin_5;		
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;			   		
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;					
	GPIO_Init(GPIOA, &GPIO_InitStructure);	

	// PB0 -> INT0 LEFT
	// PB1 -> INT1 LEFT
	// PC4 -> INT0 RIGHT
	// PD2 -> INT1 RIGHT
	/*		GPIO Initial	   */
   	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1; 						//PB1
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;					    //PC3
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;					    //PC4
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;						//PD2
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/*		GPIO to EXTI Initial	   */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource1);		//PB1
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource3);	    //PC3
 	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource4);	    //PC4
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD,GPIO_PinSource2);	    //PD2
   	
	/*		EXTI Initial	   */
	EXTI_InitStructure.EXTI_Line = EXTI_Line3 		   				// EXTI_Line3
								| EXTI_Line1 						// EXTI_Line1
								| EXTI_Line2 						// EXTI_Line2
								| EXTI_Line4;						// EXTI_Line4
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/*Sensor init terminal left*/
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11|GPIO_Pin_12;				 //PA11,PA12
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_14;				 //PB6,PB7,PB14
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);		   
	/*Sensor init terminal Right*/
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;				 //PC5,PC6,PC7
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_15;				 //PB15
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_13;				 //PA13
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);
	/* Disable JTAG  */
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);

	/* Button Configuration  **********************************************************/
 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_8 | GPIO_Pin_14 | GPIO_Pin_15;
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
 	GPIO_Init(GPIOA, &GPIO_InitStructure);	 

	/* Configure LED --------------------------------------------------*/
 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
 	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_WriteBit(GPIOC, GPIO_Pin_8, Bit_SET);
	GPIO_WriteBit(GPIOC, GPIO_Pin_9, Bit_SET);
	GPIO_WriteBit(GPIOC, GPIO_Pin_10, Bit_SET);
	GPIO_WriteBit(GPIOC, GPIO_Pin_11, Bit_SET);
}

void UART1_Configuration(void)
{
  USART_InitTypeDef USART_InitStructure;
  /* USART1 configuration ----------------------------------------------------*/
  /* USART configured as follow:
        - BaudRate = 115200 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  /* Configure USART1 */
  USART_Init(USART1, &USART_InitStructure);

  /* Enable the USART1 */
  USART_Cmd(USART1, ENABLE); 
}
/* Eng Programs ================================================================*/
#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
