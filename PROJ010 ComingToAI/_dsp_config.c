/*		DSP Part
		_dsp_config.c	
*/
#include "_dsp_config.h"
#include "stm32f10x_tim.h"
/* Private typedef -----------------------------------------------------------*/
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;

/* Extern Lib ----------------------------------------------------------------*/
extern 		void	Delay(__IO uint32_t nTime);
extern 		int 	measuredSensor(void);
extern		uint8_t		Color_Flag			;

extern __IO uint16_t		ADC1_BUFFER[4];

/*	Private Function	------------------------------------------------------*/
void Set_speed(int16_t Left,int16_t Right)
{
TIM_OCInitTypeDef  TIM_OCInitStructure;
	if(Left > 1000){
		Left = 1000;
	}else if(Left < -1000){
		Left = -1000;
	}
	if(Right > 1000){
		Right = 1000;
	}else if(Right < -1000){
		Right = -1000;
	}
/*TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

  	TIM_TimeBaseStructure.TIM_Period = 20000;
  	TIM_TimeBaseStructure.TIM_Prescaler = 20;
  	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);	
*/  		
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
				
	if(Left > 0){
		GPIO_WriteBit(GPIOA,GPIO_Pin_5,Bit_SET);
  		TIM_OCInitStructure.TIM_Pulse = 20000 - Left * 20;
  		TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	}
	else{
		Left = Left*(-1);
		GPIO_WriteBit(GPIOA,GPIO_Pin_5,Bit_RESET);
  		TIM_OCInitStructure.TIM_Pulse = Left * 20;
  		TIM_OC2Init(TIM3, &TIM_OCInitStructure);	
	}
	
	if(Right > 0){
		GPIO_WriteBit(GPIOA,GPIO_Pin_4,Bit_RESET);
  		TIM_OCInitStructure.TIM_Pulse = Right * 20;
  		TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	}
	else{
		Right = Right*(-1);
		GPIO_WriteBit(GPIOA,GPIO_Pin_4,Bit_SET);
  		TIM_OCInitStructure.TIM_Pulse = 20000 - Right * 20;
  		TIM_OC1Init(TIM3, &TIM_OCInitStructure); 	
	}
	  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);	
		TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);	
		TIM_ARRPreloadConfig(TIM3, ENABLE);		
  	TIM_Cmd(TIM3, ENABLE);			
}

void _eLeft(int16_t Left){
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

  		/* Time base configuration */
  		TIM_TimeBaseStructure.TIM_Period = 20000;
  		TIM_TimeBaseStructure.TIM_Prescaler = 20;
  		TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  		TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);	
		
	if(Left >= 0){
		GPIO_WriteBit(GPIOA,GPIO_Pin_5,Bit_SET);
  		/* PWM1 Mode configuration: Channel1 For MotorLeft */
  		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  		TIM_OCInitStructure.TIM_Pulse = 20000 - Left * 20;
  		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  		TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	}
	else{
		Left = Left*(-1);
		GPIO_WriteBit(GPIOA,GPIO_Pin_5,Bit_RESET);
  		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  		TIM_OCInitStructure.TIM_Pulse = Left * 20;
  		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  		TIM_OC2Init(TIM3, &TIM_OCInitStructure);
		
		
	}	
		//TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);	
		//TIM_ARRPreloadConfig(TIM3, ENABLE);

		/* TIM3 enable */
  		TIM_Cmd(TIM3, ENABLE);
}

void _eRight(int16_t Right){
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

  		/* Time base configuration */
  		TIM_TimeBaseStructure.TIM_Period = 20000;
  		TIM_TimeBaseStructure.TIM_Prescaler = 20;
  		TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  		TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);	
		
	if(Right >= 0){
		GPIO_WriteBit(GPIOA,GPIO_Pin_4,Bit_RESET);
  		/* PWM1 Mode configuration: Channel1 For MotorLeft */
  		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  		TIM_OCInitStructure.TIM_Pulse = Right * 20;
  		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  		TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	}
	else{
		Right = Right*(-1);
		GPIO_WriteBit(GPIOA,GPIO_Pin_4,Bit_SET);
	  	// PWM1 Mode configuration: Channel1 For MotorLeft
  		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  		TIM_OCInitStructure.TIM_Pulse = 20000 - Right * 20;
  		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  		TIM_OC1Init(TIM3, &TIM_OCInitStructure); 

		
	}	
		//TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);	
		//TIM_ARRPreloadConfig(TIM3, ENABLE);

		/* TIM3 enable */
  		TIM_Cmd(TIM3, ENABLE);	
}

/**
  * @ brief TRACKING RIGHT
  *
  */
void TrackingRight (void){
			Set_speed(0,0);
			Set_speed(800,-800);
			Delay(200);
			do{
				Set_speed(600,-600);
				Delay(1);
			}while(measuredSensor() != -3);
			Set_speed(-80,80);
			Delay(100);	
			Set_speed(0,0);
}

/**
  * @ brief TRACKING LEFT
  *
  */
void TrackingLeft (void){
			Set_speed(0,0);
			Set_speed(-800,800);
			Delay(200);
			do{
				Set_speed(-600,600);
				Delay(1);
			}while(measuredSensor() != 3);
			Set_speed(80,-80);
			Delay(100);
			Set_speed(0,0);	
}
/**
  * @ brief MOVE TO POINT V1
  *
  */
void MoveToPoint(int linecount)
{
	 	 int s,count=0;
		 
		 
	while(count!=linecount){
		
			s=measuredSensor();
			if(s==100)
			{
				
				if(++count>=linecount) {
					Set_speed(-40,-20);
					break;
				}
		 		while(s==100){
		 			//Set_speed(1000,1000);
					Set_speed(910,1000);
					s=measuredSensor();
		 		}
			}
			else if(s==-2)
			{  
				Set_speed(910,1000-200) ;
				
				Delay(1);
			}
			else if(s==2)
			{  
				Set_speed(910-350,1000)	;
			
				Delay(1);
			}
			else if(s==1)
			{  
				Set_speed(910-200,1000);
			
				Delay(1);
			}
			else if(s==-1)
			{  
				Set_speed(910,1000-100);
				Delay(1);
			}
			else if(s==0)
			{  
				Set_speed(825,1000);
				Delay(1);
			}
			else if(s==-3)
			{  
				Set_speed(910,1000-650) ;
				Delay(1);
			}
			else if(s==3)
			{  
				Set_speed(910-650,1000)	;
				Delay(1);
			}
			else if(s<-3)
			{  
				Set_speed(910,200) ;
				Delay(1);
			}
			else if(s>3)
			{  
				Set_speed(150,1000)	;
				Delay(1);
			}
			else{
				Set_speed(0,0);
			} 
	 }		
	 			
	 		  Set_speed(825,1000);
			  Delay(150);
 			Set_speed(0,0);					
}

/**
  * @ brief MOVE TO POINT V2
  *
  */
void MoveToPointV2(int linecount)
{
	 	 int s,count=0;
		 
		 
	while(count!=linecount){
		
			s=measuredSensor();
			if(s==100)
			{
				
				if(++count>=linecount) {
					Set_speed(-40,-20);
					break;
				}
		 		while(s==100){
		 			//Set_speed(1000,1000);
					Set_speed(750,1000);
					s=measuredSensor();
		 		}
			}
			else if(s==-2)
			{  
				Set_speed(750,1000-200) ;
				
				Delay(1);
			}
			else if(s==2)
			{  
				Set_speed(750-110,1000)	;
			
				Delay(1);
			}
			else if(s==1)
			{  
				Set_speed(750-75,1000);
			
				Delay(1);
			}
			else if(s==-1)
			{  
				Set_speed(750,1000-100);
				Delay(1);
			}
			else if(s==0)
			{  
				Set_speed(750,1000);
				Delay(1);
			}
			else if(s==-3)
			{  
				Set_speed(750,1000-650) ;
				Delay(1);
			}
			else if(s==3)
			{  
				Set_speed(750-330,1000)	;
				Delay(1);
			}
			else if(s<-3)
			{  
				Set_speed(750,200) ;
				Delay(1);
			}
			else if(s>3)
			{  
				Set_speed(100,1000)	;
				Delay(1);
			}
			else{
				Set_speed(0,0);
			} 
	 }		
	 			
	 		  Set_speed(750,1000);
			  Delay(150);
 			Set_speed(0,0);					
}

/**
  *	@brief MOVE TO BOX
  *
  */
void MoveToBox(void){
		 
	do{	
	 		Set_speed(600,600);
			Delay(5);
	} while(ADC1_BUFFER[1] <= 2800);
	Set_speed(600,600);
	Delay(75);
	Set_speed(-20,-20);
	Delay(150);
 	Set_speed(0,0);			
///////////////////////////
}

/**
  * @ brief SERVO CONFIGURATION
  *
  */
void TIM_Configuration(void){

	/* DC Motor*/
  	TIM_TimeBaseStructure.TIM_Period = 20000;
  	TIM_TimeBaseStructure.TIM_Prescaler = 20;
  	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);	

  	/* Time base configuration 28799*/
  	TIM_TimeBaseStructure.TIM_Period = 25000;
  	TIM_TimeBaseStructure.TIM_Prescaler = 20;
  	TIM_TimeBaseStructure.TIM_ClockDivision = 2;
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);

	TIM_ARRPreloadConfig(TIM6, ENABLE);

	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);

	/* TIM3 enable */
  	TIM_Cmd(TIM6, ENABLE);

	/* Servo Drive */	
  	TIM_TimeBaseStructure.TIM_Period = 1333;
  	TIM_TimeBaseStructure.TIM_Prescaler = 1169;
  	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  	TIM_OCInitStructure.TIM_Pulse = 750;
  	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  	TIM_OC2Init(TIM2, &TIM_OCInitStructure);

  	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);

  	TIM_OCInitStructure.TIM_Pulse = 750;
  	TIM_OC3Init(TIM2, &TIM_OCInitStructure);

  	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
	
	TIM_OCInitStructure.TIM_Pulse = 750;
  	TIM_OC4Init(TIM2, &TIM_OCInitStructure);

  	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM2, ENABLE);

  	TIM_Cmd(TIM2, ENABLE);	

}
/**
  * @ brief SERVO 1 DRIVE
  *
  */
void Servo1_Drive(uint8_t degree)
{
	TIM_OCInitStructure.TIM_Pulse = 20+(degree);
  	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
}
/**
  * @ brief SERVO 2 DRIVE
  *
  */
void Servo2_Drive(uint8_t degree)
{
	TIM_OCInitStructure.TIM_Pulse = 20+(degree);
  	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
}
/**
  * @ brief SERVO 3 DRIVE
  *
  */
void Servo3_Drive(uint8_t degree)
{
	TIM_OCInitStructure.TIM_Pulse = 20+(degree);
  	TIM_OC4Init(TIM2, &TIM_OCInitStructure);
}
/**
  *	 @ brief Drop Box
  */
void LoadBox(void){
	TIM_Cmd(TIM2,ENABLE);
	Servo1_Drive(0);
	Servo3_Drive(140);

}
/**
  *	 @ Download Box
  */
void DropBox(void){
	
	Servo1_Drive(125);
	Servo3_Drive(15);	
	Delay(500);
	TIM_Cmd(TIM2,DISABLE);
}



