/*		DSP Part
		_dsp_config.c	
*/
#include "_dsp_config.h"

/*	Private Function	--------------------------------------------------------------*/
void Set_speed(int16_t Left,int16_t Right)
{
TIM_OCInitTypeDef  TIM_OCInitStructure;
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
				
	if(Left >= 0){
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
	
	if(Right >= 0){
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



