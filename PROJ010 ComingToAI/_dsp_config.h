/*		DSP Part
		_dsp_config.h	
*/
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/*	Export Function		------------------------------------------------------*/
//////// Initial Function
void TIM_Configuration(void);

void _eRight(int16_t);
void _eLeft(int16_t);
void Set_speed(int16_t,int16_t);
void MoveToPoint(int linecount);
void MoveToPointV2(int linecount);
void MoveToBox(void);
void TrackingLeft (void);
void TrackingRight (void);
void Servo1_Drive(uint8_t);
void Servo2_Drive(uint8_t);
void Servo3_Drive(uint8_t);
void DropBox(void);
void LoadBox(void);

