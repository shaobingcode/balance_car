#ifndef __PID_H__
#define __PID_H__

#include "stm32f1xx_hal.h"

extern int Left_Encoder_Speed, Right_Encoder_Speed;
extern float roll;
extern uint8_t stopStatus;
void Control(void);	//每隔10ms调用一次

#endif
