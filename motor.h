#ifndef _MOTOR_H
#define _MOTOR_H

#include "stm32f1xx_hal.h"
void Motor_PWM_Ctrl(int left_motor_pwm,int right_motor_pwm);
void Motor_PWM_Limit(int *left_motor_pwm,int *right_motor_pwm);
void Motor_Angle_Limit(float *Med_Jiaodu,float *Jiaodu);
#endif
