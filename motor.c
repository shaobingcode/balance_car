#include "motor.h"
#include "pid.h"
#include "tim.h"


#define PWM_MAX 7200
#define PWM_MIN -7200


int abs(int p)
{
	if (p > 0)
		return p;
	else
		return -p;
}

void Motor_PWM_Ctrl(int left_motor_pwm, int right_motor_pwm) //-7200~7200
{
	if (left_motor_pwm < 0)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	}
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, abs(left_motor_pwm));
	if (right_motor_pwm < 0)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
	}
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, abs(right_motor_pwm));
}

void Motor_PWM_Limit(int *left_motor_pwm, int *right_motor_pwm)
{
	if (*left_motor_pwm > PWM_MAX)
		*left_motor_pwm = PWM_MAX;
	if (*left_motor_pwm < PWM_MIN)
		*left_motor_pwm = PWM_MIN;
	if (*right_motor_pwm > PWM_MAX)
		*right_motor_pwm = PWM_MAX;
	if (*right_motor_pwm < PWM_MIN)
		*right_motor_pwm = PWM_MIN;
}

/**
 * @brief 小车将要 摔倒之前 停止电机
 * 
 * @param med_angle 
 * @param angle 
 */
void Motor_Angle_Limit(float *med_angle, float *angle)
{
	if (abs((int)(*angle - *med_angle)) > 60)
	{
		Motor_PWM_Ctrl(0, 0);
		stopStatus = 1;
	}
}
