#include "pid.h"
#include "encoder.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu6050.h"
#include "motor.h"
#include "sr04.h"
#include "tim.h"

// 传感器数据变量
int Left_Encoder_Speed, Right_Encoder_Speed;
float pitch, roll, yaw;
short gyrox, gyroy, gyroz;
short aacx, aacy, aacz;

// 闭环控制中间变量
int Vertical_out, Velocity_out, Turn_out, Target_Speed, Target_turn, left_motor_pwm, right_motor_pwm;
float Med_Angle = 0; // 平衡时角度值偏移量（机械中值）


// 直立环PID 比例系数（Kp：0~1000、Kd：0~10）
float Vertical_Kp = 120;
float Vertical_Kd = 0.75;	

// 速度环PID 比例系数（Kp：0~1）
float Velocity_Kp = 1.1;
float Velocity_Ki = 0.0055; 

// 转向环PID 比例系数
float Turn_Kp = 10;
float Turn_Kd = 0.l;

uint8_t stopStatus = 0; // 0：代表电机未停, 1: 代表电机停止


extern uint8_t Fore, Back, Left, Right;
#define SPEED_Y 12	// 俯仰(前后)最大设定速度
#define SPEED_Z 150 // 偏航(左右)最大设定速度

// 直立环PD控制器
// 输入：期望角度、真实角度、角速度
int Vertical_PID(float Med, float Angle, float gyro_Y)
{
	int temp;
	temp = Vertical_Kp * (Angle - Med) + Vertical_Kd * gyro_Y;
	return temp;
}

// 速度环PI控制器
// 输入：期望速度、左编码器、右编码器
int Velocity_PID(int Target, int encoder_L, int encoder_R)
{
	static int Err_LowOut_last, Encoder_S;
	static float a = 0.7;
	int Err, Err_LowOut, temp;
	Velocity_Ki = Velocity_Kp / 200;
	// 1、计算偏差值
	Err = (encoder_L + encoder_R) - Target;
	// 2、低通滤波
	Err_LowOut = (1 - a) * Err + a * Err_LowOut_last;
	Err_LowOut_last = Err_LowOut;
	// 3、积分
	Encoder_S += Err_LowOut;
	// 4、积分限幅(-20000~20000)
	Encoder_S = Encoder_S > 20000 ? 20000 : (Encoder_S < (-20000) ? (-20000) : Encoder_S);
	if (stopStatus == 1)
		Encoder_S = 0, stopStatus = 0;
	// 5、速度环计算
	temp = Velocity_Kp * Err_LowOut + Velocity_Ki * Encoder_S;
	return temp;
}

// 转向环PD控制器
// 输入：角速度、角度值
int Turn_PID(float gyro_Z, int Target_turn)
{
	int temp;
	temp = Turn_Kp * Target_turn + Turn_Kd * gyro_Z;
	return temp;
}

void Control(void) // 每隔10ms调用一次
{
	int PWM_out;
	// 1、读取编码器和陀螺仪的数据
	Left_Encoder_Speed = Read_Speed(&htim2);
	Right_Encoder_Speed = -Read_Speed(&htim4);
	mpu_dmp_get_data(&pitch, &roll, &yaw);
	MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);
	MPU_Get_Accelerometer(&aacx, &aacy, &aacz);
	// 遥控
	if ((Fore == 0) && (Back == 0))
		Target_Speed = 0; // 未接受到前进后退指令-->速度清零，稳在原地
	if (Fore == 1)
	{
		if (distance < 50)
			Target_Speed++;
		else
			Target_Speed--;
	}
	if (Back == 1)
	{
		Target_Speed++;
	}																										 //
	Target_Speed = Target_Speed > SPEED_Y ? SPEED_Y : (Target_Speed < -SPEED_Y ? (-SPEED_Y) : Target_Speed); // 限幅

	/*左右*/
	if ((Left == 0) && (Right == 0))
		Target_turn = 0;
	if (Left == 1)
		Target_turn += 30; // 左转
	if (Right == 1)
		Target_turn -= 30;																				 // 右转
	Target_turn = Target_turn > SPEED_Z ? SPEED_Z : (Target_turn < -SPEED_Z ? (-SPEED_Z) : Target_turn); // 限幅( (20*100) * 100   )

	/*转向约束*/
	if ((Left == 0) && (Right == 0))
		Turn_Kd = 0.6; // 若无左右转向指令，则开启转向约束
	else if ((Left == 1) || (Right == 1))
		Turn_Kd = 0; // 若左右转向指令接收到，则去掉转向约束


	// 2、将数据传入PID控制器，计算输出结果，即左右电机转速值
	Velocity_out = Velocity_PID(Target_Speed, Left_Encoder_Speed, Right_Encoder_Speed);
	Vertical_out = Vertical_PID(Velocity_out + Med_Angle, roll, gyrox);
	Turn_out = Turn_PID(gyroz, Target_turn);

	PWM_out = Vertical_out;
	left_motor_pwm = PWM_out - Turn_out;
	right_motor_pwm = PWM_out + Turn_out;

	
	Motor_PWM_Limit(&left_motor_pwm, &right_motor_pwm);
	Motor_PWM_Ctrl(left_motor_pwm, right_motor_pwm);
	Motor_Angle_Limit(&Med_Angle, &roll); // 安全检测
}
