#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"
#define MAX_PWM_VALUE 10000 
#define DEAD_ZONE 50       // 死区阈值，防止电机在此电压下仅发热不转
void Motor_Init(void);
void Motor_SetSpeed(int8_t Speed);
//void SetMotorPWM(float pwm_left, float pwm_right);

#endif
