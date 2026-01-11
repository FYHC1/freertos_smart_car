#include "Motor.h"
#include "main.h"
#include "stm32_hal_legacy.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_tim.h"
#include "tim.h"
#include <stdint.h>

void Motor_Init(void)
{
	// RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	// GPIO_InitTypeDef GPIO_InitStructure;
	// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	// GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	// GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	// PWM_Init();

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(MOTOR1_OT1_GPIO_Port, MOTOR1_OT1_Pin|MOTOR1_OT2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR2_OT1_GPIO_Port, MOTOR2_OT1_Pin|MOTOR2_OT2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : Motor_OT_Pin */
	GPIO_InitStruct.Pin = MOTOR1_OT1_Pin|MOTOR1_OT2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(MOTOR1_OT1_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = MOTOR2_OT1_Pin|MOTOR2_OT2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(MOTOR2_OT1_GPIO_Port, &GPIO_InitStruct);

	/* Start PWM */
	HAL_TIM_PWM_Start(&htim1, PWM_MOTOR1_Pin);
	HAL_TIM_PWM_Start(&htim1, PWM_MOTOR2_Pin);
}

// void Motor_SetSpeed(int8_t Speed)
// {
// 	if (Speed >= 0)
// 	{
// 		GPIO_SetBits(GPIOA, GPIO_Pin_4);
// 		GPIO_ResetBits(GPIOA, GPIO_Pin_5);
// 		PWM_SetCompare3(Speed);
// 	}
// 	else
// 	{
// 		GPIO_ResetBits(GPIOA, GPIO_Pin_4);
// 		GPIO_SetBits(GPIOA, GPIO_Pin_5);
// 		PWM_SetCompare3(-Speed);
// 	}

// }

void MOTOR_FORWARD(int8_t speed) {
	HAL_GPIO_WritePin(MOTOR1_OT1_GPIO_Port, MOTOR1_OT1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR1_OT2_GPIO_Port, MOTOR1_OT2_Pin, GPIO_PIN_RESET);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);

	HAL_GPIO_WritePin(MOTOR2_OT1_GPIO_Port, MOTOR2_OT1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR2_OT2_GPIO_Port, MOTOR2_OT2_Pin, GPIO_PIN_SET);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);
}

void MOTOR_BACKWARD(uint8_t speed){
	HAL_GPIO_WritePin(MOTOR1_OT1_GPIO_Port, MOTOR1_OT1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR1_OT2_GPIO_Port, MOTOR1_OT2_Pin, GPIO_PIN_SET);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, -speed);

	HAL_GPIO_WritePin(MOTOR2_OT1_GPIO_Port, MOTOR2_OT1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR2_OT2_GPIO_Port, MOTOR2_OT2_Pin, GPIO_PIN_RESET);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, -speed);
}

void MOTOR_TURN_LEFT(uint8_t speed){
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,speed);
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,speed * 0.6);
}

void MOTOR_TURN_RIGHT(uint8_t speed){
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,speed * 0.6);
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,speed);
}

void SetMotor(float pwm_left, float pwm_right) {
    // 1. 确保 STBY 为高电平，开启驱动芯片 [cite: 86, 136]
    // HAL_GPIO_WritePin(STBY_PORT, STBY_PIN, GPIO_PIN_SET);

    // --- 左电机控制 (Motor A) ---
    if (pwm_left > DEAD_ZONE) {
        // 正转 (Forward): AIN1=H, AIN2=L 
        HAL_GPIO_WritePin(MOTOR1_OT1_GPIO_Port, MOTOR1_OT1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR1_OT2_GPIO_Port, MOTOR1_OT2_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(TIM_HANDLE, TIM_CH_LEFT, pwm_left);
    } 
    else if (pwm_left < -DEAD_ZONE) {
        // 反转 (Reverse): AIN1=L, AIN2=H 
        HAL_GPIO_WritePin(MOTOR1_OT1_GPIO_Port, MOTOR1_OT1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR1_OT2_GPIO_Port, MOTOR1_OT2_Pin, GPIO_PIN_SET);
        // PWM 寄存器只接受正数，取绝对值
        __HAL_TIM_SET_COMPARE(TIM_HANDLE, TIM_CH_LEFT, -pwm_left);
    } 
    else {
        // 停止 (Stop): AIN1=L, AIN2=L 
        HAL_GPIO_WritePin(MOTOR1_OT1_GPIO_Port, MOTOR1_OT1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR1_OT2_GPIO_Port, MOTOR1_OT2_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(TIM_HANDLE, TIM_CH_LEFT, 0);
    }

    // --- 右电机控制 (Motor B) ---
    if (pwm_right > DEAD_ZONE) {
        // 正转 (Forward): BIN1=H, BIN2=L 
        HAL_GPIO_WritePin(MOTOR2_OT1_GPIO_Port, MOTOR2_OT1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR2_OT2_GPIO_Port, MOTOR2_OT2_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(TIM_HANDLE, TIM_CH_RIGHT, pwm_right);
    } 
    else if (pwm_right < -DEAD_ZONE) {
        // 反转 (Reverse): BIN1=L, BIN2=H 
        HAL_GPIO_WritePin(MOTOR2_OT1_GPIO_Port, MOTOR2_OT1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR2_OT2_GPIO_Port, MOTOR2_OT2_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(TIM_HANDLE, TIM_CH_RIGHT, -pwm_right);
    } 
    else {
        // 停止 (Stop): BIN1=L, BIN2=L 
        HAL_GPIO_WritePin(MOTOR2_OT1_GPIO_Port, MOTOR2_OT1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR2_OT2_GPIO_Port, MOTOR2_OT2_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(TIM_HANDLE, TIM_CH_RIGHT, 0);
    }
}

