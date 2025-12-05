#include "Motor.h"
#include "main.h"
#include "stm32_hal_legacy.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_tim.h"
#include "tim.h"

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
	HAL_TIM_PWM_Start(PWM_MOTOR1_GPIO_Port, PWM_MOTOR1_Pin);
	HAL_TIM_PWM_Start(PWM_MOTOR2_GPIO_Port, PWM_MOTOR2_Pin);
}

void Motor_SetSpeed(int8_t Speed)
{
	// if (Speed >= 0)
	// {
	// 	GPIO_SetBits(GPIOA, GPIO_Pin_4);
	// 	GPIO_ResetBits(GPIOA, GPIO_Pin_5);
	// 	PWM_SetCompare3(Speed);
	// }
	// else
	// {
	// 	GPIO_ResetBits(GPIOA, GPIO_Pin_4);
	// 	GPIO_SetBits(GPIOA, GPIO_Pin_5);
	// 	PWM_SetCompare3(-Speed);
	// }

	if (Speed >= 0)
	{
		HAL_GPIO_WritePin(MOTOR1_OT1_GPIO_Port, MOTOR1_OT1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MOTOR1_OT2_GPIO_Port, MOTOR1_OT2_Pin, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Speed);

		HAL_GPIO_WritePin(MOTOR2_OT1_GPIO_Port, MOTOR2_OT1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MOTOR2_OT2_GPIO_Port, MOTOR2_OT2_Pin, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, Speed);
	}
	else
	{
		HAL_GPIO_WritePin(MOTOR1_OT1_GPIO_Port, MOTOR1_OT1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MOTOR1_OT2_GPIO_Port, MOTOR1_OT2_Pin, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, -Speed);

		HAL_GPIO_WritePin(MOTOR2_OT1_GPIO_Port, MOTOR2_OT1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MOTOR2_OT2_GPIO_Port, MOTOR2_OT2_Pin, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, -Speed);
	}
}
