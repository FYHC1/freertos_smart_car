//
// Created by hgl on 25-10-22.
//

#include "sr04.h"


void sr04_start(void )
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
	HAL_Delay(15);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
}

double sr04_read(void )
{
	uint32_t i =0;
	double dist = 0;
	sr04_start();
	while (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_11) == GPIO_PIN_RESET)
	{
		i++;
		HAL_Delay(1);
		if(i > 100000)
			return 0;
	}
	i = 0;
	while (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_11) == GPIO_PIN_RESET)
	{
		i++;
		HAL_Delay(1);
		if(i > 100000)
			return 0;
	}

	dist = i*2*0.033/2;//这里乘2的原因是上面是2微妙
	return dist	;
}