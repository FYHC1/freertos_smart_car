#include "hc_05.h"

static void HC05_Init(void)
{
	HAL_UART_Init(&huart1);
}