/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "drv8833.h"
#include "queue.h"
#include "semphr.h"
#include "tim.h"
#include "task.h"
#include "sr04.h"
#include "usart.h"
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTOR_BRAKE 'b'
#define MOTOR_FORWARD 'f'
#define MOTOR_BACKWARD 'g'
#define MOTOR_COAST 'c'

#define SERVO_MAXANGLE 90
#define SERVO_MINANGLE 0

#define QUEUE_LENGTH 10
#define ITEM_SIZE sizeof( uint8_t )*2

struct Motor_Struct{
  GPIO_TypeDef* Motor_GPIO_Port;
  uint16_t Motor_GPIO_Pin;
  TIM_HandleTypeDef* htim;
};

#define RING_BUFFER_SIZE 64



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static StackType_t Motor_TaskStack[128];
static StaticTask_t Motor_TaskTCB;
static TaskHandle_t Motor_TaskHandle;

static StackType_t BLE_Parser_TaskStack[128];
static StaticTask_t BLE_Parser_TaskTCB;
static TaskHandle_t BLE_Parser_TaskHandle;
//static TaskHandle_t BLE_Parser_TaskHandle;

// static StackType_t Slave_TaskStack[128];
// static StaticTask_t Slave_TaskTCB;
// static TaskHandle_t Slave_TaskHandle;

static StackType_t SR04_TaskStack[128];
static StaticTask_t SR04_TaskTCB;
static TaskHandle_t SR04_TaskHandle;

//static uint8_t UartRecevieData[2];
static char motorState = MOTOR_BRAKE;
static uint8_t speed = 0;
static uint8_t servoAngle = 45;
static uint8_t duty = 0;

static StaticQueue_t xStaticQueue;
static uint8_t UartQueueStorageArea[QUEUE_LENGTH*ITEM_SIZE];
QueueHandle_t UartDataQueue;

SemaphoreHandle_t motorMux_Handle;


// 实际用于 DMA 接收的数组
uint8_t RxRingBuffer[RING_BUFFER_SIZE];

// 读写指针
// ReadPtr: 下一次 FreeRTOS 任务应该从哪里开始读取数据（消费者）
volatile uint16_t RxReadPtr = 0; 

// DMA 是写指针的生产者，它的位置由硬件寄存器决定，
// 通常不需要显式定义 WritePtr 变量，而是通过查询 DMA 计数器获得。

/* USER CODE END Variables */
/* Definitions for defaultTask */

osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void MotorPlay(void *pvParameters);
void BLEParserTask(void *pvParameters);
void SlaveControl(void *pvParameters);
void SR04Play(void *pvParameters);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  DRV8833_Init();
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

  // 启动 DMA 循环接收
  // DMA 会从 huart1 持续接收数据到 RxRingBuffer，直到 512 字节满后自动回到起点
  HAL_UART_Receive_DMA(&huart1, RxRingBuffer, RING_BUFFER_SIZE);

  // 启用 UART 空闲中断 (必须启用)
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  motorMux_Handle = xSemaphoreCreateMutex();
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  UartDataQueue = xQueueCreateStatic(QUEUE_LENGTH,ITEM_SIZE,UartQueueStorageArea,&xStaticQueue);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  /*创建任务电机运行*/
  Motor_TaskHandle = xTaskCreateStatic(MotorPlay,"MotorPlayTask",128,NULL,osPriorityNormal,Motor_TaskStack,&Motor_TaskTCB);
  /*创建任务蓝牙控制*/
  BLE_Parser_TaskHandle = xTaskCreateStatic(BLEParserTask,"BLEParserTask",128,NULL,osPriorityNormal2,BLE_Parser_TaskStack,&BLE_Parser_TaskTCB);
  // /*创建任务舵机方向控制*/
  // Slave_TaskHandle = xTaskCreateStatic(SlaveControl,"SlaveControlTask",128,NULL,osPriorityNormal1,Slave_TaskStack,&Slave_TaskTCB);
  /*创建任务超声波避障*/
  SR04_TaskHandle = xTaskCreateStatic(SR04Play,"SR04PlayTask",128,NULL,osPriorityNormal2,SR04_TaskStack,&SR04_TaskTCB);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void BLEParserTask(void *pvParameters)
{
  

  while(1){
    
  }
    
}

void MotorPlay(void *pvParameters)
{
  while(1)
  {
    if (motorState == MOTOR_BRAKE)
      DRV8833_Brake();
    else if (motorState == MOTOR_FORWARD)
      DRV8833_Forward(speed);
    else if (motorState == MOTOR_BACKWARD)
      DRV8833_Backward(speed);
    else if (motorState == MOTOR_COAST)
      DRV8833_Coast();
  }
}

void SR04Play(void *pvParameters)
{

  if(sr04_read() > 25)//前方无障碍物
  {
    DRV8833_Forward(speed);//前运动
    HAL_Delay(100);
  }
  else{	//前方有障碍物
    //右边运动 原地
    if (servoAngle <=75)
    {
      servoAngle = servoAngle + 15;
      duty =  (10* servoAngle/(float)SERVO_MAXANGLE +2.5)/100 *2000;
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, duty);
    }
    HAL_Delay(500);
    if(sr04_read() > 25)//右边无障碍物
    {
      DRV8833_Forward(speed);//前运动
      HAL_Delay(100);
    }
    else{//右边有障碍物
      //左边运动 原地
      if (servoAngle >=15)
      {
        servoAngle = servoAngle - 15;
        duty =  (10* servoAngle/(float)SERVO_MAXANGLE +2.5)/100 *2000;
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, duty);
      }
      HAL_Delay(1000);
      if(sr04_read() >25)//左边无障碍物
      {
        DRV8833_Forward(speed);//前运动
        HAL_Delay(100);
      }
      else{
        DRV8833_Brake();
        DRV8833_Backward(speed);//后运动
        HAL_Delay(1000);
        //右边运动
        if (servoAngle <=75)
        {
          servoAngle = servoAngle + 15;
          duty =  (10* servoAngle/(float)SERVO_MAXANGLE +2.5)/100 *2000;
          __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, duty);
        }
        HAL_Delay(50);
      }
    }
  }
 
}

// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
//   if(huart->Instance != USART1)
//     return;
//   uint8_t UartRecevieData[2] = {0};
//   HAL_UART_Receive_IT(huart,UartRecevieData,2);
//   xQueueSendToBackFromISR(UartDataQueue,&UartRecevieData,(BaseType_t*)pdTRUE);
//   xTaskNotifyGive(BLE_Parser_TaskHandle);
// }
/* USER CODE END Application */

