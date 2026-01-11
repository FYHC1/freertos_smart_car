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
#include <stdlib.h>
#include "Motor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// #define MOTOR_BRAKE 'b'
// #define MOTOR_FORWARD 'f'
// #define MOTOR_BACKWARD 'g'
// #define MOTOR_COAST 'c'

#define MAX_PWM_VALUE 10000  // 最大 PWM 占空比对应的值
#define SERVO_MAXANGLE 90
#define SERVO_MINANGLE 0

#define QUEUE_LENGTH 10
#define ITEM_SIZE sizeof( uint8_t )*2

struct Motor_Struct{
  GPIO_TypeDef* Motor_GPIO_Port;
  uint16_t Motor_GPIO_Pin;
  TIM_HandleTypeDef* htim;
};

typedef struct {
    int16_t speed;  // -1000 到 +1000
    int16_t turn;   // -1000 到 +1000
    uint8_t func_flags;
} CarControl_t;

#define RING_BUFFER_SIZE 64
//const uint16_t RING_BUFFER_SIZE = 64;
// 帧定义，方便解析
#define FRAME_SIZE 8
#define HEADER_BYTE 0x5A
#define TAIL_BYTE   0xA5


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
TaskHandle_t BLE_Parser_TaskHandle;
//static TaskHandle_t BLE_Parser_TaskHandle;

// static StackType_t Slave_TaskStack[128];
// static StaticTask_t Slave_TaskTCB;
// static TaskHandle_t Slave_TaskHandle;

static StackType_t SR04_TaskStack[128];
static StaticTask_t SR04_TaskTCB;
static TaskHandle_t SR04_TaskHandle;

//static uint8_t UartRecevieData[2];
//static char motorState = MOTOR_BRAKE;
static uint8_t speed = 0;
static uint8_t servoAngle = 45;
static uint8_t duty = 0;

static StaticQueue_t xStaticQueue;
static uint8_t MotorCmdQueueStorageArea[QUEUE_LENGTH*ITEM_SIZE];
QueueHandle_t MotorCmdQueue;

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
void MotorTask(void *pvParameters);
void BLEParserTask(void *pvParameters);
void SlaveControl(void *pvParameters);
void SR04Play(void *pvParameters);
//void SetMotorPWM(float pwm_left, float pwm_right);


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
  MotorCmdQueue = xQueueCreateStatic(QUEUE_LENGTH,ITEM_SIZE,MotorCmdQueueStorageArea,&xStaticQueue);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  /*创建任务电机运行*/
  Motor_TaskHandle = xTaskCreateStatic(MotorTask,"MotorTaskTask",128,NULL,osPriorityNormal,Motor_TaskStack,&Motor_TaskTCB);
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

uint16_t GetAvailableDataLength(void){
  //保存当前写指针位置
  extern uint16_t RxWritePtr;
  //关闭中断，防止读写指针变化
  __disable_irq();
  uint16_t temp_WritePtr = RxWritePtr;
  __enable_irq(); 
  if (temp_WritePtr >= RxReadPtr)
  {
    //正常情况，写指针在读指针之后
    return temp_WritePtr - RxReadPtr;
  }
  else
  {
    //写指针越界回到起点
    return RING_BUFFER_SIZE - (RxReadPtr - temp_WritePtr);
  }

}


void BLEParserTask(void *pvParameters)
{
  CarControl_t cmd;

  while(1){
    //等待IDLE中断通知，永久阻塞
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    while (GetAvailableDataLength() >= FRAME_SIZE)
    {
      // 检查帧头
      while(RxRingBuffer[RxReadPtr] != HEADER_BYTE && GetAvailableDataLength() >= FRAME_SIZE)
      {
        // 丢弃无效数据
        RxReadPtr = (RxReadPtr + 1) % RING_BUFFER_SIZE;
      }
    
      //提取和校验一帧数据
      uint8_t buffer[FRAME_SIZE];
      for (int i = 0; i < FRAME_SIZE; i++)
      {
        buffer[i] = RxRingBuffer[(RxReadPtr + i) % RING_BUFFER_SIZE];
      }
      if (buffer[FRAME_SIZE - 1] != TAIL_BYTE)
      {
        // 帧尾错误，丢弃帧头，继续寻找下一帧
        RxReadPtr = (RxReadPtr + 1) % RING_BUFFER_SIZE;
        continue;
      
      }

      //检查校验和
      uint8_t checksum = 0;
      for (int i = 1; i <= 5; i++) {
        checksum += buffer[i];
      }
      if (checksum != buffer[6]) {
        // 校验和错误，丢弃帧头，继续寻找下一帧
        RxReadPtr = (RxReadPtr + 1) % RING_BUFFER_SIZE;
        continue;
      }

      // 解析有效帧
      cmd.speed = (int16_t)(buffer[1] | (buffer[2]) << 8);
      cmd.turn = (int16_t)(buffer[3] | (buffer[4]) << 8);
      cmd.func_flags = buffer[5];

      // 更新队列，跳过已处理的数据
      xQueueOverwrite(MotorCmdQueue, &cmd);
      RxReadPtr = (RxReadPtr + FRAME_SIZE) % RING_BUFFER_SIZE;
    }
  }
    
}

void MotorTask(void *pvParameters)
{
  CarControl_t target;    // 目标值
  float current_spd = 0;  // 当前平滑后的速度
  float current_turn = 0; // 当前平滑后的转向

  // 平滑系数 (值越小，惯性越大)
  // 0.1 表示每次循环只向目标值靠近 10%
  const float SMOOTH_FACTOR_ACC = 0.15f; // 加速平滑
  const float SMOOTH_FACTOR_DEC = 0.30f; // 减速/刹车要快一点

  const float PWM_SCALE = (float)MAX_PWM_VALUE / 1000.0f;

  while (1) {
    // 1. 取出最新指令 (如果没有新指令，保持 target 不变，或者加超时归零逻辑)
    if (xQueuePeek(MotorCmdQueue, &target, 0) == pdTRUE) {
      // 2. 速度平滑处理 (低通滤波算法)
      if (abs(target.speed) > abs(current_spd))
        // 正在加速
        current_spd += (target.speed - current_spd) * SMOOTH_FACTOR_ACC;
      else
        // 正在减速/松开摇杆
        current_spd += (target.speed - current_spd) * SMOOTH_FACTOR_DEC;

      // 转向通常不需要太大的延迟，直接赋值或给很大系数
      current_turn = target.turn;

      // 3. 差速混合算法 (Mixing)
      // 左轮 = 前进 + 转向
      // 右轮 = 前进 - 转向
      float motor_l_float  = current_spd + current_turn;
      float motor_r_float = current_spd - current_turn;

      int16_t pwm_l = (int16_t)(motor_l_float * PWM_SCALE);
      int16_t pwm_r = (int16_t)(motor_r_float * PWM_SCALE);

      // 限幅保护 (防止超过 PWM 最大值)
      if (pwm_l > MAX_PWM_VALUE)
        pwm_l = MAX_PWM_VALUE;
      else if (pwm_l < -MAX_PWM_VALUE)
        pwm_l = -MAX_PWM_VALUE;
      if (pwm_r > MAX_PWM_VALUE)
        pwm_r = MAX_PWM_VALUE;
      else if (pwm_r < -MAX_PWM_VALUE)
        pwm_r = -MAX_PWM_VALUE;

      // 4. 调用上一条回答中的驱动函数
      // 注意：这里需要把 +/-100 的范围映射到 PWM (例如 +/- 1000)
      SetMotor(motor_l_float * 10, motor_r_float * 10);
      //SetMotorPWM(motor_l_float, motor_r_float);
    } else {
      // (可选) 增加看门狗逻辑：如果队列 500ms 没刷新，强制停车
    }

    // 5. 严格控制控制频率 50Hz
    vTaskDelay(20);
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
//   xQueueSendToBackFromISR(MotorCmdQueue,&UartRecevieData,(BaseType_t*)pdTRUE);
//   xTaskNotifyGive(BLE_Parser_TaskHandle);
// }
/* USER CODE END Application */

