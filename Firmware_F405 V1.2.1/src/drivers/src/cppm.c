#include "cppm.h"
#include "commander.h"
/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ATKflight飞控固件
 * 遥控PPM信号捕获驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2018/5/2
 * 版本：V1.0
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

#define CPPM_TIMER                   TIM9
#define CPPM_TIMER_RCC               RCC_APB2Periph_TIM9
#define CPPM_GPIO_RCC                RCC_AHB1Periph_GPIOA
#define CPPM_GPIO_PORT               GPIOA
#define CPPM_GPIO_PIN                GPIO_Pin_3
#define CPPM_GPIO_SOURCE             GPIO_PinSource3
#define CPPM_GPIO_AF                 GPIO_AF_TIM9

#define CPPM_TIM_PRESCALER           (168-1) 

static xQueueHandle captureQueue;
static uint16_t prevCapureVal;
static bool captureFlag;
static bool isAvailible;

void cppmInit(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(CPPM_GPIO_RCC, ENABLE);
	RCC_APB2PeriphClockCmd(CPPM_TIMER_RCC, ENABLE);

	//配置PPM信号输入引脚（PA3）
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = CPPM_GPIO_PIN;
	GPIO_Init(CPPM_GPIO_PORT, &GPIO_InitStructure);

	GPIO_PinAFConfig(CPPM_GPIO_PORT, CPPM_GPIO_SOURCE, CPPM_GPIO_AF);

	//配置定时器1us tick
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = CPPM_TIM_PRESCALER;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(CPPM_TIMER, &TIM_TimeBaseStructure);

	//配置输入捕获
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInit(CPPM_TIMER, &TIM_ICInitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_TIM9_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	captureQueue = xQueueCreate(64, sizeof(uint16_t));

	TIM_ITConfig(CPPM_TIMER, TIM_IT_Update | TIM_IT_CC2, ENABLE);
	TIM_Cmd(CPPM_TIMER, ENABLE);
}

bool cppmIsAvailible(void)
{
	return isAvailible;
}

int cppmGetTimestamp(uint16_t *timestamp)
{
	ASSERT(timestamp);

	return xQueueReceive(captureQueue, timestamp, 20);
}

void cppmClearQueue(void)
{
	xQueueReset(captureQueue);
}

uint16_t capureVal;
uint16_t capureValDiff;

void TIM1_BRK_TIM9_IRQHandler(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	if(TIM_GetITStatus(CPPM_TIMER, TIM_IT_CC2) != RESET)
	{
		if(TIM_GetFlagStatus(CPPM_TIMER, TIM_FLAG_CC2OF) != RESET)
		{
		  //TODO: Handle overflow error
		}

		capureVal = TIM_GetCapture2(CPPM_TIMER);
		capureValDiff = capureVal - prevCapureVal;
		prevCapureVal = capureVal;

		xQueueSendFromISR(captureQueue, &capureValDiff, &xHigherPriorityTaskWoken);
 
		captureFlag = true;
		TIM_ClearITPendingBit(CPPM_TIMER, TIM_IT_CC2);
	}

	if(TIM_GetITStatus(CPPM_TIMER, TIM_IT_Update) != RESET)
	{
		// Update input status
		isAvailible = (captureFlag == true);
		captureFlag = false;
		TIM_ClearITPendingBit(CPPM_TIMER, TIM_IT_Update);
	}
}
