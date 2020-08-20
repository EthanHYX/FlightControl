#include <string.h>
#include "ws2812.h"
#include "delay.h"

#include "FreeRTOS.h"
#include "semphr.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ATKflight�ɿع̼�
 * ws2812 RGB_LED��������	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2018/5/2
 * �汾��V1.0
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

#define TIMING_ONE  71	// �ߵ�ƽʱ�����ֵ
#define TIMING_ZERO 33	// �͵�ƽʱ�����ֵ

u32 dmaBuffer0[24];
u32 dmaBuffer1[24];

static xSemaphoreHandle allLedDone = NULL;

//ws2812��ʼ��
void ws2812Init(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	
	//��ʼ��LED_STRIP pin PA15
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;				
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;			
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;		
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_TIM2);

	TIM_TimeBaseStructure.TIM_Period = (105 - 1); //800kHz
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
 
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_DMACmd(TIM2, TIM_DMA_CC1, ENABLE);
//	TIM_Cmd(TIM2, ENABLE); 
	
	DMA_DeInit(DMA1_Stream5);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&TIM2->CCR1;
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)dmaBuffer0;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Word;
	DMA_InitStructure.DMA_BufferSize = 24;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
	DMA_InitStructure.DMA_Channel = DMA_Channel_3;
	
	DMA_DoubleBufferModeCmd(DMA1_Stream5, ENABLE);//ʹ��˫����
	DMA_DoubleBufferModeConfig(DMA1_Stream5, (u32)dmaBuffer1, DMA_Memory_0);
	DMA_Init(DMA1_Stream5, &DMA_InitStructure);
	DMA_ITConfig(DMA1_Stream5, DMA_IT_TC, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 9;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	vSemaphoreCreateBinary(allLedDone);
}

//ws2812��ɫ���
static void fillLed(u32 *buffer, u8 *color)
{
    int i;

    for(i=0; i<8; i++) // GREEN
	{
	    buffer[i] = ((color[1]<<i) & 0x0080) ? TIMING_ONE:TIMING_ZERO;
	}
	for(i=0; i<8; i++) // RED
	{
	    buffer[8+i] = ((color[0]<<i) & 0x0080) ? TIMING_ONE:TIMING_ZERO;
	}
	for(i=0; i<8; i++) // BLUE
	{
	    buffer[16+i] = ((color[2]<<i) & 0x0080) ? TIMING_ONE:TIMING_ZERO;
	}
}

static int current_led = 0;
static int total_led = 0;
static u8(*color_led)[3] = NULL;

//ws2812��ɫ������DMA
void ws2812Send(u8 (*color)[3], u16 len)
{
	if(len<1) return;

	xSemaphoreTake(allLedDone, portMAX_DELAY);//�ȴ���һ�η������

	current_led = 0;
	total_led = len;
	color_led = color;
	
	fillLed(dmaBuffer0, color_led[current_led]);
	current_led++;
	fillLed(dmaBuffer1, color_led[current_led]);
	current_led++;
	
	DMA_Cmd(DMA1_Stream5, ENABLE);	//ʹ��DMA
	TIM_Cmd(TIM2, ENABLE);			//ʹ�ܶ�ʱ��
}

//DMA�жϴ���
void DMA1_Stream5_IRQHandler(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken;

	if (total_led == 0)
	{
		TIM_Cmd(TIM2, DISABLE);
		DMA_Cmd(DMA1_Stream5, DISABLE);
	}
	
	if (DMA_GetITStatus(DMA1_Stream5, DMA_IT_TCIF5))
	{
		DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF5);
		if(DMA_GetCurrentMemoryTarget(DMA1_Stream5) == DMA_Memory_0)//DMA��ǰʹ���ڴ�0
		{
			if (current_led<total_led)
				fillLed(dmaBuffer1, color_led[current_led]);
			else
				memset(dmaBuffer1, 0, sizeof(dmaBuffer1));
		}
		else//DMA��ǰʹ���ڴ�1
		{
			if (current_led<total_led)
				fillLed(dmaBuffer0, color_led[current_led]);
			else		
				memset(dmaBuffer0, 0, sizeof(dmaBuffer0));
		}
		current_led++;
	}

	if (current_led >= total_led+2) //�ഫ��2��LED����60us�ĵ͵�ƽ
	{
		xSemaphoreGiveFromISR(allLedDone, &xHigherPriorityTaskWoken);
		TIM_Cmd(TIM2, DISABLE); 					
		DMA_Cmd(DMA1_Stream5, DISABLE); 
		total_led = 0;
	}
}


