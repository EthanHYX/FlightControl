#include "sys.h"
#include "delay.h"
#include "motors.h"
#include "pm.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ATKflight飞控固件
 * 电机驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2018/5/2
 * 版本：V1.0
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/


static bool isInit = false;
u32 motor_ratios[] = {0, 0, 0, 0};
const u32 MOTORS[] = { MOTOR_M1, MOTOR_M2, MOTOR_M3, MOTOR_M4 };

/*val:0-1000*/
static u16 ratioToCCRx(u16 val)
{
	return (MOTOR_TIM_CNT_FOR_HIGH + val * MOTOR_TIM_CNT_FOR_HIGH/ 1000);//MOTOR_TIM_CNT_FOR_HIGH为固定高电平时间值
}

void motorsInit(void)	/*电机初始化*/
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  
	
	TIM_DeInit(TIM3);
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_TIM3);//PC6 复用为TIM3 CH1	MOTOR1
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_TIM3);//PC7 复用为TIM3 CH2	MOTOR2
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource8,GPIO_AF_TIM3);//PC7 复用为TIM3 CH3	MOTOR3
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource9,GPIO_AF_TIM3);//PC9 复用为TIM3 CH4	MOTOR4
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;		
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;      
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_Init(GPIOC,&GPIO_InitStructure); 	
	
	TIM_TimeBaseStructure.TIM_Period = MOTOR_TIM_PERIOD;			//自动重装载值
	TIM_TimeBaseStructure.TIM_Prescaler = MOTOR_TIM_PRESCALE;		//定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;		//向上计数模式	
	TIM_TimeBaseStructure.TIM_ClockDivision=0; 						//时钟分频
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;					//重复计数次数
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);					//初始化TIM3
	
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;				//PWM模式1
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;	//使能输出
	TIM_OCInitStructure.TIM_Pulse=0;							//CCRx
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;		//低电平有效
	TIM_OCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Set;	//空闲高电平
	
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);  	//初始化TIM3 CH1输出比较
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);  	//初始化TIM3 CH2输出比较
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);  	//初始化TIM3 CH3输出比较
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);  	//初始化TIM3 CH4输出比较
	
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM3在CCR1上的预装载寄存器
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM3在CCR2上的预装载寄存器
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM3在CCR3上的预装载寄存器
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM3在CCR4上的预装载寄存器
 
	TIM_ARRPreloadConfig(TIM3,ENABLE);	//TIM3	ARPE使能 
	TIM_Cmd(TIM3, ENABLE);

	isInit = true;
}

/*设置电机PWM占空比*/
/*ithrust:0-1000*/
void motorsSetRatio(u32 id, u16 ithrust)
{
	if (isInit) 
	{
		switch(id)
		{
			case 0:		/*MOTOR_M1*/
				TIM_SetCompare1(TIM3,ratioToCCRx(ithrust));
				break;
			case 1:		/*MOTOR_M2*/
				TIM_SetCompare2(TIM3,ratioToCCRx(ithrust));
				break;
			case 2:		/*MOTOR_M3*/
				TIM_SetCompare3(TIM3,ratioToCCRx(ithrust));
				break;
			case 3:		/*MOTOR_M4*/	
				TIM_SetCompare4(TIM3,ratioToCCRx(ithrust));
				break;
			default: break;
		}	
	}
}
