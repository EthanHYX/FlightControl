#ifndef __RX_H
#define __RX_H
#include "sys.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ATKflight飞控固件
 * 遥控接收处理代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2018/5/2
 * 版本：V1.0
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

enum 
{
	ROLL = 0,
	PITCH,
	THROTTLE,
	YAW,
	AUX1,
	AUX2,
	AUX3,
	AUX4,
	AUX5,
	AUX6,
	AUX7,
	AUX8,
	CH_NUM
};

extern uint16_t rcData[CH_NUM];

typedef struct 
{
	uint32_t realLinkTime;
	bool linkState;
	bool invalidPulse;
} rcLinkState_t;

typedef struct 
{
    bool rcLinkState;				//遥控连接状态（true连接 false断开）
	bool failsafeActive;			//失控保护是否激活
	uint32_t rcLinkRealTime;		//遥控连接实时时间
	uint32_t throttleLowPeriod;		//遥控需要低油门时间（用于判断飞机是否着陆状态）
} failsafeState_t;



void rxInit(void);
void ppmTask(void *param);
bool rxLinkStatus(void);
void rxTask(void *param);

#endif
