#ifndef __COMMANDER_H
#define __COMMANDER_H
#include "atkp.h"
#include "config.h"
#include "stabilizer_types.h"
#include "rx.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ATKflight飞控固件
 * 获取遥控数据驱动代码
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2018/5/2
 * 版本：V1.0
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

typedef struct
{
	bool autoLandActive;
	uint32_t autoLandTime;
}autoLandState_t;

extern int16_t rcCommand[4];

void commanderGetSetpoint(const state_t *state, setpoint_t *setpoint);
void commanderSetupAltitudeHoldMode(void);
uint16_t commanderGetALtHoldThrottle(void);
void commanderActiveFailsafe(void);







#endif /* __COMMANDER_H */
