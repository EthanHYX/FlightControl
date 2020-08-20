#ifndef __STATE_CONTROL_H
#define __STATE_CONTROL_H
#include "stabilizer_types.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ATKflight飞控固件
 * 四轴姿态控制代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2018/5/2
 * 版本：V1.0
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

enum pidIndex
{
	RATE_ROLL = 0,
	RATE_PITCH,
	RATE_YAW,
	ANGLE_ROLL,
	ANGLE_PITCH,
	ANGLE_YAW,
	VELOCITY_Z,
	POSHOLD_Z,
	VELOCITY_XY,
	POSHOLD_XY,	
	PID_NUM
};

void stateControlInit(void);
bool stateControlTest(void);
void stateControl(const sensorData_t *sensorData, const state_t *state, setpoint_t *setpoint, control_t *control, const u32 tick);
void stateControlResetYawHolding(void);
void stateControlSetVelocityZPIDIntegration(float integ);
#endif /*__STATE_CONTROL_H */

