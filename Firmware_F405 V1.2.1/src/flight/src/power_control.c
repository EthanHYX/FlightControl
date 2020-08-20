#include "power_control.h"
#include "motors.h"
#include "maths.h"
#include "runtime_config.h"
#include "config.h"
#include "commander.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ATKflight飞控固件
 * 功率输出控制代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2018/5/2
 * 版本：V1.0
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/
motorPWM_t motorPWM;

void powerControlInit(void)
{
	motorsInit();
}

void powerControl(control_t *control)
{
	if(ARMING_FLAG(ARMED))//解锁状态
	{
		motorPWM.m1 = control->thrust - control->roll + control->pitch + control->yaw;
		motorPWM.m2 = control->thrust - control->roll - control->pitch - control->yaw;
		motorPWM.m3 = control->thrust + control->roll + control->pitch - control->yaw;
		motorPWM.m4 = control->thrust + control->roll - control->pitch + control->yaw;
		
		motorPWM.m1 = constrain(motorPWM.m1, MINTHROTTLE, MAXTHROTTLE) - 1000;//减去1000基础值，实际油门应该是0-1000
		motorPWM.m2 = constrain(motorPWM.m2, MINTHROTTLE, MAXTHROTTLE) - 1000;
		motorPWM.m3 = constrain(motorPWM.m3, MINTHROTTLE, MAXTHROTTLE) - 1000;
		motorPWM.m4 = constrain(motorPWM.m4, MINTHROTTLE, MAXTHROTTLE) - 1000;
	}
	else if (ARMING_FLAG(ARMING_DISABLED_PID_BYPASS))//电机测试模式，PID旁路
	{
		motorPWM.m1 = constrain(rcCommand[THROTTLE], RC_MIN, RC_MAX) - 1000;//减去1000基础值，实际油门应该是0-1000
		motorPWM.m2 = constrain(rcCommand[THROTTLE], RC_MIN, RC_MAX) - 1000;
		motorPWM.m3 = constrain(rcCommand[THROTTLE], RC_MIN, RC_MAX) - 1000;
		motorPWM.m4 = constrain(rcCommand[THROTTLE], RC_MIN, RC_MAX) - 1000;
	}
	else
	{
		motorPWM.m1 = 0;
		motorPWM.m2 = 0;
		motorPWM.m3 = 0;
		motorPWM.m4 = 0;
	}
	
	motorsSetRatio(MOTOR_M1, motorPWM.m1);
	motorsSetRatio(MOTOR_M2, motorPWM.m2);
	motorsSetRatio(MOTOR_M3, motorPWM.m3);
	motorsSetRatio(MOTOR_M4, motorPWM.m4);
}

