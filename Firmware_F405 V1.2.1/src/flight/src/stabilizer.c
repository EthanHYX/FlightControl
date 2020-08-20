#include "system.h"
#include "stabilizer.h"
#include "sensors.h"
#include "commander.h"
#include "state_control.h"
#include "power_control.h"
#include "pos_estimator.h"
#include "gyro.h"
#include "led.h"
#include "runtime_config.h"

/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ATKflight飞控固件
 * 四轴自稳控制代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2018/5/2
 * 版本：V1.0
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

static bool isInit;
setpoint_t		setpoint;	/*设置目标状态*/
sensorData_t 	sensorData;	/*传感器数据*/
state_t 		state;		/*四轴姿态*/
control_t 		control;	/*四轴控制参数*/


void stabilizerInit(void)
{
	if(isInit) return;
	
	stateControlInit();		/*姿态PID初始化*/
	powerControlInit();		/*电机初始化*/
	imuInit();				/*姿态解算初始化*/
	isInit = true;
}

void stabilizerTask(void* param)
{
	u32 tick = 0;
	u32 lastWakeTime = getSysTickCnt();
	
	//等待陀螺仪校准完成
	while(!gyroIsCalibrationComplete())
	{
		vTaskDelayUntil(&lastWakeTime, M2T(1));
	}
	
	while(1) 
	{
		//1KHz运行频率
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_1000_HZ));	
		
		//获取传感器数据
		if (RATE_DO_EXECUTE(MAIN_LOOP_RATE, tick))
		{
			sensorsAcquire(&sensorData, tick);				
		}
		
		//四元数和欧拉角计算
		if (RATE_DO_EXECUTE(ATTITUDE_ESTIMAT_RATE, tick))
		{
			imuUpdateAttitude(&sensorData, &state, ATTITUDE_ESTIMAT_DT);				
		}
		
		//位置预估计算
		if (RATE_DO_EXECUTE(POSITION_ESTIMAT_RATE, tick))
		{  	
			updatePositionEstimator(&sensorData, &state, POSITION_ESTIMAT_DT);
		}
		
		//目标姿态和飞行模式设定	
		if (RATE_DO_EXECUTE(MAIN_LOOP_RATE, tick))
		{
			commanderGetSetpoint(&state, &setpoint);
			updateArmingStatus();
		}
		
		//PID控制器计算控制输出
		stateControl(&sensorData, &state, &setpoint, &control, tick);
		
		//控制电机输出（500Hz）
		if (RATE_DO_EXECUTE(MAIN_LOOP_RATE, tick))
		{
			powerControl(&control);
		}
		
		tick++;
	}
}

