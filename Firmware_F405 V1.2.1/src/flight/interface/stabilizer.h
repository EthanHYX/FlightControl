#ifndef __STABALIZER_H
#define __STABALIZER_H
#include <stdbool.h>
#include <stdint.h>
#include "stabilizer_types.h"

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

#define MAIN_LOOP_RATE 			RATE_500_HZ				//主循环速率
#define MAIN_LOOP_DT			(1.0/MAIN_LOOP_RATE)	

#define ATTITUDE_ESTIMAT_RATE	RATE_500_HZ				//姿态解算速率
#define ATTITUDE_ESTIMAT_DT		(1.0/ATTITUDE_ESTIMAT_RATE)

#define POSITION_ESTIMAT_RATE	ATTITUDE_ESTIMAT_RATE	//位置预估速率（和姿态解算速率一致）
#define POSITION_ESTIMAT_DT		(1.0/POSITION_ESTIMAT_RATE)

#define RATE_PID_RATE			MAIN_LOOP_RATE 			//角速度环PID速率（和主循环速率一致）
#define RATE_PID_DT				(1.0/RATE_PID_RATE)

#define ANGLE_PID_RATE			ATTITUDE_ESTIMAT_RATE 	//角度环PID速率（和姿态解算速率一致）
#define ANGLE_PID_DT			(1.0/ANGLE_PID_RATE)

#define VELOCITY_PID_RATE		POSITION_ESTIMAT_RATE 	//位置速度环PID速率（和位置预估速率一致）
#define VELOCITY_PID_DT			(1.0/VELOCITY_PID_RATE)

#define POSITION_PID_RATE		POSITION_ESTIMAT_RATE 	//位置环PID速率（和位置预估速率一致）
#define POSITION_PID_DT			(1.0/POSITION_PID_RATE)

extern setpoint_t		setpoint;	/*设置目标状态*/
extern sensorData_t 	sensorData;	/*传感器数据*/
extern state_t 			state;		/*四轴姿态*/
extern control_t 		control;	/*四轴控制参数*/

void stabilizerInit(void);
void stabilizerTask(void* param);
bool stabilizerTest(void);

void getAttitudeData(attitude_t* get);
float getBaroData(void);
float getPositionZ(void);
bool getIsCalibPass(void);

void getState(state_t* get);
void getSensorData(sensorData_t* get);
void executeAccBiasFound(void);
#endif /* __STABALIZER_H */
