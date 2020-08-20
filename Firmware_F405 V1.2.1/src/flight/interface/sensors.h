#ifndef __SENSORS_H
#define __SENSORS_H
#include "stabilizer_types.h"
#include "config.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ATKflight飞控固件
 * 传感器控制代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2018/5/2
 * 版本：V1.0
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

/*传感器读取更新频率*/
#define GYRO_UPDATE_RATE		RATE_500_HZ
#define ACC_UPDATE_RATE			RATE_500_HZ
#define MAG_UPDATE_RATE			RATE_10_HZ
#define BARO_UPDATE_RATE		RATE_50_HZ

extern sensorData_t sensors;

void sensorsTask(void *param);
void sensorsInit(void);			/*传感器初始化*/
bool sensorsTest(void);			/*传感器测试*/
bool sensorsAreCalibrated(void);	/*传感器数据校准*/
void sensorsAcquire(sensorData_t *sensors, const u32 tick);/*获取传感器数据*/
void getSensorRawData(Axis3i16* acc, Axis3i16* gyro, Axis3i16* mag);
bool sensorsIsMagPresent(void);

/* 单独测量传感器数据 */
bool sensorsReadGyro(Axis3f *gyro);
bool sensorsReadAcc(Axis3f *acc);
bool sensorsReadMag(Axis3f *mag);
bool sensorsReadBaro(baro_t *baro);

#endif //__SENSORS_H
