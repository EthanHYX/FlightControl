#include <math.h>
#include "stdio.h"
#include "delay.h"
#include "config.h"
#include "config_param.h"
#include "sensors.h"
#include "pos_estimator.h"
#include "stabilizer.h"
#include "i2cdev.h"
#include "gyro.h"
#include "accelerometer.h"
#include "compass.h"
#include "barometer.h"
#include "sensorsalignment.h"

/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

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

sensorData_t sensors;

static bool isInit = false;

static bool isMPUPresent=false;
static bool isMagPresent=false;
static bool isBaroPresent=false;

static xQueueHandle accelerometerDataQueue;
static xQueueHandle gyroDataQueue;
static xQueueHandle magnetometerDataQueue;
static xQueueHandle barometerDataQueue;


/*从队列读取陀螺数据*/
bool sensorsReadGyro(Axis3f *gyro)
{
	return (pdTRUE == xQueueReceive(gyroDataQueue, gyro, 0));
}
/*从队列读取加速计数据*/
bool sensorsReadAcc(Axis3f *acc)
{
	return (pdTRUE == xQueueReceive(accelerometerDataQueue, acc, 0));
}
/*从队列读取磁力计数据*/
bool sensorsReadMag(Axis3f *mag)
{
	return (pdTRUE == xQueueReceive(magnetometerDataQueue, mag, 0));
}
/*从队列读取气压数据*/
bool sensorsReadBaro(baro_t *baro)
{
	return (pdTRUE == xQueueReceive(barometerDataQueue, baro, 0));
}

/* 传感器器件初始化 */
void sensorsInit(void)
{
	if (isInit) return;
	
	initBoardAlignment();

	isMPUPresent = gyroInit(GYRO_UPDATE_RATE);
	isMPUPresent &= accInit(ACC_UPDATE_RATE);
	isBaroPresent = baroInit();
	isMagPresent  = compassInit();
	
	/*创建传感器数据队列*/
	accelerometerDataQueue = xQueueCreate(1, sizeof(Axis3f));
	gyroDataQueue = xQueueCreate(1, sizeof(Axis3f));
	magnetometerDataQueue = xQueueCreate(1, sizeof(Axis3f));
	barometerDataQueue = xQueueCreate(1, sizeof(baro_t));
	
	isInit = true;
}

/*传感器任务*/
void sensorsTask(void *param)
{	
	u32 tick = 0;
	u32 lastWakeTime = getSysTickCnt();
	sensorsInit();		//传感器初始化
	
	while (1)
	{
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_1000_HZ));//1KHz运行频率

		if (isMPUPresent && RATE_DO_EXECUTE(GYRO_UPDATE_RATE, tick))
		{
			gyroUpdate(&sensors.gyro);
		}
		
		if (isMPUPresent && RATE_DO_EXECUTE(ACC_UPDATE_RATE, tick))
		{
			accUpdate(&sensors.acc);
		}
		
		if (isMagPresent && RATE_DO_EXECUTE(MAG_UPDATE_RATE, tick))
		{
			compassUpdate(&sensors.mag);
		}
		
		if (isBaroPresent && RATE_DO_EXECUTE(BARO_UPDATE_RATE, tick))
		{
			baroUpdate(&sensors.baro);
		}
		
		vTaskSuspendAll();	/*确保同一时刻把数据放入队列中*/
		xQueueOverwrite(accelerometerDataQueue, &sensors.acc);
		xQueueOverwrite(gyroDataQueue, &sensors.gyro);
		if (isMagPresent)
		{
			xQueueOverwrite(magnetometerDataQueue, &sensors.mag);
		}
		if (isBaroPresent)
		{
			xQueueOverwrite(barometerDataQueue, &sensors.baro);
		}
		xTaskResumeAll();

		tick++;
	}	
}

/*获取传感器数据*/
void sensorsAcquire(sensorData_t *sensors, const u32 tick)
{
	sensorsReadGyro(&sensors->gyro);
	sensorsReadAcc(&sensors->acc);
	sensorsReadMag(&sensors->mag);
	sensorsReadBaro(&sensors->baro);
}

bool sensorsIsMagPresent(void)
{
	return isMagPresent;
}
