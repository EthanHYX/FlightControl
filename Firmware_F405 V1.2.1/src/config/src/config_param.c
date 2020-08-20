#include <string.h>
#include "config_param.h"
#include "stmflash.h"
#include "beeper.h"
#include "runtime_config.h"

/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ATKflight飞控固件
 * 配置参数驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2018/5/2
 * 版本：V1.0
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/


#define VERSION 11	/*11 表示V1.1*/

configParam_t configParam;

static configParam_t configParamDefault=
{
	.version = VERSION,		/*软件版本号*/

	.pid = 
	{
		[RATE_ROLL]   = {85, 900, 18},
		[RATE_PITCH]  = {90, 1000, 18},
		[RATE_YAW]    = {120, 800, 0},
		[ANGLE_ROLL]  = {600, 0, 0},
		[ANGLE_PITCH] = {600, 0, 0},
		[ANGLE_YAW]   = {600, 0, 0},
		[VELOCITY_Z]  = {150, 200, 50},
		[POSHOLD_Z]   = {45, 0, 0},
		[VELOCITY_XY] = {0, 0, 0},
		[POSHOLD_XY]  = {0, 0, 0},
	},
	.accBias= 	/*加速度校准值*/
	{
		.accZero = 
		{
			0,
			0,
			0,
		},
		.accGain = 
		{
			4096,
			4096,
			4096,
		}
	},
	.magBias=
	{
		.magZero = 
		{
			0,
			0,
			0,
		},
	},
	.boardAlign=
	{
		.rollDeciDegrees = 0,
		.pitchDeciDegrees = 0,
		.yawDeciDegrees = 0,
	},
};

static u32 lenth = 0;
static bool isInit = false;
static bool isConfigParamOK = false;
static SemaphoreHandle_t  xSemaphore = NULL;
static u32 semaphoreGiveTime;

static u8 configParamCksum(configParam_t* data)
{
	int i;
	u8 cksum=0;	
	u8* c = (u8*)data;  	
	size_t len=sizeof(configParam_t);

	for (i=0; i<len; i++)
		cksum += *(c++);
	cksum-=data->cksum;
	
	return cksum;
}

void configParamInit(void)	/*参数配置初始化*/
{
	if(isInit) return;
	
	lenth=sizeof(configParam);
	lenth=lenth/4+(lenth%4 ? 1:0);

	STMFLASH_Read(CONFIG_PARAM_ADDR, (u32 *)&configParam, lenth);
	
	if (configParam.version == VERSION)	/*版本正确*/
	{
		if(configParamCksum(&configParam) == configParam.cksum)	/*校验正确*/
		{
			printf("Version V%1.1f check [OK]\r\n", configParam.version / 10.0f);
			isConfigParamOK = true;
		} else
		{
			printf("Version check [FAIL]\r\n");
			isConfigParamOK = false;
		}
	}
	else	/*版本更新*/
	{
		isConfigParamOK = false;
	}
	
	if(isConfigParamOK == false)	/*配置参数错误，写入默认参数*/
	{
		memcpy((u8 *)&configParam, (u8 *)&configParamDefault, sizeof(configParam));
		configParam.cksum = configParamCksum(&configParam);				/*计算校验值*/
		STMFLASH_Write(CONFIG_PARAM_ADDR,(u32 *)&configParam, lenth);	/*写入stm32 flash*/
		isConfigParamOK=true;
	}	
	
	xSemaphore = xSemaphoreCreateBinary();
	
	isInit=true;
}

void configParamTask(void* param)
{
	u8 cksum = 0;
	
	while(1) 
	{	
		xSemaphoreTake(xSemaphore, portMAX_DELAY);
		
		for (;;)
		{
			if ((getSysTickCnt() - semaphoreGiveTime > 2000) && (!ARMING_FLAG(ARMED)))//上次释放信号量时间至少大于2秒才写Flash，防止频繁写入
			{
				cksum = configParamCksum(&configParam);
				if((configParam.cksum != cksum))
				{
					configParam.cksum = cksum;					
					STMFLASH_Write(CONFIG_PARAM_ADDR,(u32 *)&configParam, lenth);
				}
				DISABLE_STATE(FLASH_WRITING);
			}
			else
			{
				xSemaphoreTake(xSemaphore, 100);
			}
		}
	}
}

void saveConfigAndNotify(void)
{
	if (!ARMING_FLAG(ARMED)) //锁定状态才可以操作Flash
	{
		beeperConfirmationBeeps(1);	
		xSemaphoreGive(xSemaphore);
		semaphoreGiveTime = getSysTickCnt();
		ENABLE_STATE(FLASH_WRITING);
	}
}

//恢复为默认参数
void resetConfigParam(void)
{
	configParam = configParamDefault;
	saveConfigAndNotify();
}

