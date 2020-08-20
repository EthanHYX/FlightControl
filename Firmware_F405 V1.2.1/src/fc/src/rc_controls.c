#include "rc_controls.h"
#include "rx.h"
#include "sensorsalignment.h"
#include "beeper.h"
#include "commander.h"
#include "runtime_config.h"
#include "ledstrip.h"

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ATKflight飞控固件
 * 遥控控制代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2018/5/2
 * 版本：V1.0
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

#define RC_PERIOD_MS     20

stickPositions_e rcStickPositions = (stickPositions_e) 0; //遥杆位置状态
auxPositions_e   channelPos[CH_NUM];//辅助通道位置状态

stickPositions_e getRcStickPositions(void)
{
    return rcStickPositions;
}

//更新遥杆位置
static void updateRcStickPositions(void)
{
	stickPositions_e tmp = (stickPositions_e)0;
	
	tmp |= ((rcData[ROLL] > RC_COMMANDER_MINCHECK) ? 0x02 : 0x00) << (ROLL * 2);
	tmp |= ((rcData[ROLL] < RC_COMMANDER_MAXCHECK) ? 0x01 : 0x00) << (ROLL * 2);

	tmp |= ((rcData[PITCH] > RC_COMMANDER_MINCHECK) ? 0x02 : 0x00) << (PITCH * 2);
	tmp |= ((rcData[PITCH] < RC_COMMANDER_MAXCHECK) ? 0x01 : 0x00) << (PITCH * 2);

	tmp |= ((rcData[THROTTLE] > RC_COMMANDER_MINCHECK) ? 0x02 : 0x00) << (THROTTLE * 2);
	tmp |= ((rcData[THROTTLE] < RC_COMMANDER_MAXCHECK) ? 0x01 : 0x00) << (THROTTLE * 2);

	tmp |= ((rcData[YAW] > RC_COMMANDER_MINCHECK) ? 0x02 : 0x00) << (YAW * 2);
	tmp |= ((rcData[YAW] < RC_COMMANDER_MAXCHECK) ? 0x01 : 0x00) << (YAW * 2);
	
    rcStickPositions = tmp;
}

//处理遥杆位置和对应命令
void processRcStickPositions(void)
{
	static u32 lastTickTimeMs;
	static uint8_t rcDelayCommand;
	static uint32_t rcSticks;
	u32 currentTimeMs = getSysTickCnt();
	
    updateRcStickPositions();
	uint32_t stTmp = getRcStickPositions();
	if(stTmp == rcSticks)
	{
		if(rcDelayCommand<250 && (currentTimeMs - lastTickTimeMs) >= RC_PERIOD_MS)
		{	
			lastTickTimeMs = currentTimeMs;
			rcDelayCommand++;
		}
	}
	else
		rcDelayCommand = 0;
	
	rcSticks = stTmp;
	
	if (rcDelayCommand != 20) return;
	
	//加锁
	if (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_CE)
	{
		mwDisarm();
	}
	//解锁
	if (rcSticks == THR_LO + YAW_HI + PIT_CE + ROL_CE)
	{
		if (channelPos[AUX1] == AUX_LO)
		{
			mwArm();
		}
	}
	
	//加速度微调
    if (rcSticks == THR_HI + YAW_CE + PIT_HI + ROL_CE) 
	{
        applyAndSaveBoardAlignmentDelta(0, -1);
        rcDelayCommand = 10;
        return;
    } 
	else if (rcSticks == THR_HI + YAW_CE + PIT_LO + ROL_CE) 
	{
        applyAndSaveBoardAlignmentDelta(0, 1);
        rcDelayCommand = 10;
        return;
    } 
	else if (rcSticks == THR_HI + YAW_CE + PIT_CE + ROL_HI) 
	{
        applyAndSaveBoardAlignmentDelta(-1, 0);
        rcDelayCommand = 10;
        return;
    } 
	else if (rcSticks == THR_HI + YAW_CE + PIT_CE + ROL_LO) 
	{
        applyAndSaveBoardAlignmentDelta(1, 0);
        rcDelayCommand = 10;
        return;
    }
	
	//打开和关闭RGB灯
    if (rcSticks == THR_LO + YAW_CE + PIT_LO + ROL_HI) 
	{
		setLedStripAllwaysIsON(true);
	}
	else if (rcSticks == THR_LO + YAW_CE + PIT_LO + ROL_LO) 
	{
		setLedStripAllwaysIsON(false);
	}
}

//处理辅助通道位置和对应命令
void processRcAUXPositions(void)
{
	for (int i=AUX1; i<CH_NUM; i++)
	{
		if (rcData[i] < (RC_MID-200))
			channelPos[i] = AUX_LO;
		else if (rcData[i] > (RC_MID+200))
			channelPos[i] = AUX_HI;
		else
			channelPos[i] = AUX_CE;
	}

	//AUX3通道（三段开关）对应三个模式（自稳模式、定高模式、定点模式）
	if(channelPos[AUX3] == AUX_LO)//自稳模式
	{
		if (!FLIGHT_MODE(ANGLE_MODE))
			ENABLE_FLIGHT_MODE(ANGLE_MODE);
		
		if (FLIGHT_MODE(NAV_ALTHOLD_MODE))
			DISABLE_FLIGHT_MODE(NAV_ALTHOLD_MODE);
	
		if (FLIGHT_MODE(NAV_POSHOLD_MODE))
			DISABLE_FLIGHT_MODE(NAV_POSHOLD_MODE);
	}
	else if(channelPos[AUX3] == AUX_CE)//定高模式
	{
		if (!FLIGHT_MODE(NAV_ALTHOLD_MODE))
		{
			ENABLE_FLIGHT_MODE(NAV_ALTHOLD_MODE);
			commanderSetupAltitudeHoldMode();
		}
		
		if (FLIGHT_MODE(NAV_POSHOLD_MODE))
			DISABLE_FLIGHT_MODE(NAV_POSHOLD_MODE);
	}
	else if(channelPos[AUX3] == AUX_HI)//定点模式
	{
		if (!FLIGHT_MODE(NAV_POSHOLD_MODE))
			ENABLE_FLIGHT_MODE(NAV_POSHOLD_MODE);
	}
	
	//AUX4通道（两段开关）对应两个模式（有头模式、无头模式）
	if(channelPos[AUX4] == AUX_LO)
	{
		if (FLIGHT_MODE(HEADFREE_MODE))//有头模式
			DISABLE_FLIGHT_MODE(HEADFREE_MODE);
	}
	else if(channelPos[AUX4] == AUX_HI)
	{
		if (!FLIGHT_MODE(HEADFREE_MODE))//无头模式
			ENABLE_FLIGHT_MODE(HEADFREE_MODE);
	}
	
	//AUX2通道（两段开关）打开和关闭蜂鸣器（炸机时寻机用）
	if(channelPos[AUX2] == AUX_LO)
	{
		if (FLIGHT_MODE(BEEPER_ON_MODE))//关闭蜂鸣器
			DISABLE_FLIGHT_MODE(BEEPER_ON_MODE);
	}
	else if(channelPos[AUX2] == AUX_HI)
	{
		if (!FLIGHT_MODE(BEEPER_ON_MODE))//打开蜂鸣器
			ENABLE_FLIGHT_MODE(BEEPER_ON_MODE);
	}
	
	//AUX1通道（两段开关）紧急停机
	if(channelPos[AUX1] == AUX_LO)
	{
		
	}
	else if(channelPos[AUX1] == AUX_HI)
	{
		if(ARMING_FLAG(ARMED))
		{
			mwDisarm();//锁定
		}
	}	
}

//获取油门遥杆状态
throttleStatus_e calculateThrottleStatus(void)
{
    if (rcData[THROTTLE] < RC_COMMANDER_MINCHECK)
        return THROTTLE_LOW;

    return THROTTLE_HIGH;
}






