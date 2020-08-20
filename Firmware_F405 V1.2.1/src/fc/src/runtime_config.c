#include "runtime_config.h"
#include "beeper.h"
#include "led.h"
#include "ledstrip.h"
#include "sensors.h"
#include "gyro.h"
#include "accelerometer.h"
#include "compass.h"
#include "barometer.h"

#include "pos_estimator.h"
#include "state_control.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ATKflight飞控固件
 * 运行监测代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2018/5/2
 * 版本：V1.0
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

uint32_t armingFlags = 0;
uint32_t stateFlags = 0;
uint32_t flightModeFlags = 0;


//使能给定飞行模式
uint32_t enableFlightMode(flightModeFlags_e mask)
{
    uint32_t oldVal = flightModeFlags;

    flightModeFlags |= (mask);
    if (flightModeFlags != oldVal)
        beeperConfirmationBeeps(1);
    return flightModeFlags;
}

//禁能给定飞行模式
uint32_t disableFlightMode(flightModeFlags_e mask)
{
    uint32_t oldVal = flightModeFlags;

    flightModeFlags &= ~(mask);
    if (flightModeFlags != oldVal)
        beeperConfirmationBeeps(1);
    return flightModeFlags;
}

//校准进行中
bool isCalibrating(void)
{
	if (!baroIsCalibrationComplete()) 
	{
		return true;
	}

	if (!posEstimatorIsCalibrationComplete()) 
	{
		return true;
	}

	if (!accIsCalibrationComplete()) 
	{
		return true;
	}

	if (!gyroIsCalibrationComplete()) 
	{
		return true;
	}

    return false;
}

//更新解锁标志位状态
void updateArmingStatus(void)
{
    if (ARMING_FLAG(ARMED)) 
	{
        LED0_ON;
    } 
	else 
	{
		//传感器校准中
		static bool calibratingFinishedBeep = false;
		if (isCalibrating()) 
		{
			ENABLE_ARMING_FLAG(ARMING_DISABLED_SENSORS_CALIBRATING);
			calibratingFinishedBeep = false;
		}
		else
		{
			DISABLE_ARMING_FLAG(ARMING_DISABLED_SENSORS_CALIBRATING);

			if (!calibratingFinishedBeep) 
			{
				calibratingFinishedBeep = true;
				beeper(BEEPER_RUNTIME_CALIBRATION_DONE);
			}
		}

		//检查水平最小角
		if (!STATE(SMALL_ANGLE)) 
		{
			ENABLE_ARMING_FLAG(ARMING_DISABLED_NOT_LEVEL);
		}
		else 
		{
			DISABLE_ARMING_FLAG(ARMING_DISABLED_NOT_LEVEL);
		}

		//检查加速度计是否已经校准完成
		if (!STATE(ACCELEROMETER_CALIBRATED)) 
		{
			ENABLE_ARMING_FLAG(ARMING_DISABLED_ACCELEROMETER_NOT_CALIBRATED);
		}
		else 
		{
			DISABLE_ARMING_FLAG(ARMING_DISABLED_ACCELEROMETER_NOT_CALIBRATED);
		}

		//检查磁力计是否校准
		if (sensorsIsMagPresent() && !STATE(COMPASS_CALIBRATED)) 
		{
			ENABLE_ARMING_FLAG(ARMING_DISABLED_COMPASS_NOT_CALIBRATED);
		}
		else 
		{
			DISABLE_ARMING_FLAG(ARMING_DISABLED_COMPASS_NOT_CALIBRATED);
		}
		
		//检查是否在写Flash
		if (STATE(FLASH_WRITING))
		{
			ENABLE_ARMING_FLAG(ARMING_DISABLED_FLASH_WRITING);
		}
		else
		{
			DISABLE_ARMING_FLAG(ARMING_DISABLED_FLASH_WRITING);
		}
		
		//刷新LED灯状态
        if (!isArmingDisabled()) 
		{
            warningLedFlash();
        } 
		else 
		{
            warningLedON();
        }
		
        warningLedUpdate();
    }
}

//锁定飞控
void mwDisarm(void)
{
    if (ARMING_FLAG(ARMED)) 
	{
        DISABLE_ARMING_FLAG(ARMED);
        beeper(BEEPER_DISARMING); 
    }
}

//解锁飞控
void mwArm(void)
{
    updateArmingStatus();

    if (!isArmingDisabled()) 
	{
        if (ARMING_FLAG(ARMED)) 
		{
            return;
        }
		ENABLE_ARMING_FLAG(ARMED);
		ENABLE_ARMING_FLAG(WAS_EVER_ARMED);
		
		stateControlResetYawHolding();//复位航向角锁定值
        beeper(BEEPER_ARMING);
        return;
    }

    if (!ARMING_FLAG(ARMED)) 
	{
        beeperConfirmationBeeps(1);
    }
}

