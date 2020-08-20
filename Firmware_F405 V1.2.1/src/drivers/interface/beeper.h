#ifndef __BEEPER_H__
#define __BEEPER_H__
#include "sys.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ATKflight飞控固件
 * 蜂鸣器音乐序列驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2018/5/2
 * 版本：V1.2
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

#define BEEP_OFF	GPIO_ResetBits(GPIOC, GPIO_Pin_13);          
#define BEEP_ON		GPIO_SetBits(GPIOC, GPIO_Pin_13);      
	
typedef enum {
    // IMPORTANT: these are in priority order, 0 = Highest
    BEEPER_SILENCE = 0,             // Silence, see beeperSilence()

    BEEPER_RUNTIME_CALIBRATION_DONE,
    BEEPER_HARDWARE_FAILURE,        // HW failure
    BEEPER_RX_LOST,                 // Beeps when TX is turned off or signal lost (repeat until TX is okay)
    BEEPER_RX_LOST_LANDING,         // Beeps SOS when armed and TX is turned off or signal lost (autolanding/autodisarm)
    BEEPER_DISARMING,               // Beep when disarming the board
    BEEPER_ARMING,                  // Beep when arming the board
    BEEPER_ARMING_GPS_FIX,          // Beep a special tone when arming the board and GPS has fix
    BEEPER_BAT_CRIT_LOW,            // Longer warning beeps when battery is critically low (repeats)
    BEEPER_BAT_LOW,                 // Warning beeps when battery is getting low (repeats)
    BEEPER_GPS_STATUS,              // FIXME **** Disable beeper when connected to USB ****
    BEEPER_RX_SET,                  // Beeps when aux channel is set for beep or beep sequence how many satellites has found if GPS enabled
    BEEPER_ACTION_SUCCESS,          // Action success (various actions)
    BEEPER_ACTION_FAIL,             // Action fail (varions actions)
    BEEPER_READY_BEEP,              // Ring a tone when GPS is locked and ready
    BEEPER_MULTI_BEEPS,             // Internal value used by 'beeperConfirmationBeeps()'.
    BEEPER_DISARM_REPEAT,           // Beeps sounded while stick held in disarm position
    BEEPER_ARMED,                   // Warning beeps when board is armed (repeats until board is disarmed or throttle is increased)
    BEEPER_SYSTEM_INIT,             // Initialisation beeps when board is powered on
    BEEPER_USB,                     // Some boards have beeper powered USB connected
    BEEPER_LAUNCH_MODE_ENABLED,     // Fixed-wing launch mode enabled
    BEEPER_CAM_CONNECTION_OPEN,     // When the 5 key simulation stated 
    BEEPER_CAM_CONNECTION_CLOSE,    // When the 5 key simulation stop 
    BEEPER_ALL,                     // Turn ON or OFF all beeper conditions
    BEEPER_PREFERENCE,              // Save preferred beeper configuration
    // BEEPER_ALL and BEEPER_PREFERENCE must remain at the bottom of this enum
} beeperMode_e;

void beeper(beeperMode_e mode);
void beeperTask(void* param);
void beeperConfirmationBeeps(uint8_t beepCount);
void beeperInit(void);
void setLedStripAllwaysIsON(bool sate);


#endif
