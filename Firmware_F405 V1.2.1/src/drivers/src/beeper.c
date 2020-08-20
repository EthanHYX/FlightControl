#include "beeper.h"
#include "led.h"
#include "ledstrip.h"
#include "runtime_config.h"

/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"

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

#define MAX_MULTI_BEEPS 20   //自定义序列最大长度

#define BEEPER_COMMAND_REPEAT 0xFE
#define BEEPER_COMMAND_STOP   0xFF

//蜂鸣器音乐序列，1、3、5...元素表示蜂鸣器on的时间，2、4、6...表示off时间，单位为10ms
// short fast beep
static const uint8_t beep_shortBeep[] = {
    10, 10, BEEPER_COMMAND_STOP
};
// arming beep
static const uint8_t beep_armingBeep[] = {
    30, 5, 5, 5, BEEPER_COMMAND_STOP
};
// armed beep (first pause, then short beep)
static const uint8_t beep_armedBeep[] = {
    0, 245, 10, 5, BEEPER_COMMAND_STOP
};
// disarming beeps
static const uint8_t beep_disarmBeep[] = {
    15, 5, 15, 5, BEEPER_COMMAND_STOP
};
// beeps while stick held in disarm position (after pause)
static const uint8_t beep_disarmRepeatBeep[] = {
    0, 100, 10, BEEPER_COMMAND_STOP
};
// Long beep and pause after that
static const uint8_t beep_lowBatteryBeep[] = {
    25, 50, BEEPER_COMMAND_STOP
};
// critical battery beep
static const uint8_t beep_critBatteryBeep[] = {
    50, 2, BEEPER_COMMAND_STOP
};

// transmitter-signal-lost tone
static const uint8_t beep_txLostBeep[] = {
    50, 50, BEEPER_COMMAND_STOP
};
// SOS morse code:
static const uint8_t beep_sos[] = {
    10, 10, 10, 10, 10, 40, 40, 10, 40, 10, 40, 40, 10, 10, 10, 10, 10, 70, BEEPER_COMMAND_STOP
};
// Arming when GPS is fixed
static const uint8_t beep_armedGpsFix[] = {
    5, 5, 15, 5, 5, 5, 15, 30, BEEPER_COMMAND_STOP
};
// Ready beeps. When gps has fix and copter is ready to fly.
static const uint8_t beep_readyBeep[] = {
    4, 5, 4, 5, 8, 5, 15, 5, 8, 5, 4, 5, 4, 5, BEEPER_COMMAND_STOP
};
// 2 fast short beeps
static const uint8_t beep_2shortBeeps[] = {
    5, 5, 5, 5, BEEPER_COMMAND_STOP
};
// 2 longer beeps
static const uint8_t beep_2longerBeeps[] = {
    20, 15, 35, 5, BEEPER_COMMAND_STOP
};
// 3 beeps
static const uint8_t beep_runtimeCalibrationDone[] = {
    20, 10, 20, 10, 20, 10, BEEPER_COMMAND_STOP
};
// two short beeps and a pause (first pause, then short beep)
static const uint8_t beep_launchModeBeep[] = {
    5, 5, 5, 100, BEEPER_COMMAND_STOP
};
// short beeps
static const uint8_t beep_hardwareFailure[] = {
    10, 10, BEEPER_COMMAND_STOP
};
// Cam connection opened
static const uint8_t beep_camOpenBeep[] = { 
    5, 15, 10, 15, 20, BEEPER_COMMAND_STOP 
}; 
// Cam connection close 
static const uint8_t beep_camCloseBeep[] = { 
    10, 8, 5, BEEPER_COMMAND_STOP 
};

//自定义序列
static uint8_t beep_multiBeeps[MAX_MULTI_BEEPS + 2];

#define BEEPER_CONFIRMATION_BEEP_DURATION 2
#define BEEPER_CONFIRMATION_BEEP_GAP_DURATION 20


// Beeper off = 0 Beeper on = 1
static uint8_t beeperIsOn = 0;

// Place in current sequence
static uint16_t beeperPos = 0;
// Time when beeper routine must act next time
static uint32_t beeperNextToggleTime = 0;

static bool ledStripAllwaysON = true;

typedef struct beeperTableEntry_s 
{
    uint8_t mode;
    uint8_t priority; // 0 = Highest
    const uint8_t *sequence;
} beeperTableEntry_t;


const beeperTableEntry_t beeperTable[] = {
    {BEEPER_RUNTIME_CALIBRATION_DONE, 0, beep_runtimeCalibrationDone,},
    {BEEPER_HARDWARE_FAILURE ,        1, beep_hardwareFailure,       },
    {BEEPER_RX_LOST,                  2, beep_txLostBeep,            },
    {BEEPER_RX_LOST_LANDING,          3, beep_sos,                   },
    {BEEPER_DISARMING,                4, beep_disarmBeep,            },
    {BEEPER_ARMING,                   5, beep_armingBeep,            },
    {BEEPER_ARMING_GPS_FIX,           6, beep_armedGpsFix,           },
    {BEEPER_BAT_CRIT_LOW,             7, beep_critBatteryBeep,       },
    {BEEPER_BAT_LOW,                  8, beep_lowBatteryBeep,        },
    {BEEPER_GPS_STATUS,               9, beep_multiBeeps,            },
    {BEEPER_RX_SET,                   10, beep_shortBeep,            },
    {BEEPER_ACTION_SUCCESS,           11, beep_2shortBeeps,          },
    {BEEPER_ACTION_FAIL,              12, beep_2longerBeeps,         },
    {BEEPER_READY_BEEP,               13, beep_readyBeep,            },
    {BEEPER_MULTI_BEEPS,              14, beep_multiBeeps,           }, 
    {BEEPER_DISARM_REPEAT,            15, beep_disarmRepeatBeep,     },
    {BEEPER_ARMED,                    16, beep_armedBeep,            },
    {BEEPER_SYSTEM_INIT,              17, NULL,                      },
    {BEEPER_USB,                      18, NULL,                      },
    {BEEPER_LAUNCH_MODE_ENABLED,      19, beep_launchModeBeep,       },
    {BEEPER_CAM_CONNECTION_OPEN,      20, beep_camOpenBeep,          },
    {BEEPER_CAM_CONNECTION_CLOSE,     21, beep_camCloseBeep,         },
    {BEEPER_ALL,                      22, NULL,                      },
    {BEEPER_PREFERENCE,               23, NULL,                      },
};

#define BEEPER_TABLE_ENTRY_COUNT (sizeof(beeperTable) / sizeof(beeperTableEntry_t))

static const beeperTableEntry_t *currentBeeperEntry = NULL;


void beeperInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
    GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOC, GPIO_Pin_13);
}

//关闭蜂鸣器
void beeperSilence(void)
{
    BEEP_OFF;
    beeperIsOn = 0;
    beeperNextToggleTime = 0;
    beeperPos = 0;
    currentBeeperEntry = NULL;
	
	if (ledStripAllwaysON)
		ledStripON();
	else
		ledStripOFF();
}

//激活蜂鸣器对应的序列
void beeper(beeperMode_e mode)
{
    if (mode == BEEPER_SILENCE) 
	{
        beeperSilence();
        return;
    }

    const beeperTableEntry_t *selectedCandidate = NULL;
    for (uint32_t i = 0; i < BEEPER_TABLE_ENTRY_COUNT; i++) 
	{
        const beeperTableEntry_t *candidate = &beeperTable[i];
        if (candidate->mode != mode)//查找序列
		{
            continue;
        }

        if (!currentBeeperEntry)//选定序列
		{
            selectedCandidate = candidate;
            break;
        }

        if (candidate->priority < currentBeeperEntry->priority)//优先级 
		{
            selectedCandidate = candidate;
        }
		
        break;
    }

    if (!selectedCandidate) 
	{
        return;
    }

    currentBeeperEntry = selectedCandidate;

    beeperPos = 0;
    beeperNextToggleTime = 0;
}

//计算当前序列的位置和下次触发时间
static void beeperProcessCommand(void)
{
    if (currentBeeperEntry->sequence[beeperPos] == BEEPER_COMMAND_REPEAT) 
	{
        beeperPos = 0;
    } 
	else if (currentBeeperEntry->sequence[beeperPos] == BEEPER_COMMAND_STOP) 
	{
        beeperSilence();
    }
	else
	{
		u32 currentTick = getSysTickCnt();
        beeperNextToggleTime = currentTick + 10 * currentBeeperEntry->sequence[beeperPos];
        beeperPos++;
    }
}

//蜂鸣器任务
void beeperTask(void* param)
{
	u32 lastWakeTime = getSysTickCnt();
	
	while(1)
	{
		vTaskDelayUntil(&lastWakeTime, 10);//10ms
		
		if (FLIGHT_MODE(BEEPER_ON_MODE))//辅助通道激活蜂鸣器
		{
			beeper(BEEPER_RX_SET);
		}
		
		if (currentBeeperEntry == NULL) 
		{
			continue;
		}
		
		u32 currentTick = getSysTickCnt();
		if (beeperNextToggleTime > currentTick) 
		{
			continue;
		}
	
		if (!beeperIsOn) 
		{
			beeperIsOn = 1;
			if (currentBeeperEntry->sequence[beeperPos] != 0) 
			{
				BEEP_ON;
				ledStripON();
				warningLedON();
				warningLedRefresh();
			}
		} 
		else 
		{
			beeperIsOn = 0;
			if (currentBeeperEntry->sequence[beeperPos] != 0) 
			{
				BEEP_OFF;
				ledStripOFF();
				warningLedOFF();
				warningLedRefresh();				
			}
		}

		beeperProcessCommand();
	}
}

//发出20ms打开，200ms关闭的叫声
//beepCount表示重复叫声的次数
void beeperConfirmationBeeps(uint8_t beepCount)
{
    int i;
    int cLimit;

    i = 0;
    cLimit = beepCount * 2;
    if (cLimit > MAX_MULTI_BEEPS)
        cLimit = MAX_MULTI_BEEPS;  //stay within array size
    do 
	{
        beep_multiBeeps[i++] = BEEPER_CONFIRMATION_BEEP_DURATION;       // 20ms beep
        beep_multiBeeps[i++] = BEEPER_CONFIRMATION_BEEP_GAP_DURATION;   // 200ms pause
    } while (i < cLimit);
    beep_multiBeeps[i] = BEEPER_COMMAND_STOP;     //sequence end
    beeper(BEEPER_MULTI_BEEPS);    //initiate sequence
}

//设置LED灯带在蜂鸣器报警完成后为常亮或常灭
void setLedStripAllwaysIsON(bool sate)
{
	ledStripAllwaysON = sate;
	if (ledStripAllwaysON)
		ledStripON();
	else
		ledStripOFF();
}


