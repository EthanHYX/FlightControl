#include "ledstrip.h"
#include "ws2812.h"

/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "timers.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ATKflight飞控固件
 * WS2812灯带驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2018/5/2
 * 版本：V1.2
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

#define NBR_LEDS  8	//ws2812 RGB灯个数

enum ledStripColor
{
	RED = 0,
	GREEN,
	BLUE,
	WHITE,
	BLACK,//黑色（不亮）
	COLOR_NUM,
};

const uint8_t colorTable[COLOR_NUM][3] =
{
	{0xff, 0x00, 0x00},//红
	{0x00, 0xFF, 0x00},//绿
	{0x00, 0x00, 0xFF},//蓝
	{0xff, 0xff, 0xff},//白
	{0x00, 0x00, 0x00},//黑（不亮）
};

typedef enum
{
    WARNING_LED_OFF = 0,
    WARNING_LED_ON,
} ledStripState_e;

static uint8_t colorBuffer[NBR_LEDS][3];


//填充颜色
static void ledStripFillBufferWitchColor(enum ledStripColor color)
{
	for (int i = 0; i < NBR_LEDS; i++)
	{
		colorBuffer[i][0] = colorTable[color][0];
		colorBuffer[i][1] = colorTable[color][1];
		colorBuffer[i][2] = colorTable[color][2];
	}
}


//灯带初始化
void ledStripInit(void)
{
	ws2812Init();
	ledStripOFF();
}

void ledStripON(void)
{
	ledStripFillBufferWitchColor(RED);
	ws2812Send(colorBuffer, NBR_LEDS);
}

void ledStripOFF(void)
{
	ledStripFillBufferWitchColor(BLACK);
	ws2812Send(colorBuffer, NBR_LEDS);
}



