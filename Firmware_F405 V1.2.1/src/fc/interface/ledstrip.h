#ifndef __LED_STRIP_H__
#define __LED_STRIP_H__
#include "sys.h"

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

void ledStripInit(void);
void ledStripON(void);
void ledStripOFF(void);
void ledstripTask(void *param);

#endif

