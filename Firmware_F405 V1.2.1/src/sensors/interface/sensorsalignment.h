#ifndef __SENSORS_ALIGNMENT_H
#define __SENSORS_ALIGNMENT_H
#include "sys.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ATKflight飞控固件
 * 传感器方向对齐驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2018/5/2
 * 版本：V1.2
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

typedef enum 
{
    ALIGN_DEFAULT = 0, 
    CW0_DEG = 1,
    CW90_DEG = 2,
    CW180_DEG = 3,
    CW270_DEG = 4,
    CW0_DEG_FLIP = 5,
    CW90_DEG_FLIP = 6,
    CW180_DEG_FLIP = 7,
    CW270_DEG_FLIP = 8
} sensor_align_e;


void initBoardAlignment(void);
void updateBoardAlignment(int16_t roll, int16_t pitch);
void applySensorAlignment(int16_t * dest, int16_t * src, uint8_t rotation);
void applyBoardAlignment(int16_t *vec);
void applyAndSaveBoardAlignmentDelta(int16_t roll, int16_t pitch);

#endif
