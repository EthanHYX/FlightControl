#ifndef __SENSORS_ALIGNMENT_H
#define __SENSORS_ALIGNMENT_H
#include "sys.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ATKflight�ɿع̼�
 * ���������������������	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2018/5/2
 * �汾��V1.2
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
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
