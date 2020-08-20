#ifndef __BAROMETER_H
#define __BAROMETER_H

#include "sys.h"
#include "stabilizer_types.h"
/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ATKflight�ɿع̼�
 * ��ѹ����������	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2018/5/2
 * �汾��V1.2
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

extern float baroAltitude;

bool baroInit(void);
bool baroIsCalibrationComplete(void);
void baroStartCalibration(void);
void baroUpdate(baro_t *baro);

#endif

