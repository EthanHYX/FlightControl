#ifndef __ACCELEROMETER_H
#define __ACCELEROMETER_H

#include "sys.h"
#include "stabilizer_types.h"
/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ATKflight�ɿع̼�
 * ���ٶȼ���������	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2018/5/2
 * �汾��V1.2
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/


#define GRAVITY_CMSS    980.665f
#define GRAVITY_MSS     9.80665f

#define CALIBRATING_ACC_CYCLES          400

			
extern Axis3i16 accADC;

bool accIsCalibrationComplete(void);
void accSetCalibrationCycles(uint16_t calibrationCyclesRequired);
bool accInit(float accUpdateRate);
void accUpdate(Axis3f *acc);

#endif

