#include <math.h>
#include "accelerometer.h"
#include "mpu6000.h"
#include "axis.h"
#include "maths.h"
#include "config_param.h"
#include "filter.h"
#include "sensors.h"
#include "sensorsalignment.h"
#include "beeper.h"
#include "runtime_config.h"

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

//������׵�ͨ�˲���������λHz��
#define ACCEL_LPF_CUTOFF_FREQ 	15.0f

//���������붨��Ļ�������ϵ�İ�װ����
#define ACCEL_ALIGN		CW270_DEG

//mpu6000��ʼ�����ٶ�����ΪFSR_8G��ACC_1G_ADC = 65536 / (2 * 8) = 4096
#define ACC_1G_ADC	4096.0f 

typedef struct accCalibration_s
{
	int32_t accSamples[6][3];
	int calibratedAxisCount;
	bool calibratedAxis[6];
	uint16_t cycleCount;
} accCalibration_t;


Axis3i16 accADCRaw;		//���ٶ�ԭʼAD����
Axis3i16 accADC;		//У׼���AD����			
Axis3f accf;			//ת����λΪG������

static accCalibration_t accCalibration;	//���ٶ�У׼�ṹ�����

biquadFilter_t accFilterLPF[XYZ_AXIS_COUNT];//���׵�ͨ�˲���

void accSetCalibrationCycles(uint16_t calibrationCyclesRequired)
{
    accCalibration.cycleCount = calibrationCyclesRequired;
}

bool accIsCalibrationComplete(void)
{
    return accCalibration.cycleCount == 0;
}

static bool isOnFinalAccelerationCalibrationCycle(void)
{
    return accCalibration.cycleCount == 1;
}

static bool isOnFirstAccelerationCalibrationCycle(void)
{
    return accCalibration.cycleCount == CALIBRATING_ACC_CYCLES;
}

//��ȡ����������������������ݼ��ٶ�ֵ����
static int getPrimaryAxisIndex(Axis3i16 accADCSample)
{
    // Tolerate up to atan(1 / 1.5) = 33 deg tilt (in worst case 66 deg separation between points)
    if ((ABS(accADCSample.z) / 1.5f) > ABS(accADCSample.x) && (ABS(accADCSample.z) / 1.5f) > ABS(accADCSample.y)) 
	{
        //Z-axis
        return (accADCSample.z > 0) ? 0 : 1;
    }
    else if ((ABS(accADCSample.x) / 1.5f) > ABS(accADCSample.y) && (ABS(accADCSample.x) / 1.5f) > ABS(accADCSample.z)) 
	{
        //X-axis
        return (accADCSample.x > 0) ? 2 : 3;
    }
    else if ((ABS(accADCSample.y) / 1.5f) > ABS(accADCSample.x) && (ABS(accADCSample.y) / 1.5f) > ABS(accADCSample.z)) 
	{
        //Y-axis
        return (accADCSample.y > 0) ? 4 : 5;
    }
    else
        return -1;
}

//ִ������У׼
static void performAcclerationCalibration(Axis3i16 accADCSample)
{
    int positionIndex = getPrimaryAxisIndex(accADCSample);

    if (positionIndex < 0) 
	{
        return;
    }

    //�ɿ�ˮƽ���ϣ��տ�ʼУ׼����λ����ֵ�������������
    if (positionIndex == 0 && isOnFirstAccelerationCalibrationCycle()) 
	{
        for (int axis = 0; axis < 6; axis++) 
		{
            accCalibration.calibratedAxis[axis] = false;
            accCalibration.accSamples[axis][X] = 0;
            accCalibration.accSamples[axis][Y] = 0;
            accCalibration.accSamples[axis][Z] = 0;
        }
        accCalibration.calibratedAxisCount = 0;
		DISABLE_STATE(ACCELEROMETER_CALIBRATED);
    }
	
	//��������ļ��ٶ�ֵ
    if (!accCalibration.calibratedAxis[positionIndex]) 
	{
        accCalibration.accSamples[positionIndex][X] += accADCSample.x;
        accCalibration.accSamples[positionIndex][Y] += accADCSample.y;
        accCalibration.accSamples[positionIndex][Z] += accADCSample.z;

        if (isOnFinalAccelerationCalibrationCycle()) 
		{
            accCalibration.calibratedAxis[positionIndex] = true;
			
			accCalibration.accSamples[positionIndex][X] = accCalibration.accSamples[positionIndex][X] / CALIBRATING_ACC_CYCLES;
			accCalibration.accSamples[positionIndex][Y] = accCalibration.accSamples[positionIndex][Y] / CALIBRATING_ACC_CYCLES;
			accCalibration.accSamples[positionIndex][Z] = accCalibration.accSamples[positionIndex][Z] / CALIBRATING_ACC_CYCLES;
			
            accCalibration.calibratedAxisCount++;
            beeperConfirmationBeeps(2);
        }
    }
	
	//�����������ƫ������
    if (accCalibration.calibratedAxisCount == 6) 
	{
		sensorCalibrationState_t calState;
        float accTmp[3];

        //������ƫ
		sensorCalibrationResetState(&calState);
		for (int axis = 0; axis < 6; axis++) 
		{
			sensorCalibrationPushSampleForOffsetCalculation(&calState, accCalibration.accSamples[axis]);
		}
        sensorCalibrationSolveForOffset(&calState, accTmp);
        for (int axis = 0; axis < 3; axis++) 
		{
            configParam.accBias.accZero[axis] = lrintf(accTmp[axis]);
        }
		
        //��������
        sensorCalibrationResetState(&calState);
        for (int axis = 0; axis < 6; axis++) 
		{
			int32_t accSample[3];
			
            accSample[X] = accCalibration.accSamples[axis][X] - configParam.accBias.accZero[X];
            accSample[Y] = accCalibration.accSamples[axis][Y] - configParam.accBias.accZero[Y];
            accSample[Z] = accCalibration.accSamples[axis][Z] - configParam.accBias.accZero[Z];

            sensorCalibrationPushSampleForScaleCalculation(&calState, axis / 2, accSample, ACC_1G_ADC);
        }
        sensorCalibrationSolveForScale(&calState, accTmp);
        for (int axis = 0; axis < 3; axis++) 
		{
            configParam.accBias.accGain[axis] = lrintf(accTmp[axis] * 4096);
        }
		
		ENABLE_STATE(ACCELEROMETER_CALIBRATED);
		//�������
		saveConfigAndNotify();
    }

    accCalibration.cycleCount--;
}

bool accInit(float accUpdateRate)
{
	if(mpu6000Init() == false)
		return false;
	
	//��ʼ��������׵�ͨ�˲�
	for (int axis = 0; axis < 3; axis++)
	{
		biquadFilterInitLPF(&accFilterLPF[axis], accUpdateRate, ACCEL_LPF_CUTOFF_FREQ);
	}

	//��־���ٶȼ��Ƿ���У׼
	if ((configParam.accBias.accZero[X] == 0) && (configParam.accBias.accZero[Y] == 0) && (configParam.accBias.accZero[Z] == 0) &&
		(configParam.accBias.accGain[X] == 4096) && (configParam.accBias.accGain[Y] == 4096) && (configParam.accBias.accGain[Z] == 4096))
	{
		DISABLE_STATE(ACCELEROMETER_CALIBRATED);
	}
	else
	{
		ENABLE_STATE(ACCELEROMETER_CALIBRATED);
	}
	
	return true;
}

void accUpdate(Axis3f *acc)
{
	//��ȡԭʼ����
	if (!mpu6000AccRead(&accADCRaw))
	{
		return;
	}
	
	//���������ݷ������
	applySensorAlignment(accADCRaw.axis, accADCRaw.axis, ACCEL_ALIGN);
	
	//���ٶȼ�����У׼
	if (!accIsCalibrationComplete()) 
	{
		performAcclerationCalibration(accADCRaw);
		return;
	}
	
	//����accADCֵ
	accADC.x = (accADCRaw.x - configParam.accBias.accZero[X]) * configParam.accBias.accGain[X] / 4096;
	accADC.y = (accADCRaw.y - configParam.accBias.accZero[Y]) * configParam.accBias.accGain[X] / 4096;
	accADC.z = (accADCRaw.z - configParam.accBias.accZero[Z]) * configParam.accBias.accGain[X] / 4096;
	
	//�����
	applyBoardAlignment(accADC.axis);
	
	//ת��Ϊ��λ g (9.8m/s^2)
	accf.x = (float)accADC.x / ACC_1G_ADC;
	accf.y = (float)accADC.y / ACC_1G_ADC;
	accf.z = (float)accADC.z / ACC_1G_ADC;
	
	//�����ͨ�˲�
	for (int axis = 0; axis < 3; axis++) 
	{
		accf.axis[axis] = biquadFilterApply(&accFilterLPF[axis], accf.axis[axis]);
	}
	
	*acc = accf;
}


