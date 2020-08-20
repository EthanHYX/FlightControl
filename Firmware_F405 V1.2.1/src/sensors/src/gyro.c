#include <math.h>
#include "gyro.h"
#include "mpu6000.h"
#include "axis.h"
#include "maths.h"
#include "config_param.h"
#include "filter.h"
#include "sensors.h"
#include "sensorsalignment.h"
#include "beeper.h"

/*������У׼��ֵ*/
#define GYRO_CALIBRATION_THRESHOLD	4.0f

/*��ͨ�˲�����*/
#define GYRO_LPF_CUTOFF_FREQ  	80.0f

/*���������붨��Ļ�������ϵ�İ�װ����*/
#define GYRO_ALIGN		CW270_DEG

//mpu6000��ʼ������������ΪFSR_2000DPS����
#define GYRO_SCALE	16.4f

typedef struct gyroCalibration_s 
{
    Axis3i32 gyroSum;
	Axis3i16 gyroZero;
    stdev_t var[XYZ_AXIS_COUNT];
    uint16_t cycleCount;
} gyroCalibration_t;


//������У׼�ṹ�����
gyroCalibration_t gyroCalibration = 
{
	.cycleCount = CALIBRATING_GYRO_CYCLES,
};	

Axis3i16 gyroADCRaw;	//������ԭʼAD����
Axis3i16 gyroADC;		//У׼��AD����
Axis3f gyrof;			//ת����λΪ��/s������

biquadFilter_t gyroFilterLPF[XYZ_AXIS_COUNT];//���׵�ͨ�˲���

void gyroSetCalibrationCycles(uint16_t calibrationCyclesRequired)
{
    gyroCalibration.cycleCount = calibrationCyclesRequired;
}

bool gyroIsCalibrationComplete(void)
{
    return gyroCalibration.cycleCount == 0;
}

static bool isOnFinalGyroCalibrationCycle(void)
{
    return gyroCalibration.cycleCount == 1;
}

static bool isOnFirstGyroCalibrationCycle(void)
{
    return gyroCalibration.cycleCount == CALIBRATING_GYRO_CYCLES;
}

void performGyroCalibration(Axis3i16 gyroADCSample)
{
    for (int axis = 0; axis < 3; axis++) 
	{
        //��λ����
        if (isOnFirstGyroCalibrationCycle()) 
		{
            gyroCalibration.gyroSum.axis[axis] = 0;
            devClear(&gyroCalibration.var[axis]);
        }

        //���������ۼ�
        gyroCalibration.gyroSum.axis[axis] += gyroADCSample.axis[axis];
        devPush(&gyroCalibration.var[axis], gyroADCSample.axis[axis]);

        //��λ��ƫ 
        gyroCalibration.gyroZero.axis[axis] = 0;

        if (isOnFinalGyroCalibrationCycle()) 
		{
            const float stddev = devStandardDeviation(&gyroCalibration.var[axis]);
			
            //��ⷽ��ֵ�Ƿ�����������ܵ��ƶ�����ֵ
			//��������趨��ֵ�򷵻�����У׼
            if ((stddev > GYRO_CALIBRATION_THRESHOLD) || (stddev == 0)) 
			{
                gyroSetCalibrationCycles(CALIBRATING_GYRO_CYCLES);
                return;
            }
			
            //У׼���
            gyroCalibration.gyroZero.axis[axis] = (gyroCalibration.gyroSum.axis[axis] + (CALIBRATING_GYRO_CYCLES / 2)) / CALIBRATING_GYRO_CYCLES;
        }
    }
	
    gyroCalibration.cycleCount--;
}

bool gyroInit(float gyroUpdateRate)
{
	if(mpu6000Init() == false)
		return false;
	
	//��ʼ����������ƫУ׼
	gyroSetCalibrationCycles(CALIBRATING_GYRO_CYCLES);
	
	//��ʼ�����׵�ͨ�˲�
	for (int axis = 0; axis < 3; axis++)
	{
		biquadFilterInitLPF(&gyroFilterLPF[axis], gyroUpdateRate, GYRO_LPF_CUTOFF_FREQ);
	}
	return true;
}


void gyroUpdate(Axis3f *gyro)
{
	//��ȡԭʼ����
	if (!mpu6000GyroRead(&gyroADCRaw))
	{
		return;
	}
	
	//���������ݷ������
	applySensorAlignment(gyroADCRaw.axis, gyroADCRaw.axis, GYRO_ALIGN);
	
	//������У׼
	if (!gyroIsCalibrationComplete()) 
	{
		performGyroCalibration(gyroADCRaw);
		gyrof.x = 0.0f;
		gyrof.y = 0.0f;
		gyrof.z = 0.0f;
		*gyro = gyrof;
		return;
	}

	//����gyroADCֵ����ȥ��ƫ
	gyroADC.x = gyroADCRaw.x - gyroCalibration.gyroZero.x;
	gyroADC.y = gyroADCRaw.y - gyroCalibration.gyroZero.y;
	gyroADC.z = gyroADCRaw.z - gyroCalibration.gyroZero.z;
	
	//�����
	applyBoardAlignment(gyroADC.axis);
	
	//ת��Ϊ��λ ��/s 
	gyrof.x = (float)gyroADC.x / GYRO_SCALE;
	gyrof.y = (float)gyroADC.y / GYRO_SCALE;
	gyrof.z = (float)gyroADC.z / GYRO_SCALE;
	
	//������׵�ͨ�˲�
	for (int axis = 0; axis < 3; axis++) 
	{
		gyrof.axis[axis] = biquadFilterApply(&gyroFilterLPF[axis], gyrof.axis[axis]);
	}
	
	*gyro = gyrof;
}
