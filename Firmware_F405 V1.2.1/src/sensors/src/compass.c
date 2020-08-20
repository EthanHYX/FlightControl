#include <math.h>
#include "compass.h"
#include "hmc5883l.h"
#include "sensorsalignment.h"
#include "beeper.h"
#include "sensorsalignment.h"
#include "config_param.h"
#include "nvic.h"
#include "maths.h"
#include "axis.h"
#include "runtime_config.h"
#include "led.h"

/*传感器对齐定义的机体坐标系的安装方向*/
#define MAG_ALIGN			CW270_DEG_FLIP

#define MAG_CALIBRATION_TIME	30000 //3000ms


Axis3i16 magADCRaw;		//磁力计原始AD数据
Axis3i16 magADC;		//校准后并对齐的AD数据

static bool isInit = false;
static uint8_t magUpdatedAtLeastOnce = 0;
static uint32_t calStartTime = 0;
static Axis3i16 magPrev;
static sensorCalibrationState_t calState;

bool compassInit(void)
{
	isInit = hmc5883lInit();
    return isInit;
}

bool compassIsHealthy(void)
{
    return (magADC.x != 0) || (magADC.y != 0) || (magADC.z != 0);
}

bool compassIsReady(void)
{
    return magUpdatedAtLeastOnce;
}

void compassSetCalibrationStart(void)
{
	if (isInit)
	{
		calStartTime = getSysTickCnt();

		magPrev.x = 0;
		magPrev.y = 0;
		magPrev.z = 0;

		beeper(BEEPER_ACTION_SUCCESS);
		sensorCalibrationResetState(&calState);
	}
}

void compassUpdate(Axis3f *mag)
{
	//读取原始数据
	if(!hmc5883lRead(&magADCRaw))
	{
		magADC.x = 0;
		magADC.y = 0;
		magADC.z = 0;
        return;
	}
	
	//传感器数据方向对齐
	applySensorAlignment(magADC.axis, magADCRaw.axis, MAG_ALIGN);
	
    //执行磁力计校准
    if (calStartTime > 0)
	{
        if (getSysTickCnt() - calStartTime < MAG_CALIBRATION_TIME)
		{
			LED0_TOGGLE;
			
            float diffMag = 0.0;
            float avgMag = 0.0;
            for (int axis = 0; axis < 3; axis++) 
			{
                diffMag += (magADC.axis[axis] - magPrev.axis[axis]) * (magADC.axis[axis] - magPrev.axis[axis]);
                avgMag += (magADC.axis[axis] + magPrev.axis[axis]) * (magADC.axis[axis] + magPrev.axis[axis]) / 4.0f;
            }

            // sqrtf(diffMag / avgMag) is a rough approximation of tangent of angle between magADC and magPrev. tan(8 deg) = 0.14
            if ((avgMag > 0.01f) && ((diffMag / avgMag) > (0.14f * 0.14f))) 
			{
				int32_t magADCSample[3];
				magADCSample[0] = magADC.x;
				magADCSample[1] = magADC.y;
				magADCSample[2] = magADC.z;
				
                sensorCalibrationPushSampleForOffsetCalculation(&calState, magADCSample);
				
				magPrev.x = magADC.x;
				magPrev.y = magADC.y;
				magPrev.z = magADC.z;
            }
        } 
		else 
		{
			calStartTime = 0;
            float magZerof[3];
            sensorCalibrationSolveForOffset(&calState, magZerof);

            for (int axis = 0; axis < 3; axis++) 
			{
                configParam.magBias.magZero[axis] = lrintf(magZerof[axis]);
            }
            saveConfigAndNotify();
        }
    }
	
    applyBoardAlignment(magADC.axis);
	
	magADC.x -= configParam.magBias.magZero[X];
	magADC.y -= configParam.magBias.magZero[Y];
	magADC.z -= configParam.magBias.magZero[Z];
	
	mag->x = magADC.x;
	mag->y = magADC.y;
	mag->z = magADC.z;
	
	if ((configParam.magBias.magZero[X] == 0) && (configParam.magBias.magZero[Y] == 0) && (configParam.magBias.magZero[Z] == 0)) 
	{
		DISABLE_STATE(COMPASS_CALIBRATED);
	}
	else 
	{
		ENABLE_STATE(COMPASS_CALIBRATED);
	}
}

