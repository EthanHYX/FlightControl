#include <math.h>
#include "sensorsalignment.h"
#include "maths.h"
#include "axis.h"
#include "config_param.h"

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


static bool standardBoardAlignment = true;     
static float boardRotation[3][3];//旋转矩阵（微调用）


static bool isBoardAlignmentStandard(const boardAlignment_t *boardAlignment)
{
    return !boardAlignment->rollDeciDegrees && !boardAlignment->pitchDeciDegrees && !boardAlignment->yawDeciDegrees;
}

void initBoardAlignment(void)
{
	if (isBoardAlignmentStandard(&configParam.boardAlign)) 
	{
		standardBoardAlignment = true;
	} 
	else 
	{
		fp_angles_t rotationAngles;

		standardBoardAlignment = false;

		rotationAngles.angles.roll  = DECIDEGREES_TO_RADIANS(configParam.boardAlign.rollDeciDegrees );
		rotationAngles.angles.pitch = DECIDEGREES_TO_RADIANS(configParam.boardAlign.pitchDeciDegrees);
		rotationAngles.angles.yaw   = DECIDEGREES_TO_RADIANS(configParam.boardAlign.yawDeciDegrees  );

		buildRotationMatrix(&rotationAngles, boardRotation);
	}
}

void updateBoardAlignment(int16_t roll, int16_t pitch)
{
    const float sinAlignYaw = sin_approx(DECIDEGREES_TO_RADIANS(configParam.boardAlign.yawDeciDegrees));
    const float cosAlignYaw = cos_approx(DECIDEGREES_TO_RADIANS(configParam.boardAlign.yawDeciDegrees));
	
    configParam.boardAlign.rollDeciDegrees += -sinAlignYaw * pitch + cosAlignYaw * roll;
	configParam.boardAlign.rollDeciDegrees = constrain(configParam.boardAlign.rollDeciDegrees, -30, 30);//限制微调范围-3~3度
	
    configParam.boardAlign.pitchDeciDegrees += cosAlignYaw * pitch + sinAlignYaw * roll;
	configParam.boardAlign.pitchDeciDegrees = constrain(configParam.boardAlign.pitchDeciDegrees, -30, 30);//限制微调范围-3~3度
	
    initBoardAlignment();
}

void applyBoardAlignment(int16_t *vec)
{
	if (standardBoardAlignment) 
	{
		return;
	}

	int16_t x = vec[X];
	int16_t y = vec[Y];
	int16_t z = vec[Z];

	vec[X] = lrintf(boardRotation[0][X] * x + boardRotation[1][X] * y + boardRotation[2][X] * z);
	vec[Y] = lrintf(boardRotation[0][Y] * x + boardRotation[1][Y] * y + boardRotation[2][Y] * z);
	vec[Z] = lrintf(boardRotation[0][Z] * x + boardRotation[1][Z] * y + boardRotation[2][Z] * z);
}


void applySensorAlignment(int16_t * dest, int16_t * src, uint8_t rotation)
{
    const int16_t x = src[X];
    const int16_t y = src[Y];
    const int16_t z = src[Z];

    switch (rotation) {
    default:
    case CW0_DEG:
        dest[X] = x;
        dest[Y] = y;
        dest[Z] = z;
        break;
    case CW90_DEG:
        dest[X] = y;
        dest[Y] = -x;
        dest[Z] = z;
        break;
    case CW180_DEG:
        dest[X] = -x;
        dest[Y] = -y;
        dest[Z] = z;
        break;
    case CW270_DEG:
        dest[X] = -y;
        dest[Y] = x;
        dest[Z] = z;
        break;
    case CW0_DEG_FLIP:
        dest[X] = -x;
        dest[Y] = y;
        dest[Z] = -z;
        break;
    case CW90_DEG_FLIP:
        dest[X] = y;
        dest[Y] = x;
        dest[Z] = -z;
        break;
    case CW180_DEG_FLIP:
        dest[X] = x;
        dest[Y] = -y;
        dest[Z] = -z;
        break;
    case CW270_DEG_FLIP:
        dest[X] = -y;
        dest[Y] = -x;
        dest[Z] = -z;
        break;
    }
}

void applyAndSaveBoardAlignmentDelta(int16_t roll, int16_t pitch)
{
    updateBoardAlignment(roll, pitch);
    saveConfigAndNotify();
}
