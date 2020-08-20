#include "pos_estimator.h"
#include "maths.h"
#include "axis.h"
#include "runtime_config.h"
#include "accelerometer.h"
#include "imu.h"
#include "nvic.h"
#include "barometer.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ATKflight飞控固件
 * 位置预估驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2018/5/2
 * 版本：V1.2
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

#define  W_Z_BARO_P   0.28f		//气压修正权重
#define  W_ACC_BIAS   0.01f		//加速度计修正权重

typedef struct 
{
    float       alt; //(cm)
    float       epv;
} navPositionEstimatorBARO_t;

typedef struct 
{
    Axis3f 		pos;
    Axis3f 		vel;
    float       eph;
    float       epv;
} posEstimatorEst_t;

typedef struct 
{
    Axis3f     accelNEU;
    Axis3f     accelBias;
    bool       gravityCalibrationComplete;
} navPosisitonEstimatorIMU_t;

typedef struct 
{
    // Data sources
    navPositionEstimatorBARO_t  baro;
    navPosisitonEstimatorIMU_t  imu;

    // Estimate
    posEstimatorEst_t  est;
	
} posEstimator_t;

posEstimator_t posEstimator;


//加速度预估位移和速度
static void posAndVelocityPredict(int axis, float dt, float acc)
{
    posEstimator.est.pos.axis[axis] += posEstimator.est.vel.axis[axis] * dt + acc * dt * dt / 2.0f;
    posEstimator.est.vel.axis[axis] += acc * dt;
}

//误差修正预估的位移和速度
static void posAndVelocityCorrect(int axis, float dt, float e, float w)
{
    float ewdt = e * w * dt;
    posEstimator.est.pos.axis[axis] += ewdt;
    posEstimator.est.vel.axis[axis] += w * ewdt;
}

//误差修正预估的速度
static void inavFilterCorrectVel(int axis, float dt, float e, float w)
{
    posEstimator.est.vel.axis[axis] += e * w * dt;
}

//更新气压计数据
static void updateBaroTopic(float newBaroAlt)
{
	if (baroIsCalibrationComplete()) 
	{
		posEstimator.baro.alt = newBaroAlt;
	}
	else
	{
		posEstimator.baro.alt = 0.0f;
	}
}

//更新NEU坐标系的加速度
//accBF：为机体坐标系加速度
static void updateIMUTopic(const Axis3f accBF)
{
    static float calibratedGravityCMSS = GRAVITY_CMSS;
    static u32 gravityCalibrationTimeout = 0;
	Axis3f accelCMSS;

	//机体坐标系的加速度转为（cm/ss）
	accelCMSS.x = accBF.x * GRAVITY_CMSS;
	accelCMSS.y = accBF.y * GRAVITY_CMSS;
	accelCMSS.z = accBF.z * GRAVITY_CMSS;

	//去除加速度偏置
	accelCMSS.x -= posEstimator.imu.accelBias.x;
	accelCMSS.y -= posEstimator.imu.accelBias.y;
	accelCMSS.z -= posEstimator.imu.accelBias.z;

	//旋转机体坐标系加速度到NEU坐标系
	imuTransformVectorBodyToEarth(&accelCMSS);
	
	//在水平状态校准重力方向的加速度零偏
	if (!posEstimator.imu.gravityCalibrationComplete && STATE(SMALL_ANGLE)) 
	{
		//慢慢收敛校准重力零偏
		const float gravityOffsetError = accelCMSS.z - calibratedGravityCMSS;
		calibratedGravityCMSS += gravityOffsetError * 0.0025f;

		if (ABS(gravityOffsetError) < 5)//误差要小于5cm/ss
		{
			if ((getSysTickCnt() - gravityCalibrationTimeout) > 250) 
			{
				posEstimator.imu.gravityCalibrationComplete = true;
			}
		}
		else 
		{
			gravityCalibrationTimeout = getSysTickCnt();
		}
	}
	
	//NEU坐标系加速度处理
	if (posEstimator.imu.gravityCalibrationComplete) 
	{
		accelCMSS.z -= calibratedGravityCMSS;//去除重力
		for (int axis = 0; axis < 3; axis++)
		{
			applyDeadband(accelCMSS.axis[axis], 4);//去除4(cm/ss)死区
			posEstimator.imu.accelNEU.axis[axis] += (accelCMSS.axis[axis] - posEstimator.imu.accelNEU.axis[axis]) * 0.3f;//一阶低通
		}
	}
	else 
	{
		posEstimator.imu.accelNEU.x = 0;
		posEstimator.imu.accelNEU.y = 0;
		posEstimator.imu.accelNEU.z = 0;
	}
}

//速度预估和位置预估
static void updateEstimatedTopic(float dt)
{
	//使用加速度预估位移和速度
	posAndVelocityPredict(Z, dt, posEstimator.imu.accelNEU.z);
	posAndVelocityPredict(Y, dt, posEstimator.imu.accelNEU.y);
	posAndVelocityPredict(X, dt, posEstimator.imu.accelNEU.x);
	
    //加速度偏置值
    const bool updateAccBias = (W_ACC_BIAS > 0);
    Axis3f accelBiasCorr = { { 0, 0, 0} };

	//使用气压计高度误差修正预估的位移和速度（Z轴）
	const float baroResidual =   posEstimator.baro.alt - posEstimator.est.pos.z;
	posAndVelocityCorrect(Z, dt, baroResidual, W_Z_BARO_P);
	if (updateAccBias) 
	{
		accelBiasCorr.z -= baroResidual * sq(W_Z_BARO_P);
	}
	
	//修正加速度偏置值
    if (updateAccBias) 
	{
        const float accelBiasCorrMagnitudeSq = sq(accelBiasCorr.x) + sq(accelBiasCorr.y) + sq(accelBiasCorr.z);
        if (accelBiasCorrMagnitudeSq < sq(GRAVITY_CMSS * 0.25f))//偏置小于0.25G时可以修正 
		{
			//将加速度偏置值由世界坐标系转换为机体坐标系
            imuTransformVectorEarthToBody(&accelBiasCorr);
			
            posEstimator.imu.accelBias.x += accelBiasCorr.x * W_ACC_BIAS * dt;
            posEstimator.imu.accelBias.y += accelBiasCorr.y * W_ACC_BIAS * dt;
            posEstimator.imu.accelBias.z += accelBiasCorr.z * W_ACC_BIAS * dt;
        }
    }
}

//预估器初始化
static void initializePositionEstimator(void)
{
    posEstimator.imu.gravityCalibrationComplete = false;
	
    for (int axis = 0; axis < 3; axis++) 
	{
        posEstimator.imu.accelBias.axis[axis] = 0;
        posEstimator.est.pos.axis[axis] = 0;
        posEstimator.est.vel.axis[axis] = 0;
    }
}

//发布预估位置和速度
static void publishEstimatedTopic(state_t *state)
{
	static u32 publishTime;
	
	//更新世界坐标系的加速度
	state->acc.x = posEstimator.imu.accelNEU.x;
	state->acc.y = posEstimator.imu.accelNEU.y;
	state->acc.z = posEstimator.imu.accelNEU.z;
	
	//更新估计的位置和速度（10ms->100Hz）
	if ((getSysTickCnt() - publishTime) >= 10)
	{
		state->position.x = posEstimator.est.pos.x;
		state->position.y = posEstimator.est.pos.y;
		state->position.z = posEstimator.est.pos.z;
		
		state->velocity.x = posEstimator.est.vel.x;
		state->velocity.y = posEstimator.est.vel.y;
		state->velocity.z = constrainf(posEstimator.est.vel.z, -150.0f, 150.0f);//限制Z轴的速度为150cm/s
		
		publishTime = getSysTickCnt();
	}
}

//更新预估器，由主循环调用
void updatePositionEstimator(const sensorData_t *sensorData, state_t *state, float dt)
{
    static bool isInitialized = false;
	
	//初始化预估器
	if (!isInitialized) 
	{
		initializePositionEstimator();
		isInitialized = true;
	}
	
	//更新预估器的气压高度
	updateBaroTopic(sensorData->baro.asl);
	
    //更新预估器的加速度（世界坐标系）
    updateIMUTopic(sensorData->acc);
	
    //预估速度和位置
    updateEstimatedTopic(dt);

    //发布预估的速度和位置
    publishEstimatedTopic(state);
}

bool posEstimatorIsCalibrationComplete(void)
{
    return posEstimator.imu.gravityCalibrationComplete;
}

void posEstimatorReset(void)
{
	for (int axis = 0; axis < 3; axis++) 
	{
		posEstimator.imu.accelBias.axis[axis] = 0;
		posEstimator.est.pos.axis[axis] = 0;
		posEstimator.est.vel.axis[axis] = 0;
	}
	posEstimator.est.pos.z = posEstimator.baro.alt;
}

