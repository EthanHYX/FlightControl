#include <math.h>
#include "commander.h"
#include "rx.h"
#include "rc_controls.h"
#include "runtime_config.h"
#include "config.h"
#include "pos_estimator.h"
#include "state_control.h"
#include "beeper.h"
#include "maths.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ATKflight飞控固件
 * 获取遥控数据驱动代码
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2018/5/2
 * 版本：V1.0
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

/*commonder配置参数*/
#define DEAD_BAND			5			//设置roll\pitch中点死区值
#define DEAD_BAND_YAW	   	10			//yaw中点死区值

#define MAX_ANGLE_ROLL		(30.0f)		//自稳最大角度
#define MAX_ANGLE_PITCH  	(30.0f)

#define MAX_RATE_ROLL		(200.0f)	//手动模式最大速率
#define MAX_RATE_PITCH		(200.0f)
#define MAX_RATE_YAW     	(200.0f)

#define ALT_HOLD_DEADBAND	50			//定高油门遥杆死区值
#define POS_HOLD_DEADBAND	30			//定点方向遥杆死区值

#define CLIMB_RATE_UP		150.0f		//定高最大上升速率 cm/s
#define CLIMB_RATE_DOWN		110.0f		//定高最大下降速率 cm/s

#define LAND_THROTTLE		1200		//失控降落油门值

uint16_t altHoldThrottleBase = 1500;	//定高油门基础值
bool setupAltitudeHoldFlag = false;		//标志定高设定完成
int16_t rcCommand[4];//[1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW

autoLandState_t autoLandState = //自动降落
{
	.autoLandActive = 0,
	.autoLandTime = 0,
};	


//遥控数据转换为速率
static float rcCommandToRate(int16_t stick, float rate)
{
    return scaleRangef((float) stick, -500, 500, -rate, rate);
}

//遥控数据转换为角度
static float rcCommandToAngle(int16_t stick, float maxInclination)
{
    stick = constrain(stick, -500, 500);
    return scaleRangef((float) stick, -500, 500, -maxInclination, maxInclination);
}

//遥控数据限制量程和去除死区
static int16_t getAxisRcCommand(int16_t rawData, int16_t deadband)
{
    int16_t stickDeflection;

    stickDeflection = constrain(rawData - RC_MID, -500, 500);
    stickDeflection = applyDeadband(stickDeflection, deadband);
    return stickDeflection;
}

//复位控制模式
static void resetSetpointMode(setpoint_t *setpoint)
{
	setpoint->mode.roll = modeDisable;
	setpoint->mode.pitch = modeDisable;
	setpoint->mode.yaw = modeDisable;
	
	setpoint->mode.x = modeDisable;
	setpoint->mode.y = modeDisable;
	setpoint->mode.z = modeDisable;
}

//配置定高模式
static void setupAltitudeHold(const state_t *state, setpoint_t *setpoint)
{
	throttleStatus_e throttleStatus = calculateThrottleStatus();//获取当前油门遥杆位置

	if (throttleStatus == THROTTLE_LOW) //如果油门低位时切换的定高，基础油门为1500
	{
		altHoldThrottleBase = 1500;
		stateControlSetVelocityZPIDIntegration(-400);//设置Z轴速度控制器积分为-400防止油门跳跃
	}
	else
	{
		altHoldThrottleBase = rcCommand[THROTTLE];
		stateControlSetVelocityZPIDIntegration(0);//清除Z轴速度控制器积分
	}

	posEstimatorReset();//复位位置预估器的预估位置和速度，防止撞击导致速度和位置异常大

	setpoint->position.z = state->position.z;//设定目标高度为当前估测的高度
}

//设定控制命令和飞行模式
void commanderGetSetpoint(const state_t *state, setpoint_t *setpoint)
{
	if (autoLandState.autoLandActive != true)//正常飞行
	{
		rcCommand[ROLL] = getAxisRcCommand(rcData[ROLL], DEAD_BAND);
		rcCommand[PITCH] = getAxisRcCommand(rcData[PITCH], DEAD_BAND);
		rcCommand[YAW] = -getAxisRcCommand(rcData[YAW], DEAD_BAND_YAW);//传感器Z轴右手定则逆时针为正，所以为负号
		rcCommand[THROTTLE] = constrain(rcData[THROTTLE], RC_MIN, RC_MAX);
	}
	else//失控保护自动降落
	{
		ENABLE_FLIGHT_MODE(ANGLE_MODE);//激活自稳模式
		rcCommand[ROLL] = 0;
		rcCommand[PITCH] = 0;
		rcCommand[YAW] = 0;
		if ((state->acc.z > 800.f) || (getSysTickCnt() - autoLandState.autoLandTime > 20000))//Z轴加速度大于800cm/ss说明掉落到地上了或超过20秒则关闭电机
		{
			rcCommand[THROTTLE] = 0;
			mwDisarm();
		}
		else
		{
			rcCommand[THROTTLE] = LAND_THROTTLE;//降落油门
		}
		beeper(BEEPER_RX_LOST_LANDING);//蜂鸣器报警
	}
	
	//如果油门值小于或等于MINCHECK时，yaw控制不起作用
	//目的是在加锁和解锁动作时防止四轴自转
	if(rcCommand[THROTTLE] <= RC_COMMANDER_MINCHECK)
	{
		rcCommand[YAW] = 0;
	}
	
	if(FLIGHT_MODE(HEADFREE_MODE))//无头模式
	{
		float yawRad = degreesToRadians(-state->attitude.yaw);
		float cosy = cosf(yawRad);
		float siny = sinf(yawRad);
		int16_t rcCommand_PITCH = rcCommand[PITCH]*cosy + rcCommand[ROLL]*siny;
		rcCommand[ROLL] = rcCommand[ROLL]*cosy - rcCommand[PITCH]*siny;
		rcCommand[PITCH] = rcCommand_PITCH;
	}

	resetSetpointMode(setpoint);//复位所有模式
	
	//手动模式和自稳模式
	if (FLIGHT_MODE(ACRO_MODE))
	{
		setpoint->mode.roll = modeVelocity;
		setpoint->mode.pitch = modeVelocity;
		setpoint->mode.yaw = modeVelocity;
		
		setpoint->attitudeRate.roll = rcCommandToRate(rcCommand[ROLL], MAX_RATE_ROLL);
		setpoint->attitudeRate.pitch = rcCommandToRate(rcCommand[PITCH], MAX_RATE_PITCH);
		setpoint->attitudeRate.yaw = rcCommandToRate(rcCommand[YAW], MAX_RATE_YAW);
	}
	else if (FLIGHT_MODE(ANGLE_MODE))
	{
		setpoint->mode.yaw = modeVelocity;
		
		setpoint->attitude.roll = rcCommandToAngle(rcCommand[ROLL], MAX_ANGLE_ROLL);
		setpoint->attitude.pitch = rcCommandToAngle(rcCommand[PITCH], MAX_ANGLE_PITCH);
		setpoint->attitudeRate.yaw = rcCommandToRate(rcCommand[YAW], MAX_RATE_YAW);
	}

	//定高模式
	if (FLIGHT_MODE(NAV_ALTHOLD_MODE))
	{
		static bool isAdjustAlttitude = false;
		
		//初始化定高模式
		if (setupAltitudeHoldFlag == false)
		{
			setupAltitudeHold(state, setpoint);
			setupAltitudeHoldFlag = true;
		}
		
		//调整设定高度
		int16_t rcThrottleAdjustment = applyDeadband(rcCommand[THROTTLE] - altHoldThrottleBase, ALT_HOLD_DEADBAND);
		if (rcThrottleAdjustment == 0 && isAdjustAlttitude == true)
		{
			setpoint->mode.z = modeAbs;
			setpoint->position.z = state->position.z;
			isAdjustAlttitude = false;
		}
		else if (rcThrottleAdjustment > 0)
		{
			setpoint->mode.z = modeVelocity;
			setpoint->velocity.z = rcThrottleAdjustment * CLIMB_RATE_UP / (RC_MAX - altHoldThrottleBase - ALT_HOLD_DEADBAND);
			isAdjustAlttitude = true;
		}
		else
		{
			setpoint->mode.z = modeVelocity;
			setpoint->velocity.z = rcThrottleAdjustment * CLIMB_RATE_DOWN / (altHoldThrottleBase - RC_MIN - ALT_HOLD_DEADBAND);
			isAdjustAlttitude = true;
		}
	}
	
	setpoint->thrust = rcCommand[THROTTLE];
}

//设置定高模式
void commanderSetupAltitudeHoldMode(void)
{
	setupAltitudeHoldFlag = false;
}

//获取定高基础油门值
uint16_t commanderGetALtHoldThrottle(void)
{
	return altHoldThrottleBase;
}

//激活失控保护模式
void commanderActiveFailsafe(void)
{
	autoLandState.autoLandActive = true;
	autoLandState.autoLandTime = getSysTickCnt();
}
