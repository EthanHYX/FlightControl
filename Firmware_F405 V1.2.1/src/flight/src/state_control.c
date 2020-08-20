#include "state_control.h"
#include "stabilizer.h"
#include "config_param.h"
#include "rc_controls.h"
#include "commander.h"
#include "rx.h"
#include "maths.h"
#include "pid.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ATKflight飞控固件
 * 四轴姿态控制代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2018/5/2
 * 版本：V1.0
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

/*角速度PID积分限幅（单位：deg/s）*/
#define PID_RATE_ROLL_INTEGRATION_LIMIT		200.0
#define PID_RATE_PITCH_INTEGRATION_LIMIT	200.0
#define PID_RATE_YAW_INTEGRATION_LIMIT		100.0

/*角速度PID输出限幅（单位：油门值）*/
#define PID_RATE_ROLL_OUTPUT_LIMIT			500.0
#define PID_RATE_PITCH_OUTPUT_LIMIT			500.0
#define PID_RATE_YAW_OUTPUT_LIMIT			300.0

/*角度PID输出限幅（单位：deg/s）*/
#define PID_ANGLE_ROLL_OUTPUT_LIMIT    		300.0
#define PID_ANGLE_PITCH_OUTPUT_LIMIT   		300.0
#define PID_ANGLE_YAW_OUTPUT_LIMIT     		150.0

//Z轴速度PID积分限幅（单位cm/s）
#define PID_VZ_INTEGRATION_LIMIT 			500.0
//Z轴速度PID输出限幅（单位油门值）
#define PID_VZ_OUTPUT_LIMIT					800.0

//XY轴速度PID积分限幅（单位cm/s）
#define PID_VXY_INTEGRATION_LIMIT			300.0
//XY轴速度PID输出限幅（单位deg）
#define PID_VXY_OUTPUT_LIMIT				30.0

//XYZ位置PID输出限幅（单位cm/s）
#define PID_POS_OUTPUT_LIMIT				150.0

//角速度PID D项低通截止频率（单位Hz）
#define PID_RATE_LPF_CUTOFF_FREQ			80.0

//Z轴速度PID D项低通截止频率（单位Hz）
#define PID_VZ_LPF_CUTOFF_FREQ				15.0

float altThrustAdj;
float altHoldThrust;
attitude_t attitudeDesired;
attitude_t rateDesired;

PidObject pid[PID_NUM];

static void allPidInit(void)
{
	//pid参数缩放
	pidInit_t pidParam[PID_NUM];
	for (int i = 0; i < PID_NUM; i++)
	{
		pidParam[i].kp = configParam.pid[i].kp / 100.0f;
		pidParam[i].ki = configParam.pid[i].ki / 100.0f;
		pidParam[i].kd = configParam.pid[i].kd / 1000.0f;
	}
	
	//角速度PID（roll\pitch\yaw）
	pidInit(&pid[RATE_ROLL], pidParam[RATE_ROLL].kp, pidParam[RATE_ROLL].ki, pidParam[RATE_ROLL].kd, 
		PID_RATE_ROLL_INTEGRATION_LIMIT, PID_RATE_ROLL_OUTPUT_LIMIT, RATE_PID_DT, true, PID_RATE_LPF_CUTOFF_FREQ);
	pidInit(&pid[RATE_PITCH], pidParam[RATE_PITCH].kp, pidParam[RATE_PITCH].ki, pidParam[RATE_PITCH].kd, 
		PID_RATE_PITCH_INTEGRATION_LIMIT, PID_RATE_PITCH_OUTPUT_LIMIT, RATE_PID_DT, true, PID_RATE_LPF_CUTOFF_FREQ);
	pidInit(&pid[RATE_YAW], pidParam[RATE_YAW].kp, pidParam[RATE_YAW].ki, pidParam[RATE_YAW].kd, 
		PID_RATE_YAW_INTEGRATION_LIMIT, PID_RATE_YAW_OUTPUT_LIMIT, RATE_PID_DT, true, PID_RATE_LPF_CUTOFF_FREQ);
	
	//角度PID（roll\pitch\yaw）
	pidInit(&pid[ANGLE_ROLL], pidParam[ANGLE_ROLL].kp, pidParam[ANGLE_ROLL].ki, pidParam[ANGLE_ROLL].kd, 
		0, PID_ANGLE_ROLL_OUTPUT_LIMIT, ANGLE_PID_DT, false, 0);
	pidInit(&pid[ANGLE_PITCH], pidParam[ANGLE_PITCH].kp, pidParam[ANGLE_PITCH].ki, pidParam[ANGLE_PITCH].kd, 
		0, PID_ANGLE_PITCH_OUTPUT_LIMIT, ANGLE_PID_DT, false, 0);
	pidInit(&pid[ANGLE_YAW], pidParam[ANGLE_YAW].kp, pidParam[ANGLE_YAW].ki, pidParam[ANGLE_YAW].kd, 
		0, PID_ANGLE_YAW_OUTPUT_LIMIT, ANGLE_PID_DT, false, 0);
	
	//Z轴速度PID
	pidInit(&pid[VELOCITY_Z], pidParam[VELOCITY_Z].kp, pidParam[VELOCITY_Z].ki, pidParam[VELOCITY_Z].kd,
		PID_VZ_INTEGRATION_LIMIT, PID_VZ_OUTPUT_LIMIT, VELOCITY_PID_DT, true, PID_VZ_LPF_CUTOFF_FREQ);
	//Z轴位置PID
	pidInit(&pid[POSHOLD_Z], pidParam[POSHOLD_Z].kp, pidParam[POSHOLD_Z].ki, pidParam[POSHOLD_Z].kd, 
		0, PID_POS_OUTPUT_LIMIT, POSITION_PID_DT, false, 0);
		
	//XY轴速度PID
	pidInit(&pid[VELOCITY_XY], pidParam[VELOCITY_XY].kp, pidParam[VELOCITY_XY].ki, pidParam[VELOCITY_XY].kd, 
		PID_VXY_INTEGRATION_LIMIT, PID_VXY_OUTPUT_LIMIT, VELOCITY_PID_DT, false, 0);
	//XY轴位置PID 
	pidInit(&pid[POSHOLD_XY], pidParam[POSHOLD_XY].kp, pidParam[POSHOLD_XY].ki, pidParam[POSHOLD_XY].kd, 
		0, PID_POS_OUTPUT_LIMIT, POSITION_PID_DT, false, 0);
}

void stateControlInit(void)
{
	allPidInit();
}

//姿态控制
void stateControl(const sensorData_t *sensorData, const state_t *state, setpoint_t *setpoint, control_t *control, const u32 tick)
{
	//位置PID
	if (RATE_DO_EXECUTE(POSITION_PID_RATE, tick))
	{
		//Z轴
		if (setpoint->mode.z == modeAbs)
		{
			setpoint->velocity.z = pidUpdate(&pid[POSHOLD_Z], setpoint->position.z - state->position.z);
		}
		//XY轴
	}
	
	//速度PID
	if (RATE_DO_EXECUTE(VELOCITY_PID_RATE, tick))
	{
		//Z轴
		if (setpoint->mode.z != modeDisable)
		{
			altThrustAdj = pidUpdate(&pid[VELOCITY_Z], setpoint->velocity.z - state->velocity.z);
			
			altHoldThrust = altThrustAdj + commanderGetALtHoldThrottle();
		}
		//XY轴
	}
	
	//角度PID（外环）
	if (RATE_DO_EXECUTE(ANGLE_PID_RATE, tick))
	{
		if (setpoint->mode.x == modeDisable || setpoint->mode.y == modeDisable) 
		{
			attitudeDesired.roll = setpoint->attitude.roll;
			attitudeDesired.pitch = setpoint->attitude.pitch;
		}
		rateDesired.roll = pidUpdate(&pid[ANGLE_ROLL], attitudeDesired.roll - state->attitude.roll);
		rateDesired.pitch = pidUpdate(&pid[ANGLE_PITCH], attitudeDesired.pitch - state->attitude.pitch);
		
		if (setpoint->attitudeRate.yaw == 0)//yaw遥杆在中位
		{
			if (attitudeDesired.yaw == 0)//偏航后yaw遥杆并回中时
			{
				attitudeDesired.yaw = state->attitude.yaw;//锁定当前航向角
			}
			float yawError = attitudeDesired.yaw - state->attitude.yaw;
			if (yawError >= +180)
				yawError -= 360;
			if (yawError <= -180)
				yawError += 360;
			rateDesired.yaw = pidUpdate(&pid[ANGLE_YAW], yawError);
		} 
		else//偏航时只进行角速度环控制
		{
			attitudeDesired.yaw = 0;
			rateDesired.yaw = setpoint->attitudeRate.yaw;
		}
	}
	
	//角速度PID（内环）
	if (RATE_DO_EXECUTE(RATE_PID_RATE, tick))
	{
		//速率模式则断开外环
		if (setpoint->mode.roll == modeVelocity || setpoint->mode.pitch == modeVelocity)
		{
			rateDesired.roll = setpoint->attitudeRate.roll;
			rateDesired.pitch = setpoint->attitudeRate.pitch;
			pidReset(&pid[ANGLE_ROLL]);
			pidReset(&pid[ANGLE_PITCH]);
		}
		
		//如果油门值小于MINCHECK时即怠速状态，清除PID积分，防止积分累计导致电机转速不一致
		if(rcCommand[THROTTLE] <= RC_COMMANDER_MINCHECK)
		{
			pidResetIntegral(&pid[RATE_ROLL]);
			pidResetIntegral(&pid[RATE_PITCH]);
			pidResetIntegral(&pid[RATE_YAW]);
			stateControlResetYawHolding();//复位航向角锁定
		}
		
		control->roll = pidUpdate(&pid[RATE_ROLL], rateDesired.roll - sensorData->gyro.x);
		control->pitch = pidUpdate(&pid[RATE_PITCH], rateDesired.pitch - sensorData->gyro.y);
		control->yaw = pidUpdate(&pid[RATE_YAW], rateDesired.yaw - sensorData->gyro.z);
	}
	
	//油门
	if (setpoint->mode.z == modeDisable)
	{
		control->thrust = setpoint->thrust;
	}
	else
	{
		control->thrust = altHoldThrust;
	}
}

//锁定当前航向角
void stateControlResetYawHolding(void)
{
	attitudeDesired.yaw = state.attitude.yaw;
}

//设置Z轴速度PID的积分项
void stateControlSetVelocityZPIDIntegration(float integ)
{
	pidSetIntegral(&pid[VELOCITY_Z], integ / pid[VELOCITY_Z].ki);
}
