#include "atkp.h"
#include "usblink.h"
#include "stabilizer.h"
#include "motors.h"
#include "commander.h"
#include "pm.h"
#include "sensors.h"
#include "config_param.h"
#include "power_control.h"
#include "rx.h"
#include "runtime_config.h"
#include "axis.h"
#include "imu.h"

#include "gyro.h"
#include "accelerometer.h"
#include "compass.h"
#include "barometer.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 无线通信驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/2
 * 版本：V1.0
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
 * 说明：此文件程序基于于匿名科创地面站V4.34通信协议下位机
 *     示例代码修改。
********************************************************************************/

//数据拆分宏定义
#define  BYTE0(dwTemp)       ( *( (u8 *)(&dwTemp)	)  )
#define  BYTE1(dwTemp)       ( *( (u8 *)(&dwTemp) + 1) )
#define  BYTE2(dwTemp)       ( *( (u8 *)(&dwTemp) + 2) )
#define  BYTE3(dwTemp)       ( *( (u8 *)(&dwTemp) + 3) )

//数据返回周期时间（单位ms）
#define  PERIOD_STATUS		30
#define  PERIOD_SENSOR 		10
#define  PERIOD_RCDATA 		40
#define  PERIOD_POWER 		100
#define  PERIOD_MOTOR		40
#define  PERIOD_SENSOR2 	40
#define  PERIOD_SPEED   	50

#define ATKP_RX_QUEUE_SIZE 	10 /*ATKP包接收队列消息个数*/

bool isInit = false;
static xQueueHandle rxQueue;

static void atkpSendPacket(atkp_t *p)
{
	usblinkSendPacket(p);
}

/***************************发送至匿名上位机指令******************************/
static void sendStatus(float roll, float pitch, float yaw, s32 alt, u8 fly_model, u8 armed) //alt单位为cm
{
	u8 _cnt=0;
	atkp_t p;
	vs16 _temp;
	vs32 _temp2 = alt;
	
	p.msgID = UP_STATUS;
	
	_temp = (int)(roll*100);
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = (int)(pitch*100);
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = (int)(yaw*100);
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	
	p.data[_cnt++]=BYTE3(_temp2);
	p.data[_cnt++]=BYTE2(_temp2);
	p.data[_cnt++]=BYTE1(_temp2);
	p.data[_cnt++]=BYTE0(_temp2);
	
	p.data[_cnt++] = fly_model;
	p.data[_cnt++] = armed;
	
	p.dataLen = _cnt;
	atkpSendPacket(&p);
}

static void sendSenser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z)
{
	u8 _cnt=0;
	atkp_t p;
	vs16 _temp;
	
	p.msgID = UP_SENSER;

	_temp = a_x;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = a_y;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = a_z;	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	
	_temp = g_x;	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = g_y;	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = g_z;	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	
	_temp = m_x;	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = m_y;	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = m_z;	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = 0;	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);	
	
	p.dataLen = _cnt;
	atkpSendPacket(&p);
}
static void sendRCData(u16 thrust,u16 yaw,u16 roll,u16 pitch,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6)
{
	u8 _cnt=0;
	atkp_t p;
	
	p.msgID = UP_RCDATA;
	p.data[_cnt++]=BYTE1(thrust);
	p.data[_cnt++]=BYTE0(thrust);
	p.data[_cnt++]=BYTE1(yaw);
	p.data[_cnt++]=BYTE0(yaw);
	p.data[_cnt++]=BYTE1(roll);
	p.data[_cnt++]=BYTE0(roll);
	p.data[_cnt++]=BYTE1(pitch);
	p.data[_cnt++]=BYTE0(pitch);
	p.data[_cnt++]=BYTE1(aux1);
	p.data[_cnt++]=BYTE0(aux1);
	p.data[_cnt++]=BYTE1(aux2);
	p.data[_cnt++]=BYTE0(aux2);
	p.data[_cnt++]=BYTE1(aux3);
	p.data[_cnt++]=BYTE0(aux3);
	p.data[_cnt++]=BYTE1(aux4);
	p.data[_cnt++]=BYTE0(aux4);
	p.data[_cnt++]=BYTE1(aux5);
	p.data[_cnt++]=BYTE0(aux5);
	p.data[_cnt++]=BYTE1(aux6);
	p.data[_cnt++]=BYTE0(aux6);

	p.dataLen = _cnt;
	atkpSendPacket(&p);
}

static void sendPower(u16 votage, u16 current)
{
	u8 _cnt=0;
	atkp_t p;
	
	p.msgID = UP_POWER;
	
	p.data[_cnt++]=BYTE1(votage);
	p.data[_cnt++]=BYTE0(votage);
	p.data[_cnt++]=BYTE1(current);
	p.data[_cnt++]=BYTE0(current);
	
	p.dataLen = _cnt;
	atkpSendPacket(&p);
}

static void sendMotorPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5,u16 m_6,u16 m_7,u16 m_8)
{
	u8 _cnt=0;
	atkp_t p;
	
	p.msgID = UP_MOTOR;
	
	p.data[_cnt++]=BYTE1(m_1);
	p.data[_cnt++]=BYTE0(m_1);
	p.data[_cnt++]=BYTE1(m_2);
	p.data[_cnt++]=BYTE0(m_2);
	p.data[_cnt++]=BYTE1(m_3);
	p.data[_cnt++]=BYTE0(m_3);
	p.data[_cnt++]=BYTE1(m_4);
	p.data[_cnt++]=BYTE0(m_4);
	p.data[_cnt++]=BYTE1(m_5);
	p.data[_cnt++]=BYTE0(m_5);
	p.data[_cnt++]=BYTE1(m_6);
	p.data[_cnt++]=BYTE0(m_6);
	p.data[_cnt++]=BYTE1(m_7);
	p.data[_cnt++]=BYTE0(m_7);
	p.data[_cnt++]=BYTE1(m_8);
	p.data[_cnt++]=BYTE0(m_8);
	
	p.dataLen = _cnt;
	atkpSendPacket(&p);
}

static void sendSenser2(s32 bar_alt,u16 csb_alt)//bar_alt csb_alt单位为cm
{
	u8 _cnt=0;
	atkp_t p;
	
	p.msgID = UP_SENSER2;
	
	p.data[_cnt++]=BYTE3(bar_alt);
	p.data[_cnt++]=BYTE2(bar_alt);
	p.data[_cnt++]=BYTE1(bar_alt);
	p.data[_cnt++]=BYTE0(bar_alt);
	
	p.data[_cnt++]=BYTE1(csb_alt);
	p.data[_cnt++]=BYTE0(csb_alt);
	
	p.dataLen = _cnt;
	atkpSendPacket(&p);
}

static void sendPid(u8 group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d)
{
	u8 _cnt=0;
	atkp_t p;
	vs16 _temp;
	
	p.msgID = 0x10+group-1;

	_temp = p1_p;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = p1_i;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = p1_d;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = p2_p;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = p2_i;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = p2_d;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = p3_p;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = p3_i;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = p3_d;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	
	p.dataLen = _cnt;
	atkpSendPacket(&p);
}

static void sendCheck(u8 head, u8 check_sum)
{
	atkp_t p;
	
	p.msgID = UP_CHECK;
	p.dataLen = 2;
	p.data[0] = head;
	p.data[1] = check_sum;
	atkpSendPacket(&p);
}
/****************************************************************************/

/*数据周期性发送给上位机，每1ms调用一次*/
static void atkpSendPeriod(void)
{
	static u16 count_ms = 1;

	if(!(count_ms % PERIOD_STATUS))
	{
		u8 fm = 0;
		bool armed = ARMING_FLAG(ARMED);
		if (FLIGHT_MODE(ANGLE_MODE))
			fm = 1;
		if (FLIGHT_MODE(NAV_ALTHOLD_MODE))
			fm = 2;
		if (FLIGHT_MODE(NAV_POSHOLD_MODE))
			fm = 3;
		sendStatus(state.attitude.roll, -state.attitude.pitch, -imuAttitudeYaw, state.position.z, fm, armed);				
	}
	if(!(count_ms % PERIOD_SENSOR))
	{
		sendSenser(accADC.x, accADC.y, accADC.z, gyroADC.x, gyroADC.y, gyroADC.z, magADC.x, magADC.y, magADC.z);				
	}
	if(!(count_ms % PERIOD_RCDATA))
	{
		sendRCData(	rcData[THROTTLE], rcData[YAW], rcData[ROLL],
					rcData[PITCH], rcData[AUX1], rcData[AUX2],
					rcData[AUX3], rcData[AUX4], rcData[AUX5], rcData[AUX6]);
	}
	if(!(count_ms % PERIOD_POWER))
	{
		float bat = pmGetBatteryVoltage();
		sendPower(bat * 100,0);
	}
	if(!(count_ms % PERIOD_MOTOR))
	{
		sendMotorPWM(motorPWM.m1, motorPWM.m2, motorPWM.m3, motorPWM.m4, 0,0,0,0);
	}
	if(!(count_ms % PERIOD_SENSOR2))
	{
		sendSenser2(baroAltitude * 100, state.position.z);
	}
	if(++count_ms>=65535) 
		count_ms = 1;	
}

static u8 atkpCheckSum(atkp_t *packet)
{
	u8 sum;
	sum = DOWN_BYTE1;
	sum += DOWN_BYTE2;
	sum += packet->msgID;
	sum += packet->dataLen;
	for(int i=0; i<packet->dataLen; i++)
	{
		sum += packet->data[i];
	}
	return sum;
}

static void atkpReceiveAnl(atkp_t *anlPacket)
{
	if(anlPacket->msgID	== DOWN_COMMAND)
	{
		switch(anlPacket->data[0])
		{
			case D_COMMAND_ACC_CALIB:
				break;
				
			case D_COMMAND_GYRO_CALIB:
				gyroSetCalibrationCycles(CALIBRATING_ACC_CYCLES);
				break;
			
			case D_COMMAND_MAG_CALIB:
				compassSetCalibrationStart();
				break;
			
			case D_COMMAND_BARO_CALIB:
				break;
			
			case D_COMMAND_ACC_CALIB_STEP1:
				accSetCalibrationCycles(CALIBRATING_ACC_CYCLES);
				break;
			case D_COMMAND_ACC_CALIB_STEP2:
				accSetCalibrationCycles(CALIBRATING_ACC_CYCLES);
				break;
			case D_COMMAND_ACC_CALIB_STEP3:
				accSetCalibrationCycles(CALIBRATING_ACC_CYCLES);
				break;
			case D_COMMAND_ACC_CALIB_STEP4:
				accSetCalibrationCycles(CALIBRATING_ACC_CYCLES);
				break;
			case D_COMMAND_ACC_CALIB_STEP5:
				accSetCalibrationCycles(CALIBRATING_ACC_CYCLES);
				break;
			case D_COMMAND_ACC_CALIB_STEP6:
				accSetCalibrationCycles(CALIBRATING_ACC_CYCLES);
				break;
		}
	}			
	else if(anlPacket->msgID == DOWN_ACK)
	{
		if(anlPacket->data[0] == D_ACK_READ_PID)/*读取PID参数*/
		{
			sendPid(1, configParam.pid[RATE_ROLL].kp,   configParam.pid[RATE_ROLL].ki,   configParam.pid[RATE_ROLL].kd,
					   configParam.pid[RATE_PITCH].kp,  configParam.pid[RATE_PITCH].ki,  configParam.pid[RATE_PITCH].kd,
					   configParam.pid[RATE_YAW].kp,    configParam.pid[RATE_YAW].ki,    configParam.pid[RATE_YAW].kd 
				   );
			sendPid(2, configParam.pid[ANGLE_ROLL].kp,  configParam.pid[ANGLE_ROLL].ki,  configParam.pid[ANGLE_ROLL].kd,
					   configParam.pid[ANGLE_PITCH].kp, configParam.pid[ANGLE_PITCH].ki, configParam.pid[ANGLE_PITCH].kd,
					   configParam.pid[ANGLE_YAW].kp,   configParam.pid[ANGLE_YAW].ki,   configParam.pid[ANGLE_YAW].kd 
				   );
			sendPid(3, configParam.pid[VELOCITY_Z].kp,  configParam.pid[VELOCITY_Z].ki,  configParam.pid[VELOCITY_Z].kd,
					   configParam.pid[POSHOLD_Z].kp,   configParam.pid[POSHOLD_Z].ki,   configParam.pid[POSHOLD_Z].kd,
					   configParam.pid[VELOCITY_XY].kp, configParam.pid[VELOCITY_XY].ki, configParam.pid[VELOCITY_XY].kd
				   );
			sendPid(4, configParam.pid[POSHOLD_XY].kp,  configParam.pid[POSHOLD_XY].ki,  configParam.pid[POSHOLD_XY].kd,
					   0, 0, 0,
					   0, 0, 0
				   );
			sendPid(5, configParam.accBias.accZero[X] + 1000,  configParam.accBias.accZero[Y] + 1000,  configParam.accBias.accZero[Z] + 1000,//上位机不能显示负数，统一加1000
					   configParam.accBias.accGain[X],  configParam.accBias.accGain[Y],  configParam.accBias.accGain[Z],
					   configParam.magBias.magZero[X] + 1000,  configParam.magBias.magZero[Y] + 1000,  configParam.magBias.magZero[Z] + 1000
				   );
			
			bool pidBypassFlag = ARMING_FLAG(ARMING_DISABLED_PID_BYPASS); 
			sendPid(6, 0, 0, 0,
					   pidBypassFlag, 0, 0,
					   0, 0, 0
				   );
		}
		
		if(anlPacket->data[0] == D_ACK_RESET_PARAM)/*恢复默认参数*/
		{
			resetConfigParam();
			stateControlInit();
			
			sendPid(1, configParam.pid[RATE_ROLL].kp,   configParam.pid[RATE_ROLL].ki,   configParam.pid[RATE_ROLL].kd,
					   configParam.pid[RATE_PITCH].kp,  configParam.pid[RATE_PITCH].ki,  configParam.pid[RATE_PITCH].kd,
					   configParam.pid[RATE_YAW].kp,    configParam.pid[RATE_YAW].ki,    configParam.pid[RATE_YAW].kd 
				   );
			sendPid(2, configParam.pid[ANGLE_ROLL].kp,  configParam.pid[ANGLE_ROLL].ki,  configParam.pid[ANGLE_ROLL].kd,
					   configParam.pid[ANGLE_PITCH].kp, configParam.pid[ANGLE_PITCH].ki, configParam.pid[ANGLE_PITCH].kd,
					   configParam.pid[ANGLE_YAW].kp,   configParam.pid[ANGLE_YAW].ki,   configParam.pid[ANGLE_YAW].kd 
				   );
			sendPid(3, configParam.pid[VELOCITY_Z].kp,  configParam.pid[VELOCITY_Z].ki,  configParam.pid[VELOCITY_Z].kd,
					   configParam.pid[POSHOLD_Z].kp,   configParam.pid[POSHOLD_Z].ki,   configParam.pid[POSHOLD_Z].kd,
					   configParam.pid[VELOCITY_XY].kp, configParam.pid[VELOCITY_XY].ki, configParam.pid[VELOCITY_XY].kd
				   );
			sendPid(4, configParam.pid[POSHOLD_XY].kp,  configParam.pid[POSHOLD_XY].ki,  configParam.pid[POSHOLD_XY].kd,
					   0, 0, 0,
					   0, 0, 0
				   );
			sendPid(5, configParam.accBias.accZero[X] + 1000,  configParam.accBias.accZero[Y] + 1000,  configParam.accBias.accZero[Z] + 1000,//上位机不能显示负数，统一加1000
					   configParam.accBias.accGain[X],  configParam.accBias.accGain[Y],  configParam.accBias.accGain[Z],
					   configParam.magBias.magZero[X] + 1000,  configParam.magBias.magZero[Y] + 1000,  configParam.magBias.magZero[Z] + 1000
				   );
		}
	}
	else if(anlPacket->msgID == DOWN_PID1)
	{
		configParam.pid[RATE_ROLL].kp = ((s16)(*(anlPacket->data+0)<<8)|*(anlPacket->data+1));
		configParam.pid[RATE_ROLL].ki = ((s16)(*(anlPacket->data+2)<<8)|*(anlPacket->data+3));
		configParam.pid[RATE_ROLL].kd = ((s16)(*(anlPacket->data+4)<<8)|*(anlPacket->data+5));
		
		configParam.pid[RATE_PITCH].kp = ((s16)(*(anlPacket->data+6)<<8)|*(anlPacket->data+7));
		configParam.pid[RATE_PITCH].ki = ((s16)(*(anlPacket->data+8)<<8)|*(anlPacket->data+9));
		configParam.pid[RATE_PITCH].kd = ((s16)(*(anlPacket->data+10)<<8)|*(anlPacket->data+11));
		
		configParam.pid[RATE_YAW].kp = ((s16)(*(anlPacket->data+12)<<8)|*(anlPacket->data+13));
		configParam.pid[RATE_YAW].ki = ((s16)(*(anlPacket->data+14)<<8)|*(anlPacket->data+15));
		configParam.pid[RATE_YAW].kd = ((s16)(*(anlPacket->data+16)<<8)|*(anlPacket->data+17));
		
		u8 cksum = atkpCheckSum(anlPacket);
		sendCheck(anlPacket->msgID,cksum);
	}
	else if(anlPacket->msgID == DOWN_PID2)
	{
		configParam.pid[ANGLE_ROLL].kp = ((s16)(*(anlPacket->data+0)<<8)|*(anlPacket->data+1));
		configParam.pid[ANGLE_ROLL].ki = ((s16)(*(anlPacket->data+2)<<8)|*(anlPacket->data+3));
		configParam.pid[ANGLE_ROLL].kd = ((s16)(*(anlPacket->data+4)<<8)|*(anlPacket->data+5));
		
		configParam.pid[ANGLE_PITCH].kp = ((s16)(*(anlPacket->data+6)<<8)|*(anlPacket->data+7));
		configParam.pid[ANGLE_PITCH].ki = ((s16)(*(anlPacket->data+8)<<8)|*(anlPacket->data+9));
		configParam.pid[ANGLE_PITCH].kd = ((s16)(*(anlPacket->data+10)<<8)|*(anlPacket->data+11));
		
		configParam.pid[ANGLE_YAW].kp = ((s16)(*(anlPacket->data+12)<<8)|*(anlPacket->data+13));
		configParam.pid[ANGLE_YAW].ki = ((s16)(*(anlPacket->data+14)<<8)|*(anlPacket->data+15));
		configParam.pid[ANGLE_YAW].kd = ((s16)(*(anlPacket->data+16)<<8)|*(anlPacket->data+17));
		
		u8 cksum = atkpCheckSum(anlPacket);
		sendCheck(anlPacket->msgID,cksum);
	}		
	else if(anlPacket->msgID == DOWN_PID3)
	{
		configParam.pid[VELOCITY_Z].kp = ((s16)(*(anlPacket->data+0)<<8)|*(anlPacket->data+1));
		configParam.pid[VELOCITY_Z].ki = ((s16)(*(anlPacket->data+2)<<8)|*(anlPacket->data+3));
		configParam.pid[VELOCITY_Z].kd = ((s16)(*(anlPacket->data+4)<<8)|*(anlPacket->data+5));
		
		configParam.pid[POSHOLD_Z].kp = ((s16)(*(anlPacket->data+6)<<8)|*(anlPacket->data+7));
		configParam.pid[POSHOLD_Z].ki = ((s16)(*(anlPacket->data+8)<<8)|*(anlPacket->data+9));
		configParam.pid[POSHOLD_Z].kd = ((s16)(*(anlPacket->data+10)<<8)|*(anlPacket->data+11));
		
		configParam.pid[VELOCITY_XY].kp = ((s16)(*(anlPacket->data+12)<<8)|*(anlPacket->data+13));
		configParam.pid[VELOCITY_XY].ki = ((s16)(*(anlPacket->data+14)<<8)|*(anlPacket->data+15));
		configParam.pid[VELOCITY_XY].kd = ((s16)(*(anlPacket->data+16)<<8)|*(anlPacket->data+17));
			
		u8 cksum = atkpCheckSum(anlPacket);
		sendCheck(anlPacket->msgID,cksum);
	}
	else if(anlPacket->msgID == DOWN_PID4)
	{
		configParam.pid[POSHOLD_XY].kp = ((s16)(*(anlPacket->data+0)<<8)|*(anlPacket->data+1));
		configParam.pid[POSHOLD_XY].ki = ((s16)(*(anlPacket->data+2)<<8)|*(anlPacket->data+3));
		configParam.pid[POSHOLD_XY].kd = ((s16)(*(anlPacket->data+4)<<8)|*(anlPacket->data+5));
			
		u8 cksum = atkpCheckSum(anlPacket);
		sendCheck(anlPacket->msgID,cksum);
	}
	else if(anlPacket->msgID == DOWN_PID5)
	{
		u8 cksum = atkpCheckSum(anlPacket);
		sendCheck(anlPacket->msgID,cksum);
	}
	else if(anlPacket->msgID == DOWN_PID6)//用于电机测试
	{
		bool pidBypassFlag = ((s16)(*(anlPacket->data+6)<<8)|*(anlPacket->data+7));
		
		if (pidBypassFlag && (getBatteryState()==BATTERY_NOT_PRESENT))
			ENABLE_ARMING_FLAG(ARMING_DISABLED_PID_BYPASS);
		else
			DISABLE_ARMING_FLAG(ARMING_DISABLED_PID_BYPASS);
		
		u8 cksum = atkpCheckSum(anlPacket);
		sendCheck(anlPacket->msgID,cksum);
		
		stateControlInit();//初始化PID
		saveConfigAndNotify();//保存PID至Flash
	}
} 

void atkpTxTask(void *param)
{
	while(1)
	{
		atkpSendPeriod();
		vTaskDelay(1);
		
	}
}

void atkpRxAnlTask(void *param)
{
	atkp_t p;
	while(1)
	{
		xQueueReceive(rxQueue, &p, portMAX_DELAY);
		atkpReceiveAnl(&p);
	}
}

void atkpInit(void)
{
	if(isInit) return;
	rxQueue = xQueueCreate(ATKP_RX_QUEUE_SIZE, sizeof(atkp_t));
	ASSERT(rxQueue);
	isInit = true;
}

bool atkpReceivePacketBlocking(atkp_t *p)
{
	ASSERT(p);
	ASSERT(p->dataLen <= ATKP_MAX_DATA_SIZE);
	return xQueueSend(rxQueue, p, portMAX_DELAY);	
}
