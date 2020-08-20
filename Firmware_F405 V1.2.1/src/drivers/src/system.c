#include "system.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ATKflight飞控固件
 * 系统初始化函数	
 * 包括系统和底层硬件初始化
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2018/5/2
 * 版本：V1.0
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

/*底层硬件初始化*/
void systemInit(void)
{
	nvicInit();			/*中断配置初始化*/
	delay_init(168);	/*delay初始化*/
	ledInit();			/*led初始化*/
	ledStripInit();		/*初始化WS2812RGB灯*/
	beeperInit();		/*蜂鸣器初始化*/
	configParamInit();	/*初始化配置参数*/
	pmInit();			/*电源电压管理初始化*/
	rxInit();			/*遥控器ppm信号接收初始化*/
	usblinkInit();		/*USB通信初始化*/
	atkpInit();			/*传输协议初始化*/
	consoleInit();		/*打印初始化*/
	stabilizerInit();	/*电机 PID 姿态解算初始化*/
}
