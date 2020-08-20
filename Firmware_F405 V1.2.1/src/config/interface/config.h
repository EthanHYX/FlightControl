#ifndef __CONFIG_H
#define __CONFIG_H
#include "nvic.h"
#include "stdio.h"	/*printf 调用*/

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ATKflight飞控固件
 * 配置文件代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2018/5/2
 * 版本：V1.0
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

/*遥控器配置*/
#define RC_MIN			1000
#define RC_MID			1500
#define RC_MAX			2000

#define RC_COMMANDER_MINCHECK	1100 //遥杆命令最小检查值
#define RC_COMMANDER_MAXCHECK	1900 //遥杆命令最大检查值

#define MINTHROTTLE		1100 //解锁后怠速油门值
#define MAXTHROTTLE		1850 //控制时最大油门值	


/*电调协议配置（只能选一）*/
#define USE_ESC_PROTOCOL_STANDARD  //标准PWM协议
//#define USE_ESC_PROTOCOL_ONESHOT125	//oneshot125协议	 	




#endif /* __CONFIG_H */
