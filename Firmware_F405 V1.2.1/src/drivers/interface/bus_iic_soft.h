#ifndef __MYIIC_H
#define __MYIIC_H
#include "sys.h" 

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 模拟IIC驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/2
 * 版本：V1.0
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/


void busIIC_Init(void);
bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t data);
bool i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);
bool i2cWriteBuffer(uint8_t addr, uint8_t reg, uint8_t len, const uint8_t * data);


#endif




