#ifndef __NVIC_H
#define __NVIC_H
#include "sys.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ATKflight�ɿع̼�
 * �ж�������������	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2018/5/2
 * �汾��V1.0
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

#define NVIC_LOW_PRI  13
#define NVIC_MID_PRI  10
#define NVIC_HIGH_PRI 7


void nvicInit(void);
u32 getSysTickCnt(void);	

#endif /* __NVIC_H */
