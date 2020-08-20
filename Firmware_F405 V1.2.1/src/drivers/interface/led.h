#ifndef __LED_H
#define __LED_H
#include "sys.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ATKflight�ɿع̼�
 * LED��������	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2018/5/2
 * �汾��V1.0
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

#define LED0_ON 		GPIO_ResetBits(GPIOB, GPIO_Pin_9)
#define LED0_OFF 		GPIO_SetBits(GPIOB, GPIO_Pin_9)
#define LED0_TOGGLE 	GPIO_ToggleBits(GPIOB, GPIO_Pin_9)


void ledInit(void);
void warningLedON(void);
void warningLedOFF(void);
void warningLedFlash(void);
void warningLedRefresh(void);
void warningLedUpdate(void);

#endif
