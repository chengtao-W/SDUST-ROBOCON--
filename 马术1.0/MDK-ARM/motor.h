#ifndef __MOTOR_H__
#define __MOTOR_H__

#include <stdint.h>
#include "stdio.h"
#include "rc_dog.h"

extern MOTOR_send Motor[8];    //��ȫ�ֱ�������������ƽṹ��͵�����ݽṹ�壬�����ڹ���ʱͨ��debug�鿴����ֵ
extern MOTOR_recv data[8];

void MotorParameterInit(void);
void MotorParameterSet(float angle,uint8_t id,float KP,float KD);
	
#endif

