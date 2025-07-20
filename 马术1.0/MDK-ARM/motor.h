#ifndef __MOTOR_H__
#define __MOTOR_H__

#include <stdint.h>
#include "stdio.h"
#include "rc_dog.h"

extern MOTOR_send Motor[8];    //以全局变量声明电机控制结构体和电机数据结构体，方便在故障时通过debug查看变量值
extern MOTOR_recv data[8];

void MotorParameterInit(void);
void MotorParameterSet(float angle,uint8_t id,float KP,float KD);
	
#endif

