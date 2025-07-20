#ifndef __RC_DOG_H__
#define __RC_DOG_H__

/*����------------------------------------*/
#include "usart.h"
#include "math.h"
//#include "rs485.h"
//#include "dread_me_control.h"
/*TASK-----------------------------------------*/
#include "image.h"
#include "LCD_Test.h"
#include "control.h"
#include "motor_control.h"
#include "jump.h"
#include "motor.h"
#include "gait.h"
#include "GUI.h"
#include "imu.h"
#include "pid.h"

#define YAW 0
#define ROLL 1
#define PITCH 2

//�ȳ���λ
#define LegLenthExtremeMax 35.0f //�����ˣ����ٳ��ˡ�
#define LegLenthMax 30.0f //ʵ��������Ȼ���һ����ԣ�ȡ�
#define LegLenthMin 13.0f //����΢��ѹ�Źؽڡ�
#define LegStandLenth 18.0f //��׼վ��״̬���ȳ���
#define LegSquatLenth 14.0f //�����޼�ѹ�Źؽڡ�

//������λ
#define StepLenthMin 0.0f
#define StepLenthMax  (2*LegLenthExtremeMax*0.866f*0.9f) //��С�����45cm
#define StepLenthMax_Half (LegLenthExtremeMax*0.866f*0.9f)

//״̬����������
#define StatesMaxNum 20

extern float theta1,theta2,theta3,theta4;
extern float KP,KD;
extern float x,y;//���ߵĽ��
extern PIDTypeDef Yaw_PID_Loop;
extern PIDTypeDef Roll_PID_Loop;
extern PIDTypeDef Pitch_PID_Loop;
extern ALLParam StateDetachedParams_Copy[StatesMaxNum];
extern ALLParam ALLParams[StatesMaxNum];
extern float EulerAngle[3];//32λ����
extern volatile float voltage_vrefint_proportion;
extern float x,y;//���ߵĽ��
#endif

