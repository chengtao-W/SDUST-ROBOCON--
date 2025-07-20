#ifndef __RC_DOG_H__
#define __RC_DOG_H__

/*外设------------------------------------*/
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

//腿长限位
#define LegLenthExtremeMax 35.0f //真滴最长了，别再长了。
#define LegLenthMax 30.0f //实际上离最长腿还有一定的裕度。
#define LegLenthMin 13.0f //有略微挤压髋关节。
#define LegStandLenth 18.0f //标准站立状态的腿长。
#define LegSquatLenth 14.0f //正常无挤压髋关节。

//步长限位
#define StepLenthMin 0.0f
#define StepLenthMax  (2*LegLenthExtremeMax*0.866f*0.9f) //大小大概在45cm
#define StepLenthMax_Half (LegLenthExtremeMax*0.866f*0.9f)

//状态数上限配置
#define StatesMaxNum 20

extern float theta1,theta2,theta3,theta4;
extern float KP,KD;
extern float x,y;//摆线的结果
extern PIDTypeDef Yaw_PID_Loop;
extern PIDTypeDef Roll_PID_Loop;
extern PIDTypeDef Pitch_PID_Loop;
extern ALLParam StateDetachedParams_Copy[StatesMaxNum];
extern ALLParam ALLParams[StatesMaxNum];
extern float EulerAngle[3];//32位变量
extern volatile float voltage_vrefint_proportion;
extern float x,y;//摆线的结果
#endif

