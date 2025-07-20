#include "rc_dog.h"

//惯导姿态控制PID结构体
PIDTypeDef Yaw_PID_Loop;
PIDTypeDef Roll_PID_Loop;
PIDTypeDef Pitch_PID_Loop;

//PID位置环目标确定
void SetPoint(PIDTypeDef *pid,float want)
{
	pid->Setpoint=want;
}

//PID基础配置
void PID_Set_KP_KI_KD(PIDTypeDef *pid,float kp,float ki,float kd)
{
	pid->P=kp;pid->I=ki;pid->D=kd;
}

void IMU_YAW_PID_SET(float kp,float ki,float kd,float SumError_limit)
{
	PID_Set_KP_KI_KD(&Yaw_PID_Loop,kp,ki,kd);
	Yaw_PID_Loop.Output_limit = 180;
	Yaw_PID_Loop.SumError_limit = SumError_limit;
	
//	memcpy(&Roll_PID_Loop,&Yaw_PID_Loop,sizeof(PIDTypeDef));	
//	memcpy(&Pitch_PID_Loop,&Yaw_PID_Loop,sizeof(PIDTypeDef));	
}

void IMU_ROLL_PID_SET(float kp,float ki,float kd,float SumError_limit)
{
	
	PID_Set_KP_KI_KD(&Roll_PID_Loop,kp,ki,kd);
	Roll_PID_Loop.Output_limit = 180;
	Roll_PID_Loop.SumError_limit = SumError_limit;
//	memcpy(&Roll_PID_Loop,&Yaw_PID_Loop,sizeof(PIDTypeDef));	
//	memcpy(&Pitch_PID_Loop,&Yaw_PID_Loop,sizeof(PIDTypeDef));	
}

void IMU_PITCH_PID_SET(float kp,float ki,float kd,float SumError_limit)
{
	
	PID_Set_KP_KI_KD(&Pitch_PID_Loop,kp,ki,kd);
	Pitch_PID_Loop.Output_limit = 180;
	Pitch_PID_Loop.SumError_limit = SumError_limit;

//	memcpy(&Roll_PID_Loop,&Yaw_PID_Loop,sizeof(PIDTypeDef));	
//	memcpy(&Pitch_PID_Loop,&Yaw_PID_Loop,sizeof(PIDTypeDef));	
}


//位置式PID计算(用于位置（角度）环，其输出作为速度环的目标值)
void PID_PosLocCalc(PIDTypeDef *pid,float Now_Point)//位置式
{
  float Now_Error,d_Error;
	Now_Error=pid->Setpoint-Now_Point;
	pid->SumError+=Now_Error;//这部分进行了累加，从而导致积分饱和现象。
	//积分限幅（而增量式PID不需要积分限幅）积分限幅有两种思路，一种是限制sum（相对限制），另一种是限制I*sum（绝对限制），后者我认为更加合理，但考虑到新老代码的兼容性，仍然采用前者。
  if(pid->SumError     >  pid->SumError_limit) pid->SumError= pid->SumError_limit;
	else if(pid->SumError< -pid->SumError_limit) pid->SumError=-pid->SumError_limit;
	
	d_Error = Now_Error - pid->Last_error;//误差之差，表示微分。
    pid->Last_error=Now_Error;
	pid->Out_put =  pid->P*Now_Error + pid->I*pid->SumError + pid->D*d_Error;
	//限幅输出
	if(pid->Out_put     > pid->Output_limit) pid->Out_put= pid->Output_limit;
	else if(pid->Out_put<-pid->Output_limit) pid->Out_put=-pid->Output_limit;
}

