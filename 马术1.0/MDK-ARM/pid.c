#include "rc_dog.h"

//�ߵ���̬����PID�ṹ��
PIDTypeDef Yaw_PID_Loop;
PIDTypeDef Roll_PID_Loop;
PIDTypeDef Pitch_PID_Loop;

//PIDλ�û�Ŀ��ȷ��
void SetPoint(PIDTypeDef *pid,float want)
{
	pid->Setpoint=want;
}

//PID��������
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


//λ��ʽPID����(����λ�ã��Ƕȣ������������Ϊ�ٶȻ���Ŀ��ֵ)
void PID_PosLocCalc(PIDTypeDef *pid,float Now_Point)//λ��ʽ
{
  float Now_Error,d_Error;
	Now_Error=pid->Setpoint-Now_Point;
	pid->SumError+=Now_Error;//�ⲿ�ֽ������ۼӣ��Ӷ����»��ֱ�������
	//�����޷���������ʽPID����Ҫ�����޷��������޷�������˼·��һ��������sum��������ƣ�����һ��������I*sum���������ƣ�����������Ϊ���Ӻ��������ǵ����ϴ���ļ����ԣ���Ȼ����ǰ�ߡ�
  if(pid->SumError     >  pid->SumError_limit) pid->SumError= pid->SumError_limit;
	else if(pid->SumError< -pid->SumError_limit) pid->SumError=-pid->SumError_limit;
	
	d_Error = Now_Error - pid->Last_error;//���֮���ʾ΢�֡�
    pid->Last_error=Now_Error;
	pid->Out_put =  pid->P*Now_Error + pid->I*pid->SumError + pid->D*d_Error;
	//�޷����
	if(pid->Out_put     > pid->Output_limit) pid->Out_put= pid->Output_limit;
	else if(pid->Out_put<-pid->Output_limit) pid->Out_put=-pid->Output_limit;
}

