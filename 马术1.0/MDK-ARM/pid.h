#ifndef __PID_H__
#define	__PID_H__


//PID结构体对象
typedef struct
{
	float Setpoint;
	
	float SumError;
	float SumError_limit;
	
	float P;
	float D;
	float I;
	
	float Last_error;
	float LLast_error;
	
	double Out_put;
	float Output_limit;
}PIDTypeDef;

void SetPoint(PIDTypeDef *pid,float want);
void IMU_YAW_PID_SET(float kp,float ki,float kd,float SumError_limit);
void IMU_ROLL_PID_SET(float kp,float ki,float kd,float SumError_limit);
void IMU_PITCH_PID_SET(float kp,float ki,float kd,float SumError_limit);
void PID_Set_KP_KI_KD(PIDTypeDef *pid,float kp,float ki,float kd);
void PID_PosLocCalc(PIDTypeDef *pid,float Now_Point);//位置式
#endif
