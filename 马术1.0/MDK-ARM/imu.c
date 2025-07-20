#include "rc_dog.h"


float yawwant = 0.0f;
float pitchwant = 0.0f;
float rollwant = 0.0f;

void ImuInit(void)
{
	/****IMU的PID初始化****///Pitch和Roll的PID初始化（全局姿态控制，其输出直接控制电机转角）
	IMU_YAW_PID_SET(2,0,0,3600);
	IMU_ROLL_PID_SET(0.5,0,0,3600);
	IMU_PITCH_PID_SET(0.15,0,0,3600);
	
}

/*
	功能：基于IMU角度数据的姿态控制
	float roll_set,float pitch_set,float yaw_set：欧拉角（横滚、俯仰、水平）的目标值。
	GaitParams params：输入state_detached_params[i].detached_params_j，即某种步态的某条腿的姿态。该参数作为一种状态基准。j取0即可，应为四条腿参数都是一样的。
	paramID：用来将对状态基准做出改变后的新步态信息来调整state_detached_params[paramID]的值。paramID应与上一条中的i一样。
	问题：该函数充分暴露出了我们目前的控制的欠缺，即各个步态中，四条腿的参数配置都是一样的。
*/
void YawControl(float yaw_set,ALLParam *State_Detached_Params,int direction)
{
	float normal_step_left,normal_step_right;

		/*******IMUのPID相关*******/
		//PID目标设定（一般都是0，）
		SetPoint(&Yaw_PID_Loop,yaw_set);

		PID_PosLocCalc(&Yaw_PID_Loop,EulerAngle[YAW]);//32位变量);
		if(direction != 1)
		{		
			Yaw_PID_Loop.Out_put = -Yaw_PID_Loop.Out_put;
		}
		//死区设置
		if((Yaw_PID_Loop.Out_put<1.0f) && (Yaw_PID_Loop.Out_put>-1.0f)) 	Yaw_PID_Loop.Out_put=0;

		/**********步态控制*********/
		//Yaw输出给步长参数v22
		normal_step_left  = StateDetachedParams_Copy[State_Detached_Params->GaitID].detached_params_0.step_length - Yaw_PID_Loop.Out_put;//左腿步长减小
		normal_step_right = StateDetachedParams_Copy[State_Detached_Params->GaitID].detached_params_0.step_length + Yaw_PID_Loop.Out_put ;//右腿步长增加
		//步长限幅
		normal_step_left  = ((normal_step_left>StepLenthMax - 2) ? StepLenthMax- 2 : normal_step_left);
		normal_step_right = ((normal_step_right>StepLenthMax -2) ? StepLenthMax- 2 : normal_step_right);
		normal_step_left  = ((normal_step_left<StepLenthMin - 2) ? StepLenthMin- 2 : normal_step_left);
		normal_step_right = ((normal_step_right<StepLenthMin- 2) ? StepLenthMin- 2 : normal_step_right);
		//最终赋值
		State_Detached_Params->detached_params_0.step_length = normal_step_right;
		State_Detached_Params->detached_params_1.step_length = normal_step_left;
		State_Detached_Params->detached_params_2.step_length = normal_step_right;
		State_Detached_Params->detached_params_3.step_length = normal_step_left;
		
//		Paint_DrawFloatNum (0, 0, Yaw_PID_Loop.Out_put,3, &Font24, WHITE, BLACK);
}

/*
	功能：基于IMU角度数据的姿态控制
	float roll_set,float pitch_set,float yaw_set：欧拉角（横滚、俯仰、水平）的目标值。
	GaitParams params：输入state_detached_params[i].detached_params_j，即某种步态的某条腿的姿态。该参数作为一种状态基准。j取0即可，应为四条腿参数都是一样的。
	paramID：用来将对状态基准做出改变后的新步态信息来调整state_detached_params[paramID]的值。paramID应与上一条中的i一样。
	问题：该函数充分暴露出了我们目前的控制的欠缺，即各个步态中，四条腿的参数配置都是一样的。
*/
void PitchControl(float pitch_set,ALLParam *State_Detached_Params)
{
		float normal_stance_0,normal_stance_1,normal_stance_2,normal_stance_3;

		/*******IMUのPID相关*******/
		//PID目标设定（一般都是0，）
		SetPoint(&Pitch_PID_Loop,pitch_set);

		PID_PosLocCalc(&Pitch_PID_Loop,EulerAngle[PITCH]);//32位变量);

	
		if((Pitch_PID_Loop.Out_put<1.0f) && (Pitch_PID_Loop.Out_put>-1.0f)) Pitch_PID_Loop.Out_put=0;

		/**********步态控制*********/
		if(pitch_set>=0)
		{
		//狗腿长度控制（还有一种思路是只增不减，即保证最基本的情况，这样可以避免变数太多而增加调节负担）
		normal_stance_0 = StateDetachedParams_Copy[State_Detached_Params->GaitID].detached_params_0.stance_height;//先+后－，是为了抑制前倾和右翻。
		normal_stance_1 = StateDetachedParams_Copy[State_Detached_Params->GaitID].detached_params_1.stance_height;
		normal_stance_2 = StateDetachedParams_Copy[State_Detached_Params->GaitID].detached_params_2.stance_height	+ Pitch_PID_Loop.Out_put;
		normal_stance_3 = StateDetachedParams_Copy[State_Detached_Params->GaitID].detached_params_3.stance_height	+ Pitch_PID_Loop.Out_put;
		}
		else
		{
		//狗腿长度控制（还有一种思路是只增不减，即保证最基本的情况，这样可以避免变数太多而增加调节负担）
		normal_stance_0 = StateDetachedParams_Copy[State_Detached_Params->GaitID].detached_params_0.stance_height	+ Pitch_PID_Loop.Out_put;//先+后－，是为了抑制前倾和右翻。
		normal_stance_1 = StateDetachedParams_Copy[State_Detached_Params->GaitID].detached_params_1.stance_height	+ Pitch_PID_Loop.Out_put;
		normal_stance_2 = StateDetachedParams_Copy[State_Detached_Params->GaitID].detached_params_2.stance_height;
		normal_stance_3 = StateDetachedParams_Copy[State_Detached_Params->GaitID].detached_params_3.stance_height;
		}
		//狗腿长度上限控制
		normal_stance_0  = ( (normal_stance_0>=LegLenthMax) ? LegLenthMax : normal_stance_0 );
		normal_stance_1  = ( (normal_stance_1>=LegLenthMax) ? LegLenthMax : normal_stance_1 );
		normal_stance_2  = ( (normal_stance_2>=LegLenthMax) ? LegLenthMax : normal_stance_2 );
		normal_stance_3  = ( (normal_stance_3>=LegLenthMax) ? LegLenthMax : normal_stance_3 );
		//狗腿长度下限控制
		normal_stance_0  = ( (normal_stance_0<=LegLenthMin) ? LegLenthMin : normal_stance_0 );
		normal_stance_1  = ( (normal_stance_1<=LegLenthMin) ? LegLenthMin : normal_stance_1 );
		normal_stance_2  = ( (normal_stance_2<=LegLenthMin) ? LegLenthMin : normal_stance_2 );
		normal_stance_3  = ( (normal_stance_3<=LegLenthMin) ? LegLenthMin : normal_stance_3 );
		//Pitch和Roll输出给stance_height参数
		State_Detached_Params->detached_params_0.stance_height = normal_stance_0;
		State_Detached_Params->detached_params_1.stance_height = normal_stance_1;
		State_Detached_Params->detached_params_2.stance_height = normal_stance_2;
		State_Detached_Params->detached_params_3.stance_height = normal_stance_3;
		
//		Paint_DrawFloatNum (0, 0, Yaw_PID_Loop.Out_put,3, &Font24, WHITE, BLACK);
}


/*
	功能：基于IMU角度数据的姿态控制
	float roll_set,float pitch_set,float yaw_set：欧拉角（横滚、俯仰、水平）的目标值。
	GaitParams params：输入state_detached_params[i].detached_params_j，即某种步态的某条腿的姿态。该参数作为一种状态基准。j取0即可，应为四条腿参数都是一样的。
	paramID：用来将对状态基准做出改变后的新步态信息来调整state_detached_params[paramID]的值。paramID应与上一条中的i一样。
	问题：该函数充分暴露出了我们目前的控制的欠缺，即各个步态中，四条腿的参数配置都是一样的。
*/
void AttitudeControl(float roll_set,float pitch_set,ALLParam *State_Detached_Params)
{
		float normal_stance_0,normal_stance_1,normal_stance_2,normal_stance_3;

		/*******IMUのPID相关*******/
		//PID目标设定（一般都是0，除了Pitch有时要求它是一定角度;另外还有可能是为了微调Yaw）
		SetPoint(&Pitch_PID_Loop,pitch_set);
		SetPoint(&Roll_PID_Loop,roll_set);
		//PID计算（利用到了串口解算出的IMU_EulerAngle进行位置式PID计算）
		PID_PosLocCalc(&Pitch_PID_Loop,EulerAngle[PITCH]);
		PID_PosLocCalc(&Roll_PID_Loop,EulerAngle[ROLL]);

		//死区设置
		if((Pitch_PID_Loop.Out_put<1.0f) && (Pitch_PID_Loop.Out_put>-1.0f)) Pitch_PID_Loop.Out_put=0;
		if((Roll_PID_Loop.Out_put<1.0f) && (Roll_PID_Loop.Out_put>-1.0f)) 	Roll_PID_Loop.Out_put=0;
		/**********步态控制*********/

		//腿号0123分别对应左前、左后、右前、右后，即1、2对应左腿，3、4对应右腿。注意配置对，否则正反馈。
		//狗腿长度控制（还有一种思路是只增不减，即保证最基本的情况，这样可以避免变数太多而增加调节负担）
		normal_stance_0 = StateDetachedParams_Copy[State_Detached_Params->GaitID].detached_params_0.stance_height + Roll_PID_Loop.Out_put	+ Pitch_PID_Loop.Out_put ;//先+后－，是为了抑制前倾和右翻。//- Roll_PID_Loop.Out_put
		normal_stance_1 = StateDetachedParams_Copy[State_Detached_Params->GaitID].detached_params_1.stance_height	+ Roll_PID_Loop.Out_put	- Pitch_PID_Loop.Out_put ;//+ Roll_PID_Loop.Out_put
		normal_stance_2 = StateDetachedParams_Copy[State_Detached_Params->GaitID].detached_params_2.stance_height - Roll_PID_Loop.Out_put + Pitch_PID_Loop.Out_put;//- Roll_PID_Loop.Out_put
		normal_stance_3 = StateDetachedParams_Copy[State_Detached_Params->GaitID].detached_params_3.stance_height - Roll_PID_Loop.Out_put - Pitch_PID_Loop.Out_put;//+ Roll_PID_Loop.Out_put
		//狗腿长度上限控制
		normal_stance_0  = ( (normal_stance_0>=LegLenthMax) ? LegLenthMax : normal_stance_0 );
		normal_stance_1  = ( (normal_stance_1>=LegLenthMax) ? LegLenthMax : normal_stance_1 );
		normal_stance_2  = ( (normal_stance_2>=LegLenthMax) ? LegLenthMax : normal_stance_2 );
		normal_stance_3  = ( (normal_stance_3>=LegLenthMax) ? LegLenthMax : normal_stance_3 );
		//狗腿长度下限控制
		normal_stance_0  = ( (normal_stance_0<=LegLenthMin) ? LegLenthMin : normal_stance_0 );
		normal_stance_1  = ( (normal_stance_1<=LegLenthMin) ? LegLenthMin : normal_stance_1 );
		normal_stance_2  = ( (normal_stance_2<=LegLenthMin) ? LegLenthMin : normal_stance_2 );
		normal_stance_3  = ( (normal_stance_3<=LegLenthMin) ? LegLenthMin : normal_stance_3 );
		//Pitch和Roll输出给stance_height参数
		State_Detached_Params->detached_params_0.stance_height = normal_stance_0;
		State_Detached_Params->detached_params_1.stance_height = normal_stance_1;
		State_Detached_Params->detached_params_2.stance_height = normal_stance_2;
		State_Detached_Params->detached_params_3.stance_height = normal_stance_3;
}