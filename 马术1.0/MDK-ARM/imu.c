#include "rc_dog.h"


float yawwant = 0.0f;
float pitchwant = 0.0f;
float rollwant = 0.0f;

void ImuInit(void)
{
	/****IMU��PID��ʼ��****///Pitch��Roll��PID��ʼ����ȫ����̬���ƣ������ֱ�ӿ��Ƶ��ת�ǣ�
	IMU_YAW_PID_SET(2,0,0,3600);
	IMU_ROLL_PID_SET(0.5,0,0,3600);
	IMU_PITCH_PID_SET(0.15,0,0,3600);
	
}

/*
	���ܣ�����IMU�Ƕ����ݵ���̬����
	float roll_set,float pitch_set,float yaw_set��ŷ���ǣ������������ˮƽ����Ŀ��ֵ��
	GaitParams params������state_detached_params[i].detached_params_j����ĳ�ֲ�̬��ĳ���ȵ���̬���ò�����Ϊһ��״̬��׼��jȡ0���ɣ�ӦΪ�����Ȳ�������һ���ġ�
	paramID����������״̬��׼�����ı����²�̬��Ϣ������state_detached_params[paramID]��ֵ��paramIDӦ����һ���е�iһ����
	���⣺�ú�����ֱ�¶��������Ŀǰ�Ŀ��Ƶ�Ƿȱ����������̬�У������ȵĲ������ö���һ���ġ�
*/
void YawControl(float yaw_set,ALLParam *State_Detached_Params,int direction)
{
	float normal_step_left,normal_step_right;

		/*******IMU��PID���*******/
		//PIDĿ���趨��һ�㶼��0����
		SetPoint(&Yaw_PID_Loop,yaw_set);

		PID_PosLocCalc(&Yaw_PID_Loop,EulerAngle[YAW]);//32λ����);
		if(direction != 1)
		{		
			Yaw_PID_Loop.Out_put = -Yaw_PID_Loop.Out_put;
		}
		//��������
		if((Yaw_PID_Loop.Out_put<1.0f) && (Yaw_PID_Loop.Out_put>-1.0f)) 	Yaw_PID_Loop.Out_put=0;

		/**********��̬����*********/
		//Yaw�������������v22
		normal_step_left  = StateDetachedParams_Copy[State_Detached_Params->GaitID].detached_params_0.step_length - Yaw_PID_Loop.Out_put;//���Ȳ�����С
		normal_step_right = StateDetachedParams_Copy[State_Detached_Params->GaitID].detached_params_0.step_length + Yaw_PID_Loop.Out_put ;//���Ȳ�������
		//�����޷�
		normal_step_left  = ((normal_step_left>StepLenthMax - 2) ? StepLenthMax- 2 : normal_step_left);
		normal_step_right = ((normal_step_right>StepLenthMax -2) ? StepLenthMax- 2 : normal_step_right);
		normal_step_left  = ((normal_step_left<StepLenthMin - 2) ? StepLenthMin- 2 : normal_step_left);
		normal_step_right = ((normal_step_right<StepLenthMin- 2) ? StepLenthMin- 2 : normal_step_right);
		//���ո�ֵ
		State_Detached_Params->detached_params_0.step_length = normal_step_right;
		State_Detached_Params->detached_params_1.step_length = normal_step_left;
		State_Detached_Params->detached_params_2.step_length = normal_step_right;
		State_Detached_Params->detached_params_3.step_length = normal_step_left;
		
//		Paint_DrawFloatNum (0, 0, Yaw_PID_Loop.Out_put,3, &Font24, WHITE, BLACK);
}

/*
	���ܣ�����IMU�Ƕ����ݵ���̬����
	float roll_set,float pitch_set,float yaw_set��ŷ���ǣ������������ˮƽ����Ŀ��ֵ��
	GaitParams params������state_detached_params[i].detached_params_j����ĳ�ֲ�̬��ĳ���ȵ���̬���ò�����Ϊһ��״̬��׼��jȡ0���ɣ�ӦΪ�����Ȳ�������һ���ġ�
	paramID����������״̬��׼�����ı����²�̬��Ϣ������state_detached_params[paramID]��ֵ��paramIDӦ����һ���е�iһ����
	���⣺�ú�����ֱ�¶��������Ŀǰ�Ŀ��Ƶ�Ƿȱ����������̬�У������ȵĲ������ö���һ���ġ�
*/
void PitchControl(float pitch_set,ALLParam *State_Detached_Params)
{
		float normal_stance_0,normal_stance_1,normal_stance_2,normal_stance_3;

		/*******IMU��PID���*******/
		//PIDĿ���趨��һ�㶼��0����
		SetPoint(&Pitch_PID_Loop,pitch_set);

		PID_PosLocCalc(&Pitch_PID_Loop,EulerAngle[PITCH]);//32λ����);

	
		if((Pitch_PID_Loop.Out_put<1.0f) && (Pitch_PID_Loop.Out_put>-1.0f)) Pitch_PID_Loop.Out_put=0;

		/**********��̬����*********/
		if(pitch_set>=0)
		{
		//���ȳ��ȿ��ƣ�����һ��˼·��ֻ������������֤�������������������Ա������̫������ӵ��ڸ�����
		normal_stance_0 = StateDetachedParams_Copy[State_Detached_Params->GaitID].detached_params_0.stance_height;//��+�󣭣���Ϊ������ǰ����ҷ���
		normal_stance_1 = StateDetachedParams_Copy[State_Detached_Params->GaitID].detached_params_1.stance_height;
		normal_stance_2 = StateDetachedParams_Copy[State_Detached_Params->GaitID].detached_params_2.stance_height	+ Pitch_PID_Loop.Out_put;
		normal_stance_3 = StateDetachedParams_Copy[State_Detached_Params->GaitID].detached_params_3.stance_height	+ Pitch_PID_Loop.Out_put;
		}
		else
		{
		//���ȳ��ȿ��ƣ�����һ��˼·��ֻ������������֤�������������������Ա������̫������ӵ��ڸ�����
		normal_stance_0 = StateDetachedParams_Copy[State_Detached_Params->GaitID].detached_params_0.stance_height	+ Pitch_PID_Loop.Out_put;//��+�󣭣���Ϊ������ǰ����ҷ���
		normal_stance_1 = StateDetachedParams_Copy[State_Detached_Params->GaitID].detached_params_1.stance_height	+ Pitch_PID_Loop.Out_put;
		normal_stance_2 = StateDetachedParams_Copy[State_Detached_Params->GaitID].detached_params_2.stance_height;
		normal_stance_3 = StateDetachedParams_Copy[State_Detached_Params->GaitID].detached_params_3.stance_height;
		}
		//���ȳ������޿���
		normal_stance_0  = ( (normal_stance_0>=LegLenthMax) ? LegLenthMax : normal_stance_0 );
		normal_stance_1  = ( (normal_stance_1>=LegLenthMax) ? LegLenthMax : normal_stance_1 );
		normal_stance_2  = ( (normal_stance_2>=LegLenthMax) ? LegLenthMax : normal_stance_2 );
		normal_stance_3  = ( (normal_stance_3>=LegLenthMax) ? LegLenthMax : normal_stance_3 );
		//���ȳ������޿���
		normal_stance_0  = ( (normal_stance_0<=LegLenthMin) ? LegLenthMin : normal_stance_0 );
		normal_stance_1  = ( (normal_stance_1<=LegLenthMin) ? LegLenthMin : normal_stance_1 );
		normal_stance_2  = ( (normal_stance_2<=LegLenthMin) ? LegLenthMin : normal_stance_2 );
		normal_stance_3  = ( (normal_stance_3<=LegLenthMin) ? LegLenthMin : normal_stance_3 );
		//Pitch��Roll�����stance_height����
		State_Detached_Params->detached_params_0.stance_height = normal_stance_0;
		State_Detached_Params->detached_params_1.stance_height = normal_stance_1;
		State_Detached_Params->detached_params_2.stance_height = normal_stance_2;
		State_Detached_Params->detached_params_3.stance_height = normal_stance_3;
		
//		Paint_DrawFloatNum (0, 0, Yaw_PID_Loop.Out_put,3, &Font24, WHITE, BLACK);
}


/*
	���ܣ�����IMU�Ƕ����ݵ���̬����
	float roll_set,float pitch_set,float yaw_set��ŷ���ǣ������������ˮƽ����Ŀ��ֵ��
	GaitParams params������state_detached_params[i].detached_params_j����ĳ�ֲ�̬��ĳ���ȵ���̬���ò�����Ϊһ��״̬��׼��jȡ0���ɣ�ӦΪ�����Ȳ�������һ���ġ�
	paramID����������״̬��׼�����ı����²�̬��Ϣ������state_detached_params[paramID]��ֵ��paramIDӦ����һ���е�iһ����
	���⣺�ú�����ֱ�¶��������Ŀǰ�Ŀ��Ƶ�Ƿȱ����������̬�У������ȵĲ������ö���һ���ġ�
*/
void AttitudeControl(float roll_set,float pitch_set,ALLParam *State_Detached_Params)
{
		float normal_stance_0,normal_stance_1,normal_stance_2,normal_stance_3;

		/*******IMU��PID���*******/
		//PIDĿ���趨��һ�㶼��0������Pitch��ʱҪ������һ���Ƕ�;���⻹�п�����Ϊ��΢��Yaw��
		SetPoint(&Pitch_PID_Loop,pitch_set);
		SetPoint(&Roll_PID_Loop,roll_set);
		//PID���㣨���õ��˴��ڽ������IMU_EulerAngle����λ��ʽPID���㣩
		PID_PosLocCalc(&Pitch_PID_Loop,EulerAngle[PITCH]);
		PID_PosLocCalc(&Roll_PID_Loop,EulerAngle[ROLL]);

		//��������
		if((Pitch_PID_Loop.Out_put<1.0f) && (Pitch_PID_Loop.Out_put>-1.0f)) Pitch_PID_Loop.Out_put=0;
		if((Roll_PID_Loop.Out_put<1.0f) && (Roll_PID_Loop.Out_put>-1.0f)) 	Roll_PID_Loop.Out_put=0;
		/**********��̬����*********/

		//�Ⱥ�0123�ֱ��Ӧ��ǰ�������ǰ���Һ󣬼�1��2��Ӧ���ȣ�3��4��Ӧ���ȡ�ע�����öԣ�������������
		//���ȳ��ȿ��ƣ�����һ��˼·��ֻ������������֤�������������������Ա������̫������ӵ��ڸ�����
		normal_stance_0 = StateDetachedParams_Copy[State_Detached_Params->GaitID].detached_params_0.stance_height + Roll_PID_Loop.Out_put	+ Pitch_PID_Loop.Out_put ;//��+�󣭣���Ϊ������ǰ����ҷ���//- Roll_PID_Loop.Out_put
		normal_stance_1 = StateDetachedParams_Copy[State_Detached_Params->GaitID].detached_params_1.stance_height	+ Roll_PID_Loop.Out_put	- Pitch_PID_Loop.Out_put ;//+ Roll_PID_Loop.Out_put
		normal_stance_2 = StateDetachedParams_Copy[State_Detached_Params->GaitID].detached_params_2.stance_height - Roll_PID_Loop.Out_put + Pitch_PID_Loop.Out_put;//- Roll_PID_Loop.Out_put
		normal_stance_3 = StateDetachedParams_Copy[State_Detached_Params->GaitID].detached_params_3.stance_height - Roll_PID_Loop.Out_put - Pitch_PID_Loop.Out_put;//+ Roll_PID_Loop.Out_put
		//���ȳ������޿���
		normal_stance_0  = ( (normal_stance_0>=LegLenthMax) ? LegLenthMax : normal_stance_0 );
		normal_stance_1  = ( (normal_stance_1>=LegLenthMax) ? LegLenthMax : normal_stance_1 );
		normal_stance_2  = ( (normal_stance_2>=LegLenthMax) ? LegLenthMax : normal_stance_2 );
		normal_stance_3  = ( (normal_stance_3>=LegLenthMax) ? LegLenthMax : normal_stance_3 );
		//���ȳ������޿���
		normal_stance_0  = ( (normal_stance_0<=LegLenthMin) ? LegLenthMin : normal_stance_0 );
		normal_stance_1  = ( (normal_stance_1<=LegLenthMin) ? LegLenthMin : normal_stance_1 );
		normal_stance_2  = ( (normal_stance_2<=LegLenthMin) ? LegLenthMin : normal_stance_2 );
		normal_stance_3  = ( (normal_stance_3<=LegLenthMin) ? LegLenthMin : normal_stance_3 );
		//Pitch��Roll�����stance_height����
		State_Detached_Params->detached_params_0.stance_height = normal_stance_0;
		State_Detached_Params->detached_params_1.stance_height = normal_stance_1;
		State_Detached_Params->detached_params_2.stance_height = normal_stance_2;
		State_Detached_Params->detached_params_3.stance_height = normal_stance_3;
}