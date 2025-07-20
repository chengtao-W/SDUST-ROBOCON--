#include "jump.h"

float start_time_ = 0.0f;
extern float time;
int jumpflag=0;
extern float KP,KD;
void Start_Jump(float start_time_s) 
{
	start_time_ = start_time_s;
}

///*******************************************************************************************
//	*@ �������ƣ�void ExecuteJump(void) 
//	*@ ���ܣ���ǰ��Ծ
//	*@ ��ע��ѡ��״̬
// *******************************************************************************************/	
void ExecuteJump(void) 
{
//     min radius = 0.8;
//     max radius = 0.25;
		float t; //������Ծ��ʼʱ�� 
    const float prep_time = 0.8; //��Ծǰ����ʱ��[s]
    const float launch_time = 0.25 ; //����ǰ�ĳ���ʱ��[s]
    const float fall_time = 1.75; //���Ȼָ�������Ϊ��ĳ���ʱ��[s]

//    const float stance_height = 10; //��Ծǰ���Ȳ���չ[m]
//    const float jump_extension = 15; //����Ȳ���չ[m]
//    const float fall_extension = 18; //�ڵ���ʱ�������Ȳ���չ[m]
//		const float jump_angle = 35;  //��ǰ��Ծ�ĽǶ�

		t = HAL_GetTick()/1000.0f - start_time_/1000.0f; // ��Ծ��ʼ���ʱ��
    if (t < prep_time) 
		{
			ChangeKpKd(0.08,0.02);
			FrontLegCoordinateControl(-12*sin(15*PI/180),12*cos(15*PI/180));
			BackLegCoordinateControl(-12*sin(10*PI/180),12*cos(10*PI/180));
    } 
		else if (t >= prep_time && t < prep_time + launch_time) 
		{
			ChangeKpKd(20.0,0.08);
			FrontLegCoordinateControl(-22*sin(15*PI/180),22*cos(15*PI/180));
			BackLegCoordinateControl(-22*sin(10*PI/180),22*cos(10*PI/180));
    }
		else if (t >= prep_time + launch_time && t < prep_time + launch_time + fall_time) 
		{
			ChangeKpKd(0.3,0.08);
			AllLegCoordinateControl(5,12);
    } 
		else 
			{
			}
}

///*******************************************************************************************
//	*@ �������ƣ�void ExecuteJump(void) 
//	*@ ���ܣ���ǰ��Ծ
//	*@ ��ע��ѡ��״̬
// *******************************************************************************************/	
void ExecuteJump1(void) 
{
//     min radius = 0.8;
//     max radius = 0.25;
		float t; //������Ծ��ʼʱ�� 
    const float prep_time = 0.8; //��Ծǰ����ʱ��[s]
    const float launch_time = 0.25 ; //����ǰ�ĳ���ʱ��[s]
    const float fall_time = 1.75; //���Ȼָ�������Ϊ��ĳ���ʱ��[s]

//    const float stance_height = 10; //��Ծǰ���Ȳ���չ[m]
//    const float jump_extension = 15; //����Ȳ���չ[m]
//    const float fall_extension = 18; //�ڵ���ʱ�������Ȳ���չ[m]
//		const float jump_angle = 35;  //��ǰ��Ծ�ĽǶ�

		t = HAL_GetTick()/1000.0f - start_time_/1000.0f; // ��Ծ��ʼ���ʱ��
    if (t < prep_time) 
		{
			ChangeKpKd(0.08,0.02);
			FrontLegCoordinateControl(-12*sin(40*PI/180),12*cos(40*PI/180));
			BackLegCoordinateControl(-12*sin(30*PI/180),12*cos(30*PI/180));
    } 
		else if (t >= prep_time && t < prep_time + launch_time) 
		{
			ChangeKpKd(30.0,0.08);
			FrontLegCoordinateControl(-24*sin(40*PI/180),24*cos(40*PI/180));
			BackLegCoordinateControl(-24*sin(30*PI/180),24*cos(30*PI/180));
    }
		else if (t >= prep_time + launch_time && t < prep_time + launch_time + fall_time) 
		{
			ChangeKpKd(0.5,0.08);
			AllLegCoordinateControl(5,12);
    } 
		else 
			{
				
			}
}


///*******************************************************************************************
//	*@ �������ƣ�void ExecuteJump(void) 
//	*@ ���ܣ���ǰ��Ծ
//	*@ ��ע��ѡ��״̬
// *******************************************************************************************/	
void ExecuteJump2(void) 
{
//     min radius = 0.8;
//     max radius = 0.25;
		float t; //������Ծ��ʼʱ�� 
    const float prep_time = 0.8; //��Ծǰ����ʱ��[s]
    const float launch_time = 0.25 ; //����ǰ�ĳ���ʱ��[s]
    const float fall_time = 1.75; //���Ȼָ�������Ϊ��ĳ���ʱ��[s]

//    const float stance_height = 10; //��Ծǰ���Ȳ���չ[m]
//    const float jump_extension = 15; //����Ȳ���չ[m]
//    const float fall_extension = 18; //�ڵ���ʱ�������Ȳ���չ[m]
//		const float jump_angle = 35;  //��ǰ��Ծ�ĽǶ�

		t = HAL_GetTick()/1000.0f - start_time_/1000.0f; // ��Ծ��ʼ���ʱ��
    if (t < prep_time) 
		{
			ChangeKpKd(0.08,0.02);
			FrontLegCoordinateControl(-12*sin(0*PI/180),12*cos(0*PI/180));
			BackLegCoordinateControl(-12*sin(0*PI/180),12*cos(0*PI/180));
    } 
		else if (t >= prep_time && t < prep_time + launch_time) 
		{
			ChangeKpKd(20.0,0.08);
			FrontLegCoordinateControl(-22*sin(0*PI/180),22*cos(0*PI/180));
			BackLegCoordinateControl(-22*sin(0*PI/180),22*cos(0*PI/180));
    }
		else if (t >= prep_time + launch_time && t < prep_time + launch_time + fall_time) 
		{
			ChangeKpKd(0.3,0.08);
			AllLegCoordinateControl(5,12);
    } 
		else 
			{
			}
}

///*******************************************************************************************
//	*@ �������ƣ�void ExecuteJump(void) 
//	*@ ���ܣ���ǰ��Ծ
//	*@ ��ע��ѡ��״̬
// *******************************************************************************************/	
void BigJump(void) 
{
//     min radius = 0.8;
//     max radius = 0.25;
		float t; //������Ծ��ʼʱ�� 
    const float prep_time = 0.8; //��Ծǰ����ʱ��[s]
    const float launch_time = 0.25 ; //����ǰ�ĳ���ʱ��[s]
    const float fall_time = 1.75; //���Ȼָ�������Ϊ��ĳ���ʱ��[s]

//    const float stance_height = 10; //��Ծǰ���Ȳ���չ[m]
//    const float jump_extension = 15; //����Ȳ���չ[m]
//    const float fall_extension = 18; //�ڵ���ʱ�������Ȳ���չ[m]
//		const float jump_angle = 35;  //��ǰ��Ծ�ĽǶ�

		t = HAL_GetTick()/1000.0f - start_time_/1000.0f; // ��Ծ��ʼ���ʱ��
    if (t < prep_time) 
		{
			ChangeKpKd(0.08,0.02);
			FrontLegCoordinateControl(-12*sin(25*PI/180),12*cos(25*PI/180));
			BackLegCoordinateControl(-12*sin(15*PI/180),12*cos(15*PI/180));
    } 
		else if (t >= prep_time && t < prep_time + launch_time) 
		{
			ChangeKpKd(10.0,0.02);
			FrontLegCoordinateControl(-30*sin(25*PI/180),30*cos(25*PI/180));
			BackLegCoordinateControl(-30*sin(15*PI/180),30*cos(15*PI/180));
    }
		else if (t >= prep_time + launch_time && t < prep_time + launch_time + fall_time) 
		{
			ChangeKpKd(0.15,0.08);
			AllLegCoordinateControl(5,12);
    } 
		else 
			{
			}
}


/////*******************************************************************************************
////	*@ �������ƣ�void ExecuteJump(void) 
////	*@ ���ܣ���ǰ��Ծ
////	*@ ��ע��ѡ��״̬
//// *******************************************************************************************/	
//void ExecuteJump3(void) 
//{
////     min radius = 0.8;
////     max radius = 0.25;
//		float t; //������Ծ��ʼʱ�� 
//    const float prep_time = 0.8; //��Ծǰ����ʱ��[s]
//    const float launch_time = 0.25 ; //����ǰ�ĳ���ʱ��[s]
//    const float fall_time = 1.75; //���Ȼָ�������Ϊ��ĳ���ʱ��[s]

////    const float stance_height = 10; //��Ծǰ���Ȳ���չ[m]
////    const float jump_extension = 15; //����Ȳ���չ[m]
////    const float fall_extension = 18; //�ڵ���ʱ�������Ȳ���չ[m]
////		const float jump_angle = 35;  //��ǰ��Ծ�ĽǶ�

//		t = HAL_GetTick()/1000.0f - start_time_/1000.0f; // ��Ծ��ʼ���ʱ��
//    if (t < prep_time) 
//		{
//			ChangeKpKd(0.08,0.02);
//			FrontLegCoordinateControl(-12*sin(0*PI/180),12*cos(0*PI/180));
//			BackLegCoordinateControl(-12*sin(0*PI/180),12*cos(0*PI/180));
//    } 
//		else if (t >= prep_time && t < prep_time + launch_time) 
//		{
//			ChangeKpKd(20.0,0.08);
//			FrontLegCoordinateControl(-15*sin(10*PI/180),15*cos(10*PI/180));
//			BackLegCoordinateControl(-12*sin(0*PI/180),12*cos(0*PI/180));
//    }
//		else if (t >= prep_time + launch_time && t < prep_time + launch_time + fall_time) 
//		{
//			ChangeKpKd(0.3,0.08);
//			AllLegCoordinateControl(5,12);
//    } 
//		else 
//			{
//			}
//}

///*******************************************************************************************
//	*@ �������ƣ�void ExecuteJump(void) 
//	*@ ���ܣ���ǰ��Ծ
//	*@ ��ע��ѡ��״̬
// *******************************************************************************************/	
void ExecuteJump4(void) 
{
//     min radius = 0.8;
//     max radius = 0.25;
		float t; //������Ծ��ʼʱ�� 
    const float prep_time = 0.8; //��Ծǰ����ʱ��[s]
		const float backleg_jump_time  = 0.15;   //�������ĳ���ʱ��				   [ms]
		const float frontleg_jump_time = 0.18;   //ǰ�����ĳ���ʱ��           [ms]
    const float launch_time = 0.25 ; //����ǰ�ĳ���ʱ��[s]
    const float fall_time = 1.75; //���Ȼָ�������Ϊ��ĳ���ʱ��[s]

//    const float stance_height = 10; //��Ծǰ���Ȳ���չ[m]
//    const float jump_extension = 15; //����Ȳ���չ[m]
//    const float fall_extension = 18; //�ڵ���ʱ�������Ȳ���չ[m]
//		const float jump_angle = 35;  //��ǰ��Ծ�ĽǶ�

		t = HAL_GetTick()/1000.0f - start_time_/1000.0f; // ��Ծ��ʼ���ʱ��
    if (t < prep_time) 
		{
			ChangeKpKd(0.08,0.02);
			FrontLegCoordinateControl(-12*sin(60*PI/180),12*cos(60*PI/180));
			BackLegCoordinateControl(-12*sin(50*PI/180),12*cos(50*PI/180));
    } 
		else if (t >= prep_time && t < prep_time + launch_time) 
		{
			ChangeKpKd(20.0,0.08);
			FrontLegCoordinateControl(-22*sin(60*PI/180),22*cos(60*PI/180));
			BackLegCoordinateControl(-22*sin(50*PI/180),22*cos(50*PI/180));
    }
		else if (t >= prep_time + launch_time && t < prep_time + launch_time + fall_time) 
		{
			ChangeKpKd(0.3,0.08);
			AllLegCoordinateControl(5,12);
    } 
		else 
			{
			}
}
void BackFlip()
{
		float t; //������Ծ��ʼʱ�� 
	
		//IMU�����ض��Ƕȷ�Χ��Ծ�Ŀ��Ʊ���
		uint8_t imu_wait_lock = 1;//IMU����
		float imu_fullscale_correction = 3;//��Ϊ���������̣�90�ȣ���ʵ��������֮�
		float takeoff_inclination = -(67-imu_fullscale_correction);//���Ĳ��� ��inclination������
		float imu_angle_half_range = 1.5;//��������ʹ��
	
		uint8_t SeconTime=0;//�ж��Ƿ��ǵڶ��ε���
		unsigned long timedelay=0;//��������ʱ������Ծ��IMU��ϸ�����
    const float prep_time = 0.8; //��Ծǰ����ʱ��[s]
		const float backleg_jump_time  = 0.15;   //�������ĳ���ʱ��				   [ms]
		const float frontleg_jump_time = 0.18;   //ǰ�����ĳ���ʱ��           [ms]
    const float launch_time = 0.25 ; //����ǰ�ĳ���ʱ��[s]
    const float fall_time = 1.75; //���Ȼָ�������Ϊ��ĳ���ʱ��[s]
	
		/*��Ծ����̬�ѿ�*/
		const float stance_height = LegLenthMin+1.0f;//��Ծ֮ǰ�ȵĸ߶�  [cm]
		const float jump_extension = LegLenthExtremeMax;//�����LegLenthExtremeMax
		const float backleg_jump_angle  =  81.0f;//������ǰ�Ⱥ�����Ҫ�ɸýǶȲ���������
		const float frontleg_squat_angle = 60.0f;
		const float delta_angle = 10;

		t = HAL_GetTick()/1000.0f - start_time_/1000.0f; // ��Ծ��ʼ���ʱ��
	  
		if (t < prep_time) 
		{
			ChangeKpKd(0.3,0.08);
			BackLegCoordinateControl(-stance_height*cos(backleg_jump_angle*PI/180),stance_height*sin(backleg_jump_angle*PI/180));
			FrontLegCoordinateControl(-stance_height*cos(frontleg_squat_angle*PI/180),stance_height*sin(frontleg_squat_angle*PI/180));
    } 
		else if (t >= prep_time && t < prep_time + launch_time) 
		{
			ChangeKpKd(50.0,0.08);
//			FrontLegCoordinateControl(-22*sin(60*PI/180),22*cos(60*PI/180));
			BackLegCoordinateControl(-jump_extension*cos(frontleg_squat_angle*PI/180),jump_extension*sin(frontleg_squat_angle*PI/180));
			ChangeKpKd(2.0,0.08);
			FrontLegCoordinateControl(-stance_height*cos((frontleg_squat_angle+delta_angle)*PI/180),stance_height*sin((frontleg_squat_angle+delta_angle)*PI/180));
			while(imu_wait_lock)
			{
				/***********���ȵȴ�������ʱ��************/
					timedelay++;
					if(timedelay == backleg_jump_time/5) //��������ʱ��Լ5msһ�Ρ�
						{
							ChangeKpKd(0.05,0.08);
//						LegPID_Set(1,7,0.1f,1.1f,200,0.1f);
//						LegPID_Set(3,7,0.1f,1.1f,200,0.1f);
							/******������������̬����Ҫ������Ῠס��******/
//						LegSpeedLimit(1,SpeedMode_SLOW + 150);
//						LegSpeedLimit(3,SpeedMode_SLOW + 150);
							//���ȷ���ת
							theta3=	360;
							theta4=-180;
							SetCoupledThetaPosition(3);
							SetCoupledThetaPosition(4);
						}
					FrontLegCoordinateControl(LegSquatLenth,-0.15f);
							//һֱ���ǶȺ��ʣ�Ȼ���л�����һ����̬��
					if(SeconTime==0 &&  EulerAngle[ROLL]> (takeoff_inclination-imu_angle_half_range) && EulerAngle[ROLL] < (takeoff_inclination+imu_angle_half_range) )
						{
							SeconTime=2;
						}
					else if(SeconTime==2 && EulerAngle[ROLL]	< (-88-imu_fullscale_correction) && EulerAngle[ROLL]> (-90-imu_fullscale_correction) )//��pitch�Ƕȳ���ϵͳ���ʱ�����ܽǶȷ�Χ�ﲻ���������޷�ǰ������������imu.c�����ж�������ϵͳ��
						{	
							SeconTime=1;
						}
					else if(SeconTime==1 && EulerAngle[ROLL]> (takeoff_inclination-imu_angle_half_range) && EulerAngle[ROLL]< (takeoff_inclination+imu_angle_half_range) )
						{
							imu_wait_lock=0;
							ChangeKpKd(0.05,0.08);
							//���ȷ���ת
 							theta3=	360;
							theta4=-180;
							SetCoupledThetaPosition(3);
							SetCoupledThetaPosition(4);
						}
			}
			ChangeKpKd(20.0,0.08);
			FrontLegCoordinateControl(jump_extension,0.15f);
	}
		else
		{
				ChangeKpKd(0.5,0.02);
//				theta1=0;theta2=180;
//				SetCoupledThetaPosition(1);
//				SetCoupledThetaPosition(2);
//				HAL_Delay(200);
			
				//���ȷ��������ת��
				BackLegCoordinateControl(0,-LegStandLenth);
				
				FrontLegCoordinateControl(0,-LegStandLenth);
		}
}
