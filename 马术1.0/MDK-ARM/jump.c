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
//	*@ 任务名称：void ExecuteJump(void) 
//	*@ 功能：向前跳跃
//	*@ 备注：选择状态
// *******************************************************************************************/	
void ExecuteJump(void) 
{
//     min radius = 0.8;
//     max radius = 0.25;
		float t; //计算跳跃开始时间 
    const float prep_time = 0.8; //跳跃前持续时间[s]
    const float launch_time = 0.25 ; //收腿前的持续时间[s]
    const float fall_time = 1.75; //收腿恢复正常行为后的持续时间[s]

//    const float stance_height = 10; //跳跃前的腿部伸展[m]
//    const float jump_extension = 15; //最大腿部伸展[m]
//    const float fall_extension = 18; //在跌倒时期望的腿部伸展[m]
//		const float jump_angle = 35;  //向前跳跃的角度

		t = HAL_GetTick()/1000.0f - start_time_/1000.0f; // 跳跃开始后的时间
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
//	*@ 任务名称：void ExecuteJump(void) 
//	*@ 功能：向前跳跃
//	*@ 备注：选择状态
// *******************************************************************************************/	
void ExecuteJump1(void) 
{
//     min radius = 0.8;
//     max radius = 0.25;
		float t; //计算跳跃开始时间 
    const float prep_time = 0.8; //跳跃前持续时间[s]
    const float launch_time = 0.25 ; //收腿前的持续时间[s]
    const float fall_time = 1.75; //收腿恢复正常行为后的持续时间[s]

//    const float stance_height = 10; //跳跃前的腿部伸展[m]
//    const float jump_extension = 15; //最大腿部伸展[m]
//    const float fall_extension = 18; //在跌倒时期望的腿部伸展[m]
//		const float jump_angle = 35;  //向前跳跃的角度

		t = HAL_GetTick()/1000.0f - start_time_/1000.0f; // 跳跃开始后的时间
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
//	*@ 任务名称：void ExecuteJump(void) 
//	*@ 功能：向前跳跃
//	*@ 备注：选择状态
// *******************************************************************************************/	
void ExecuteJump2(void) 
{
//     min radius = 0.8;
//     max radius = 0.25;
		float t; //计算跳跃开始时间 
    const float prep_time = 0.8; //跳跃前持续时间[s]
    const float launch_time = 0.25 ; //收腿前的持续时间[s]
    const float fall_time = 1.75; //收腿恢复正常行为后的持续时间[s]

//    const float stance_height = 10; //跳跃前的腿部伸展[m]
//    const float jump_extension = 15; //最大腿部伸展[m]
//    const float fall_extension = 18; //在跌倒时期望的腿部伸展[m]
//		const float jump_angle = 35;  //向前跳跃的角度

		t = HAL_GetTick()/1000.0f - start_time_/1000.0f; // 跳跃开始后的时间
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
//	*@ 任务名称：void ExecuteJump(void) 
//	*@ 功能：向前跳跃
//	*@ 备注：选择状态
// *******************************************************************************************/	
void BigJump(void) 
{
//     min radius = 0.8;
//     max radius = 0.25;
		float t; //计算跳跃开始时间 
    const float prep_time = 0.8; //跳跃前持续时间[s]
    const float launch_time = 0.25 ; //收腿前的持续时间[s]
    const float fall_time = 1.75; //收腿恢复正常行为后的持续时间[s]

//    const float stance_height = 10; //跳跃前的腿部伸展[m]
//    const float jump_extension = 15; //最大腿部伸展[m]
//    const float fall_extension = 18; //在跌倒时期望的腿部伸展[m]
//		const float jump_angle = 35;  //向前跳跃的角度

		t = HAL_GetTick()/1000.0f - start_time_/1000.0f; // 跳跃开始后的时间
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
////	*@ 任务名称：void ExecuteJump(void) 
////	*@ 功能：向前跳跃
////	*@ 备注：选择状态
//// *******************************************************************************************/	
//void ExecuteJump3(void) 
//{
////     min radius = 0.8;
////     max radius = 0.25;
//		float t; //计算跳跃开始时间 
//    const float prep_time = 0.8; //跳跃前持续时间[s]
//    const float launch_time = 0.25 ; //收腿前的持续时间[s]
//    const float fall_time = 1.75; //收腿恢复正常行为后的持续时间[s]

////    const float stance_height = 10; //跳跃前的腿部伸展[m]
////    const float jump_extension = 15; //最大腿部伸展[m]
////    const float fall_extension = 18; //在跌倒时期望的腿部伸展[m]
////		const float jump_angle = 35;  //向前跳跃的角度

//		t = HAL_GetTick()/1000.0f - start_time_/1000.0f; // 跳跃开始后的时间
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
//	*@ 任务名称：void ExecuteJump(void) 
//	*@ 功能：向前跳跃
//	*@ 备注：选择状态
// *******************************************************************************************/	
void ExecuteJump4(void) 
{
//     min radius = 0.8;
//     max radius = 0.25;
		float t; //计算跳跃开始时间 
    const float prep_time = 0.8; //跳跃前持续时间[s]
		const float backleg_jump_time  = 0.15;   //后退跳的持续时间				   [ms]
		const float frontleg_jump_time = 0.18;   //前腿跳的持续时间           [ms]
    const float launch_time = 0.25 ; //收腿前的持续时间[s]
    const float fall_time = 1.75; //收腿恢复正常行为后的持续时间[s]

//    const float stance_height = 10; //跳跃前的腿部伸展[m]
//    const float jump_extension = 15; //最大腿部伸展[m]
//    const float fall_extension = 18; //在跌倒时期望的腿部伸展[m]
//		const float jump_angle = 35;  //向前跳跃的角度

		t = HAL_GetTick()/1000.0f - start_time_/1000.0f; // 跳跃开始后的时间
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
		float t; //计算跳跃开始时间 
	
		//IMU到达特定角度范围跳跃的控制变量
		uint8_t imu_wait_lock = 1;//IMU锁定
		float imu_fullscale_correction = 3;//即为理论满量程（90度）与实际满量程之差。
		float takeoff_inclination = -(67-imu_fullscale_correction);//核心参数 （inclination：倾向）
		float imu_angle_half_range = 1.5;//可以满足使用
	
		uint8_t SeconTime=0;//判断是否是第二次到达
		unsigned long timedelay=0;//非阻塞延时，让跳跃与IMU结合更合理
    const float prep_time = 0.8; //跳跃前持续时间[s]
		const float backleg_jump_time  = 0.15;   //后退跳的持续时间				   [ms]
		const float frontleg_jump_time = 0.18;   //前腿跳的持续时间           [ms]
    const float launch_time = 0.25 ; //收腿前的持续时间[s]
    const float fall_time = 1.75; //收腿恢复正常行为后的持续时间[s]
	
		/*跳跃的姿态把控*/
		const float stance_height = LegLenthMin+1.0f;//跳跃之前腿的高度  [cm]
		const float jump_extension = LegLenthExtremeMax;//最大是LegLenthExtremeMax
		const float backleg_jump_angle  =  81.0f;//理论上前腿后移主要由该角度不合适引起
		const float frontleg_squat_angle = 60.0f;
		const float delta_angle = 10;

		t = HAL_GetTick()/1000.0f - start_time_/1000.0f; // 跳跃开始后的时间
	  
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
				/***********后腿等待倒立的时机************/
					timedelay++;
					if(timedelay == backleg_jump_time/5) //非阻塞延时，约5ms一次。
						{
							ChangeKpKd(0.05,0.08);
//						LegPID_Set(1,7,0.1f,1.1f,200,0.1f);
//						LegPID_Set(3,7,0.1f,1.1f,200,0.1f);
							/******后腿慢慢倒立态（必要，否则会卡住）******/
//						LegSpeedLimit(1,SpeedMode_SLOW + 150);
//						LegSpeedLimit(3,SpeedMode_SLOW + 150);
							//后腿反向转
							theta3=	360;
							theta4=-180;
							SetCoupledThetaPosition(3);
							SetCoupledThetaPosition(4);
						}
					FrontLegCoordinateControl(LegSquatLenth,-0.15f);
							//一直到角度合适，然后切换到下一个步态。
					if(SeconTime==0 &&  EulerAngle[ROLL]> (takeoff_inclination-imu_angle_half_range) && EulerAngle[ROLL] < (takeoff_inclination+imu_angle_half_range) )
						{
							SeconTime=2;
						}
					else if(SeconTime==2 && EulerAngle[ROLL]	< (-88-imu_fullscale_correction) && EulerAngle[ROLL]> (-90-imu_fullscale_correction) )//当pitch角度出现系统误差时，可能角度范围达不到这里，因此无法前翻，必须先在imu.c接收中断中修正系统误差。
						{	
							SeconTime=1;
						}
					else if(SeconTime==1 && EulerAngle[ROLL]> (takeoff_inclination-imu_angle_half_range) && EulerAngle[ROLL]< (takeoff_inclination+imu_angle_half_range) )
						{
							imu_wait_lock=0;
							ChangeKpKd(0.05,0.08);
							//后腿反向转
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
			
				//后腿反方向（向后）转动
				BackLegCoordinateControl(0,-LegStandLenth);
				
				FrontLegCoordinateControl(0,-LegStandLenth);
		}
}
