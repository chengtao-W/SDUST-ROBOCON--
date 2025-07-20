#include "gait.h"
#include "rc_dog.h"


/*
  1-----2
     |
	   |
  3-----4	
*/
float now_time;
extern float time;

ALLParam ALLParams[StatesMaxNum]=
{
		{//NONE
			0,
     {0,0,0,0,0,0},//{步高, 步长, 抬腿高, 压腿高, 飞行占比, 频率}	单位 cm
		 {0,0,0,0,0,0},//{stance_height, step_length, up_amp, down_amp, flight_percent, freq}  cm
		 {0,0,0,0,0,0},
		 {0,0,0,0,0,0}
		},// TROT	双拍步态 对角小跑
	
		{
		//TROT
			1,
		 {17,10.0,3,1,0.5,4.0},//15,8.0,6,0,0.5,3.5
		 {17,10.0,3,1,0.5,4.0},// 6个参数变量为stance_height; step_length; up_amp; down_amp; flight_percent; freq
		 {17,10.0,3,1,0.5,4.0},
		 {17,10.0,3,1,0.5,4.0}
    },
		{
			//踱步
			2,
     {17,0.0,3,2,0.5,5.5},
     {17,0.0,3,2,0.5,5.5},
     {17,0.0,3,2,0.5,5.5},// 6个参数变量为stance_height; step_length; up_amp; down_amp; flight_percent; freq
     {17,0.0,3,2,0.5,5.5}
    },
		{
		//TROT
			3,
		 {17,10.0,3,10,0.5,2.0},//15,8.0,6,0,0.5,3.5
		 {17,10.0,3,10,0.5,2.0},// 6个参数变量为stance_height; step_length; up_amp; down_amp; flight_percent; freq
		 {17,10.0,3,10,0.5,2.0},
		 {17,10.0,3,10,0.5,2.0}
    },
		{
		//TROT
			4,
		 {17,0.0,0,0,0.0,3},//15,8.0,6,0,0.5,3.5
		 {17,0.0,0,0,0.0,3},// 6个参数变量为stance_height; step_length; up_amp; down_amp; flight_percent; freq
		 {17,0.0,0,0,0.0,3},
		 {17,0.0,0,0,0.0,3}
    },
};
//用于复制上方状态数组作为永恒基准。
ALLParam StateDetachedParams_Copy[StatesMaxNum] = {0};

//正常参数
GaitParams state_gait_params[]={

{0,0,0,0,0,0},//NONE 0  ok
{17,10.0,4,0,0.5,5.5},//TROT 1  ok  步高 步幅 
{17,0.0,3,2,0.5,5.5},//BACKWARD 2  ok 
{19,10.0,2,0,0.5,2.5},//WALK 3 直走 ok
{19,3,0,2,0.5,2},//S_WALK
{19,3,0,2,0.5,2},//S_BACKWARD

};

int flag=1;

void InitPosture()
{
	if(flag == 1)
	{
		ChangeKpKd(0.3,0.08);
		FrontLegCoordinateControl(15,15);
		BackLegCoordinateControl(-15,15);
		HAL_Delay(50);
		FrontLegCoordinateControl(14,15);
		BackLegCoordinateControl(-14,15);
		HAL_Delay(50);
		FrontLegCoordinateControl(13,15);
		BackLegCoordinateControl(-13,15);
		HAL_Delay(50);
		FrontLegCoordinateControl(12,15);
		BackLegCoordinateControl(-12,15);
		HAL_Delay(50);
		FrontLegCoordinateControl(11,15);
		BackLegCoordinateControl(-11,15);
		HAL_Delay(50);
		FrontLegCoordinateControl(10,15);
		BackLegCoordinateControl(-10,15);
		HAL_Delay(50);
		FrontLegCoordinateControl(9,15);
		BackLegCoordinateControl(-9,15);
		HAL_Delay(50);
		FrontLegCoordinateControl(8,15);
		BackLegCoordinateControl(-8,15);
		HAL_Delay(50);
		FrontLegCoordinateControl(7,15);
		BackLegCoordinateControl(-7,15);
		HAL_Delay(50);
		FrontLegCoordinateControl(6,15);
		BackLegCoordinateControl(-6,15);
		HAL_Delay(50);
		FrontLegCoordinateControl(5,15);
		BackLegCoordinateControl(-5,15);
		HAL_Delay(50);
		FrontLegCoordinateControl(4,15);
		BackLegCoordinateControl(-4,15);
		HAL_Delay(50);
		FrontLegCoordinateControl(3,15);
		BackLegCoordinateControl(-3,15);
		HAL_Delay(50);
		FrontLegCoordinateControl(2,15);
		BackLegCoordinateControl(-2,15);
		HAL_Delay(50);
		FrontLegCoordinateControl(1,15);
		BackLegCoordinateControl(-1,15);
		HAL_Delay(50);
		FrontLegCoordinateControl(0,15);
		BackLegCoordinateControl(0,15);
		HAL_Delay(50);
		flag=0;
	}
	if(flag==0)
	{
		ChangeKpKd(1.0,0.08);
		FrontLegCoordinateControl(0,15);
		BackLegCoordinateControl(0,15);
	}
}

void StopPosture(void)
{
		ChangeKpKd(1.0,0.08);
}

/********对角测验函数*********/
void gait(	ALLParam ALLParams,
                    float leg0_offset, float leg1_offset,float leg2_offset, float leg3_offset,
                    float leg0_direction, float leg1_direction,float leg2_direction, float leg3_direction)
{
	 float t ;
	 t = HAL_GetTick()/1000.0f-now_time/1000.0f;

	 CoupledMoveLeg(t, ALLParams.detached_params_0, leg0_offset, 	leg0_direction,	1);
   CoupledMoveLeg(t, ALLParams.detached_params_1, leg1_offset,	leg1_direction, 2);
   CoupledMoveLeg(t, ALLParams.detached_params_2, leg2_offset,	leg2_direction, 3);
   CoupledMoveLeg(t, ALLParams.detached_params_3, leg3_offset,	leg3_direction, 4);						
}		

void DogAttitudeControl(void)
{
		ChangeKpKd(1.0,0.08);
		AttitudeControl(0.0f,0.0f,&ALLParams[4]);
		gait(ALLParams[4],0,0,0,0,1.0,1.0,1.0,1.0);
}
/********Tort步态**********/
void TortPosture(uint8_t direction)
{
	ChangeKpKd(1.0,0.08);
	if(direction == 0)//直走
	{
//		YawControl(0.0f,&ALLParams[1],direction);
//		PitchControl(10.0f,&ALLParams[1]);
		gait(ALLParams[1],0.5,0,0,0.5,1.0,1.0,1.0,1.0);
	}
	else//后退
	{
//		YawControl(0.0f,&ALLParams[1],direction);
//		PitchControl(-10.0f,&ALLParams[1],direction);
		gait(ALLParams[1],0.5,0,0,0.5,1.0,1.0,1.0,1.0);
	}
}

/********旋转函数**********/
void RotatePosture(uint8_t direction)
{
	ChangeKpKd(1.0,0.08);
	if(direction == 0)//左转
	{
		gait(ALLParams[1],0.5,0,0,0.5,-1.0,1.0,-1.0,1.0);
	}
	else//右转
	{
		gait(ALLParams[1],0.5,0,0,0.5,1.0,-1.0,1.0,-1.0);
	}
}


///*********原地踏步**********/
void MarkingPosture(void)
{
	ChangeKpKd(1.0,0.08);
	gait(ALLParams[2],0.5,0,0,0.5,-1.0,-1.0,-1.0,-1.0);
}

///*********后退函数**********/
void you_pingyi(void)
{
//	change_KP_KD(0.7,0.01);
	 float t ;
	 t = HAL_GetTick()/1000.0f-now_time/1000.0f;

	 CoupledMoveLeg(t, state_gait_params[2],0, 1.0,2);
//   CoupledMoveLeg(t, params, leg1_offset,	leg1_direction,2);
   CoupledMoveLeg(t, state_gait_params[2],0, 1.0,4);
//   CoupledMoveLeg(t, params, leg3_offset,	leg3_direction,4);			
}

void zuo_pingyi(void)
{
//	change_KP_KD(0.7,0.01);
//	gait(state_gait_params[1],0.5,0,0,0.5,-1.0,-1.0,-1.0,-1.0);
	 float t ;
	 t = HAL_GetTick()/1000.0f-now_time/1000.0f;

	 CoupledMoveLeg(t, state_gait_params[2],0,1.0,1);
//   CoupledMoveLeg(t, params, leg1_offset,	leg1_direction,2);
   CoupledMoveLeg(t, state_gait_params[2],0,1.0,3);
//   CoupledMoveLeg(t, params, leg3_offset,	leg3_direction,4);				
}
