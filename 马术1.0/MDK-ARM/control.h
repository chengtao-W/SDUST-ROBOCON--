#ifndef __CONTROL_H__
#define __CONTROL_H__

#define PI 3.14159265358979324f
#define StanceHeight  25
#define L1 12
#define L2 23.5

typedef struct  {		// 腿部参数结构体
    float stance_height ; // 狗身到地面的距离 (cm)
    float step_length ; // 一步的距离 (cm)
    float up_amp ; // 上部振幅y (cm)
    float down_amp ; // 下部振幅 (cm)
    float flight_percent ; // 摆动相百分比 %
    float freq ; // 一步的频率 (Hz)
} GaitParams;

typedef struct
{
		uint8_t GaitID;
    GaitParams detached_params_0;
    GaitParams detached_params_1;
    GaitParams detached_params_2;
    GaitParams detached_params_3;
} ALLParam;//Detached：分离的

void CartesianToTheta(void);
void SinTrajectory (float t,GaitParams params, float gaitOffset,float leg_diretion) ;
void BeSSeLTrajectory (float t,GaitParams params, float gaitOffset,float leg_diretion) ;
void SetCoupledThetaPosition(int LegId);//单独版，需要给腿号
void CoupledMoveLeg(float Times,GaitParams params,float gaitOffset,float direction,int id);

void ChangeKpKd(float kpSet,float kdSet);

void SingleLegCoordinateControl(float set_x,float set_y,int LegId);
void BackLegCoordinateControl(float set_x,float set_y);
void FrontLegCoordinateControl(float set_x,float set_y);
void AllLegCoordinateControl(float set_x,float set_y);

void MotorInit(void);

void ReverseMoveOpen(void);
void ReverseMoveClose(void);

#endif

