#ifndef __CONTROL_H__
#define __CONTROL_H__

#define PI 3.14159265358979324f
#define StanceHeight  25
#define L1 12
#define L2 23.5

typedef struct  {		// �Ȳ������ṹ��
    float stance_height ; // ��������ľ��� (cm)
    float step_length ; // һ���ľ��� (cm)
    float up_amp ; // �ϲ����y (cm)
    float down_amp ; // �²���� (cm)
    float flight_percent ; // �ڶ���ٷֱ� %
    float freq ; // һ����Ƶ�� (Hz)
} GaitParams;

typedef struct
{
		uint8_t GaitID;
    GaitParams detached_params_0;
    GaitParams detached_params_1;
    GaitParams detached_params_2;
    GaitParams detached_params_3;
} ALLParam;//Detached�������

void CartesianToTheta(void);
void SinTrajectory (float t,GaitParams params, float gaitOffset,float leg_diretion) ;
void BeSSeLTrajectory (float t,GaitParams params, float gaitOffset,float leg_diretion) ;
void SetCoupledThetaPosition(int LegId);//�����棬��Ҫ���Ⱥ�
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

