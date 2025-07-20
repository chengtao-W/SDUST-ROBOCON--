#include "rc_dog.h"

float theta1=0,theta2=0,theta3=0,theta4=0;
float x=0,y=0;//���ߵĽ��
double L=0;//L�ǵ����ĩ�˵�ֱ�߾���
float omega1=0,omega2=0,omega3=0,omega4=0,omega5=0,omega6=0,omega7=0,omega8=0;
float KP=0;
float KD=0;

//�˶��������
uint8_t reverse_move_flag = 0;
uint8_t Invert_coordinate_system = 0;

//��ʼ̬����ֵ������ͳһ����ϵ��
#define offset_front_1 0.0  //40.2f
#define offset_front_2 5.0 //34.1f
#define offset_back_1  0.0
#define offset_back_2  5.0

void CartesianToTheta(void)
{
    float L=0,N=0;
    double M=0;
    float A1=0,A2=0;
		//����ǰ��
	//�����ȳ����㼰�ȳ���λ
    L=sqrt(pow(x,2) + pow(y,2));
	  if(L<LegLenthMin) L=LegLenthMin;
		  else if(L>LegLenthExtremeMax) L=LegLenthExtremeMax;
		//�����ȳ����㡰�м�Ƕȡ�N��M������Ƕȷ�Χ-180��~180�㡣
		N=asin(y/L)*180.0/PI;//�Ƕȷ�ΧΪ-90��~90�㡣
		if((x<0)&&(y>0)) N=180-N;//�Ƕȷ�ΧΪ90��~180��
		else if((x<0)&&(y<0)) N=-180-N;//�Ƕȷ�ΧΪ-180��~-90��
    M=acos(	( pow(L,2)+pow(L1,2)-pow(L2,2) )/(2*L1*L) )*180.0/PI;//���ԵĽǶȴ�С���Ƕȷ�ΧΪ0��~90�㡣
		//����ת�����Ȳ�����offset�������ո�ֵ�ٽ��е�������
    A1=M+N-90;
    A2=90-(N-M);
		//����ȷ������Ƕȡ��Ƕȷ�Χ�ֱ�Ϊ0��~360���-360��~0�㡣
		theta1=-(A1-90);
		theta2=-(A2-270);//���Կ�����theta2=A2+90��������һ��������������õĶԳ��ԣ�ʹ��������Ҳ������x�������ҵĶԳƣ����A2�ڼ�ȥ90�Ļ����ϣ��ټ�ȥ��180��
		if(reverse_move_flag==1)
		{
			//�����ǶȵĴ���ͬ��
			theta1-=360;
			theta2+=360;
		}
		
		//�������
		//�����ȳ����㼰�ȳ���λ
    L=sqrt(pow(-x,2) + pow(y,2));
	  if(L<LegLenthMin) L=LegLenthMin;
		  else if(L>LegLenthExtremeMax) L=LegLenthExtremeMax;
		//�����ȳ����㡰�м�Ƕȡ�N��M������Ƕȷ�Χ-180��~180�㡣
		N=asin(y/L)*180.0/PI;//�Ƕȷ�ΧΪ-90��~90�㡣
		if((-x<0)&&(y>0)) N=180-N;//�Ƕȷ�ΧΪ90��~180��
		else if((-x<0)&&(y<0)) N=-180-N;//�Ƕȷ�ΧΪ-180��~-90��
    M=acos(	( pow(L,2)+pow(L1,2)-pow(L2,2) )/(2*L1*L) )*180.0/PI;//���ԵĽǶȴ�С���Ƕȷ�ΧΪ0��~90�㡣
		//����ת�����Ȳ�����offset�������ո�ֵ�ٽ��е�������
    A1=M+N-90;
    A2=90-(N-M);
		//����ȷ������Ƕȡ��Ƕȷ�Χ�ֱ�Ϊ0��~360���-360��~0�㡣
		theta3=-(A1-90);
		theta4=-(A2-270);//���Կ�����theta2=A2+90��������һ��������������õĶԳ��ԣ�ʹ��������Ҳ������x�������ҵĶԳƣ����A2�ڼ�ȥ90�Ļ����ϣ��ټ�ȥ��180��
		if(reverse_move_flag==1)
		{
			//�����ǶȵĴ���ͬ��
			theta3-=360;
			theta4+=360;
		}
}

/*
* NAME: SinTrajectory (float t,GaitParams params, float gaitOffset)
* FUNCTION : ���ҹ켣�����������ĺ�����������CoupledMoveLeg�����С�
* ��ڲ�����
			t���������Ʊ�������������ʱ������š��������ϲ��������ʱ�������ʵ�����ǣ�tt=times*5/1000;��ttԼÿ5ms�仯5/1000����0.005��
			GaitParams����̬����
			gaitOffset����λ����ڹ��ɲ�ͬ��̬�ĺ��Ĳ�������
			leg_diretion�������ȵ�ǰ�������
			angle��
* ���Ż�����ʼ��λ�Ŀ��ƱȽ���Ҫ���������𲽵�ʱ���Ƿ�ƽ�ȣ�
*/
void SinTrajectory (float t,GaitParams params, float gaitOffset,float leg_diretion) 
{
	//t=times*5/1000����ÿ1s�仯1
	//��ȡ���Һ�������Ҫ���õĲ���
		float gp;
    float stanceHeight = params.stance_height;//��������ظ߶�
    float downAMP = params.down_amp;//����ֵ
    float upAMP = params.up_amp;//����ֵ
    float flightPercent = params.flight_percent;//�ڶ���ռ��
    float stepLength = params.step_length ;//����
    float FREQ = params.freq;//Ƶ��
		
	if(leg_diretion<0) stepLength = -stepLength;//�������

	/******��λ��ʱ�䡢����ѭ��������******/
    static float p = 0,prev_t = 0;//��λʱ���ۼ�(Ҫ��ʵ�ֲ�ͬ�Ȳ�ͬƵ�ʣ��Ͳ��ܹ���һ���������Ӧ�ý����Ϊ�Ȳ���������)��
	//����tÿ�ν��뺯���仯����0.005�����FREQ������ҪС��200������p�ı仯�������ڵ���1���Ӷ������˶�����
	//���統FREQ=1ʱ��ÿ����1s��t�仯1����p�պñ仯1���ʴ�ʱƵ��Ϊ1Hz����FREQ=nʱ��Ƶ����Ȼ��ΪnHz����Ƶ�����Ϊ200Hz��
	//����Ƶ�ʲ�Ҫ������ΪƵ��Խ����ζ�Ų�������Խ�١���ʵ�������ǲ���Ҫ��ô��Ƶ�ʣ�Ӧ��Ƶ��������0-5�����䷶Χ�ڡ�
    p += FREQ * (t - prev_t);//Ƶ��*ʱ��仯����Ϊ��λ�仯����pÿ�α仯��������ʱ���ǹ̶���5ms�������ǿ���ͨ���ı�ÿ�α仯�Ĵ�С����Ӵ���仯Ƶ�ʡ�FREQԽ�󣬵��α仯�ľ�Խ��
		gp = fmod((p+gaitOffset),1.0);//�ú������� x/y ����������1.0����ȡС�����֣�����gp������0-1��Χ�ڡ�
    prev_t = t;//����ǰtֵ����������
	/******���ҹ켣����******/
	//���ڶ���
    if (gp <= flightPercent) //gp����gaitOffset��ʼ����˵�gaitOffset����flightPercentʱ����ֱ��ת��֧���ࡣ
	{
        x = (gp/flightPercent)*stepLength - stepLength/2.0f;//��-stepLength/2��+stepLength/2���ƶ�ʱ�䲻��stepLength�ı䣬��stepLengthԽ��ʵ���ƶ��ٶ�Խ�졣
        y = -upAMP*sin(PI*gp/flightPercent) + stanceHeight;//Χ��stanceHeightΪ�����������Ҳ�����ͬ����upAMPԽ���ƶ��ٶ�Խ�졣
    }
	//���֧����
    else //�ڶ����Ǵ����ҹ켣����ʼλ�ô�ִ�С�
	{
        float percentBack = (gp-flightPercent)/(1.0f-flightPercent);//percentBack��(gp/flightPercent)��һ������
        x = -percentBack*stepLength + stepLength/2.0f;//һ����˵���״ν���ʱ���Ǵ�stepLength/2��ʼ��Ȼ��֮�������˶���
        y = downAMP*sin(PI*percentBack) + stanceHeight;//
    }

}

//�����˶�����
void ReverseMoveOpen(void)
{
	reverse_move_flag=1;
}
//�ر��˶�����
void ReverseMoveClose(void)
{
	reverse_move_flag=0;
}

/*
* NAME: SinTrajectory (float t,GaitParams params, float gaitOffset)
* FUNCTION : �������켣�����������ĺ�����������CoupledMoveLeg�����С�
* ��ڲ�����
			t���������Ʊ�������������ʱ������š��������ϲ��������ʱ�������ʵ�����ǣ�tt=times*5/1000;��ttԼÿ5ms�仯5/1000����0.005��
			GaitParams����̬����
			gaitOffset����λ����ڹ��ɲ�ͬ��̬�ĺ��Ĳ�������
			leg_diretion�������ȵ�ǰ�������
			angle��
* ���Ż�����ʼ��λ�Ŀ��ƱȽ���Ҫ���������𲽵�ʱ���Ƿ�ƽ�ȣ�
*/
void BeSSeLTrajectory (float t,GaitParams params, float gaitOffset,float leg_diretion) 
{
	//t=times*5/1000����ÿ1s�仯1
	//��ȡ���Һ�������Ҫ���õĲ���
		float gp;
    float stanceHeight = params.stance_height;//��������ظ߶�
    float downAMP = params.down_amp;//����ֵ
    float upAMP = params.up_amp;//����ֵ
    float flightPercent = params.flight_percent;//�ڶ���ռ��
    float stepLength = params.step_length ;//����
    float FREQ = params.freq;//Ƶ��
		
	if(leg_diretion<0) stepLength = -stepLength;//�������

	/******��λ��ʱ�䡢����ѭ��������******/
    static float p = 0,prev_t = 0;//��λʱ���ۼ�(Ҫ��ʵ�ֲ�ͬ�Ȳ�ͬƵ�ʣ��Ͳ��ܹ���һ���������Ӧ�ý����Ϊ�Ȳ���������)��
    p += FREQ * (t - prev_t);//Ƶ��*ʱ��仯����Ϊ��λ�仯����pÿ�α仯��������ʱ���ǹ̶���5ms�������ǿ���ͨ���ı�ÿ�α仯�Ĵ�С����Ӵ���仯Ƶ�ʡ�FREQԽ�󣬵��α仯�ľ�Խ��
		gp = fmod((p+gaitOffset),1.0);//�ú������� x/y ����������1.0����ȡС�����֣�����gp������0-1��Χ�ڡ�
    prev_t = t;//����ǰtֵ����������

	//���ڶ���
    if (gp <= 0.5) //gp����gaitOffset��ʼ����˵�gaitOffset����flightPercentʱ����ֱ��ת��֧���ࡣ
	{
        x = -stepLength/2+stepLength*(10*(pow(2*gp,3))*(pow(1-(2*gp),2))+5*(pow((2*gp),4))*(1-(2*gp))+pow(2*gp,5));
				if(gp<0.25)
				{
        y =stanceHeight+upAMP*(10*(pow(4*gp,3))*(pow(1-(4*gp),2))+5*(pow((4*gp),4))*(1-(4*gp))+pow(4*gp,5));
				}
				else
				{
					y =stanceHeight+upAMP*(10*(pow(2-(4*gp),3))*(pow((4*gp)-1,2))+5*(pow(2-(4*gp),4))*((4*gp)-1)+pow(2-(4*gp),5));
				}
    }
	//���֧����
    else //�ڶ����Ǵ����ҹ켣����ʼλ�ô�ִ�С�
	{
		
//        float percentBack = (gp-flightPercent)/(1.0f-flightPercent);//percentBack��(gp/flightPercent)��һ������
        x = -stepLength/2+stepLength*(10*(pow(2-(2*gp),3))*(pow((2*gp)-1,2))+5*(pow(2-(2*gp),4))*((2*gp)-1)+pow(2-(2*gp),5));
        y = stanceHeight;
    }

}

void MotorInit(void)
{
	MotorParameterInit();
	
	SERVO_Send_recv(&Motor[0], &data[0]);	//������ָ��͸������ͬʱ���շ���ֵ
	omega1=data[0].Pos;
	HAL_Delay(5);
	
	SERVO_Send_recv(&Motor[1], &data[1]);	//������ָ��͸������ͬʱ���շ���ֵ
	omega2=data[1].Pos;
	HAL_Delay(5);
	
	SERVO_Send_recv(&Motor[2], &data[2]);	//������ָ��͸������ͬʱ���շ���ֵ
	omega3=data[2].Pos;
	HAL_Delay(5);
	
	SERVO_Send_recv(&Motor[3], &data[3]);	//������ָ��͸������ͬʱ���շ���ֵ
	omega4=data[3].Pos;
	HAL_Delay(5);
	
	SERVO_Send_recv(&Motor[4], &data[4]);	//������ָ��͸������ͬʱ���շ���ֵ
	omega5=data[4].Pos;
	HAL_Delay(5);
	
	SERVO_Send_recv(&Motor[5], &data[5]);	//������ָ��͸������ͬʱ���շ���ֵ
	omega6=data[5].Pos;
	HAL_Delay(5);
	
	SERVO_Send_recv(&Motor[6], &data[6]);	//������ָ��͸������ͬʱ���շ���ֵ
	omega7=data[6].Pos;
	HAL_Delay(5);
	
	SERVO_Send_recv(&Motor[7], &data[7]);	//������ָ��͸������ͬʱ���շ���ֵ
	omega8=data[7].Pos;
	HAL_Delay(5);
}

void SingleLegCoordinateControl(float set_x,float set_y,int LegId)//վ������
{
	x=set_x;
	y=set_y;
	CartesianToTheta();
	SetCoupledThetaPosition(LegId);
}

void BackLegCoordinateControl(float set_x,float set_y)
{
	x=set_x;
	y=set_y;
	CartesianToTheta();
	SetCoupledThetaPosition(3);
	SetCoupledThetaPosition(4);
	
}

void FrontLegCoordinateControl(float set_x,float set_y)
{
	x=set_x;
	y=set_y;
	CartesianToTheta();
	SetCoupledThetaPosition(1);
	SetCoupledThetaPosition(2);
	
}

void AllLegCoordinateControl(float set_x,float set_y)
{
	x=set_x;
	y=set_y;
	CartesianToTheta();
	SetCoupledThetaPosition(1);
	SetCoupledThetaPosition(2);
	SetCoupledThetaPosition(3);
	SetCoupledThetaPosition(4);
}


void ChangeKpKd(float kpSet,float kdSet)
{
	KP=kpSet;
	KD=kdSet;
}

void CoupledMoveLeg(float Times,GaitParams params,float gaitOffset,float direction,int id)
{
	SinTrajectory(Times,params,gaitOffset,direction);
	CartesianToTheta();
	SetCoupledThetaPosition(id);
}



void SetCoupledThetaPosition(int LegId)//�����棬��Ҫ���Ⱥ�
{
		switch(LegId)
	{
		case 1:
		{
		MotorParameterSet(omega1+(theta1+offset_front_1)/360*6.2832f,1,KP,KD);
		SERVO_Send_recv(&Motor[0], &data[0]);	//������ָ��͸������ͬʱ���շ���ֵ

		MotorParameterSet(omega2+(theta2+offset_front_2)/360*6.2832f,2,KP,KD);
		SERVO_Send_recv(&Motor[1], &data[1]);	//������ָ��͸������ͬʱ���շ���ֵ
		}break;
		case 2:
		{
		MotorParameterSet(omega3-(theta2+offset_front_2)/360*6.2832f,3,KP,KD);
		SERVO_Send_recv(&Motor[2], &data[2]);	//������ָ��͸������ͬʱ���շ���ֵ

		MotorParameterSet(omega4-(theta1+offset_front_1)/360*6.2832f,4,KP,KD);
		SERVO_Send_recv(&Motor[3], &data[3]);	//������ָ��͸������ͬʱ���շ���ֵ
		}break;
		case 3:
		{
		MotorParameterSet(omega5-(theta3+offset_back_1)/360*6.2832f,5,KP,KD);
		SERVO_Send_recv(&Motor[4], &data[4]);	//������ָ��͸������ͬʱ���շ���ֵ
	
		MotorParameterSet(omega6-(theta4+offset_back_2)/360*6.2832f,6,KP,KD);
		SERVO_Send_recv(&Motor[5], &data[5]);	//������ָ��͸������ͬʱ���շ���ֵ
		}break;
		case 4:
		{
		MotorParameterSet(omega7+(theta4+offset_back_2)/360*6.2832f,7,KP,KD);
		SERVO_Send_recv(&Motor[6], &data[6]);	//������ָ��͸������ͬʱ���շ���ֵ
	
		MotorParameterSet(omega8+(theta3+offset_back_1)/360*6.2832f,8,KP,KD);
		SERVO_Send_recv(&Motor[7], &data[7]);	//������ָ��͸������ͬʱ���շ���ֵ
		}break;
	}
}
