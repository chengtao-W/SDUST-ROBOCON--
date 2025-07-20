#include "rc_dog.h"

float theta1=0,theta2=0,theta3=0,theta4=0;
float x=0,y=0;//摆线的结果
double L=0;//L是电机到末端的直线距离
float omega1=0,omega2=0,omega3=0,omega4=0,omega5=0,omega6=0,omega7=0,omega8=0;
float KP=0;
float KD=0;

//运动反向控制
uint8_t reverse_move_flag = 0;
uint8_t Invert_coordinate_system = 0;

//初始态修正值（用于统一坐标系）
#define offset_front_1 0.0  //40.2f
#define offset_front_2 5.0 //34.1f
#define offset_back_1  0.0
#define offset_back_2  5.0

void CartesianToTheta(void)
{
    float L=0,N=0;
    double M=0;
    float A1=0,A2=0;
		//计算前腿
	//所需腿长计算及腿长限位
    L=sqrt(pow(x,2) + pow(y,2));
	  if(L<LegLenthMin) L=LegLenthMin;
		  else if(L>LegLenthExtremeMax) L=LegLenthExtremeMax;
		//根据腿长计算“中间角度”N和M。总体角度范围-180°~180°。
		N=asin(y/L)*180.0/PI;//角度范围为-90°~90°。
		if((x<0)&&(y>0)) N=180-N;//角度范围为90°~180°
		else if((x<0)&&(y<0)) N=-180-N;//角度范围为-180°~-90°
    M=acos(	( pow(L,2)+pow(L1,2)-pow(L2,2) )/(2*L1*L) )*180.0/PI;//绝对的角度大小，角度范围为0°~90°。
		//坐标转换（先不考虑offset，在最终赋值再进行调整）。
    A1=M+N-90;
    A2=90-(N-M);
		//最终确定电机角度。角度范围分别为0°~360°和-360°~0°。
		theta1=-(A1-90);
		theta2=-(A2-270);//可以看作是theta2=A2+90。即由于一条腿两个电机放置的对称性，使其坐标轴也产生了x方向左右的对称，因此A2在减去90的基础上，再减去了180。
		if(reverse_move_flag==1)
		{
			//两个角度的处理不同！
			theta1-=360;
			theta2+=360;
		}
		
		//计算后腿
		//所需腿长计算及腿长限位
    L=sqrt(pow(-x,2) + pow(y,2));
	  if(L<LegLenthMin) L=LegLenthMin;
		  else if(L>LegLenthExtremeMax) L=LegLenthExtremeMax;
		//根据腿长计算“中间角度”N和M。总体角度范围-180°~180°。
		N=asin(y/L)*180.0/PI;//角度范围为-90°~90°。
		if((-x<0)&&(y>0)) N=180-N;//角度范围为90°~180°
		else if((-x<0)&&(y<0)) N=-180-N;//角度范围为-180°~-90°
    M=acos(	( pow(L,2)+pow(L1,2)-pow(L2,2) )/(2*L1*L) )*180.0/PI;//绝对的角度大小，角度范围为0°~90°。
		//坐标转换（先不考虑offset，在最终赋值再进行调整）。
    A1=M+N-90;
    A2=90-(N-M);
		//最终确定电机角度。角度范围分别为0°~360°和-360°~0°。
		theta3=-(A1-90);
		theta4=-(A2-270);//可以看作是theta2=A2+90。即由于一条腿两个电机放置的对称性，使其坐标轴也产生了x方向左右的对称，因此A2在减去90的基础上，再减去了180。
		if(reverse_move_flag==1)
		{
			//两个角度的处理不同！
			theta3-=360;
			theta4+=360;
		}
}

/*
* NAME: SinTrajectory (float t,GaitParams params, float gaitOffset)
* FUNCTION : 正弦轨迹生成器（核心函数），用在CoupledMoveLeg函数中。
* 入口参数：
			t：心跳控制变量，用来体现时间的流逝。我们在上层参数输入时，输入的实际上是：tt=times*5/1000;即tt约每5ms变化5/1000即，0.005。
			GaitParams：步态控制
			gaitOffset：相位差，用于构成不同步态的核心参数！！
			leg_diretion：代表腿的前进或后退
			angle：
* 待优化：初始相位的控制比较重要，决定了起步的时候是否平稳！
*/
void SinTrajectory (float t,GaitParams params, float gaitOffset,float leg_diretion) 
{
	//t=times*5/1000，即每1s变化1
	//获取正弦函数的所要配置的参数
		float gp;
    float stanceHeight = params.stance_height;//狗底盘离地高度
    float downAMP = params.down_amp;//负峰值
    float upAMP = params.up_amp;//正峰值
    float flightPercent = params.flight_percent;//摆动相占比
    float stepLength = params.step_length ;//步长
    float FREQ = params.freq;//频率
		
	if(leg_diretion<0) stepLength = -stepLength;//方向控制

	/******相位（时间、周期循环）控制******/
    static float p = 0,prev_t = 0;//相位时间累计(要想实现不同腿不同频率，就不能共用一个这个，而应该将其变为腿部参数特征)。
	//由于t每次进入函数变化至少0.005，因此FREQ理论上要小于200。否则，p的变化量将大于等于1，从而导致运动出错。
	//例如当FREQ=1时，每经过1s，t变化1，而p刚好变化1，故此时频率为1Hz，当FREQ=n时，频率显然就为nHz。故频率最大为200Hz。
	//建议频率不要过大，因为频率越大意味着采样点数越少。而实际上我们不需要那么高频率，应将频率限制在0-5开区间范围内。
    p += FREQ * (t - prev_t);//频率*时间变化量即为相位变化量。p每次变化所经历的时间是固定的5ms，但我们可以通过改变每次变化的大小来间接代替变化频率。FREQ越大，单次变化的就越大。
		gp = fmod((p+gaitOffset),1.0);//该函数返回 x/y 的余数，除1.0表明取小数部分，即将gp限制在0-1范围内。
    prev_t = t;//将当前t值保存下来。
	/******正弦轨迹生成******/
	//足尖摆动相
    if (gp <= flightPercent) //gp将从gaitOffset开始，因此当gaitOffset大于flightPercent时，将直接转到支撑相。
	{
        x = (gp/flightPercent)*stepLength - stepLength/2.0f;//从-stepLength/2到+stepLength/2，移动时间不随stepLength改变，故stepLength越大实际移动速度越快。
        y = -upAMP*sin(PI*gp/flightPercent) + stanceHeight;//围绕stanceHeight为基础进行正弦波动。同样是upAMP越大移动速度越快。
    }
	//足尖支撑相
    else //摆动总是从正弦轨迹的起始位置处执行。
	{
        float percentBack = (gp-flightPercent)/(1.0f-flightPercent);//percentBack与(gp/flightPercent)是一个道理
        x = -percentBack*stepLength + stepLength/2.0f;//一般来说，首次进入时总是从stepLength/2开始，然后之后就向后运动。
        y = downAMP*sin(PI*percentBack) + stanceHeight;//
    }

}

//开启运动反向
void ReverseMoveOpen(void)
{
	reverse_move_flag=1;
}
//关闭运动反向
void ReverseMoveClose(void)
{
	reverse_move_flag=0;
}

/*
* NAME: SinTrajectory (float t,GaitParams params, float gaitOffset)
* FUNCTION : 贝塞尔轨迹生成器（核心函数），用在CoupledMoveLeg函数中。
* 入口参数：
			t：心跳控制变量，用来体现时间的流逝。我们在上层参数输入时，输入的实际上是：tt=times*5/1000;即tt约每5ms变化5/1000即，0.005。
			GaitParams：步态控制
			gaitOffset：相位差，用于构成不同步态的核心参数！！
			leg_diretion：代表腿的前进或后退
			angle：
* 待优化：初始相位的控制比较重要，决定了起步的时候是否平稳！
*/
void BeSSeLTrajectory (float t,GaitParams params, float gaitOffset,float leg_diretion) 
{
	//t=times*5/1000，即每1s变化1
	//获取正弦函数的所要配置的参数
		float gp;
    float stanceHeight = params.stance_height;//狗底盘离地高度
    float downAMP = params.down_amp;//负峰值
    float upAMP = params.up_amp;//正峰值
    float flightPercent = params.flight_percent;//摆动相占比
    float stepLength = params.step_length ;//步长
    float FREQ = params.freq;//频率
		
	if(leg_diretion<0) stepLength = -stepLength;//方向控制

	/******相位（时间、周期循环）控制******/
    static float p = 0,prev_t = 0;//相位时间累计(要想实现不同腿不同频率，就不能共用一个这个，而应该将其变为腿部参数特征)。
    p += FREQ * (t - prev_t);//频率*时间变化量即为相位变化量。p每次变化所经历的时间是固定的5ms，但我们可以通过改变每次变化的大小来间接代替变化频率。FREQ越大，单次变化的就越大。
		gp = fmod((p+gaitOffset),1.0);//该函数返回 x/y 的余数，除1.0表明取小数部分，即将gp限制在0-1范围内。
    prev_t = t;//将当前t值保存下来。

	//足尖摆动相
    if (gp <= 0.5) //gp将从gaitOffset开始，因此当gaitOffset大于flightPercent时，将直接转到支撑相。
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
	//足尖支撑相
    else //摆动总是从正弦轨迹的起始位置处执行。
	{
		
//        float percentBack = (gp-flightPercent)/(1.0f-flightPercent);//percentBack与(gp/flightPercent)是一个道理
        x = -stepLength/2+stepLength*(10*(pow(2-(2*gp),3))*(pow((2*gp)-1,2))+5*(pow(2-(2*gp),4))*((2*gp)-1)+pow(2-(2*gp),5));
        y = stanceHeight;
    }

}

void MotorInit(void)
{
	MotorParameterInit();
	
	SERVO_Send_recv(&Motor[0], &data[0]);	//将控制指令发送给电机，同时接收返回值
	omega1=data[0].Pos;
	HAL_Delay(5);
	
	SERVO_Send_recv(&Motor[1], &data[1]);	//将控制指令发送给电机，同时接收返回值
	omega2=data[1].Pos;
	HAL_Delay(5);
	
	SERVO_Send_recv(&Motor[2], &data[2]);	//将控制指令发送给电机，同时接收返回值
	omega3=data[2].Pos;
	HAL_Delay(5);
	
	SERVO_Send_recv(&Motor[3], &data[3]);	//将控制指令发送给电机，同时接收返回值
	omega4=data[3].Pos;
	HAL_Delay(5);
	
	SERVO_Send_recv(&Motor[4], &data[4]);	//将控制指令发送给电机，同时接收返回值
	omega5=data[4].Pos;
	HAL_Delay(5);
	
	SERVO_Send_recv(&Motor[5], &data[5]);	//将控制指令发送给电机，同时接收返回值
	omega6=data[5].Pos;
	HAL_Delay(5);
	
	SERVO_Send_recv(&Motor[6], &data[6]);	//将控制指令发送给电机，同时接收返回值
	omega7=data[6].Pos;
	HAL_Delay(5);
	
	SERVO_Send_recv(&Motor[7], &data[7]);	//将控制指令发送给电机，同时接收返回值
	omega8=data[7].Pos;
	HAL_Delay(5);
}

void SingleLegCoordinateControl(float set_x,float set_y,int LegId)//站立函数
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



void SetCoupledThetaPosition(int LegId)//单独版，需要给腿号
{
		switch(LegId)
	{
		case 1:
		{
		MotorParameterSet(omega1+(theta1+offset_front_1)/360*6.2832f,1,KP,KD);
		SERVO_Send_recv(&Motor[0], &data[0]);	//将控制指令发送给电机，同时接收返回值

		MotorParameterSet(omega2+(theta2+offset_front_2)/360*6.2832f,2,KP,KD);
		SERVO_Send_recv(&Motor[1], &data[1]);	//将控制指令发送给电机，同时接收返回值
		}break;
		case 2:
		{
		MotorParameterSet(omega3-(theta2+offset_front_2)/360*6.2832f,3,KP,KD);
		SERVO_Send_recv(&Motor[2], &data[2]);	//将控制指令发送给电机，同时接收返回值

		MotorParameterSet(omega4-(theta1+offset_front_1)/360*6.2832f,4,KP,KD);
		SERVO_Send_recv(&Motor[3], &data[3]);	//将控制指令发送给电机，同时接收返回值
		}break;
		case 3:
		{
		MotorParameterSet(omega5-(theta3+offset_back_1)/360*6.2832f,5,KP,KD);
		SERVO_Send_recv(&Motor[4], &data[4]);	//将控制指令发送给电机，同时接收返回值
	
		MotorParameterSet(omega6-(theta4+offset_back_2)/360*6.2832f,6,KP,KD);
		SERVO_Send_recv(&Motor[5], &data[5]);	//将控制指令发送给电机，同时接收返回值
		}break;
		case 4:
		{
		MotorParameterSet(omega7+(theta4+offset_back_2)/360*6.2832f,7,KP,KD);
		SERVO_Send_recv(&Motor[6], &data[6]);	//将控制指令发送给电机，同时接收返回值
	
		MotorParameterSet(omega8+(theta3+offset_back_1)/360*6.2832f,8,KP,KD);
		SERVO_Send_recv(&Motor[7], &data[7]);	//将控制指令发送给电机，同时接收返回值
		}break;
	}
}
