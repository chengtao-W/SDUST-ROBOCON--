#include "rc_dog.h" 
	
MOTOR_send Motor[8];    //以全局变量声明电机控制结构体和电机数据结构体，方便在故障时通过debug查看变量值
MOTOR_recv data[8];


void MotorParameterInit(void)
{
	
		Motor[0].id=0; 			//给电机控制指令结构体赋值
		Motor[0].mode=0;
		Motor[0].T=0;
		Motor[0].W=0;
		Motor[0].Pos=0;
		Motor[0].K_P=0;
		Motor[0].K_W=0;
	
		Motor[1].id=1; 			//给电机控制指令结构体赋值
		Motor[1].mode=0;
		Motor[1].T=0;
		Motor[1].W=0;
		Motor[1].Pos=0;
		Motor[1].K_P=0;
		Motor[1].K_W=0;
	
	
		Motor[2].id=2; 			//给电机控制指令结构体赋值
		Motor[2].mode=0;
		Motor[2].T=0;
		Motor[2].W=0;
		Motor[2].Pos=0;
		Motor[2].K_P=0;
		Motor[2].K_W=0;
		
		
		Motor[3].id=3; 			//给电机控制指令结构体赋值
		Motor[3].mode=0;
		Motor[3].T=0;
		Motor[3].W=0;
		Motor[3].Pos=0;
		Motor[3].K_P=0;
		Motor[3].K_W=0;
		
		Motor[4].id=4; 			//给电机控制指令结构体赋值
		Motor[4].mode=0;
		Motor[4].T=0;
		Motor[4].W=0;
		Motor[4].Pos=0;
		Motor[4].K_P=0;
		Motor[4].K_W=0;
		
		Motor[5].id=5; 			//给电机控制指令结构体赋值
		Motor[5].mode=0;
		Motor[5].T=0;
		Motor[5].W=0;
		Motor[5].Pos=0;
		Motor[5].K_P=0;
		Motor[5].K_W=0;
		
		Motor[6].id=6; 			//给电机控制指令结构体赋值
		Motor[6].mode=0;
		Motor[6].T=0;
		Motor[6].W=0;
		Motor[6].Pos=0;
		Motor[6].K_P=0;
		Motor[6].K_W=0;
		
		Motor[7].id=7; 			//给电机控制指令结构体赋值
		Motor[7].mode=0;
		Motor[7].T=0;
		Motor[7].W=0;
		Motor[7].Pos=0;
		Motor[7].K_P=0;
		Motor[7].K_W=0;
		
}

void MotorParameterSet(float angle,uint8_t id,float KP,float KD)
{
	switch(id)
	{
		case 1 :
		{
			Motor[0].id=0; 			//给电机控制指令结构体赋值
			Motor[0].mode=1;
			Motor[0].T=0.0;
			Motor[0].W=0.0;
			Motor[0].Pos=angle;
			Motor[0].K_P=KP;
			Motor[0].K_W=KD;
		}
		case 2 :
		{
			Motor[1].id=1; 			//给电机控制指令结构体赋值
			Motor[1].mode=1;
			Motor[1].T=0.0;
			Motor[1].W=0.0;
			Motor[1].Pos=angle;
			Motor[1].K_P=KP;
			Motor[1].K_W=KD;
		}
		case 3 :
		{
			Motor[2].id=2; 			//给电机控制指令结构体赋值
			Motor[2].mode=1;
			Motor[2].T=0.0;
			Motor[2].W=0.0;
			Motor[2].Pos=angle;
			Motor[2].K_P=KP;
			Motor[2].K_W=KD;
		}
		case 4 :
		{
			Motor[3].id=3; 			//给电机控制指令结构体赋值
			Motor[3].mode=1;
			Motor[3].T=0.0;
			Motor[3].W=0.0;
			Motor[3].Pos=angle;
			Motor[3].K_P=KP;
			Motor[3].K_W=KD;
		}
		case 5 :
		{
			Motor[4].id=4; 			//给电机控制指令结构体赋值
			Motor[4].mode=1;
			Motor[4].T=0.0;
			Motor[4].W=0.0;
			Motor[4].Pos=angle;
			Motor[4].K_P=KP;
			Motor[4].K_W=KD;
		}
		case 6 :
		{
			Motor[5].id=5; 			//给电机控制指令结构体赋值
			Motor[5].mode=1;
			Motor[5].T=0.0;
			Motor[5].W=0.0;
			Motor[5].Pos=angle;
			Motor[5].K_P=KP;
			Motor[5].K_W=KD;
		}
		case 7 :
		{
			Motor[6].id=6; 			//给电机控制指令结构体赋值
			Motor[6].mode=1;
			Motor[6].T=0.0;
			Motor[6].W=0.0;
			Motor[6].Pos=angle;
			Motor[6].K_P=KP;
			Motor[6].K_W=KD;
		}
		case 8 :
		{
			Motor[7].id=7; 			//给电机控制指令结构体赋值
			Motor[7].mode=1;
			Motor[7].T=0.0;
			Motor[7].W=0.0;
			Motor[7].Pos=angle;
			Motor[7].K_P=KP;
			Motor[7].K_W=KD;
		}
	}
}
