#ifndef __IMU_H__
#define __IMU_H__

void ImuInit(void);
void YawControl(float yaw_set,ALLParam *State_Detached_Params,int direction);
void PitchControl(float pitch_set,ALLParam *State_Detached_Params);
void AttitudeControl(float roll_set,float pitch_set,ALLParam *State_Detached_Params);
//void Can_Send_Msg(uint8_t ucStdId, uint8_t* msg, uint32_t len);
//void MIT_CtrlMotor(CAN_HandleTypeDef* hcan,uint16_t ID, float _pos, float _vel,float _KP, float _KD, float _torq);


#endif


