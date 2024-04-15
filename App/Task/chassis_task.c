/***      
《 底盘控制代码 》
***/
#include "chassis_task.h"
#include "Device.h"
#include "tim.h"
#include "math.h"


/* Init start */
Chassis_Date_t Chassis ={
	.State=SYSTEM_LOST,
	.Ctrl_Mode = Ctrl_Err,
	.PID_Type = Clear_Away,
};


/**
 * @brief Chassis总控
 * @param 
 */
uint16_t i;
void Chassis_Task(void)
{
	Chassis_GET_Info();
	CAN_cmd_capacitance(100);
	if(RC_S1== RC_SW_UP)
	{
		Chassis_RC_Ctrl();
//		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,i);//开启
	}
	else if(RC_S1 == RC_SW_DOWN)
	{
		Chassis_KEY_Ctrl();
	}
	else if(RC_S1 == RC_SW_MID)
	{
		Chassis_Stop();
	}
}



/**
* @brief 底盘信息获取
* @param 
*/
void Chassis_GET_Info(void)
{
	Chassis.State = System.System_State;
	Chassis.PID_Type = System.System_Pid;
	Chassis_Set_PID_Type();
	Chassis_InitPID();
}



/**
 * @brief PID种类获取
 * @note  本地同步
 */
void Chassis_Get_PID_Type(PID_Info_t *str )
{
  str->PID_Type = Chassis.PID_Type;
}

void Chassis_Set_PID_Type(void)
{
  Chassis_Get_PID_Type(&Chassis.Motor_Data[CHAS_LF].PID);
  
  Chassis_Get_PID_Type(&Chassis.Motor_Data[CHAS_RF].PID);
  
  Chassis_Get_PID_Type(&Chassis.Motor_Data[CHAS_LB].PID); 
  
  Chassis_Get_PID_Type(&Chassis.Motor_Data[CHAS_RB].PID);  
}



/**
* @brief 底盘PID设置总函数
* @param 
*/
void Chassis_InitPID(void)
{
	Chassis_PID_Switch(&Chassis.Motor_Data[CHAS_LF].PID);
	Chassis_PID_Switch(&Chassis.Motor_Data[CHAS_RF].PID);
	Chassis_PID_Switch(&Chassis.Motor_Data[CHAS_LB].PID);
	Chassis_PID_Switch(&Chassis.Motor_Data[CHAS_RB].PID);
}


/**
 * @brief PID切换
 * @param 
 */
void Chassis_PID_Switch(PID_Info_t *str)
{
  static int length = sizeof(PID_Parameter_t);
  switch (str->PID_Type)
  {
    case RC:{
      memcpy(&(str->Speed_Loop.PID_Param),&PID_Speed_Param[M_3508][RC],length);
    }break; 
    case KEY:{
      memcpy(&(str->Speed_Loop.PID_Param),&PID_Speed_Param[M_3508][KEY],length);
    }break;  
    default :{
      memcpy(&(str->Speed_Loop.PID_Param),&PID_Speed_Param[M_3508][Clear_Away],length);
		}break;  
	}
};


/**
 * @brief 遥控器
 * @param 
 */
int16_t yaw_angle_t;
#if(Robot==HERO)
void Chassis_RC_Ctrl(void)
{
	yaw_angle_t=YAW_MotorAngle_Proc(Gimbal.YAW.Motor_Data.CAN_GetData.Motor_Angle);
	if(RC_S2==1)
	{
		Chassis.RC_Move.FB_Speed = RC_CH3*6000/660;
		Chassis.RC_Move.LR_Speed = RC_CH2*6000/660;
		Chassis.RC_Move.Z_Speed  = 0;
	}
	if(RC_S2 == 3)
	{
		Chassis.RC_Move.FB_Speed = RC_CH3*6000/660;
		Chassis.RC_Move.LR_Speed = RC_CH2*6000/660;
		Chassis.RC_Move.Z_Speed  = -PID_Position(&Chassis.YAW_Follow.PID.Speed_Loop,
																							MECH_Mid_Angle,yaw_angle_t);
	}
	if(RC_S2 == 2)
	{
		static float Yaw_Angle;
		Yaw_Angle = MECH_Mid_Angle -YAW_MotorAngle_Proc( Gimbal.YAW.Motor_Data.CAN_GetData.Motor_Angle);
		Chassis.RC_Move.Z_Speed = 8000;
		Chassis.RC_Move.FB_Speed = SPIN_GetSpeed_FB(Yaw_Angle,(float)(RC_CH3*6000/660),(float)-(RC_CH2*6000/660));
		Chassis.RC_Move.LR_Speed = SPIN_GetSpeed_LR(Yaw_Angle,(float)(RC_CH3*6000/660),(float)-(RC_CH2*6000/660));
	}		
  /*PID参数调试*/
//	PID_Debug(&Chassis.Motor_Data[CHAS_LF].PID);
//	PID_Debug(&Chassis.Motor_Data[CHAS_RF].PID);
//	PID_Debug(&Chassis.Motor_Data[CHAS_LB].PID);
//	PID_Debug(&Chassis.Motor_Data[CHAS_RB].PID);

	Chassis_Handler(&Chassis.RC_Move);
}	
/**
 * @brief 键盘
 * @param 
 */
void Chassis_KEY_Ctrl(void)
{
	yaw_angle_t=YAW_MotorAngle_Proc(Gimbal.YAW.Motor_Data.CAN_GetData.Motor_Angle);

	if(KEY_W == 1){
		Chassis.KEY_Move.FB_Speed += 500;
	}
	else if(KEY_S == 1){
		Chassis.KEY_Move.FB_Speed -= 500;
	}
	else if(KEY_A == 1){
		Chassis.KEY_Move.LR_Speed -= 500;
	}
	else if(KEY_D == 1){
		Chassis.KEY_Move.LR_Speed += 500;
	}
	else{
		Chassis.KEY_Move.FB_Speed = 0;
		Chassis.KEY_Move.LR_Speed = 0;
	}
	XY_Speed(&Chassis.KEY_Move.FB_Speed,5000,-5000);
	XY_Speed(&Chassis.KEY_Move.LR_Speed,5000,-5000);
	Chassis.KEY_Move.Z_Speed  = PID_Position(&Chassis.YAW_Follow.PID.Speed_Loop,
																							MECH_Mid_Angle,yaw_angle_t);
	Chassis_Handler(&Chassis.KEY_Move);
}
#elif(Robot==Infantry_3)
void Chassis_RC_Ctrl(void)
{
	yaw_angle_t=YAW_MotorAngle_Proc(Gimbal.YAW.Motor_Data.CAN_GetData.Motor_Angle);

	if(RC_S2==1)
	{
		Chassis.RC_Move.FB_Speed = RC_CH3*8000/660;
		Chassis.RC_Move.LR_Speed = RC_CH2*8000/660;
		Chassis.RC_Move.Z_Speed  = 0;
	}
	if(RC_S2 == 3)
	{
		Chassis.RC_Move.FB_Speed = RC_CH3*6000/660;
		Chassis.RC_Move.LR_Speed = RC_CH2*6000/660;
		Chassis.RC_Move.Z_Speed  = PID_Position(&Chassis.YAW_Follow.PID.Speed_Loop,
																							MECH_Mid_Angle,yaw_angle_t);
	}
	if(RC_S2 == 2)
	{
		static float Yaw_Angle;
		Yaw_Angle = MECH_Mid_Angle -YAW_MotorAngle_Proc( Gimbal.YAW.Motor_Data.CAN_GetData.Motor_Angle);
		Chassis.RC_Move.Z_Speed = 8000;
		Chassis.RC_Move.FB_Speed = SPIN_GetSpeed_FB(Yaw_Angle,(float)(RC_CH3*6000/660),(float)(RC_CH2*6000/660));
		Chassis.RC_Move.LR_Speed = SPIN_GetSpeed_LR(Yaw_Angle,(float)(RC_CH3*6000/660),(float)(RC_CH2*6000/660));
	}		
  /*PID参数调试*/
//	PID_Debug(&Chassis.Motor_Data[CHAS_LF].PID);
//	PID_Debug(&Chassis.Motor_Data[CHAS_RF].PID);
//	PID_Debug(&Chassis.Motor_Data[CHAS_LB].PID);
//	PID_Debug(&Chassis.Motor_Data[CHAS_RB].PID);

	Chassis_Handler(&Chassis.RC_Move);
}	
/**
 * @brief 键盘
 * @param 
 */
void Chassis_KEY_Ctrl(void)
{
	yaw_angle_t=YAW_MotorAngle_Proc(Gimbal.YAW.Motor_Data.CAN_GetData.Motor_Angle);

	if(KEY_W == 1){
		Chassis.KEY_Move.FB_Speed += 500;
	}
	else if(KEY_S == 1){
		Chassis.KEY_Move.FB_Speed -= 500;
	}
	else if(KEY_A == 1){
		Chassis.KEY_Move.LR_Speed -= 500;
	}
	else if(KEY_D == 1){
		Chassis.KEY_Move.LR_Speed += 500;
	}
	else{
		Chassis.KEY_Move.FB_Speed = 0;
		Chassis.KEY_Move.LR_Speed = 0;
	}
	XY_Speed(&Chassis.KEY_Move.FB_Speed,5000,-5000);
	XY_Speed(&Chassis.KEY_Move.LR_Speed,5000,-5000);
	Chassis.KEY_Move.Z_Speed  = PID_Position(&Chassis.YAW_Follow.PID.Speed_Loop,
																							MECH_Mid_Angle,yaw_angle_t);
	Chassis_Handler(&Chassis.KEY_Move);
}

#elif(Robot==Infantry_5)
void Chassis_RC_Ctrl(void)
{
	yaw_angle_t=YAW_MotorAngle_Proc(Gimbal.YAW.Motor_Data.CAN_GetData.Motor_Angle);

	if(RC_S2==1)
	{
		Chassis.RC_Move.FB_Speed = RC_CH3*8000/660;
		Chassis.RC_Move.LR_Speed = RC_CH2*8000/660;
		Chassis.RC_Move.Z_Speed  = 0;
	}
	if(RC_S2 == 3)
	{
		Chassis.RC_Move.FB_Speed = RC_CH3*8000/660;
		Chassis.RC_Move.LR_Speed = RC_CH2*8000/660;
		Chassis.RC_Move.Z_Speed  = PID_Position(&Chassis.YAW_Follow.PID.Speed_Loop,
																							MECH_Mid_Angle,yaw_angle_t);
	}
	if(RC_S2 == 2)
	{
		static float Yaw_Angle;
		Yaw_Angle = MECH_Mid_Angle -YAW_MotorAngle_Proc( Gimbal.YAW.Motor_Data.CAN_GetData.Motor_Angle); 
		Chassis.RC_Move.Z_Speed = 2000;
		Chassis.RC_Move.FB_Speed = SPIN_GetSpeed_FB(Yaw_Angle,(float)(RC_CH3*6000/660),(float)(RC_CH2*6000/660));
		Chassis.RC_Move.LR_Speed = SPIN_GetSpeed_LR(Yaw_Angle,(float)(RC_CH3*6000/660),(float)(RC_CH2*6000/660));
	}		
  /*PID参数调试*/
//	PID_Debug(&Chassis.Motor_Data[CHAS_LF].PID);
//	PID_Debug(&Chassis.Motor_Data[CHAS_RF].PID);
//	PID_Debug(&Chassis.Motor_Data[CHAS_LB].PID);
//	PID_Debug(&Chassis.Motor_Data[CHAS_RB].PID);

	Chassis_Handler(&Chassis.RC_Move);
}	
/**
 * @brief 键盘
 * @param 
 */
void Chassis_KEY_Ctrl(void)
{
	yaw_angle_t=YAW_MotorAngle_Proc(Gimbal.YAW.Motor_Data.CAN_GetData.Motor_Angle);

	if(KEY_W == 1){
		Chassis.KEY_Move.FB_Speed += 500;
	}
	else if(KEY_S == 1){
		Chassis.KEY_Move.FB_Speed -= 500;
	}
	else if(KEY_A == 1){
		Chassis.KEY_Move.LR_Speed -= 500;
	}
	else if(KEY_D == 1){
		Chassis.KEY_Move.LR_Speed += 500;
	}
	else{
		Chassis.KEY_Move.FB_Speed = 0;
		Chassis.KEY_Move.LR_Speed = 0;
	}
	XY_Speed(&Chassis.KEY_Move.FB_Speed,5000,-5000);
	XY_Speed(&Chassis.KEY_Move.LR_Speed,5000,-5000);
	Chassis.KEY_Move.Z_Speed  = PID_Position(&Chassis.YAW_Follow.PID.Speed_Loop,
																							MECH_Mid_Angle,yaw_angle_t);
	Chassis_Handler(&Chassis.KEY_Move);
}
#endif

/**
 * @brief 得到底盘四个电机的解算速度目标值
 * @note  电机的方向控制 
 */
float Speed_Adapt(float FB,float LR,float Z)
{
  float res;
  float Adapt_K = 1;
//  if(abs(FB)+abs(LR)+abs(Z) > Speed_MAX)
//    Adapt_K = (Speed_MAX)/(abs(FB)+abs(LR)+abs(Z));
  res = FB*Adapt_K  + LR*Adapt_K  + Z*Adapt_K ;
  return res;
}
//整车速度分配到轮子速度
#if(Robot == HERO)
void Chassis_Allot_Speed(Chassis_Move_t *str)
{
	str->Motor_SpeedTarget[CHAS_LF] = Speed_Adapt(-str->FB_Speed,
																								 str->LR_Speed,	
																								 str->Z_Speed);
	str->Motor_SpeedTarget[CHAS_RF] = Speed_Adapt( str->FB_Speed,
																								 str->LR_Speed,	
																								 str->Z_Speed);
	str->Motor_SpeedTarget[CHAS_LB] = Speed_Adapt( str->FB_Speed,
																								-str->LR_Speed,	
																								 str->Z_Speed);
	str->Motor_SpeedTarget[CHAS_RB] = Speed_Adapt(-str->FB_Speed,
																								-str->LR_Speed,	
																								 str->Z_Speed);
}
#elif(Robot==Infantry_3)
void Chassis_Allot_Speed(Chassis_Move_t *str)
{
	str->Motor_SpeedTarget[CHAS_LF] = Speed_Adapt( str->FB_Speed,
																								 str->LR_Speed,	
																								 str->Z_Speed);
	str->Motor_SpeedTarget[CHAS_RF] = Speed_Adapt(-str->FB_Speed,
																								 str->LR_Speed,	
																								 str->Z_Speed);
	str->Motor_SpeedTarget[CHAS_LB] = Speed_Adapt( str->FB_Speed,
																								-str->LR_Speed,	
																								 str->Z_Speed);
	str->Motor_SpeedTarget[CHAS_RB] = Speed_Adapt(-str->FB_Speed,
																								-str->LR_Speed,	
																								 str->Z_Speed);
}
#elif(Robot==Infantry_5)
void Chassis_Allot_Speed(Chassis_Move_t *str)
{
	str->Motor_SpeedTarget[CHAS_LF] = Speed_Adapt( str->LR_Speed,
																								 str->FB_Speed,	
																								 str->Z_Speed);
	str->Motor_SpeedTarget[CHAS_RF] = Speed_Adapt(-str->LR_Speed,
																								 str->FB_Speed,	
																								 str->Z_Speed);
	str->Motor_SpeedTarget[CHAS_LB] = Speed_Adapt(-str->LR_Speed,
																								-str->FB_Speed,	
																								 str->Z_Speed);
	str->Motor_SpeedTarget[CHAS_RB] = Speed_Adapt( str->LR_Speed,
																								-str->FB_Speed,	
																								 str->Z_Speed);
}
#endif

/**
 * @brief 将MMKNM的速度解算 斜坡分给四个电机的PID目标值
 * @param 
 */
void Chassis_MotorSpeed(Chassis_Move_t *str)
{
	Chassis.Motor_Data[CHAS_LF].Motor_Data.PID_Speed_target = str->Motor_SpeedTarget[CHAS_LF];
	Chassis.Motor_Data[CHAS_RF].Motor_Data.PID_Speed_target = str->Motor_SpeedTarget[CHAS_RF];
	Chassis.Motor_Data[CHAS_LB].Motor_Data.PID_Speed_target = str->Motor_SpeedTarget[CHAS_LB];
	Chassis.Motor_Data[CHAS_RB].Motor_Data.PID_Speed_target = str->Motor_SpeedTarget[CHAS_RB];
}


/**
 * @brief PID输出
*/
float Chassis_GetOutPut(Chassis_Motor_t *str)
{
	PID_Info_t *PID = &(str->PID);
//  Motor_Data_t *DATA = &(str->Motor_Data);
  float res;
	  /*返回值 = 速度环PID输出*/
  res = PID_Position(&PID->Speed_Loop,  										 \
										str->Motor_Data.PID_Speed_target,				 \
										str->Motor_Data.CAN_GetData.Motor_Speed);\
  return res;
}

/**
 * @brief Chassis电机输出
 * @param 
 */
void Chassis_CanOutPut(void)
{
	static int16_t pid_out[CHAS_MOTOR_CNT] = {0,0,0,0};
	pid_out[CHAS_LF] = (int16_t)Chassis_GetOutPut(&Chassis.Motor_Data[CHAS_LF]);
	pid_out[CHAS_RF] = (int16_t)Chassis_GetOutPut(&Chassis.Motor_Data[CHAS_RF]);
	pid_out[CHAS_LB] = (int16_t)Chassis_GetOutPut(&Chassis.Motor_Data[CHAS_LB]);
	pid_out[CHAS_RB] = (int16_t)Chassis_GetOutPut(&Chassis.Motor_Data[CHAS_RB]);
	CAN_cmd_chassis(pid_out[CHAS_LF],pid_out[CHAS_RF],pid_out[CHAS_LB],pid_out[CHAS_RB]);
}

/**
 * @brief Chassis输出调用
 * @param 
 */
void Chassis_Handler(Chassis_Move_t *str)
{
	Chassis_Allot_Speed(str);
	Chassis_MotorSpeed(str);
	Chassis_CanOutPut();
}


/**
 * @brief 卸力函数
 * @param 
 */
void Chassis_Stop(void)
{
	static int16_t pid_out[CHAS_MOTOR_CNT] = {0, 0, 0, 0};
	
	/* 速度环最终输出 */
  pid_out[CHAS_LF] = 0;
  pid_out[CHAS_RF] = 0;
  pid_out[CHAS_LB] = 0;
  pid_out[CHAS_RB] = 0;
  
	CAN_cmd_chassis(pid_out[CHAS_LF],pid_out[CHAS_RF],pid_out[CHAS_LB],pid_out[CHAS_RB]);	  
}

#if(Robot==HERO)
float SPIN_GetSpeed_LR(float Angle , float FB_ctrl , float LR_ctrl)
{
  float res;
  res = cos(Angle*2*3.1415926f/8192.f) * LR_ctrl - sin(Angle*2*3.14f/8192.f) * FB_ctrl;
  return res;
}
float SPIN_GetSpeed_FB(float Angle , float FB_ctrl , float LR_ctrl)
{
  float res;
  res = sin(Angle*2*3.1415926f/8192.f) * LR_ctrl + cos(Angle*2*3.14f/8192.f) * FB_ctrl;
  return res;  
}
#elif(Robot==Infantry_3)
float SPIN_GetSpeed_LR(float Angle , float FB_ctrl , float LR_ctrl)
{
  float res;
  res = cos(Angle*2*3.1415926f/8192.f) * LR_ctrl + sin(Angle*2*3.14f/8192.f) * FB_ctrl;
  return res;
}
float SPIN_GetSpeed_FB(float Angle , float FB_ctrl , float LR_ctrl)
{
  float res;
  res = -sin(Angle*2*3.1415926f/8192.f) * LR_ctrl + cos(Angle*2*3.14f/8192.f) * FB_ctrl;
  return res;  
}
#elif(Robot==Infantry_5)
float SPIN_GetSpeed_LR(float Angle , float FB_ctrl , float LR_ctrl)
{
  float res;
  res = cos(Angle*2*3.1415926f/8192.f) * LR_ctrl + sin(Angle*2*3.14f/8192.f) * FB_ctrl;
  return res;
}
float SPIN_GetSpeed_FB(float Angle , float FB_ctrl , float LR_ctrl)
{
  float res;
  res = -sin(Angle*2*3.1415926f/8192.f) * LR_ctrl + cos(Angle*2*3.14f/8192.f) * FB_ctrl;
  return res;  
}
#endif


void XY_Speed(float *speed, float MAX,float MIX)
{
	if(*speed > MAX){
		*speed = MAX;
	}
	if(*speed < MIX){ 
	*speed = MIX;
	}
}
