#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H
/************************************** Includes **************************************/
//#include "can_receive.h"

#include "Device.h"
#include "system.h"
//#include "pid.h"


/********************************* Exported functions *********************************/
/* 底盘电机数组下标 */
typedef enum 
{
  CHAS_LF = 0,			//左前
  CHAS_RF = 1,			//右前
  CHAS_LB = 2,			//左后
  CHAS_RB = 3,			//右后
  CHAS_MOTOR_CNT = 4,
}CHAS_Motor_cnt_t;

/*地盘模式*/
typedef enum 
{
	COMMON = 0,
	
}Chassis_Mode;

typedef struct 
{
  float FB_Speed,LR_Speed,Z_Speed ;
//  float FB_Speed_k,LR_Speed_k,Z_Speed_k; //遥控->速度 变化的幅度调节因子
  float Motor_SpeedTarget[CHAS_MOTOR_CNT];
}Chassis_Move_t;






typedef struct 
{
 	Motor_Data_t	Motor_Data;
  PID_Info_t		PID;
}Chassis_Motor_t;

/* 底盘结构体 */
typedef struct
{
  System_State_t State;
	System_Ctrl_Mode_t Ctrl_Mode;
	Motor_Info_t  Motor_Info;
	Chassis_Motor_t Motor_Data[CHAS_MOTOR_CNT];
	Chassis_Motor_t YAW_Follow;
	PID_Type_t PID_Type;
	
	
	Chassis_Move_t RC_Move;
	Chassis_Move_t KEY_Move;

}Chassis_Date_t;




extern Chassis_Date_t Chassis;


void Chassis_Task(void);
void Chassis_GET_Info(void);
void Chassis_Get_PID_Type(PID_Info_t *str);
void Chassis_Set_PID_Type(void);
void Chassis_InitPID(void);
void Chassis_PID_Switch(PID_Info_t *str);
void Chassis_RC_Ctrl(void);
void Chassis_KEY_Ctrl(void);

float Speed_Adapt(float FB,float LR,float Z);
void Chassis_Allot_Speed(Chassis_Move_t *str);
void Chassis_MotorSpeed(Chassis_Move_t *str);

float Chassis_GetOutPut(Chassis_Motor_t *str);
void Chassis_CanOutPut(void);
void Chassis_Handler(Chassis_Move_t *str);

void Chassis_Stop(void);
float SPIN_GetSpeed_LR(float Angle , float FB_ctrl , float LR_ctrl);

float SPIN_GetSpeed_FB(float Angle , float FB_ctrl , float LR_ctrl);
void XY_Speed(float *speed, float MAX,float MIX);

#endif
