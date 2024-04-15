#ifndef __PID_H_
#define __PID_H_
/************************************** Includes **************************************/
//#include "Device.h"

//#include "gimbal_task.h"
/********************************* Exported functions *********************************/

typedef enum
{
	Clear_Away  =0,	    //清除
	KEY         =1,			//键盘
	RC          =2,			//遥控
	PID_TYPE_CNT,
}PID_Type_t;


/* PID参数 */
typedef __packed struct 
{
  float P,I,D;
}PID_Parameter_t;


typedef __packed struct
{
  PID_Parameter_t PID_Param;
	float Target;
	float Actual;
  float Err;
  float Last_Err;
	
  float I_Limit;				//I积分限幅
  float I_Limit_Max;	
	
  float PID_P_Out;
  float PID_I_Out;
  float PID_D_Out;
  float PID_Output;
  float PID_Err_Dead;			//死区
  float PID_Output_Max;		//输出限幅
  float PID_I_Out_Max;		
  float PID_P_Out_Max;
}PID_Loop_t;

/*PID数据结构体*/
typedef struct 
{
  PID_Loop_t Speed_Loop;
	PID_Loop_t Angle_Loop;
  PID_Type_t PID_Type;
}PID_Info_t;


#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

void PID_Debug(PID_Info_t *str);

float PID_Position(PID_Loop_t *pid, float target, float actual);

void Motor_Init(void);
		
void M_3508_Init(void);
void M_3508_ParamInit(PID_Loop_t *str);
		
//void GIM_YawParamInit(Gimbal_Info_t *str);
//void GIM_PitParamInit(Gimbal_Info_t *str);
void YAW_Follow_PitParamInit(PID_Loop_t *str);

void GM_6020_Init(void);
		
void M_2006_Init(void);
void M_2006_ParamInit(void);
void Fric_3508_Init(void);

#endif
