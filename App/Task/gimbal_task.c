/***      
�� ��̨���ƴ��� ��
***/
#include "gimbal_task.h"
#include "Device.h"
#include "math.h"



/* Init start */
Gimbal_Date_t Gimbal ={
	.State = SYSTEM_LOST,
	.Ctrl_Mode = Ctrl_Err,
	.PID_Type = Clear_Away,		//Clear_Away
};

/*Init end*/

static bool fire_flag = 0; // �����־λ
static uint32_t start_fire_time = 0; // ��ʼ�����ʱ��
extern int16_t Shoot_pid_out;				//����M2006

static PID_Type_t YAW_prev_pid_type = Clear_Away; 
/**
 * @brief Gimbal�ܿ�
 * @param 
 */
void Gimbal_Task(void)
{
	Gimbal_GET_Info();

	if(RC_S1== RC_SW_UP)
	{
		Gimbal_RC_Ctrl();
	}
	else if(RC_S1 == RC_SW_DOWN)
	{
		Gimbal_KRY_Ctrl();
	}
	else if(RC_S1 == RC_SW_MID)
	{
			if(YAW_prev_pid_type != Gimbal.PID_Type)
		{/* ��һ�ν���ģʽ */
			YAW_prev_pid_type = Gimbal.PID_Type;
		}
		Gimbal_Stop();
	}
}


/**
 * @brief ��̨��Ϣ��ȡ
 * @param 
 */
void Gimbal_GET_Info(void)
{
	Gimbal.State = System.System_State;
	Gimbal.PID_Type = System.System_Pid;
	Gimbal_Set_PID_type();
	Gimbal_InitPID();
	IMU_Data_Report();
}



/**
 * @brief PID�����ȡ
 * @note  ����ͬ��
 */
void Gimbal_Get_PID_Type(Gimbal_Info_t *strGim)
{
  strGim->PID.PID_Type = Gimbal.PID_Type;
}

void Gimbal_Set_PID_type(void)
{
  Gimbal_Get_PID_Type(&Gimbal.YAW);
  Gimbal_Get_PID_Type(&Gimbal.PITCH);    
}

/**
 * @brief ��̨PID�����ܺ���
 * @param 
 */
void Gimbal_InitPID(void)
{  
  Gimbal_PID_Switch(&Gimbal.YAW);
  Gimbal_PID_Switch(&Gimbal.PITCH); 
}



/**
 * @brief ��̨PID�л�
 * @param 
 */
void Gimbal_PID_Switch(Gimbal_Info_t *strGim)
{
	PID_Info_t *str = &(strGim->PID);
	Motor_Info_t *strx = &(strGim->Motor_Info);

	static int length = sizeof(PID_Parameter_t);
 switch (strx->Motor_Type)
 {
	case GM_6020_YAW:{
		switch (str->PID_Type)
      {
        case RC:{
          memcpy(&(str->Speed_Loop.PID_Param),&PID_Speed_Param[GM_6020_YAW][RC],length);
          memcpy(&(str->Angle_Loop.PID_Param),&PID_Angle_Param[GM_6020_YAW][RC],length);
        }break;
        case KEY:{
          memcpy(&(str->Speed_Loop.PID_Param),&PID_Speed_Param[GM_6020_YAW][KEY],length);
          memcpy(&(str->Angle_Loop.PID_Param),&PID_Angle_Param[GM_6020_YAW][KEY],length);
        }break;
         default:{
          memcpy(&(str->Speed_Loop.PID_Param),&PID_Speed_Param[GM_6020_YAW][Clear_Away],length);
          memcpy(&(str->Angle_Loop.PID_Param),&PID_Angle_Param[GM_6020_YAW][Clear_Away],length);
        }break;
			}
	}
	 case GM_6020_PIT:{
		 	switch (str->PID_Type)
      {
        case RC:{
          memcpy(&(str->Speed_Loop.PID_Param),&PID_Speed_Param[GM_6020_PIT][RC],length);
          memcpy(&(str->Angle_Loop.PID_Param),&PID_Angle_Param[GM_6020_PIT][RC],length);
        }break;
        case KEY:{
          memcpy(&(str->Speed_Loop.PID_Param),&PID_Speed_Param[GM_6020_PIT][KEY],length);
          memcpy(&(str->Angle_Loop.PID_Param),&PID_Angle_Param[GM_6020_PIT][KEY],length);
        }break;
         default:{
          memcpy(&(str->Speed_Loop.PID_Param),&PID_Speed_Param[GM_6020_PIT][Clear_Away],length);
          memcpy(&(str->Angle_Loop.PID_Param),&PID_Angle_Param[GM_6020_PIT][Clear_Away],length);
        }break;
			}
   }	 
	}
}




/**
 * @brief ң����������̨
 * @param 
 */
void Gimbal_RC_Ctrl(void)
{
	RC_Ctrl_YAW(&Gimbal.YAW);
	RC_Ctrl_PITCH(&Gimbal.PITCH);

	Gimbal_CanOutPut();
}

/**
 * @brief Gimbal_YAW��Ŀ��� ң����ң��ģʽ
 * @param 
 */
void RC_Ctrl_YAW(Gimbal_Info_t *str)
{
//	PID_Debug(&str->PID);
	if(YAW_prev_pid_type != Gimbal.PID_Type)
		{/* ��һ�ν���ģʽ */
			YAW_prev_pid_type = Gimbal.PID_Type;
			Gimbal_FirstYawAngle(&Gimbal.YAW);
		}
	
	str->RC_Move.Angle_Inc = str->RC_Move.Angle_k * (-RC_CH0);
		
	str->Motor_Data.PID_Angle_target += str->RC_Move.Angle_Inc;
//	if(RC_S2 == 1){
//	str->Motor_Data.PID_Angle_target = 100;}
//	if(RC_S2 == 3){
//	str->Motor_Data.PID_Angle_target = 80;}
//	if(RC_S2 == 2){
//	str->Motor_Data.PID_Angle_target = 60;}
	YAW_Angle_Over_Zero(&str->Motor_Data.PID_Angle_target);
	
	/*ͬ����������YAW����*/
	str->Motor_Data.PID_Speed = str->Motor_Data.IMU_GetData.IMU_Speed;
	str->Motor_Data.PID_Angle = str->Motor_Data.IMU_GetData.IMU_Angle;
}
/**
 * @brief Gimbal_PITCH��Ŀ���ң����ң��ģʽ
 * @param 
 */
void RC_Ctrl_PITCH(Gimbal_Info_t *str)
{
//	PID_Debug(&str->PID);
	if(YAW_prev_pid_type != Gimbal.PID_Type)
	{/* ��һ�ν���ģʽ */
		YAW_prev_pid_type = Gimbal.PID_Type;
		Gimbal_FirstPitchAngle(&Gimbal.PITCH);
	}

	str->RC_Move.Angle_Inc = str->RC_Move.Angle_k * RC_CH1;
	str->Motor_Data.PID_Angle_target += str->RC_Move.Angle_Inc;
//	if(RC_S2 == 1){
//	str->Motor_Data.PID_Angle_target = -20;}
//	if(RC_S2 == 3){
//	str->Motor_Data.PID_Angle_target = 0;}
//	if(RC_S2 == 2){
//	str->Motor_Data.PID_Angle_target = 20;}
	PITCH_Angle_Limit(&str->Motor_Data.PID_Angle_target);
	
	/*ͬ����������YAW����*/
	str->Motor_Data.PID_Speed = str->Motor_Data.IMU_GetData.IMU_Speed;
	str->Motor_Data.PID_Angle = str->Motor_Data.IMU_GetData.IMU_Angle;
}

/**
 * @brief ���������̨
 * @param 
 */
void Gimbal_KRY_Ctrl(void)
{
	KEY_Ctrl_YAW(&Gimbal.YAW);
	KEY_Ctrl_PITCH(&Gimbal.PITCH);
	
	Gimbal_CanOutPut();
}
/**
 * @brief Gimbal_YAW��Ŀ��� KEYң��ģʽ
 * @param 
 */
extern struct SolveTrajectoryParams st;
extern Vision_Info_t Vision_cj;
void KEY_Ctrl_YAW(Gimbal_Info_t *str)
{
	if(YAW_prev_pid_type != Gimbal.PID_Type)
		{/* ��һ�ν���ģʽ */
			YAW_prev_pid_type = Gimbal.PID_Type;
			Gimbal_FirstYawAngle(&Gimbal.YAW);
		}
	if(MOUSE_RIGH == 0)
	{		
		str->KEY_Move.Angle_Inc = str->KEY_Move.Angle_k * (MOUSE_X_MOVE_SPEED);
		str->Motor_Data.PID_Angle_target -= str->KEY_Move.Angle_Inc;
	}
	else/*��������*/
	{
		if(Vision_cj.VisionRTx.AutoAim_Rx.Packet.RxData.tracking == 0)
		{
			str->KEY_Move.Angle_Inc = str->KEY_Move.Angle_k * (MOUSE_X_MOVE_SPEED);
			str->Motor_Data.PID_Angle_target -= str->KEY_Move.Angle_Inc;
		}
		else if(Vision_cj.VisionRTx.AutoAim_Rx.Packet.RxData.tracking == 1)
		{
			str->Motor_Data.PID_Angle_target = st.yaw;
		}

	}
	YAW_Angle_Over_Zero(&str->Motor_Data.PID_Angle_target);
	/*ͬ����������YAW����*/
	str->Motor_Data.PID_Speed = str->Motor_Data.IMU_GetData.IMU_Speed;
	str->Motor_Data.PID_Angle = str->Motor_Data.IMU_GetData.IMU_Angle;
}

/**
 * @brief Gimbal_PITCH��Ŀ���KEYң��ģʽ
 * @param 
 */

void KEY_Ctrl_PITCH(Gimbal_Info_t *str)
{
	if(YAW_prev_pid_type != Gimbal.PID_Type)
	{/* ��һ�ν���ģʽ */
		YAW_prev_pid_type = Gimbal.PID_Type;
		Gimbal_FirstPitchAngle(&Gimbal.PITCH);
	}
	if(MOUSE_RIGH == 0)
	{
		str->RC_Move.Angle_Inc = str->RC_Move.Angle_k * MOUSE_Y_MOVE_SPEED;
		str->Motor_Data.PID_Angle_target -= str->RC_Move.Angle_Inc;
	}
	else/*��������*/
	{
		if(Vision_cj.VisionRTx.AutoAim_Rx.Packet.RxData.tracking == 0)
		{
			str->KEY_Move.Angle_Inc = str->KEY_Move.Angle_k * (MOUSE_X_MOVE_SPEED);
			str->Motor_Data.PID_Angle_target -= str->KEY_Move.Angle_Inc;
		}
		else if(Vision_cj.VisionRTx.AutoAim_Rx.Packet.RxData.tracking == 1)
		{
			str->Motor_Data.PID_Angle_target = st.pitch ;
			fire_flag = JudgeFireFlag(&Gimbal);
//			if(fire_flag == 1)
//			{
//				Shoot_pid_out = (int16_t)Shoot_GetOutPut(&Shoot.Motor_Data[DRIVER]);
//			}
			
		}
	}
}

/**
 * @brief ����YAW����洦��
 * @param 
 */
float YAW_MotorAngle_Proc(int16_t Angle)
{
	if(Angle < MECH_Mid_Angle - Motor_180)
		Angle += Motor_360;
	if(Angle > MECH_Mid_Angle + Motor_180)
		Angle -= Motor_360;
	
  return (float)Angle;
}

/**
 * @brief YAW��Ƕȹ��㴦��
 * @param 
 */
float YAW_Angle_Over_Zero(float *Angle)
{
	if(*Angle - Gimbal.YAW.Motor_Data.IMU_GetData.IMU_Angle > IMU_180)    
	{
		*Angle =*Angle - IMU_360;        
	}
	else if(*Angle - Gimbal.YAW.Motor_Data.IMU_GetData.IMU_Angle < -IMU_180)
	{
		*Angle =*Angle + IMU_360;
	}
	return *Angle;
}


/**
 * @brief PICTH��Ƕ���λ
 * @param 
 */
float PITCH_Angle_Limit(float *Angle)
{
	if(*Angle > PITCH_MAX)    
	{
		*Angle = PITCH_MAX;        
	}
	if(*Angle < PITCH_MIX)
	{
		*Angle =PITCH_MIX;
	}
	return *Angle;
}





/**
 * @brief ��ȡ����PID���
 * @param 
 */
float Gimbal_GetOutPut(Gimbal_Info_t *str)
{
  PID_Info_t *PID = &(str->PID);
  Motor_Data_t *DATA = &(str->Motor_Data);
  float res;
   
  /*�ٶȻ����� = �ǶȻ�PID���*/
  DATA->PID_Speed_target = PID_Position(&PID->Angle_Loop,
																				str->Motor_Data.PID_Angle_target ,\
																				str->Motor_Data.PID_Angle); \
  /*����ֵ = �ٶȻ�PID���*/
  res = PID_Position(&PID->Speed_Loop,
										str->Motor_Data.PID_Speed_target,\
										str->Motor_Data.PID_Speed);\
  return res;
}



/**
 * @brief Gimbal������
 * @param 
 */
extern int16_t Shoot_pid_out;
void Gimbal_CanOutPut(void)
{
	static int16_t Gimbal_pid_out[4] = {0,0,0,0};

	Gimbal_pid_out[GIM_YAW]	= Gimbal_GetOutPut(&Gimbal.YAW);
	Gimbal_pid_out[GIM_PITCH]= Gimbal_GetOutPut(&Gimbal.PITCH);
	
#if(Robot==HERO)
	CAN_cmd_gimbal_yaw(-Gimbal_pid_out[GIM_YAW]);
	CAN_cmd_gimbal_pitch(Gimbal_pid_out[GIM_PITCH],Gimbal_pid_out[3]);
#elif(Robot==Infantry_3)
	CAN_cmd_gimbal_yaw(Gimbal_pid_out[GIM_YAW]);
	CAN_cmd_gimbal_pitch(Gimbal_pid_out[GIM_PITCH],Shoot_pid_out);
#elif(Robot==Infantry_5)
	CAN_cmd_gimbal_yaw(Gimbal_pid_out[GIM_YAW]);
	CAN_cmd_gimbal_pitch(-Gimbal_pid_out[GIM_PITCH],Shoot_pid_out);
#endif
}









/**
 * @brief ж������
 * @param 
 */
void Gimbal_Stop(void)
{
	static int16_t pid_out[2] = {0, 0};
	
	/* �ٶȻ�������� */
  pid_out[GIM_YAW] 	 = 0;
  pid_out[GIM_PITCH] = 0;
  
	CAN_cmd_gimbal_yaw(pid_out[GIM_YAW]);
	CAN_cmd_gimbal_pitch(pid_out[GIM_PITCH],0);
}
/**
 * @brief ��ʼ��������ģʽYAW��Ƕ�
 * @param 
 */
void Gimbal_FirstYawAngle(Gimbal_Info_t *str)
{
  str->Motor_Data.PID_Angle_target = str->Motor_Data.IMU_GetData.IMU_Angle;
}

/**
 * @brief ��ʼ��������ģʽPITCH��Ƕ�
 * @param 
 */
void Gimbal_FirstPitchAngle(Gimbal_Info_t *str)
{
  str->Motor_Data.PID_Angle_target = str->Motor_Data.IMU_GetData.IMU_Angle;
}


/**
 * @brief ��ȡIMU����
 * @param 
 */
void IMU_YawData_Report(Motor_Data_t *str)
{
  str->IMU_GetData.IMU_Speed = IMU_Get_Data.IMU_Gyo[Z]/10;
  str->IMU_GetData.IMU_Angle = IMU_Get_Data.IMU_Eular[YAW];
}
void IMU_PitData_Report(Motor_Data_t *str)
{
  str->IMU_GetData.IMU_Speed = IMU_Get_Data.IMU_Gyo[Y]/10;
  str->IMU_GetData.IMU_Angle = IMU_Get_Data.IMU_Eular[PITCH];
}
void IMU_Data_Report(void)
{
  IMU_YawData_Report(&Gimbal.YAW.Motor_Data);
  IMU_PitData_Report(&Gimbal.PITCH.Motor_Data);
}




/**
 * @brief �����Ƿ���Ͽ�������
 * @param AimTra:in ��׼�ĵ�������
 * @param gyro_angle_rad:in ��ǰ������������
 */

bool JudgeFireFlag(Gimbal_Date_t *AimTra)
{
    // �����ﷶΧ��ʱ���¿�ʼ����ʱ��
    // ��֮���0.5s��500ms���ڱ��ֿ���λΪtrue
    float error_pitch, error_yaw;
    error_pitch = fabs(AimTra->PITCH.Motor_Data.PID_Angle - AimTra->PITCH.Motor_Data.PID_Angle_target);
    error_yaw = fabs(AimTra->YAW.Motor_Data.PID_Angle - AimTra->YAW.Motor_Data.PID_Angle_target);
    bool fire = 0;
    uint32_t now_time = HAL_GetTick();
    if ((error_pitch < 1.0f) &&
        (error_yaw < 1.0f))
    {
        fire = true;
        start_fire_time = now_time;
    }
    else if (now_time - start_fire_time < 500)
    {
        fire = true;
    }
    else
    {
        fire = false;
    }

    return fire;
}


/**
 * @brief   ��ȡ�����־��(1���Կ���0�����Կ���)
 * @param   none
 * @return  �����־��
 */
bool GetFireFlag(void)
{
    return fire_flag;
}






//void yaw_pitch_gimbal(void)
//{
//	if(Yaw_Start == true) //������,Yaw�������нǶ�
//	{		
//		yaw=Eular[2];
//		Yaw_Start = false;
//	}
//	yaw_limit(&yaw);
//	PID_calc(&pid_gimbal[1],Eular[2],yaw);
//	PID_calc(&pid_gimbal[2],Gyo[2]/10,pid_gimbal[1].out);
//	//Pitch
//	pitch_limit(&pitach,25,-23);
//	PID_calc(&pid_gimbal[3],Eular[0],pitach);//pitach
//	PID_calc(&pid_gimbal[4],Gyo[1]/10,pid_gimbal[3].out);
//}

//void visual_giambal(void)
//{		
//		
//	visual_yaw=Eular[2] - Visual_Data.Vision_Yaw*0.7f;
//	yaw_limit(&visual_yaw);	
//	PID_calc(&pid_gimbal[1],Eular[2],visual_yaw);
//	PID_calc(&pid_gimbal[2],Gyo[2]/10,pid_gimbal[1].out);
//Pitch
//	visual_pitch=Eular[0] + Visual_Data.Vision_Pitch*0.5f;
//	pitch_limit(&visual_pitch,25,-23);
//	PID_calc(&pid_gimbal[3],Eular[0],visual_pitch);
//	PID_calc(&pid_gimbal[4],Gyo[1]/10,pid_gimbal[3].out);
//	 
//}


//void giambal_mode(void)	
//{
//	if(rc_ctrl.mouse.press_r==0)
//	{
//		yaw_pitch_gimbal();
//	}
//	else
//	{
//		if(Visual_Date_t==1)
//		{
//			visual_giambal();
//			yaw=Eular[2];
//			pitach=Eular[0];
//		}
//		else
//		{
//			yaw_pitch_gimbal();
//		}
//	}
