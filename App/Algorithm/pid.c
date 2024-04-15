#include "pid.h"
#include "Device.h"


//反限幅（死区）
float anti_constrain(float amt,float high,float low)
{
	if (amt > low && amt < high)
		return 0;
	else
		return amt;
}


/* 用于调试PID参数的函数 */
PID_Parameter_t PID_SpeedDebug = {
  .P = 0,
  .I = 0,
  .D = 0,
};
PID_Parameter_t PID_AngleDebug = {
  .P = 0,
  .I = 0,
  .D = 0,
};
void PID_Debug(PID_Info_t *str)
{
  str->Speed_Loop.PID_Param.P = PID_SpeedDebug.P;
  str->Speed_Loop.PID_Param.I = PID_SpeedDebug.I;
  str->Speed_Loop.PID_Param.D = PID_SpeedDebug.D;
  
  str->Angle_Loop.PID_Param.P = PID_AngleDebug.P;
  str->Angle_Loop.PID_Param.I = PID_AngleDebug.I;
  str->Angle_Loop.PID_Param.D = PID_AngleDebug.D;
}
			
/**
  * @brief  PID(速度/角度)环控制器
  * @param  目标-真实
  * @retval None
  */
float PID_Position(PID_Loop_t *pid, float target, float actual)
{
	pid->Target=target;
	pid->Actual=actual;
	//误差=目标值-实际值
	pid->Err = target - actual;
  	/*死区*/
  pid->Err = anti_constrain(pid->Err,pid->PID_Err_Dead,-pid->PID_Err_Dead);
	//积分限幅
	pid->I_Limit = pid->I_Limit + pid->Err;
	LimitMax(pid->I_Limit,pid->I_Limit_Max);

	pid->PID_P_Out = pid->PID_Param.P * pid->Err;
	pid->PID_I_Out = pid->PID_Param.I * pid->I_Limit;
	pid->PID_D_Out = pid->PID_Param.D * (pid->Err - pid->Last_Err);
	//I输出限幅
	LimitMax(pid->PID_I_Out,pid->PID_I_Out_Max);
	pid->PID_Output = pid->PID_P_Out+pid->PID_I_Out+pid->PID_D_Out;
	
	pid->Last_Err = pid->Err;
	//总输出限幅
	LimitMax(pid->PID_Output, pid->PID_Output_Max);
	
	return pid->PID_Output;
}


#if (Robot == Infantry_3)
PID_Parameter_t PID_Yaw_Follow={
	.P = 5,
	.I = 0,
	.D = 0,
};
PID_Parameter_t PID_Speed_Param[MOTOR_TYPE_CNT][PID_TYPE_CNT]={
  [M_3508] = {
		 [Clear_Away]   = {
      .P = 0,
      .I = 0,
      .D = 0,
    },
    [RC]   = {
      .P = 6,
      .I = 0,
      .D = 0,
    },
		[KEY]	 = {
      .P = 5,
      .I = 0,
      .D = 0,
		},
	},
	  [GM_6020_YAW] = {
		 [Clear_Away]   = {
      .P = 0,
      .I = 0,
      .D = 0,
    },
    [RC]   = {
      .P = 100,
      .I = 0.1,	
      .D = 0,
    },
		[KEY]	 = {
      .P = 80,	
      .I = 0.5,
      .D = 0,
		},
	},
	  [GM_6020_PIT] = {
		 [Clear_Away]   = {
      .P = 0,
      .I = 0,
      .D = 0,
    },
    [RC]   = {
      .P = 100,	/*180*/
      .I = 1,		
      .D = 0,
    },
		[KEY]	 = {
      .P = 100,
      .I = 1,
      .D = 0,
		},
	},
		[FRIC_3508] = {
		 [Clear_Away]   = {
      .P = 0,
      .I = 0,
      .D = 0,
    },
    [RC]   = {
      .P = 5,
      .I = 0,
      .D = 0,
    },
		[KEY]	 = {
      .P = 5,
      .I = 0,
      .D = 0,
		},
	},
		[M_2006] = {
			[Clear_Away] = {
      .P = 0,
      .I = 0,
      .D = 0,
    },
			[RC]   = {
      .P = 5,
      .I = 0,
      .D = 0,
    },
			[KEY]	 = {
      .P = 5,
      .I = 0,
      .D = 0,
		},
	},
};

PID_Parameter_t PID_Angle_Param[MOTOR_TYPE_CNT][PID_TYPE_CNT]={
	  [GM_6020_YAW] = {
		 [Clear_Away]   = {
      .P = 0,
      .I = 0,
      .D = 0,
    },
    [RC]   = {
      .P = 15,
      .I = 0,
      .D = 0,
    },
		[KEY]	 = {
      .P = 15,
      .I = 0,
      .D = 0,
		},
	},
	  [GM_6020_PIT] = {
		 [Clear_Away]   = {
      .P = 0,
      .I = 0,
      .D = 0,
    },
    [RC]   = {
      .P = 9,
      .I = 0,
      .D = 0,
    },
		[KEY]	 = {
      .P = 9,
      .I = 0,
      .D = 0,
		},
	},
		[M_2006] = {
		 [Clear_Away]   = {
      .P = 0,
      .I = 0,
      .D = 0,
    },
    [RC]   = {
      .P = 5 ,
      .I = 0,
      .D = 0,
    },
		[KEY]	 = {
      .P = 5,
      .I = 0,
      .D = 0,
		},
	},
};
#elif(Robot ==HERO )
PID_Parameter_t PID_Yaw_Follow=
{
	.P = 5,
	.I = 0,
	.D = 0,
};

PID_Parameter_t PID_Speed_Param[MOTOR_TYPE_CNT][PID_TYPE_CNT]={
  [M_3508] = {
		 [Clear_Away]   = {
      .P = 0,
      .I = 0,
      .D = 0,
    },
    [RC]   = {
      .P = 10.5,
      .I = 0,
      .D = 0,
    },
		[KEY]	 = {
      .P = 5,
      .I = 0,
      .D = 0,
		},
	},
	  [GM_6020_YAW] = {
		 [Clear_Away]   = {
      .P = 0,
      .I = 0,
      .D = 0,
    },
    [RC]   = {
      .P = 230,
      .I = 0,	
      .D = 0,
    },
		[KEY]	 = {
      .P = 230,	
      .I = 0,
      .D = 0,
		},
	},
	  [GM_6020_PIT] = {
		 [Clear_Away]   = {
      .P = 0,
      .I = 0,
      .D = 0,
    },
    [RC]   = {
      .P = 150,	
      .I = 1.4,		
      .D = 0,
    },
		[KEY]	 = {
      .P = 150,
      .I = 1.4,
      .D = 0,
		},
	},
		[FRIC_3508] = {
		 [Clear_Away]   = {
      .P = 0,
      .I = 0,
      .D = 0,
    },
    [RC]   = {
      .P = 3.8,
      .I = 0.018,
      .D = 0,
    },
		[KEY]	 = {
      .P = 3.8,
      .I = 0.018,
      .D = 0,
		},
	},
		[M_2006] = {
			[Clear_Away] = {
      .P = 0,
      .I = 0,
      .D = 0,
    },
			[RC]   = {
      .P = 8,
      .I = 0,
      .D = 0,
    },
			[KEY]	 = {
      .P = 8,
      .I = 0,
      .D = 0,
		},
	},
};

PID_Parameter_t PID_Angle_Param[MOTOR_TYPE_CNT][PID_TYPE_CNT]={
	  [GM_6020_YAW] = {
		 [Clear_Away]   = {
      .P = 0,
      .I = 0,
      .D = 0,
    },
    [RC]   = {
      .P = 14,
      .I = 0,
      .D = 0,
    },
		[KEY]	 = {
      .P = 14,
      .I = 0,
      .D = 0,
		},
	},
	  [GM_6020_PIT] = {
		 [Clear_Away]   = {
      .P = 0,
      .I = 0,
      .D = 0,
    },
    [RC]   = {
      .P = 10.6,
      .I = 0,
      .D = 0,
    },
		[KEY]	 = {
      .P = 10.6,
      .I = 0,
      .D = 0,
		},
	},
		[M_2006] = {
		 [Clear_Away]   = {
      .P = 0,
      .I = 0,
      .D = 0,
    },
    [RC]   = {
      .P = 9 ,
      .I = 0,
      .D = 0,
    },
		[KEY]	 = {
      .P = 8,
      .I = 0,
      .D = 0,
		},
	},
};

#elif (Robot == Infantry_5)
PID_Parameter_t PID_Yaw_Follow={
	.P = 2,
	.I = 0,
	.D = 0,
};
PID_Parameter_t PID_Speed_Param[MOTOR_TYPE_CNT][PID_TYPE_CNT]={
  [M_3508] = {
		 [Clear_Away]   = {
      .P = 0,
      .I = 0,
      .D = 0,
    },
    [RC]   = {
      .P = 6,
      .I = 0,
      .D = 0,
    },
		[KEY]	 = {
      .P = 5,
      .I = 0,
      .D = 0,
		},
	},
	  [GM_6020_YAW] = {
		 [Clear_Away]   = {
      .P = 0,
      .I = 0,
      .D = 0,
    },
    [RC]   = {
      .P = 180,
      .I = 0.001,	
      .D = 0,
    },
		[KEY]	 = {
      .P = 160,	
      .I = 0.001,
      .D = 0,
		},
	},
	  [GM_6020_PIT] = {
		 [Clear_Away]   = {
      .P = 0,
      .I = 0,
      .D = 0,
    },
    [RC]   = {
      .P = 150,
      .I = 0.5,		
      .D = 0,
    },
		[KEY]	 = {
      .P = 160,
      .I = 0,
      .D = 0.01,
		},
	},
		[FRIC_3508] = {
		 [Clear_Away]   = {
      .P = 0,
      .I = 0,
      .D = 0,
    },
    [RC]   = {
      .P = 10,
      .I = 0,
      .D = 0,
    },
		[KEY]	 = {
      .P = 10,
      .I = 0,
      .D = 0,
		},
	},
		[M_2006] = {
			[Clear_Away] = {
      .P = 0,
      .I = 0,
      .D = 0,
    },
			[RC]   = {
      .P = 8,
      .I = 0,
      .D = 0,
    },
			[KEY]	 = {
      .P = 3,
      .I = 0,
      .D = 0,
		},
	},
};

PID_Parameter_t PID_Angle_Param[MOTOR_TYPE_CNT][PID_TYPE_CNT]={
	  [GM_6020_YAW] = {
		 [Clear_Away]   = {
      .P = 0,
      .I = 0,
      .D = 0,
    },
    [RC]   = {
      .P = 10,
      .I = 0,
      .D = 0,
    },
		[KEY]	 = {
      .P = 8,
      .I = 0,
      .D = 0,
		},
	},
	  [GM_6020_PIT] = {
		 [Clear_Away]   = {
      .P = 0,
      .I = 0,
      .D = 0,
    },
    [RC]   = {
      .P = 18,
      .I = 0,
      .D = 0,
    },
		[KEY]	 = {
      .P = 12,
      .I = 0,
      .D = 0.001,
		},
	},
		[M_2006] = {
		 [Clear_Away]   = {
      .P = 0,
      .I = 0,
      .D = 0,
    },
    [RC]   = {
      .P = 4 ,
      .I = 0,
      .D = 0,
    },
		[KEY]	 = {
      .P = 2,
      .I = 0,
      .D = 0,
		},
	},
};
#endif




/**
 * @brief 总电机初始化
 * @param 
 */
void Motor_Init(void)
{
  M_3508_Init();
	GM_6020_Init();
	M_2006_Init();
	Fric_3508_Init();
}

/*底盘M3508电机---------------------------------------*/
/**
 * @brief 底盘电机初始化
 * @param 
 */
void M_3508_Init(void)
{
	Chassis.Motor_Info.Motor_Type = M_3508;
	M_3508_ParamInit(&Chassis.Motor_Data[CHAS_LF].PID.Speed_Loop);
  M_3508_ParamInit(&Chassis.Motor_Data[CHAS_RF].PID.Speed_Loop);
  M_3508_ParamInit(&Chassis.Motor_Data[CHAS_LB].PID.Speed_Loop);
  M_3508_ParamInit(&Chassis.Motor_Data[CHAS_RB].PID.Speed_Loop);	
}



/**
 * @brief PID限幅设置 
 * @param 
 */
void M_3508_ParamInit(PID_Loop_t *str)
{
  /* 这里只针对速度环 */
  str->I_Limit_Max = 50000;
  str->PID_I_Out_Max = 10000; 
  str->PID_P_Out_Max = 50000;
  str->PID_Output_Max = 10000;
  str->PID_Err_Dead = 5;
}




/*云台GM6020电机--------------------------------------*/
/**
 * @brief Yaw轴的PID限幅设置
 * @param 
 */
void GIM_YawParamInit(Gimbal_Info_t *str)
{
  /* 速度环 */
  str->PID.Speed_Loop.I_Limit_Max = 5000;
  str->PID.Speed_Loop.PID_I_Out_Max = 25000;
  str->PID.Speed_Loop.PID_P_Out_Max = 50000;
  str->PID.Speed_Loop.PID_Output_Max = 25000;
  str->PID.Speed_Loop.PID_Err_Dead = 0;
  /* 角度环 */
  str->PID.Angle_Loop.I_Limit_Max = 12000;
  str->PID.Angle_Loop.PID_I_Out_Max = 25000;
  str->PID.Angle_Loop.PID_P_Out_Max = 50000;
  str->PID.Angle_Loop.PID_Output_Max = 13000;
  str->PID.Angle_Loop.PID_Err_Dead = 0;  
  /* 遥控模式 */
  str->RC_Move.Angle_Inc = 0;
  str->RC_Move.Angle_k = 0.001;
  str->RC_Move.Slow_Inc = 0;
  /* 键鼠模式 */
  str->KEY_Move.Angle_Inc = 0;
  str->KEY_Move.Angle_k = 0.004;//0.001
  str->KEY_Move.Slow_Inc = 0;
	
}

/**
 * @brief Pitch轴的PID限幅设置
 * @param 
 */
void GIM_PitParamInit(Gimbal_Info_t *str)
{
  /* 速度环 */
  str->PID.Speed_Loop.I_Limit_Max = 8000;
  str->PID.Speed_Loop.PID_I_Out_Max = 10000;
  str->PID.Speed_Loop.PID_P_Out_Max = 50000;
  str->PID.Speed_Loop.PID_Output_Max = 18000;
  str->PID.Speed_Loop.PID_Err_Dead = 0;
  /* 角度环 */
  str->PID.Angle_Loop.I_Limit_Max = 8000;
  str->PID.Angle_Loop.PID_I_Out_Max = 1000000;
  str->PID.Angle_Loop.PID_P_Out_Max = 50000;
  str->PID.Angle_Loop.PID_Output_Max = 10000;
  str->PID.Angle_Loop.PID_Err_Dead = 0;  
  /* 遥控模式 */
  str->RC_Move.Angle_Inc = 0;
  str->RC_Move.Angle_k = 0.001;
  str->RC_Move.Slow_Inc = 0;
  /* 键鼠模式 */
  str->KEY_Move.Angle_Inc = 0;
  str->KEY_Move.Angle_k = 5;
  str->KEY_Move.Slow_Inc = 0;
}
/**
 * @brief Pitch轴的PID限幅设置
 * @param 
 */
void YAW_Follow_PitParamInit(PID_Loop_t *str)
{
	str->PID_Param = PID_Yaw_Follow;
  str->I_Limit_Max = 10000;
  str->PID_I_Out_Max = 10000; 
  str->PID_P_Out_Max = 10000;
  str->PID_Output_Max = 5000;
  str->PID_Err_Dead = 80;
}

/**
 * @brief 云台电机初始化
 * @param 
 */
void GM_6020_Init(void)
{
  Gimbal.YAW.Motor_Info.Motor_Type = GM_6020_YAW;
  Gimbal.PITCH.Motor_Info.Motor_Type = GM_6020_PIT;
  
	YAW_Follow_PitParamInit(&Chassis.YAW_Follow.PID.Speed_Loop);
  GIM_YawParamInit(&Gimbal.YAW);
  GIM_PitParamInit(&Gimbal.PITCH);
}



/*拨盘M2006电机--------------------------------------*/
/**
 * @brief 拨盘电机初始化
 * @param 
 */
void M_2006_Init(void)
{
	Shoot.Motor_Data[DRIVER].Motor_Info.Motor_Type = M_2006;
  M_2006_ParamInit();
}
void M_2006_ParamInit(void)
{
  PID_Info_t *str = &(Shoot.Motor_Data[DRIVER].PID);
  
  str->Speed_Loop.PID_I_Out_Max = 25000;
  str->Speed_Loop.PID_Output_Max = 10000;//速度环最终输出限制10000，防止电流过大，断线+疯转
  str->Speed_Loop.PID_P_Out_Max = 13000;
  str->Speed_Loop.PID_Err_Dead = 0;
  
  str->Angle_Loop.PID_I_Out_Max = 25000;
  str->Angle_Loop.PID_Output_Max = 13000;
  str->Angle_Loop.PID_P_Out_Max = 13000;
  str->Angle_Loop.PID_Err_Dead = 0;
}

/*摩擦轮M_3508电机--------------------------------------*/
/**
 * @brief 拨盘电机初始化
 * @param 
 */
void Fric_3508_Init(void)
{
	Shoot.Motor_Data[FRIC_L].Motor_Info.Motor_Type = FRIC_3508;
	Shoot.Motor_Data[FRIC_R].Motor_Info.Motor_Type = FRIC_3508;
	
	M_3508_ParamInit(&Shoot.Motor_Data[FRIC_L].PID.Speed_Loop);
	M_3508_ParamInit(&Shoot.Motor_Data[FRIC_R].PID.Speed_Loop);
}



