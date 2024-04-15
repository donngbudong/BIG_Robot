#include "vofa_task.h"
#include "Device.h"
#include "usart.h"

static uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};

Vofa_Motor_t Vofa;
Vofa_Date speed;

void Motor_Date(CAN_GET_DATA_t *str)
{
	Vofa_Motor_t *vofa;
	vofa->Chassis_LF.receive = str->Motor_Speed;
}


void VOFA_SendDate(void)
{
	static uint16_t i;
	speed.receive = 6000;
//	Vofa.Chassis_LB.receive = Chassis.Motor_Data[0].Motor_Data.CAN_GetData.Motor_Speed;
	
	Vofa.IMU_YAW.receive = IMU_Get_Data.IMU_Eular[YAW];
	Vofa.YAW.receive = Gimbal.YAW.Motor_Data.PID_Angle_target;
	Vofa.IMU_PITCH.receive = IMU_Get_Data.IMU_Eular[PITCH];
	Vofa.PITCH.receive = Gimbal.PITCH.Motor_Data.PID_Angle_target;
//	Vofa.Shoot_L.receive = Shoot.Motor_Data[FRIC_L].Motor_Data.CAN_GetData.Motor_Speed;
//	Vofa.Shoot_B.receive = Shoot.Motor_Data[FRIC_R].Motor_Data.CAN_GetData.Motor_Speed;
//	for( i = 0; i<4; i++)	
//	{
//		HAL_UART_Transmit(&huart6,&speed.send[i],1,100);
//	}
//	for( i = 0; i<4; i++)	
//	{
//		HAL_UART_Transmit(&huart6,&Vofa.Shoot_B.send[i],1,100);
//	}
//	for( i = 0; i<4; i++)	
//	{
//		HAL_UART_Transmit(&huart6,&Vofa.Shoot_L.send[i],1,100);
//	}

//	for( i = 0; i<4; i++)	
//	{
//		HAL_UART_Transmit(&huart6,&Vofa.IMU_PITCH.send[i],1,100);
//	}
//	for( i = 0; i<4; i++)	
//	{
//		HAL_UART_Transmit(&huart6,&Vofa.PITCH.send[i],1,100);
//	}
	for( i = 0; i<4; i++)	
	{
		HAL_UART_Transmit(&huart6,&Vofa.IMU_YAW.send[i],1,100);
	}
	for( i = 0; i<4; i++)	
	{
		HAL_UART_Transmit(&huart6,&Vofa.YAW.send[i],1,100);
	}
	/*֡帧尾*/
 	for( i=0; i<4; i++){
		HAL_UART_Transmit(&huart6,&tail[i],1,100);
	}
}

//void vofa_send(void)
//{
//	uint16_t i=0;

////	vofa_date_t[1].c=Eular[0];
////	vofa_date_t[2].c=Eular[2];
////	vofa_date_t[3].c=pid_gimbal[1].set;
////	vofa_date_t[4].c=pid_gimbal[3].set;
////chassis
////	vofa_date_t[0].c=4000;
////	vofa_date_t[1].c=motor_chassis[1].speed_rpm;
////	vofa_date_t[2].c=motor_chassis[2].speed_rpm;
////	vofa_date_t[3].c=motor_chassis[3].speed_rpm;
////	vofa_date_t[4].c=motor_chassis[4].speed_rpm;


///******************************0**************************/
////	for(i = 0; i<4; i++)	
////	{
////		HAL_UART_Transmit(&huart6,&vofa_date_t[0].b[i],1,100);
////	}
///******************************1**************************/
//	for(i = 0; i<4; i++)
//	{
//		HAL_UART_Transmit(&huart6,&vofa_date_t[1].b[i],1,100);
//	}
///******************************2**************************/
//	for(i = 0; i<4; i++)
//	{
//		HAL_UART_Transmit(&huart6,&vofa_date_t[2].b[i],1,100);
//	}
///******************************3**************************/
////	for(i = 0; i<4; i++)
////	{
////		HAL_UART_Transmit(&huart6,&vofa_date_t[3].b[i],1,100);
////	}
/////******************************4**************************/
////		for(i = 0; i<4; i++)
////	{
////		HAL_UART_Transmit(&huart6,&vofa_date_t[4].b[i],1,100);
////	}


//}

