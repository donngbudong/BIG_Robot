#include "system_task.h"
#include "System.h"
#include "Device.h"
#include "FreeRTOS.h"

#include "tim.h"
#include "task.h"



Sys_Info_t System = {
	.Rc_State  = RC_LOST,
	.Imu_State = IMU_LOST,
	.System_State = SYSTEM_LOST,
	.System_Pid = Clear_Away,
};



uint64_t Remote_time = 0;	//遥控器
uint64_t Imu_time = 0;		//裁判系统
uint64_t Vision_time = 0;	//视觉


//设备连接初始化
void Time_Init(void)
{
  Remote_time = micros();
  Imu_time = micros();
	Vision_time = micros();
}

void System_Task(void)
{
	System_State();
	RC_State_Report();
	IMU_State_Report();
//	ws2812_blue(200);
//	Waterfall_light();
	PID_Switch();
}

void System_State(void)
{
#if(MCU==UP)
	CAN_cmd_RC1(RC_CH0,RC_CH1,RC_CH2,RC_CH3);
	CAN_cmd_RC2(RC_S1,RC_S2,RC_SW,RC_Ctrl.kb.key);
#endif
	
    if(System.Rc_State ==RC_ERR || System.Imu_State ==IMU_ERR)
  {
    System.System_State = SYSTEM_ERR;
  } 
    if(System.Rc_State ==RC_NORMAL && System.Imu_State ==IMU_NORMAL)
  {
    System.System_State = SYSTEM_NORMAL;
  }
  if(System.Rc_State ==RC_LOST || System.Imu_State ==IMU_LOST)
  {
    System.System_State = SYSTEM_LOST;
  }
}

void PID_Switch(void)
{
	if(RC_S1==1)
	{
		System.System_Pid = RC;
	}
	if(RC_S1==3)
	{
		System.System_Pid = Clear_Away;
	}
	if(RC_S1==2)
	{
		System.System_Pid = KEY;
	}
}


void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
		HAL_TIM_PWM_Stop_DMA(&htim1,TIM_CHANNEL_1);
}
 


uint16_t static RGB_buffur[RESET_PULSE + WS2812_DATA_LEN] = { 0 };

void ws2812_set_RGB(uint8_t R, uint8_t G, uint8_t B, uint16_t num)
{
    uint16_t* p = (RGB_buffur + RESET_PULSE) + (num * LED_DATA_LEN);
    
    for (uint16_t i = 0;i < 8;i++)
    {
        p[i]      = (G << i) & (0x80)?ONE_PULSE:ZERO_PULSE;
        p[i + 8]  = (R << i) & (0x80)?ONE_PULSE:ZERO_PULSE;
        p[i + 16] = (B << i) & (0x80)?ONE_PULSE:ZERO_PULSE;
    }
}
void ws2812_blue(uint32_t led_nums)
{
	uint16_t num_data;
	num_data = 80 + led_nums * 24;
	for(uint8_t i = 0; i < led_nums; i++)
	{
		ws2812_set_RGB(0xff, 0x00, 0x00, i);
	}
	 HAL_TIM_PWM_Start_DMA(&htim1,TIM_CHANNEL_1,(uint32_t *)RGB_buffur,(num_data));
}
uint8_t x,y,z;
uint8_t big=255;
void State_LED(void)
{
	 // X
	 if (x == 0)
	 {
		ws2812_set_RGB(big, 0x00, 0x00, 0);
		ws2812_set_RGB(big, 0x00, 0x00, 1);
	 }
	 else
	 {
		ws2812_set_RGB(0x00, big, 0x00, 0);
		ws2812_set_RGB(0x00, big, 0x00, 1);
	 }
	 // Y
	 if (y == 0)
	 {
		ws2812_set_RGB(0x00, big, 0x00, 2);
		ws2812_set_RGB(0x00, big, 0x00, 3);
	 }
	 else
	 {
		ws2812_set_RGB(0x00, 0x00, big, 2);
		ws2812_set_RGB(0x00, 0x00, big, 3);
	 }
	 // Z
	 if (z == 0)
	 {
		ws2812_set_RGB(0x00, 0x00, big, 4);
		ws2812_set_RGB(0x00, 0x00, big, 5);
	 }
	 else
	 {
		ws2812_set_RGB(big, 0x00, 0x00, 4);
		ws2812_set_RGB(big, 0x00, 0x00, 5);
	 }

//    ws2812_set_RGB(0x00, 0x22, 0x00, 1);
//    ws2812_set_RGB(0x00, 0x00, 0x22, 2);
//    ws2812_set_RGB(0x22, 0x00, 0x00, 3);
//    ws2812_set_RGB(0x00, 0x22, 0x00, 4);
//    ws2812_set_RGB(0x00, 0x00, 0x22, 5);
//		ws2812_set_RGB(0x22, 0x00, 0x00, 6);
//    ws2812_set_RGB(0x00, 0x22, 0x00, 7);
    HAL_TIM_PWM_Start_DMA(&htim1,TIM_CHANNEL_1,(uint32_t *)RGB_buffur,(232)); //272 = 80 + 24 * LED_NUMS(6)
//    HAL_Delay(50);
}

uint8_t RGB[51]={0,10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170,180,190,200,210,220,230,240,250,
										240,230,220,210,200,190,180,170,160,150,140,130,120,110,100,90,80,70,60,50,40,30,20,10,0};
void Waterfall_light(void)
{
	uint8_t i;

//if(RC_S1 == 3)
//{
	ws2812_set_RGB(0xFF, 0x00, 0x00, 0);
	ws2812_set_RGB(0xFF, 0x7F, 0x00, 1);
	ws2812_set_RGB(0xFF, 0xFF, 0x00, 2);
	ws2812_set_RGB(0x00, 0xFF, 0x00, 3);
	ws2812_set_RGB(0x00, 0xFF, 0xFE, 4);
	ws2812_set_RGB(0x00, 0x00, 0xFE, 5);
	ws2812_set_RGB(0x8B, 0x00, 0xFE, 6);
	ws2812_set_RGB(0x22, 0x22, 0x22, 7);
//流水灯
	for(i=0;i<=8;i++){
		HAL_TIM_PWM_Start_DMA(&htim1,TIM_CHANNEL_1,(uint32_t *)RGB_buffur,(80+24*i)); //272 = 80 + 24 * LED_NUMS(8)80+24*i}
		vTaskDelay(25);
	}
	ws2812_set_RGB(0x00, 0x00, 0x00, 0);
	ws2812_set_RGB(0x00, 0x00, 0x00, 1);
	ws2812_set_RGB(0x00, 0x00, 0x00, 2);
	ws2812_set_RGB(0x00, 0x00, 0x00, 3);
	ws2812_set_RGB(0x00, 0x00, 0x00, 4);
	ws2812_set_RGB(0x00, 0x00, 0x00, 5);
	ws2812_set_RGB(0x00, 0x00, 0x00, 6);
	ws2812_set_RGB(0x00, 0x00, 0x00, 7);
	HAL_TIM_PWM_Start_DMA(&htim1,TIM_CHANNEL_1,(uint32_t *)RGB_buffur,(272)); //272 = 80 + 24 * LED_NUMS(8)80+24*i}
	vTaskDelay(25);
//}

//if(RC_S1 == 1)
//{
////渐变灯
//	for(i=0;i<51;i++){
//		ws2812_set_RGB(RGB[i], 0x00, 0x00, 0);
//		ws2812_set_RGB(RGB[i], 0x00, 0x00, 1);
//		ws2812_set_RGB(RGB[i], 0x00, 0x00, 2);
//		ws2812_set_RGB(RGB[i], 0x00, 0x00, 3);
//		ws2812_set_RGB(RGB[i], 0x00, 0x00, 4);
//		ws2812_set_RGB(RGB[i], 0x00, 0x00, 5);
//		HAL_TIM_PWM_Start_DMA(&htim1,TIM_CHANNEL_1,(uint32_t *)RGB_buffur,(80+24*6)); //272 = 80 + 24 * LED_NUMS(8)80+24*i}
//		vTaskDelay(10);
//		}
//	for(i=0;i<51;i++){
//		ws2812_set_RGB(0x00, RGB[i], 0x00, 0);
//		ws2812_set_RGB(0x00, RGB[i], 0x00, 1);
//		ws2812_set_RGB(0x00, RGB[i], 0x00, 2);
//		ws2812_set_RGB(0x00, RGB[i], 0x00, 3);
//		ws2812_set_RGB(0x00, RGB[i], 0x00, 4);
//		ws2812_set_RGB(0x00, RGB[i], 0x00, 5);

//		HAL_TIM_PWM_Start_DMA(&htim1,TIM_CHANNEL_1,(uint32_t *)RGB_buffur,(80+24*6)); //272 = 80 + 24 * LED_NUMS(8)80+24*i}
//		vTaskDelay(10);
//		}
//	for(i=0;i<51;i++){
//		ws2812_set_RGB(0x00, 0x00, RGB[i], 0);
//		ws2812_set_RGB(0x00, 0x00, RGB[i], 1);
//		ws2812_set_RGB(0x00, 0x00, RGB[i], 2);
//		ws2812_set_RGB(0x00, 0x00, RGB[i], 3);
//		ws2812_set_RGB(0x00, 0x00, RGB[i], 4);
//		ws2812_set_RGB(0x00, 0x00, RGB[i], 5);

//		HAL_TIM_PWM_Start_DMA(&htim1,TIM_CHANNEL_1,(uint32_t *)RGB_buffur,(80+24*6)); //272 = 80 + 24 * LED_NUMS(8)80+24*i}
//		vTaskDelay(10);
//		}
//}
}
