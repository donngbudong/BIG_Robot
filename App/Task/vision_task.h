#ifndef VISION_TASK_H
#define VISION_TASK_H
#include "System.h"

#define VISION_TX_BUFFER_LEN	28
#define VISION_FRAME_HEADER		(0x5A)

typedef enum
{
	VS_ERR		= 0, 
	VS_NORMAL   = 1, 
	VS_LOST     = 2, 
}Vs_State_t;

typedef union 
{
	uint8_t input[4];
	float		output;
}Union_t; 

typedef struct 
{
	Vs_State_t Vs_State;
//	uint16_t distance[3];									//����??2a?��
//	uint8_t Visual_State;
	Union_t	pitch;
	Union_t	yaw;
	Union_t	TX_Pitch;
	Union_t	TX_Yaw;
	
	float RX_Pitch;
	float RX_Yaw;

	float ratio;
	float MAX;
	float MIN;
	float distance[3];
	uint8_t STATE;
}Vision_Date_t;

#ifndef PI
#define PI 3.1415926535f
#endif
#define GRAVITY 9.78f
typedef unsigned char uint8_t;
enum ARMOR_ID
{
    ARMOR_OUTPOST		= 0,
    ARMOR_HERO 			= 1,
    ARMOR_ENGINEER	= 2,
    ARMOR_INFANTRY3 = 3,
    ARMOR_INFANTRY4 = 4,
    ARMOR_INFANTRY5 = 5,
    ARMOR_GUARD 		= 6,
    ARMOR_BASE 			= 7
};

enum ARMOR_NUM
{
    ARMOR_NUM_BALANCE = 2,
    ARMOR_NUM_OUTPOST = 3,
    ARMOR_NUM_NORMAL = 4
};

enum BULLET_TYPE
{
    BULLET_17 = 0,
    BULLET_42 = 1
};


//���ò���
struct SolveTrajectoryParams
{
    float k;             //����ϵ��

    //�������
    enum BULLET_TYPE bullet_type;  //������������� 0-���� 1-Ӣ��
    float current_v;      //��ǰ����
    float current_pitch;  //��ǰpitch
    float current_yaw;    //��ǰyaw

    //Ŀ�����
    float xw;             //ROS����ϵ�µ�x
    float yw;             //ROS����ϵ�µ�y
    float zw;             //ROS����ϵ�µ�z
    float vxw;            //ROS����ϵ�µ�vx
    float vyw;            //ROS����ϵ�µ�vy
    float vzw;            //ROS����ϵ�µ�vz
    float tar_yaw;        //Ŀ��yaw
    float v_yaw;          //Ŀ��yaw�ٶ�
    float r1;             //Ŀ�����ĵ�ǰ��װ�װ�ľ���
    float r2;             //Ŀ�����ĵ�����װ�װ�ľ���
    float dz;             //��һ��װ�װ������ڱ�����װ�װ�ĸ߶Ȳ�
    int bias_time;        //ƫ��ʱ��
    float s_bias;         //ǹ��ǰ�Ƶľ���
    float z_bias;         //yaw������ǹ��ˮƽ��Ĵ�ֱ����

    enum ARMOR_ID armor_id;     //װ�װ�����  0-outpost 6-guard 7-base
                                //1-Ӣ�� 2-���� 3-4-5-���� 
    enum ARMOR_NUM armor_num;   //װ�װ�����  2-balance 3-outpost 4-normal

		float yaw;
		float pitch;
};

//���ڴ洢Ŀ��װ�װ����Ϣ
struct tar_pos
{
    float x;           //װ�װ�����������ϵ�µ�x
    float y;           //װ�װ�����������ϵ�µ�y
    float z;           //װ�װ�����������ϵ�µ�z
    float yaw;         //װ�װ�����ϵ�������������ϵ��yaw��
};
//�������������ģ��
extern float monoDirectionalAirResistanceModel(float s, float v, float angle);
//��ȫ��������ģ��
extern float completeAirResistanceModel(float s, float v, float angle);
//pitch��������
extern float pitchTrajectoryCompensation(float s, float y, float v);
//�������ž��ߵó�������װ�װ� �Զ����㵯��
extern void autoSolveTrajectory(float *pitch, float *yaw, float *aim_x, float *aim_y, float *aim_z);


/* ֡ͷ�ֽ�ƫ�� */
typedef enum {
	 SOF_Vision    		= 0 ,	//֡ͷƫ��
	 DATA_Vision	 		= 1 , //������ʼƫ��
	 LEN_FRAME_HEADER = 1	,	// ֡ͷ����
	 LEN_FRAME_TAILER = 2	,	// ֡βCRC16 
}Frame_Header_Offset_t;

/* �������ݳ�����Ϣ */
typedef struct {
	/* Std */
	uint8_t LEN_RX_DATA 			;	// �������ݶγ���
  uint8_t LEN_RX_PACKET	    ;	// ���հ���������
} DataRX_Length_t;

/* �������ݳ�����Ϣ */
typedef struct {
	/* Std */
  uint8_t TX_CRC16          ;//crc16ƫ�� 
	uint8_t LEN_TX_DATA 		  ;	// �������ݶγ���
	uint8_t LEN_TX_PACKET	    ;	// ���Ͱ���������
} DataTX_Length_t;


/* �������ݶθ�ʽ */
typedef __packed struct 
{
	bool tracking : 1;
  uint8_t id : 3;          	// 0-outpost 6-guard 7-base
  uint8_t armors_num : 3;  	// 2-balance 3-outpost 4-normal
  uint8_t reserved : 1;			//����
  float x;
  float y;
  float z;
  float yaw;
  float vx;
  float vy;
  float vz;
  float v_yaw;
  float r1;
  float r2;
  float dz;
} AutoAim_Rx_Data_t;

/* �������ݶθ�ʽ */ 
typedef __packed struct
{
	uint8_t detect_color : 1 ;  // 0-red 1-blue
  bool reset_tracker : 1;
  uint8_t reserved : 6;
  float roll;
  float pitch;
  float yaw;
  float aim_x;
  float aim_y;
  float aim_z;
} AutoAim_Tx_Data_t;


/* ֡β��ʽ */
typedef __packed struct 
{
	uint16_t crc16;					// CRC16У����
} Vision_Frame_Tailer_t;

/* ���鷢�Ͱ���ʽ */
typedef __packed struct
{
	uint8_t  			sof;		// ͬ��ͷ
	AutoAim_Tx_Data_t	  TxData;		// ����
	Vision_Frame_Tailer_t FrameTailer;	// ֡β		
} AutoAim_Tx_Packet_t;


/* ������հ���ʽ */
typedef __packed struct 
{
	uint8_t  			sof;		// ͬ��ͷ
	AutoAim_Rx_Data_t	  RxData;		// ����
	Vision_Frame_Tailer_t FrameTailer;	// ֡β	
} AutoAim_Rx_Packet_t;


/*���Ͷ˵���Ϣ*/
typedef struct
{
  AutoAim_Tx_Packet_t Packet;
  DataTX_Length_t  LEN;
}AutoAim_Tx_Info_t;

/*���ն˵���Ϣ*/
typedef struct
{
  AutoAim_Rx_Packet_t Packet;
  DataRX_Length_t  LEN;
}AutoAim_Rx_Info_t;


/*���<-->�Ӿ� ͨ����Ϣ*/
typedef struct
{
  AutoAim_Tx_Info_t AutoAim_Tx;
  AutoAim_Rx_Info_t AutoAim_Rx;
}VisionRTx_t;



/*�Ӿ������ܿؽṹ��*/
typedef struct
{
  VisionRTx_t  VisionRTx;
}Vision_Info_t;


void Visual_Task(void);

void Visual_SendData(void);
void AUTO_AIM_Ctrl(void);

void Vision_read_data(uint8_t *data);


void rm_vision(uint8_t *data);


void VSIION_State_Report(void);
Vs_State_t VS_Check(void);
bool Judge_VISISON_Lost(void);

//#define    IF_VS_LOST      Judge_VISISON_Lost()

void Vision_Init(void);
void VISION_SendData(void);
void VISION_ReadData(uint8_t *rxBuf);
void AUTO_AIM_Ctrl(void);
void RM_Vision_Init(void);


extern Vision_Date_t Vision;
#endif
