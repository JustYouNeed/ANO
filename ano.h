# ifndef __ANO_TECH_H
# define __ANO_TECH_H
#if defined STM32F40_41xxx
	# include "stm32f4xx.h"
#elif defined (STM32F10X_HD)||(STM32F10X_MD)||(STM32F10X_LD)
	# include "stm32f10x.h"
#endif
	
# include "stdint.h"


// 定义通信中使用的串口，1为串口1，2为串口2.。。。
//目前在F407上实现了串口1，2，3
# define ANO_USART	1

//该宏开关决定是否使用用户自己的底层驱动函数，
//当开关为一时需要用户自己提供底层的通信部分，如基本字节的发送函数，串口的初始化函数
# define ANO_DRIVER_BY_USER		0

//自动处理数据宏开关，若打开，则程序将自动在接收完一帧数据之后处理相关数据，
//但开启自动处理会增加中断时间，可根据用户需要选择，但需要在其他地方周期性调用
//数据处理函数ANO_DataProcess（），该功能默认开启
# define ANO_DATA_PRECESS_ON	1


//是否接收OPENMV发送的数据
//1，接收，0，不接收
# define ANO_CON_OPENMV				1

#if ANO_CON_OPENMV==1
	//OpenMV摄像头发送Blobs的位置			
	//int16_t 	int16_t			int16_t			int16_t		int16_t		int16_t
	//red_blob	blue_blob		green_blob	yellow		white     black
	# define OPENMV_BLOBS1_POS	0XF1
	# define OPENMV_BLOBS2_POS	0XF2
	//OpenMV摄像头寻线发送线的位置
	# define OPENMV_LINE_POS	0XF3
#endif


//PID参数在上传及接收时使用的变换因子
# define ANO_PID_TRAN_FAC_P 10
# define ANO_PID_TRAN_FAC_I 1000
# define ANO_PID_TRAN_FAC_D	1


//默认PID参数，使用在线调参时可以保证程序初始化时有PID参数
# define DEFAULT_PIT_KP	35
# define DEFAULT_PIT_KI	0
# define DEFAULT_PIT_KD	5000

# define DEFAULT_ROL_KP 30
# define DEFAULT_ROL_KI 0
# define DEFAULT_ROL_KD 0

# define DEFAULT_YAW_KP	0
# define DEFAULT_YAW_KI	0
# define DEFAULT_YAW_KD	0


# if ANO_USART==1
	# define ANO_USART_Handler	USART1_IRQHandler
# elif	ANO_USART==2
	# define ANO_USART_Handler	USART2_IRQHandler
# elif	ANO_USART==3
	# define ANO_USART_Handler	USART3_IRQHandler
# endif


//参数调整功能码，上位机发送
//01 ACC校准
//02 GYRO校准
//03 ACC与GYRO校准
//04 MAG校准
//05 BARO校准
# define ADJ_COMMAND	0x01	

//上位机请求返回PID参数功能码
# define REQUEST_PID  0x02

//上位机调整传感器参数功能码
//int16	int16	int16 int16 int16 int16 int16 int16 int16 int16
//THR		YAW		ROL		PIT		AUX1	AUX2	AUX3	AUX4	AUX5	AUX6
# define ADJ_SENSER	0x03

//上位机调整PID参数1功能码
//数据格式如下
//int16	int16	int16 int16 int16 int16 int16 int16 int16 
//ROL_P	ROL_I	ROL_D	PIT_P	PIT_I	PIT_D	YAW_P	YAW_I	YAW_D
# define ADJ_PID1	0x10

//上位机调整PID参数2功能码
//数据格式如下
//int16	int16	int16 int16 int16 int16 int16  int16  int16 
//ALT_P	ALT_I	ALT_D	POS_P	POS_I	POS_D	PID1_P PID1_I PID3_D 
# define ADJ_PID2	0x11


//上位机调整PID参数3功能码
//数据格式如下
//int16		int16		int16	
//PID2_P	PID2_I	PID2_D		
# define ADJ_PID3	0x12

# define ADJ_PID4 0x13
# define ADJ_PID5 0x14
# define ADJ_PID6 0x15



//上位机调整静态偏差功能码
//数据格式如下
//int16 						int16
//OFFSET_ROL*1000		OFFSET_PIT*1000
# define ADJ_OFFSET	0x16

//上位机请求进入bootmode功能码
# define BOOTMODE	0xf0


//串口中断函数，接收上位机的命令
void ANO_USART_Handler(void);


//接收上位机命令开关，1 打开 0，关闭,默认打开
extern uint8_t RX_FLAG;

typedef struct{

#if ANO_DRIVER_BY_USER==1
	void (*init)(u32 bound);//用户自己的串口初始化函数
	void (*ano_sendchar)(uint8_t byte); //用户提供的字节发送函数
#endif
	
	uint8_t RecBuff[32];
	uint8_t REC_FLAG;
	
	float alt_p,alt_i,alt_d;
	float pos_p,pos_i,pos_d;
	
	float rol_p,rol_i,rol_d; // 横滚角PID数据
	
	float pit_p,pit_i,pit_d;	//俯仰角PID数据
	
	float yaw_p,yaw_i,yaw_d;	//航向角PID数据
	
	
	float Kp[12],Ki[12],Kd[12];	//保存上位机发的12组PID数据
	
#if ANO_CON_OPENMV==1
	//int16_t 	int16_t			int16_t			int16_t		int16_t		int16_t
	//red_blob	blue_blob		green_blob	yellow		white     black
	int16_t redBlob_x,redBlob_y,blueBlob_x,blueBlob_y;   //红色、蓝色小球x,y位置
	int16_t greenBlob_x, greenBlob_y, yellowBlob_x, yellowBlob_y;
	int16_t whiteBlob_x, whiteBlob_y, blackBlob_x, blackBlob_y;
	
	float linePos;		//线的位置，相对摄像头中心
#endif
}ANO_InfoStruct;


extern ANO_InfoStruct ANO_INFO;


#if ANO_DRIVER_BY_USER==1
void ANO_Init(void (*f1)(u32 bound),void (*fun2)(uint8_t byte),u32 bound);
#else
//匿名上位机初始
void ANO_Init(u32 bound);
#endif
void ANO_InfoStrInit(void);
void ANO_SendChar(uint8_t byte);
void ANO_Response(void);
void ANO_DataProcess(void);

void ANO_StatusUpload(int16_t roll, int16_t pitch, int16_t yaw,int32_t alt_use, uint8_t armed);

void ANO_MotorPWMUpload(int16_t pwm1, int16_t pwm2, int16_t pwm3, int16_t pwm4);

void ANO_SenserDataUpload(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz,short mag_x,short mag_y,short mag_z);

void ANO_PID1Upload(void); //PID1参数上传
void ANO_PID2Upload(void);
void ANO_PID3Upload(void);
void ANO_PID4Upload(void);
void ANO_PID5Upload(void);
void ANO_PID6Upload(void);

void ANO_PID1Download(void);
void ANO_PID2Download(void);
void ANO_PID3Download(void);
void ANO_PID4Download(void);
void ANO_PID5Download(void);
void ANO_PID6Download(void);

void ANO_PitchPIDDownload(void);
void ANO_RollPIDDownload(void);
void ANO_YawPIDDownload(void);

#if ANO_CON_OPENMV==1

void ANO_BlobsPosDownload(void);
void ANO_LinePosDownload(void);

#endif


# endif

