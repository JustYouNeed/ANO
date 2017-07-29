# ifndef __ANO_TECH_H
# define __ANO_TECH_H
#if defined STM32F40_41xxx
	# include "stm32f4xx.h"
#elif defined (STM32F10X_HD)||(STM32F10X_MD)||(STM32F10X_LD)
	# include "stm32f10x.h"
#endif
	
# include "stdint.h"


// ����ͨ����ʹ�õĴ��ڣ�1Ϊ����1��2Ϊ����2.������
//Ŀǰ��F407��ʵ���˴���1��2��3
# define ANO_USART	1

//�ú꿪�ؾ����Ƿ�ʹ���û��Լ��ĵײ�����������
//������Ϊһʱ��Ҫ�û��Լ��ṩ�ײ��ͨ�Ų��֣�������ֽڵķ��ͺ��������ڵĳ�ʼ������
# define ANO_DRIVER_BY_USER		0

//�Զ��������ݺ꿪�أ����򿪣�������Զ��ڽ�����һ֡����֮����������ݣ�
//�������Զ�����������ж�ʱ�䣬�ɸ����û���Ҫѡ�񣬵���Ҫ�������ط������Ե���
//���ݴ�����ANO_DataProcess�������ù���Ĭ�Ͽ���
# define ANO_DATA_PRECESS_ON	1


//�Ƿ����OPENMV���͵�����
//1�����գ�0��������
# define ANO_CON_OPENMV				1

#if ANO_CON_OPENMV==1
	//OpenMV����ͷ����Blobs��λ��			
	//int16_t 	int16_t			int16_t			int16_t		int16_t		int16_t
	//red_blob	blue_blob		green_blob	yellow		white     black
	# define OPENMV_BLOBS1_POS	0XF1
	# define OPENMV_BLOBS2_POS	0XF2
	//OpenMV����ͷѰ�߷����ߵ�λ��
	# define OPENMV_LINE_POS	0XF3
#endif


//PID�������ϴ�������ʱʹ�õı任����
# define ANO_PID_TRAN_FAC_P 10
# define ANO_PID_TRAN_FAC_I 1000
# define ANO_PID_TRAN_FAC_D	1


//Ĭ��PID������ʹ�����ߵ���ʱ���Ա�֤�����ʼ��ʱ��PID����
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


//�������������룬��λ������
//01 ACCУ׼
//02 GYROУ׼
//03 ACC��GYROУ׼
//04 MAGУ׼
//05 BAROУ׼
# define ADJ_COMMAND	0x01	

//��λ�����󷵻�PID����������
# define REQUEST_PID  0x02

//��λ����������������������
//int16	int16	int16 int16 int16 int16 int16 int16 int16 int16
//THR		YAW		ROL		PIT		AUX1	AUX2	AUX3	AUX4	AUX5	AUX6
# define ADJ_SENSER	0x03

//��λ������PID����1������
//���ݸ�ʽ����
//int16	int16	int16 int16 int16 int16 int16 int16 int16 
//ROL_P	ROL_I	ROL_D	PIT_P	PIT_I	PIT_D	YAW_P	YAW_I	YAW_D
# define ADJ_PID1	0x10

//��λ������PID����2������
//���ݸ�ʽ����
//int16	int16	int16 int16 int16 int16 int16  int16  int16 
//ALT_P	ALT_I	ALT_D	POS_P	POS_I	POS_D	PID1_P PID1_I PID3_D 
# define ADJ_PID2	0x11


//��λ������PID����3������
//���ݸ�ʽ����
//int16		int16		int16	
//PID2_P	PID2_I	PID2_D		
# define ADJ_PID3	0x12

# define ADJ_PID4 0x13
# define ADJ_PID5 0x14
# define ADJ_PID6 0x15



//��λ��������̬ƫ�����
//���ݸ�ʽ����
//int16 						int16
//OFFSET_ROL*1000		OFFSET_PIT*1000
# define ADJ_OFFSET	0x16

//��λ���������bootmode������
# define BOOTMODE	0xf0


//�����жϺ�����������λ��������
void ANO_USART_Handler(void);


//������λ������أ�1 �� 0���ر�,Ĭ�ϴ�
extern uint8_t RX_FLAG;

typedef struct{

#if ANO_DRIVER_BY_USER==1
	void (*init)(u32 bound);//�û��Լ��Ĵ��ڳ�ʼ������
	void (*ano_sendchar)(uint8_t byte); //�û��ṩ���ֽڷ��ͺ���
#endif
	
	uint8_t RecBuff[32];
	uint8_t REC_FLAG;
	
	float alt_p,alt_i,alt_d;
	float pos_p,pos_i,pos_d;
	
	float rol_p,rol_i,rol_d; // �����PID����
	
	float pit_p,pit_i,pit_d;	//������PID����
	
	float yaw_p,yaw_i,yaw_d;	//�����PID����
	
	
	float Kp[12],Ki[12],Kd[12];	//������λ������12��PID����
	
#if ANO_CON_OPENMV==1
	//int16_t 	int16_t			int16_t			int16_t		int16_t		int16_t
	//red_blob	blue_blob		green_blob	yellow		white     black
	int16_t redBlob_x,redBlob_y,blueBlob_x,blueBlob_y;   //��ɫ����ɫС��x,yλ��
	int16_t greenBlob_x, greenBlob_y, yellowBlob_x, yellowBlob_y;
	int16_t whiteBlob_x, whiteBlob_y, blackBlob_x, blackBlob_y;
	
	float linePos;		//�ߵ�λ�ã��������ͷ����
#endif
}ANO_InfoStruct;


extern ANO_InfoStruct ANO_INFO;


#if ANO_DRIVER_BY_USER==1
void ANO_Init(void (*f1)(u32 bound),void (*fun2)(uint8_t byte),u32 bound);
#else
//������λ����ʼ
void ANO_Init(u32 bound);
#endif
void ANO_InfoStrInit(void);
void ANO_SendChar(uint8_t byte);
void ANO_Response(void);
void ANO_DataProcess(void);

void ANO_StatusUpload(int16_t roll, int16_t pitch, int16_t yaw,int32_t alt_use, uint8_t armed);

void ANO_MotorPWMUpload(int16_t pwm1, int16_t pwm2, int16_t pwm3, int16_t pwm4);

void ANO_SenserDataUpload(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz,short mag_x,short mag_y,short mag_z);

void ANO_PID1Upload(void); //PID1�����ϴ�
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

