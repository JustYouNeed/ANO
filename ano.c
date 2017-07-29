/**
  ******************************************************************************
  * �ļ����ƣ�ano.c
  * ����  		Vecoter
  * �汾��    V1.0.0
  * �������  2017 - 7 - 25
  * ˵��	    ���ļ�Ϊ������λ��ͨ������������λ���汾ΪV4.0.1��ʹ��оƬΪSTM32F407ZGT6
  *         	���ݵ��ϴ�������           
  *           PID���ߵ��ι���
  *      
  *           ����ѧϰʹ�ã�����������ҵ��;
  ******************************************************************************
**/


//2017.7.25 ��ɻ����汾�����������������λ����ͨ�ţ������ϴ���PID���ߵ���.
//					�ݲ�֧��f1оƬ����Ҫ�ֶ���ֲ


//2017.7.26 ����һ�汾������֧��F1оƬ�������ɳ����Զ��жϵ�ǰʹ�õ�оƬ��f4��f1��
//					���ݴ��������ֶ�����ĵ�����ѡ���Զ�������ֶ�����

//2017.7.29 �޸�ͨ�����ݳ��ȴ����޸����ջ������ڴ�Խ�����
//					������OPENMV��ͨ�Ų��֣����Խ��մ�OpenMV���͵�Ѱ����Ϣ����С��λ����Ϣ
//					


//ʹ��˵������ͨ��ģ����ֻ���Զ�֧��STM32F4��STM32F1ϵ�У����ʹ�õ�оƬΪF4��F1��оƬ��
//ֻҪ����ANO_Init��ʼ����ģ����У���Ҫע�͵������ط��Ĵ����жϽ��պ���,����Ҫ֧������ϵ�е�оƬ��
//�뽫ano.h�е�ANO_DRIVER_BY_USER�궨���Ϊ1�����ڵ���ANO_Init����ʱ����������������һ��Ϊ
//����ͨ�ų�ʼ���������޷���ֵ��ֻ��һ��u32���͵��βΣ����ڶ���Ϊ���͵����ֽں�����������Ϊ���ڲ����ʣ�
//��������Ҫ�Լ���д���պ�������ʹ���жϽ��գ�
//��ʹ����λ�����͵�����ʱ��������Ҫ�õ��ĵط�include "ano.h"��Ȼ��Ϳ���ֱ�ӵ���ANO_INFO����ṹ������ȡ
//��λ�����͵����ݣ�


# include "ano.h"
# include "led.h"
//
//���ݽ��սṹ��
//����λ�����յ������ݶ�������ṹ������
//��Ҫʹ�øýṹ�����������ʱ��ֱ��#include "ano.h"�Ϳ���ֱ��ʹ�øýṹ��
//
ANO_InfoStruct ANO_INFO;

//������û��ṩ�ײ��������������û���Ҫ������������ָ��
//����ģ����ɳ�ʼ��
#if ANO_DRIVER_BY_USER==1
void ANO_Init(void (*f1)(u32 bound),void (*fun2)(uint8_t byte),u32 bound)
{
	ANO_INFO.init = f1;  //��ȡ��ʼ��������ַ
	ANO_INFO.ano_sendchar = fun2; //��ȡ�ֽڷ��ͺ�����ַ
	
	ANO_INFO.init(bound);  //��ɳ�ʼ��
}
#else
////////////////////////////////////////
//�������ܣ���ʼ����������λ��ͨ�Ź���
//����˵�����ú�������ֲ��ʱ����Ҫ����оƬ�ͺ��޸�
//��ڲ�����������
//����ֵ����
////////////////////////////////////////
#if defined STM32F407xx	//���ʹ�õ���f4оƬ
void ANO_Init(u32 bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
#if ANO_USART==1			//ʹ�ô���1
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹ��GPIOAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//ʹ��USART1ʱ��
		
	//����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9����ΪUSART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10����ΪUSART1
		//USART1�˿�����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9��GPIOA10
	
# elif ANO_USART==2  		//ʹ�ô���2
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹ��GPIOAʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//ʹ��USART2ʱ��
	
	//����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); //GPIOA2����ΪUSART2
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //GPIOA3����ΪUSART2
	
		//USART1�˿�����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; //GPIOA9��GPIOA10

# elif	ANO_USART==3		//ʹ�ô���3
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //ʹ��GPIOBʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//ʹ��USART3ʱ��
	
	//����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3); //GPIOA10����ΪUSART3
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); //GPIOA11����ΪUSART3
	
	//USART1�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOB10��GPIOB11
# endif
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
			
#if ANO_USART==1
	GPIO_Init(GPIOA,&GPIO_InitStructure); 
#elif ANO_USART==2
	GPIO_Init(GPIOA,&GPIO_InitStructure); //
#elif ANO_USART==3
	GPIO_Init(GPIOB,&GPIO_InitStructure); //
#endif

   //USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	
#if ANO_USART==1
  USART_Init(USART1, &USART_InitStructure); //��ʼ������1
  USART_Cmd(USART1, ENABLE);  //ʹ�ܴ���1 
#elif ANO_USART==2
	USART_Init(USART2, &USART_InitStructure); //��ʼ������2
  USART_Cmd(USART2, ENABLE);  //ʹ�ܴ���1 
#elif ANO_USART==3
  USART_Init(USART3, &USART_InitStructure); //��ʼ������3
  USART_Cmd(USART3, ENABLE);  //ʹ�ܴ���1 
#endif

#if ANO_USART==1
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//��������ж�
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//����1�ж�ͨ
#elif ANO_USART==2
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//��������ж�
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//����2�ж�ͨ
#elif ANO_USART==3
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//��������ж�
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;//����3�ж�ͨ
#endif

	//USART NVIC ����
	//���ȼ�����Ϊ��ߣ���ֹ��������ʱ����ϣ����¶���
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//��ռ���ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//�����ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
	
	//��ؽṹ���ʼ��
	ANO_InfoStrInit();
}
#elif defined (STM32F10X_HD)||(STM32F10X_MD)||(STM32F10X_LD)
void ANO_Init(u32 bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
#if ANO_USART==1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��USART1��GPIOAʱ��
#elif	ANO_USART==2
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART2|RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��USART2��GPIOAʱ��
#elif ANO_USART==3
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART3|RCC_APB2Periph_GPIOB, ENABLE);	//ʹ��USART3��GPIOBʱ��
#endif

#if ANO_USART==1
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.9
   
  //USART1_RX	  GPIOA.10��ʼ��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.10 
#elif ANO_USART==2
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA2
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA2
   
  //USART1_RX	  GPIOA.3��ʼ��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PA3
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.3
#elif ANO_USART==3
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
  GPIO_Init(GPIOB, &GPIO_InitStructure);//
   
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
  GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIOB
#endif
 
 
 

  //Usart1 NVIC ����

	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
  
   //USART ��ʼ������

	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
#if ANO_USART==1
  USART_Init(USART1, &USART_InitStructure); //��ʼ������1
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
  USART_Cmd(USART1, ENABLE);  
#elif	ANO_USART==2
  USART_Init(USART2, &USART_InitStructure); //��ʼ������1
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
  USART_Cmd(USART2, ENABLE);  
#elif	ANO_USART==3
  USART_Init(USART3, &USART_InitStructure); //��ʼ������1
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
  USART_Cmd(USART3, ENABLE);  
#endif

	ANO_InfoStrInit();
}

#endif


#endif 


////////////////////////////////////////////////////////
//�������ܣ���ʼ��ANO_INFO�ṹ��
//��ڲ�������
//����ֵ:��
//˵����
////////////////////////////////////////////////////////
void ANO_InfoStrInit(void)
{
	ANO_INFO.Kd[0] = 0;
	ANO_INFO.Ki[0] = 0;
	ANO_INFO.Kp[0] = 0;
	
	ANO_INFO.pit_p = DEFAULT_PIT_KP;
	ANO_INFO.pit_i = DEFAULT_PIT_KI;
	ANO_INFO.pit_d = DEFAULT_PIT_KD;
	
	ANO_INFO.rol_p = DEFAULT_ROL_KP;
	ANO_INFO.rol_i = DEFAULT_ROL_KI;
	ANO_INFO.rol_d = DEFAULT_ROL_KD;
	
	ANO_INFO.yaw_p = DEFAULT_YAW_KP;
	ANO_INFO.yaw_i = DEFAULT_YAW_KI;
	ANO_INFO.yaw_d = DEFAULT_YAW_KD;
	
	ANO_INFO.REC_FLAG = 0;
}
///////////////////////////////////
//�������ܣ�����֡���պ���
//����˵�����ú�������ֲ��ʱ��Ϊ����ʵ�ֺ���
//��ڲ�������
//����ֵ����
///////////////////////////////////
void ANO_USART_Handler(void)
{
	uint8_t uByte;  //�ֽڽ����ݴ�
	uint8_t i = 0;  //
	uint8_t checkSum = 0; //
	static uint8_t uCnt = 0;
#if ANO_USART==1															//���ʹ�ô���1
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  
	{
		uByte =USART_ReceiveData(USART1);
#elif ANO_USART==2														//���ʹ�ô���2
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  
	{
		uByte =USART_ReceiveData(USART2);
#elif	ANO_USART==3														//���ʹ�ô���3
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  
	{
		uByte =USART_ReceiveData(USART3);
#endif
		
		switch(uCnt)
		{
			case 0X00:
			{
					if(0XAA == uByte)	
					{
						
						ANO_INFO.RecBuff[uCnt++] = uByte;  //֡��ʼ�ж�
					}
					else uCnt = 0X00;
			}break;
			case 0X01:
			{
				if(0XAF == uByte)	
				{
					ANO_INFO.RecBuff[uCnt++] = uByte;//֡��ʼ�ж�
					
				}
				else uCnt = 0X00;
			}break;
			case 0X02:ANO_INFO.RecBuff[uCnt++] = uByte; break;  //������
			case 0X03:ANO_INFO.RecBuff[uCnt++] = uByte; break; //���ݳ��ȣ���ȥ�������Լ���ʼ֡������
			default:if(uCnt < (ANO_INFO.RecBuff[3] + 0X05)) ANO_INFO.RecBuff[uCnt++] = uByte;break;//��������
		}

		if(uCnt == (ANO_INFO.RecBuff[3] + 0X05))  //�Ѿ���������������֡
		{
			uCnt = 0;
			
			for(i = 0;i < ANO_INFO.RecBuff[3] + 0X04; i++) checkSum += ANO_INFO.RecBuff[i]; //��������
			if((checkSum&0xff) != ANO_INFO.RecBuff[ANO_INFO.RecBuff[3] + 0X04]) ANO_INFO.REC_FLAG = 0; //���մ���
			else 
			{
				LED1 = ~LED1;
				ANO_INFO.REC_FLAG = ANO_INFO.RecBuff[2];		//���ݼ������󣬱��湦����
		#if ANO_DATA_PRECESS_ON==1  //ѡ���Ƿ������һ֡���ݺ��Զ�����
				ANO_DataProcess();
		#endif
			}
		}
		
  } 
}
	
#if ANO_DRIVER_BY_USER==0
////////////////////////////////////////
//�������ܣ�ͨ�����ڷ���һ���ֽڵ�����
//����˵�����ú�������ֲ��ʱ��Ϊ����ʵ�ֺ���
//��ڲ�����byte��Ҫ���͵�8λ����
//����ֵ����
/////////////////////////////////////////
void ANO_SendChar(uint8_t byte)
{
#if ANO_USART==1
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET); 
    USART_SendData(USART1,byte);   
#elif ANO_USART==2
	while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET); 
    USART_SendData(USART2,byte); 
#elif ANO_USART==3
	while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET); 
		USART_SendData(USART3,byte); 
#endif
}
#endif
///////////////////////////////////////////////////////////////////////////////////
//�������ܣ�������λ�������ϴ�
//��ڲ�����buff,Ҫ���͵��������飬funCode��������;len���ݳ��ȣ���󲻳���27���ֽ�
//����ֵ��0�����ݳ��ȳ������ƣ�1�����������2�����ͳɹ�
///////////////////////////////////////////////////////////////////////////////////
uint8_t ANO_DataUpload(uint8_t *buff, uint8_t funCode, uint8_t len)
{
	uint8_t SendBuff[32];
	uint8_t i;
	if(len>27)return 0;//���ݳ��ȳ�������
	if(funCode>0xff) return 1;//���������
		
	SendBuff[0] = 0XAA; //֡��ʼ
	SendBuff[1] = 0XAA; //֡��ʼ
	SendBuff[2] = funCode;  //������
	SendBuff[3] = len;   //���ݳ��ȣ���ȥ��ʼ��͹������Լ�����
	
	//��Ҫ���͵����ݸ��Ƶ�������
	for(i = 0; i < len; i++) SendBuff[i + 4] = buff[i];
	//����У���
	for(i = 0; i< len + 4; i++) SendBuff[len + 4] += buff[i];
	//ѭ����������
	for(i = 0; i< len + 5; i++) 
#if ANO_DRIVER_BY_USER==1
		ANO_INFO.ano_sendchar(SendBuff[i]);
#else
		ANO_SendChar(SendBuff[i]);
#endif
	return 2;
}

///////////////////////////////////////////////////////////////////////////////////
//�������ܣ��ɿ�״̬�ϴ�
//��ڲ�����roll,pitch,yaw,alt_use,����״̬
//����ֵ����
//////////////////////////////////////////////////////////////////////////////////
void ANO_StatusUpload(int16_t roll, int16_t pitch, int16_t yaw,int32_t alt_use, uint8_t armed)
{
	uint8_t Buff[11];
	uint8_t i;
	for(i = 0;i< 11; i++) Buff[i] = 0x00;
	Buff[0] = (roll>>8)&0xff;
	Buff[1] = roll&0xff;
	Buff[2] = (pitch >> 8) & 0xff;
	Buff[3] = pitch & 0xff;
	Buff[4] = (yaw >> 8) & 0xff;
	Buff[5] = yaw & 0xff;
	
	Buff[6] = (alt_use >> 24) & 0xff;
	Buff[7] = (alt_use >> 16) & 0xff;
	Buff[8] = (alt_use >> 8) & 0xff;
	Buff[9] = alt_use & 0xff;
	
	Buff[10] = armed & 0xff;
	
	ANO_DataUpload(Buff,0x01,11);
}

///////////////////////////////////////////////////////////////////////////////////
//�������ܣ��ɿ���̬�ϴ�
//��ڲ�����aacx,X����ٶ�,aacy,aacz;gyrox��X����ٶ�,gyroy,gyroz;mag_x,�Ŵ���������
//����ֵ����
//////////////////////////////////////////////////////////////////////////////////
void ANO_SenserDataUpload(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz,short mag_x,short mag_y,short mag_z)
{
	uint8_t Buff[18];
	uint8_t i;
	for(i = 0;i< 18; i++) Buff[i] = 0x00;
	
	Buff[0] = (aacx>>8)&0xff;
	Buff[1] = aacx&0xff;
	
	Buff[2] = (aacy >> 8) & 0xff;
	Buff[3] = aacy & 0xff;
	
	Buff[4] = (aacz >> 8) & 0xff;
	Buff[5] = aacz & 0xff;
	
	Buff[6] = (gyrox >> 8) & 0xff;
	Buff[7] = gyrox & 0xff;
	
	Buff[8] = (gyroy >> 8) & 0xff;
	Buff[9] = gyroy & 0xff;
	
	Buff[10] = (gyroz >> 8) & 0xff;
	Buff[11] = gyroz & 0xff;
	
	Buff[12] = (mag_x >> 8) & 0xff;
	Buff[13] = mag_x & 0xff;
	
	Buff[14] = (mag_y >> 8) & 0xff;
	Buff[15] = mag_y & 0xff;
	
	Buff[16] = (mag_z >> 8) & 0xff;
	Buff[17] = mag_z & 0xff;
	
	ANO_DataUpload(Buff,0x02,18);
}

///////////////////////////////////////////////////////////////////////////////////
//�������ܣ��ϴ���һ��PID����
//��ڲ������ޣ�����Ҫ�ϴ������ݶ���ANO_INFO�ṹ����
//����ֵ����
//////////////////////////////////////////////////////////////////////////////////
void ANO_PID1Upload(void)
{
	uint8_t Buff[18];
	uint8_t i;
	short temp;
	for(i = 0;i< 18; i++) Buff[i] = 0x00;
	
	temp = (short)(ANO_INFO.rol_p*ANO_PID_TRAN_FAC_P);
	Buff[0] = (temp>>8)&0xff;
	Buff[1] = temp&0xff;
	
	temp = (short)(ANO_INFO.rol_i*ANO_PID_TRAN_FAC_I);
	Buff[2] = (temp >> 8) & 0xff;
	Buff[3] = temp & 0xff;
	
	temp = (short)(ANO_INFO.rol_d*ANO_PID_TRAN_FAC_D);
	Buff[4] = (temp >> 8) & 0xff;
	Buff[5] = temp & 0xff;
	
	temp = (short)(ANO_INFO.pit_p*ANO_PID_TRAN_FAC_P);
	Buff[6] = (temp >> 8) & 0xff;
	Buff[7] = temp & 0xff;
	
	temp = (short)(ANO_INFO.pit_i*ANO_PID_TRAN_FAC_I);
	Buff[8] = (temp >> 8) & 0xff;
	Buff[9] = temp & 0xff;
	
	temp = (short)(ANO_INFO.pit_d*ANO_PID_TRAN_FAC_D);
	Buff[10] = (temp >> 8) & 0xff;
	Buff[11] = temp & 0xff;
	
	temp = (short)(ANO_INFO.yaw_p*ANO_PID_TRAN_FAC_P);
	Buff[12] = (temp >> 8) & 0xff;
	Buff[13] = temp & 0xff;
	
	temp = (short)(ANO_INFO.yaw_i*ANO_PID_TRAN_FAC_I);
	Buff[14] = (temp >> 8) & 0xff;
	Buff[15] = temp & 0xff;
	
	temp = (short)(ANO_INFO.yaw_d*ANO_PID_TRAN_FAC_D);
	Buff[16] = (temp >> 8) & 0xff;
	Buff[17] = temp & 0xff;
//	
	ANO_DataUpload(Buff,0x10,18);
}

///////////////////////////////////////////////////////////////////////////////////
//�������ܣ��ϴ��ڶ���PID����
//��ڲ�������
//����ֵ����
//////////////////////////////////////////////////////////////////////////////////
void ANO_PID2Upload(void)
{
	uint8_t Buff[18];
	uint8_t i;
	short temp = 0;
	for(i = 0;i< 18; i++) Buff[i] = 0x00;
	
	temp = (short)(ANO_INFO.alt_p*ANO_PID_TRAN_FAC_P);
	Buff[0] = (temp>>8)&0xff;
	Buff[1] = temp&0xff;
	
	temp = (short)(ANO_INFO.alt_i*ANO_PID_TRAN_FAC_I);
	Buff[2] = (temp >> 8) & 0xff;
	Buff[3] = temp & 0xff;
	
	temp = (short)(ANO_INFO.alt_d*ANO_PID_TRAN_FAC_D);
	Buff[4] = (temp >> 8) & 0xff;
	Buff[5] = temp & 0xff;
	
	temp = (short)(ANO_INFO.pos_p*ANO_PID_TRAN_FAC_P);
	Buff[6] = (temp >> 8) & 0xff;
	Buff[7] = temp & 0xff;
	
	temp = (short)(ANO_INFO.pos_i*ANO_PID_TRAN_FAC_I);
	Buff[8] = (temp >> 8) & 0xff;
	Buff[9] = temp & 0xff;
	
	temp = (short)(ANO_INFO.pos_d*ANO_PID_TRAN_FAC_D);
	Buff[10] = (temp >> 8) & 0xff;
	Buff[11] = temp & 0xff;
	
	temp = (short)(ANO_INFO.Kp[0]*ANO_PID_TRAN_FAC_P);
	Buff[12] = (temp >> 8) & 0xff;
	Buff[13] = temp & 0xff;
	
	temp = (short)(ANO_INFO.Ki[0]*ANO_PID_TRAN_FAC_I);
	Buff[14] = (temp >> 8) & 0xff;
	Buff[15] = temp & 0xff;
	
	temp = (short)(ANO_INFO.Kd[0]*ANO_PID_TRAN_FAC_D);
	Buff[16] = (temp >> 8) & 0xff;
	Buff[17] = temp & 0xff;
	//	
	ANO_DataUpload(Buff,0x11,18);
}

///////////////////////////////////////////////////////////////////////////////////
//�������ܣ��ϴ�������PID����
//��ڲ�������
//����ֵ����
//////////////////////////////////////////////////////////////////////////////////
void ANO_PID3Upload(void)
{
	uint8_t Buff[18];
	uint8_t i;
	short temp = 0;
	for(i = 0;i< 18; i++) Buff[i] = 0x00;
	
	temp = (short)(ANO_INFO.Kp[1]*ANO_PID_TRAN_FAC_P);
	Buff[0] = (temp>>8)&0xff;
	Buff[1] = temp&0xff;
	
	temp = (short)(ANO_INFO.Ki[1]*ANO_PID_TRAN_FAC_I);
	Buff[2] = (temp >> 8) & 0xff;
	Buff[3] = temp & 0xff;
	
	temp = (short)(ANO_INFO.Kd[1]*ANO_PID_TRAN_FAC_D);
	Buff[4] = (temp >> 8) & 0xff;
	Buff[5] = temp & 0xff;

		temp = (short)(ANO_INFO.Kp[2]*ANO_PID_TRAN_FAC_P);
	Buff[6] = (temp>>8)&0xff;
	Buff[7] = temp&0xff;
	
	temp = (short)(ANO_INFO.Ki[2]*ANO_PID_TRAN_FAC_I);
	Buff[8] = (temp >> 8) & 0xff;
	Buff[9] = temp & 0xff;
	
	temp = (short)(ANO_INFO.Kd[2]*ANO_PID_TRAN_FAC_D);
	Buff[10] = (temp >> 8) & 0xff;
	Buff[11] = temp & 0xff;

		temp = (short)(ANO_INFO.Kp[3]*ANO_PID_TRAN_FAC_P);
	Buff[12] = (temp>>8)&0xff;
	Buff[13] = temp&0xff;
	
	temp = (short)(ANO_INFO.Ki[3]*ANO_PID_TRAN_FAC_I);
	Buff[14] = (temp >> 8) & 0xff;
	Buff[15] = temp & 0xff;
	
	temp = (short)(ANO_INFO.Kd[3]*ANO_PID_TRAN_FAC_D);
	Buff[16] = (temp >> 8) & 0xff;
	Buff[17] = temp & 0xff;
//	
	ANO_DataUpload(Buff,0x12,18);
}

///////////////////////////////////////////////////////////////////////////////////
//�������ܣ��ϴ�������PID����
//��ڲ�������
//����ֵ����
//////////////////////////////////////////////////////////////////////////////////
void ANO_PID4Upload(void)
{
	uint8_t Buff[18];
	uint8_t i;
	short temp = 0;
	for(i = 0;i< 18; i++) Buff[i] = 0x00;
	
	temp = (short)(ANO_INFO.Kp[4]*ANO_PID_TRAN_FAC_P);
	Buff[0] = (temp>>8)&0xff;
	Buff[1] = temp&0xff;
	
	temp = (short)(ANO_INFO.Ki[4]*ANO_PID_TRAN_FAC_I);
	Buff[2] = (temp >> 8) & 0xff;
	Buff[3] = temp & 0xff;
	
	temp = (short)(ANO_INFO.Kd[4]*ANO_PID_TRAN_FAC_D);
	Buff[4] = (temp >> 8) & 0xff;
	Buff[5] = temp & 0xff;

		temp = (short)(ANO_INFO.Kp[5]*ANO_PID_TRAN_FAC_P);
	Buff[6] = (temp>>8)&0xff;
	Buff[7] = temp&0xff;
	
	temp = (short)(ANO_INFO.Ki[5]*ANO_PID_TRAN_FAC_I);
	Buff[8] = (temp >> 8) & 0xff;
	Buff[9] = temp & 0xff;
	
	temp = (short)(ANO_INFO.Kd[5]*ANO_PID_TRAN_FAC_D);
	Buff[10] = (temp >> 8) & 0xff;
	Buff[11] = temp & 0xff;

		temp = (short)(ANO_INFO.Kp[6]*ANO_PID_TRAN_FAC_P);
	Buff[12] = (temp>>8)&0xff;
	Buff[13] = temp&0xff;
	
	temp = (short)(ANO_INFO.Ki[6]*ANO_PID_TRAN_FAC_I);
	Buff[14] = (temp >> 8) & 0xff;
	Buff[15] = temp & 0xff;
	
	temp = (short)(ANO_INFO.Kd[6]*ANO_PID_TRAN_FAC_D);
	Buff[16] = (temp >> 8) & 0xff;
	Buff[17] = temp & 0xff;
//	
	ANO_DataUpload(Buff,0x13,18);
}

///////////////////////////////////////////////////////////////////////////////////
//�������ܣ��ϴ�������PID����
//��ڲ�������
//����ֵ����
//////////////////////////////////////////////////////////////////////////////////
void ANO_PID5Upload(void)
{
	uint8_t Buff[18];
	uint8_t i;
	short temp = 0;
	for(i = 0;i< 18; i++) Buff[i] = 0x00;
	
	temp = (short)(ANO_INFO.Kp[7]*ANO_PID_TRAN_FAC_P);
	Buff[0] = (temp>>8)&0xff;
	Buff[1] = temp&0xff;
	
	temp = (short)(ANO_INFO.Ki[7]*ANO_PID_TRAN_FAC_I);
	Buff[2] = (temp >> 8) & 0xff;
	Buff[3] = temp & 0xff;
	
	temp = (short)(ANO_INFO.Kd[7]*ANO_PID_TRAN_FAC_D);
	Buff[4] = (temp >> 8) & 0xff;
	Buff[5] = temp & 0xff;

		temp = (short)(ANO_INFO.Kp[8]*ANO_PID_TRAN_FAC_P);
	Buff[6] = (temp>>8)&0xff;
	Buff[7] = temp&0xff;
	
	temp = (short)(ANO_INFO.Ki[8]*ANO_PID_TRAN_FAC_I);
	Buff[8] = (temp >> 8) & 0xff;
	Buff[9] = temp & 0xff;
	
	temp = (short)(ANO_INFO.Kd[8]*ANO_PID_TRAN_FAC_D);
	Buff[10] = (temp >> 8) & 0xff;
	Buff[11] = temp & 0xff;

		temp = (short)(ANO_INFO.Kp[9]*ANO_PID_TRAN_FAC_P);
	Buff[12] = (temp>>8)&0xff;
	Buff[13] = temp&0xff;
	
	temp = (short)(ANO_INFO.Ki[9]*ANO_PID_TRAN_FAC_I);
	Buff[14] = (temp >> 8) & 0xff;
	Buff[15] = temp & 0xff;
	
	temp = (short)(ANO_INFO.Kd[9]*ANO_PID_TRAN_FAC_D);
	Buff[16] = (temp >> 8) & 0xff;
	Buff[17] = temp & 0xff;
	
	ANO_DataUpload(Buff,0x14,18);
}

///////////////////////////////////////////////////////////////////////////////////
//�������ܣ��ϴ�������PID����
//��ڲ�������
//����ֵ����
//////////////////////////////////////////////////////////////////////////////////
void ANO_PID6Upload(void)
{
	uint8_t Buff[18];
	uint8_t i;
	short temp = 0;
	for(i = 0;i< 18; i++) Buff[i] = 0x00;
	
	temp = (short)(ANO_INFO.Kp[10]*ANO_PID_TRAN_FAC_P);
	Buff[0] = (temp>>8)&0xff;
	Buff[1] = temp&0xff;
	
	temp = (short)(ANO_INFO.Ki[10]*ANO_PID_TRAN_FAC_I);
	Buff[2] = (temp >> 8) & 0xff;
	Buff[3] = temp & 0xff;
	
	temp = (short)(ANO_INFO.Kd[10]*ANO_PID_TRAN_FAC_D);
	Buff[4] = (temp >> 8) & 0xff;
	Buff[5] = temp & 0xff;

	temp = (short)(ANO_INFO.Kp[11]*ANO_PID_TRAN_FAC_P);
	Buff[6] = (temp>>8)&0xff;
	Buff[7] = temp&0xff;
	
	temp = (short)(ANO_INFO.Ki[11]*ANO_PID_TRAN_FAC_I);
	Buff[8] = (temp >> 8) & 0xff;
	Buff[9] = temp & 0xff;
	
	temp = (short)(ANO_INFO.Kd[11]*ANO_PID_TRAN_FAC_D);
	Buff[10] = (temp >> 8) & 0xff;
	Buff[11] = temp & 0xff;

	ANO_DataUpload(Buff,0x15,12);
}

////////////////////////////////////////////////
//�������ܣ��ظ���λ��У����
//��ڲ�������
//����ֵ����
//˵�������յ���λ�����������������Ӧ���øú�����Ӧ��λ������ʾ���ݽ������󣬸ú��������ݴ�������Զ����ã�
////////////////////////////////////////////////
void ANO_Response(void)
{
	uint8_t SendBuff[8];
	uint8_t i;
	
	SendBuff[0] = 0XAA;//֡��ʼ
	SendBuff[1] = 0XAA;
	SendBuff[2] = 0XF0;
	SendBuff[3] = 0x03;
	SendBuff[4] = 0XBA;
	SendBuff[5] = (ANO_INFO.RecBuff[ANO_INFO.RecBuff[3] + 0x04]>>8)&0xff;
	SendBuff[6] = ANO_INFO.RecBuff[ANO_INFO.RecBuff[3] + 0x04]&0xff;
	
	for(i = 0;i<7;i++) SendBuff[7] += SendBuff[i];
	for(i = 0;i<8;i++)
#if ANO_DRIVER_BY_USER==1
	ANO_INFO.ano_sendchar(SendBuff[i]);
#else
	ANO_SendChar(SendBuff[i]);
#endif
}

///////////////////////////////////////////////////////////////////////////////////
//�������ܣ����յ�һ��PID����
//��ڲ�������
//����ֵ����
//////////////////////////////////////////////////////////////////////////////////
void ANO_PID1Download(void)
{
	//ȡǰ������8λ�����ݺϲ���һ��16λ�����ݣ���ǿ��ת����һ��float�͵�����
	//ת����ɺ������Ӧ�Ĵ�������
	ANO_INFO.rol_p = (float)((int16_t)((ANO_INFO.RecBuff[4]<<8)|(ANO_INFO.RecBuff[5])))/ANO_PID_TRAN_FAC_P;
	ANO_INFO.rol_i = (float)((int16_t)(ANO_INFO.RecBuff[6]<<8)|(ANO_INFO.RecBuff[7]))/ANO_PID_TRAN_FAC_I;
	ANO_INFO.rol_d = (float)((int16_t)(ANO_INFO.RecBuff[8]<<8)|(ANO_INFO.RecBuff[9]))/ANO_PID_TRAN_FAC_D;
	
	ANO_INFO.pit_p = (float)((int16_t)((ANO_INFO.RecBuff[10]<<8)|(ANO_INFO.RecBuff[11])))/ANO_PID_TRAN_FAC_P;
	ANO_INFO.pit_i = (float)((int16_t)(ANO_INFO.RecBuff[12]<<8)|(ANO_INFO.RecBuff[13]))/ANO_PID_TRAN_FAC_I;
	ANO_INFO.pit_d = (float)((int16_t)(ANO_INFO.RecBuff[14]<<8)|(ANO_INFO.RecBuff[15]))/ANO_PID_TRAN_FAC_D;

	ANO_INFO.yaw_p = (float)((int16_t)((ANO_INFO.RecBuff[16]<<8)|(ANO_INFO.RecBuff[17])))/ANO_PID_TRAN_FAC_P;
	ANO_INFO.yaw_i = (float)((int16_t)(ANO_INFO.RecBuff[18]<<8)|(ANO_INFO.RecBuff[19]))/ANO_PID_TRAN_FAC_I;
	ANO_INFO.yaw_d = (float)((int16_t)(ANO_INFO.RecBuff[20]<<8)|(ANO_INFO.RecBuff[21]))/ANO_PID_TRAN_FAC_D;
}

///////////////////////////////////////////////////////////////////////////////////
//�������ܣ����յڶ���PID����
//��ڲ�������
//����ֵ����
//////////////////////////////////////////////////////////////////////////////////
void ANO_PID2Download(void)
{
	ANO_INFO.alt_p = (float)((int16_t)((ANO_INFO.RecBuff[4]<<8)|(ANO_INFO.RecBuff[5])))/ANO_PID_TRAN_FAC_P;
	ANO_INFO.alt_i = (float)((int16_t)(ANO_INFO.RecBuff[6]<<8)|(ANO_INFO.RecBuff[7]))/ANO_PID_TRAN_FAC_I;
	ANO_INFO.alt_d = (float)((int16_t)(ANO_INFO.RecBuff[8]<<8)|(ANO_INFO.RecBuff[9]))/ANO_PID_TRAN_FAC_D;
	
	ANO_INFO.pos_p = (float)((int16_t)((ANO_INFO.RecBuff[10]<<8)|(ANO_INFO.RecBuff[11])))/ANO_PID_TRAN_FAC_P;
	ANO_INFO.pos_i = (float)((int16_t)(ANO_INFO.RecBuff[12]<<8)|(ANO_INFO.RecBuff[13]))/ANO_PID_TRAN_FAC_I;
	ANO_INFO.pos_d = (float)((int16_t)(ANO_INFO.RecBuff[14]<<8)|(ANO_INFO.RecBuff[15]))/ANO_PID_TRAN_FAC_D;

	ANO_INFO.Kp[0] = (float)((int16_t)((ANO_INFO.RecBuff[16]<<8)|(ANO_INFO.RecBuff[17])))/ANO_PID_TRAN_FAC_P;
	ANO_INFO.Ki[0] = (float)((int16_t)(ANO_INFO.RecBuff[18]<<8)|(ANO_INFO.RecBuff[19]))/ANO_PID_TRAN_FAC_I;
	ANO_INFO.Kd[0] = (float)((int16_t)(ANO_INFO.RecBuff[20]<<8)|(ANO_INFO.RecBuff[21]))/ANO_PID_TRAN_FAC_D;
	
}

///////////////////////////////////////////////////////////////////////////////////
//�������ܣ����յ�����PID����
//��ڲ�������
//����ֵ����
//////////////////////////////////////////////////////////////////////////////////
void ANO_PID3Download(void)
{
	ANO_INFO.Kp[1] = (float)((int16_t)((ANO_INFO.RecBuff[4]<<8)|(ANO_INFO.RecBuff[5])))/ANO_PID_TRAN_FAC_P;
	ANO_INFO.Ki[1] = (float)((int16_t)(ANO_INFO.RecBuff[6]<<8)|(ANO_INFO.RecBuff[7]))/ANO_PID_TRAN_FAC_I;
	ANO_INFO.Kd[1] = (float)((int16_t)(ANO_INFO.RecBuff[8]<<8)|(ANO_INFO.RecBuff[9]))/ANO_PID_TRAN_FAC_D;
	
	ANO_INFO.Kp[2] = (float)((int16_t)((ANO_INFO.RecBuff[10]<<8)|(ANO_INFO.RecBuff[11])))/ANO_PID_TRAN_FAC_P;
	ANO_INFO.Ki[2] = (float)((int16_t)(ANO_INFO.RecBuff[12]<<8)|(ANO_INFO.RecBuff[13]))/ANO_PID_TRAN_FAC_I;
	ANO_INFO.Kd[2] = (float)((int16_t)(ANO_INFO.RecBuff[14]<<8)|(ANO_INFO.RecBuff[15]))/ANO_PID_TRAN_FAC_D;

	ANO_INFO.Kp[3] = (float)((int16_t)((ANO_INFO.RecBuff[16]<<8)|(ANO_INFO.RecBuff[17])))/ANO_PID_TRAN_FAC_P;
	ANO_INFO.Ki[3] = (float)((int16_t)(ANO_INFO.RecBuff[18]<<8)|(ANO_INFO.RecBuff[19]))/ANO_PID_TRAN_FAC_I;
	ANO_INFO.Kd[3] = (float)((int16_t)(ANO_INFO.RecBuff[20]<<8)|(ANO_INFO.RecBuff[21]))/ANO_PID_TRAN_FAC_D;
}

///////////////////////////////////////////////////////////////////////////////////
//�������ܣ����յ�����PID����
//��ڲ�������
//����ֵ����
//////////////////////////////////////////////////////////////////////////////////
void ANO_PID4Download(void)
{
	ANO_INFO.Kp[4] = (float)((int16_t)((ANO_INFO.RecBuff[4]<<8)|(ANO_INFO.RecBuff[5])))/ANO_PID_TRAN_FAC_P;
	ANO_INFO.Ki[4] = (float)((int16_t)(ANO_INFO.RecBuff[6]<<8)|(ANO_INFO.RecBuff[7]))/ANO_PID_TRAN_FAC_I;
	ANO_INFO.Kd[4] = (float)((int16_t)(ANO_INFO.RecBuff[8]<<8)|(ANO_INFO.RecBuff[9]))/ANO_PID_TRAN_FAC_D;
	
	ANO_INFO.Kp[5] = (float)((int16_t)((ANO_INFO.RecBuff[10]<<8)|(ANO_INFO.RecBuff[11])))/ANO_PID_TRAN_FAC_P;
	ANO_INFO.Ki[5] = (float)((int16_t)(ANO_INFO.RecBuff[12]<<8)|(ANO_INFO.RecBuff[13]))/ANO_PID_TRAN_FAC_I;
	ANO_INFO.Kd[5] = (float)((int16_t)(ANO_INFO.RecBuff[14]<<8)|(ANO_INFO.RecBuff[15]))/ANO_PID_TRAN_FAC_D;

	ANO_INFO.Kp[6] = (float)((int16_t)((ANO_INFO.RecBuff[16]<<8)|(ANO_INFO.RecBuff[17])))/ANO_PID_TRAN_FAC_P;
	ANO_INFO.Ki[6] = (float)((int16_t)(ANO_INFO.RecBuff[18]<<8)|(ANO_INFO.RecBuff[19]))/ANO_PID_TRAN_FAC_I;
	ANO_INFO.Kd[6] = (float)((int16_t)(ANO_INFO.RecBuff[20]<<8)|(ANO_INFO.RecBuff[21]))/ANO_PID_TRAN_FAC_D;
}

///////////////////////////////////////////////////////////////////////////////////
//�������ܣ����յ�����PID����
//��ڲ�������
//����ֵ����
//////////////////////////////////////////////////////////////////////////////////
void ANO_PID5Download(void)
{
	ANO_INFO.Kp[7] = (float)((int16_t)((ANO_INFO.RecBuff[4]<<8)|(ANO_INFO.RecBuff[5])))/ANO_PID_TRAN_FAC_P;
	ANO_INFO.Ki[7] = (float)((int16_t)(ANO_INFO.RecBuff[6]<<8)|(ANO_INFO.RecBuff[7]))/ANO_PID_TRAN_FAC_I;
	ANO_INFO.Kd[7] = (float)((int16_t)(ANO_INFO.RecBuff[8]<<8)|(ANO_INFO.RecBuff[9]))/ANO_PID_TRAN_FAC_D;
	
	ANO_INFO.Kp[8] = (float)((int16_t)((ANO_INFO.RecBuff[10]<<8)|(ANO_INFO.RecBuff[11])))/ANO_PID_TRAN_FAC_P;
	ANO_INFO.Ki[8] = (float)((int16_t)(ANO_INFO.RecBuff[12]<<8)|(ANO_INFO.RecBuff[13]))/ANO_PID_TRAN_FAC_I;
	ANO_INFO.Kd[8] = (float)((int16_t)(ANO_INFO.RecBuff[14]<<8)|(ANO_INFO.RecBuff[15]))/ANO_PID_TRAN_FAC_D;

	ANO_INFO.Kp[9] = (float)((int16_t)((ANO_INFO.RecBuff[16]<<8)|(ANO_INFO.RecBuff[17])))/ANO_PID_TRAN_FAC_P;
	ANO_INFO.Ki[9] = (float)((int16_t)(ANO_INFO.RecBuff[18]<<8)|(ANO_INFO.RecBuff[19]))/ANO_PID_TRAN_FAC_I;
	ANO_INFO.Kd[9] = (float)((int16_t)(ANO_INFO.RecBuff[20]<<8)|(ANO_INFO.RecBuff[21]))/ANO_PID_TRAN_FAC_D;
}

///////////////////////////////////////////////////////////////////////////////////
//�������ܣ����յ�����PID����
//��ڲ�������
//����ֵ����
//////////////////////////////////////////////////////////////////////////////////
void ANO_PID6Download(void)
{
	ANO_INFO.Kp[10] = (float)((int16_t)((ANO_INFO.RecBuff[4]<<8)|(ANO_INFO.RecBuff[5])))/ANO_PID_TRAN_FAC_P;
	ANO_INFO.Ki[10] = (float)((int16_t)(ANO_INFO.RecBuff[6]<<8)|(ANO_INFO.RecBuff[7]))/ANO_PID_TRAN_FAC_I;
	ANO_INFO.Kd[10] = (float)((int16_t)(ANO_INFO.RecBuff[8]<<8)|(ANO_INFO.RecBuff[9]))/ANO_PID_TRAN_FAC_D;
	
	ANO_INFO.Kp[11] = (float)((int16_t)((ANO_INFO.RecBuff[10]<<8)|(ANO_INFO.RecBuff[11])))/ANO_PID_TRAN_FAC_P;
	ANO_INFO.Ki[11] = (float)((int16_t)(ANO_INFO.RecBuff[12]<<8)|(ANO_INFO.RecBuff[13]))/ANO_PID_TRAN_FAC_I;
	ANO_INFO.Kd[11] = (float)((int16_t)(ANO_INFO.RecBuff[14]<<8)|(ANO_INFO.RecBuff[15]))/ANO_PID_TRAN_FAC_D;
}

#if ANO_CON_OPENMV==1

//�������ܣ����մ�OPENMV�Ϸ��͹������ߵ�λ��
//��ڲ�������
//����ֵ����
//˵������
void ANO_LinePosDownload(void)
{
	ANO_INFO.linePos = (float)(int16_t)(((ANO_INFO.RecBuff[4]&0xff)<<8)|(ANO_INFO.RecBuff[5]&0xff))/100;
}

//�������ܣ���OPENMV����С������
//��ڲ�������
//����ֵ����
//˵������
void ANO_BlobsPosDownload(void)
{
	ANO_INFO.redBlob_x = (int16_t)(((ANO_INFO.RecBuff[4]&0xff)<<8)|(ANO_INFO.RecBuff[5]&0xff));
	ANO_INFO.redBlob_y = (int16_t)(((ANO_INFO.RecBuff[6]&0xff)<<8)|(ANO_INFO.RecBuff[7]&0xff));
	
	ANO_INFO.blueBlob_x = (int16_t)(((ANO_INFO.RecBuff[8]&0xff)<<8)|(ANO_INFO.RecBuff[9]&0xff));
	ANO_INFO.blueBlob_y = (int16_t)(((ANO_INFO.RecBuff[10]&0xff)<<8)|(ANO_INFO.RecBuff[11]&0xff));

	ANO_INFO.greenBlob_x = (int16_t)(((ANO_INFO.RecBuff[12]&0xff)<<8)|(ANO_INFO.RecBuff[13]&0xff));
	ANO_INFO.greenBlob_y = (int16_t)(((ANO_INFO.RecBuff[14]&0xff)<<8)|(ANO_INFO.RecBuff[15]&0xff));

	ANO_INFO.yellowBlob_x = (int16_t)(((ANO_INFO.RecBuff[16]&0xff)<<8)|(ANO_INFO.RecBuff[17]&0xff));
	ANO_INFO.yellowBlob_y = (int16_t)(((ANO_INFO.RecBuff[18]&0xff)<<8)|(ANO_INFO.RecBuff[19]&0xff));

	ANO_INFO.whiteBlob_x = (int16_t)(((ANO_INFO.RecBuff[20]&0xff)<<8)|(ANO_INFO.RecBuff[21]&0xff));
	ANO_INFO.whiteBlob_y = (int16_t)(((ANO_INFO.RecBuff[22]&0xff)<<8)|(ANO_INFO.RecBuff[23]&0xff));

	ANO_INFO.blackBlob_x = (int16_t)(((ANO_INFO.RecBuff[24]&0xff)<<8)|(ANO_INFO.RecBuff[25]&0xff));
	ANO_INFO.blackBlob_y = (int16_t)(((ANO_INFO.RecBuff[26]&0xff)<<8)|(ANO_INFO.RecBuff[27]&0xff));
}
#endif

///////////////////////////////////////////////////////////////////////////////////
//�������ܣ��ϴ����PWM
//��ڲ������ĸ������PWM��������λ���޷���ʾ����PWM��ֵ
//����ֵ����
//////////////////////////////////////////////////////////////////////////////////
void ANO_MotorPWMUpload(int16_t pwm1, int16_t pwm2, int16_t pwm3, int16_t pwm4)
{
	uint8_t Buff[16];
	uint8_t i;
	for(i = 0;i< 16; i++) Buff[i] = 0x00;
	Buff[0] = (pwm1>>8)&0xff;
	Buff[1] = pwm1&0xff;
	
	Buff[2] = (pwm1 >> 8) & 0xff;
	Buff[3] = pwm1 & 0xff;
	
	Buff[4] = (pwm1 >> 8) & 0xff;
	Buff[5] = pwm1 & 0xff;

	Buff[6] = (pwm2 >> 8) & 0xff;
	Buff[7] = pwm2 & 0xff;

	ANO_DataUpload(Buff,0x06,16);
}


///////////////////////////////////////////////////////////////////////////////////
//�������ܣ����ݴ�������
//��ڲ�������
//����ֵ����
//////////////////////////////////////////////////////////////////////////////////
void ANO_DataProcess(void)
{
	switch(ANO_INFO.REC_FLAG)
	{
		case ADJ_COMMAND:break;
		case REQUEST_PID: //PID������������
		{
			ANO_PID1Upload();  //�ϴ���һ��PID����
			ANO_PID2Upload();  //
			ANO_PID3Upload();	 //
			ANO_PID4Upload();  //
			ANO_PID5Upload();  //
			ANO_PID6Upload();	 //
		}break;
		case ADJ_SENSER:break;  //��������������
		case ADJ_PID1:ANO_PID1Download();	ANO_Response(); break; //���յ�һ��PID��������Ӧ
		case ADJ_PID2:ANO_PID2Download();	ANO_Response(); break; //
		case ADJ_PID3:ANO_PID3Download();	ANO_Response(); break; //
		case ADJ_PID4:ANO_PID4Download();	ANO_Response(); break; //
		case ADJ_PID5:ANO_PID5Download();	ANO_Response(); break; //
		case ADJ_PID6:ANO_PID6Download();	ANO_Response(); break; //
		case OPENMV_LINE_POS:ANO_LinePosDownload();break;
		case OPENMV_BLOBS1_POS:ANO_BlobsPosDownload();break;
		case ADJ_OFFSET:break;                                //������ƫ����
		case BOOTMODE:break;                                  //����IAP����ģʽ����
	}
	ANO_INFO.REC_FLAG = 0;     //���������ݺ���ձ�־��λ
}


