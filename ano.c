/**
  ******************************************************************************
  * 文件名称：ano.c
  * 作者  		Vecoter
  * 版本号    V1.0.0
  * 完成日期  2017 - 7 - 25
  * 说明	    该文件为匿名上位机通信驱动程序，上位机版本为V4.0.1，使用芯片为STM32F407ZGT6
  *         	数据的上传与下载           
  *           PID在线调参功能
  *      
  *           仅供学习使用，请勿用于商业用途
  ******************************************************************************
**/


//2017.7.25 完成基础版本，可以完成与匿名上位机的通信，数据上传与PID在线调参.
//					暂不支持f1芯片，需要手动移植


//2017.7.26 在上一版本上新增支持F1芯片，可以由程序自动判断当前使用的芯片是f4或f1，
//					数据处理部分由手动处理改到可以选择自动处理或手动处理。

//2017.7.29 修复通信数据长度错误，修复接收缓存区内存越界错误
//					新增与OPENMV的通信部分，可以接收从OpenMV发送的寻线信息，与小球位置信息
//					


//使用说明：本通信模块暂只能自动支持STM32F4与STM32F1系列，如果使用的芯片为F4或F1的芯片，
//只要调用ANO_Init初始化本模块就行，并要注释掉其他地方的串口中断接收函数,若需要支持其他系列的芯片，
//请将ano.h中的ANO_DRIVER_BY_USER宏定义改为1，并在调用ANO_Init函数时传递三个参数，第一个为
//串口通信初始化函数（无返回值，只有一个u32类型的形参），第二个为发送单个字节函数，第三个为串口波特率，
//还可能需要自己重写接收函数，请使用中断接收，
//在使用上位机发送的数据时，请在需要用到的地方include "ano.h"，然后就可以直接调用ANO_INFO这个结构体来获取
//上位机发送的数据，


# include "ano.h"
# include "led.h"
//
//数据接收结构体
//从上位机接收到的数据都在这个结构体里面
//需要使用该结构体里面的数据时，直接#include "ano.h"就可以直接使用该结构体
//
ANO_InfoStruct ANO_INFO;

//如果由用户提供底层驱动函数，则用户需要传入两个函数指针
//并由模块完成初始化
#if ANO_DRIVER_BY_USER==1
void ANO_Init(void (*f1)(u32 bound),void (*fun2)(uint8_t byte),u32 bound)
{
	ANO_INFO.init = f1;  //获取初始化函数地址
	ANO_INFO.ano_sendchar = fun2; //获取字节发送函数地址
	
	ANO_INFO.init(bound);  //完成初始化
}
#else
////////////////////////////////////////
//函数功能：初始化与匿名上位机通信功能
//函数说明：该函数在移植的时候需要根据芯片型号修改
//入口参数：波特率
//返回值：无
////////////////////////////////////////
#if defined STM32F407xx	//如果使用的是f4芯片
void ANO_Init(u32 bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
#if ANO_USART==1			//使用串口1
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟
		
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10复用为USART1
		//USART1端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9与GPIOA10
	
# elif ANO_USART==2  		//使用串口2
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//使能USART2时钟
	
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); //GPIOA2复用为USART2
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //GPIOA3复用为USART2
	
		//USART1端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; //GPIOA9与GPIOA10

# elif	ANO_USART==3		//使用串口3
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //使能GPIOB时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//使能USART3时钟
	
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3); //GPIOA10复用为USART3
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); //GPIOA11复用为USART3
	
	//USART1端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOB10与GPIOB11
# endif
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
			
#if ANO_USART==1
	GPIO_Init(GPIOA,&GPIO_InitStructure); 
#elif ANO_USART==2
	GPIO_Init(GPIOA,&GPIO_InitStructure); //
#elif ANO_USART==3
	GPIO_Init(GPIOB,&GPIO_InitStructure); //
#endif

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	
#if ANO_USART==1
  USART_Init(USART1, &USART_InitStructure); //初始化串口1
  USART_Cmd(USART1, ENABLE);  //使能串口1 
#elif ANO_USART==2
	USART_Init(USART2, &USART_InitStructure); //初始化串口2
  USART_Cmd(USART2, ENABLE);  //使能串口1 
#elif ANO_USART==3
  USART_Init(USART3, &USART_InitStructure); //初始化串口3
  USART_Cmd(USART3, ENABLE);  //使能串口1 
#endif

#if ANO_USART==1
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启相关中断
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//串口1中断通
#elif ANO_USART==2
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启相关中断
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//串口2中断通
#elif ANO_USART==3
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启相关中断
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;//串口3中断通
#endif

	//USART NVIC 配置
	//优先级设置为最高，防止接收数据时被打断，导致丢码
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//抢占优先级0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
	
	//相关结构体初始化
	ANO_InfoStrInit();
}
#elif defined (STM32F10X_HD)||(STM32F10X_MD)||(STM32F10X_LD)
void ANO_Init(u32 bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
#if ANO_USART==1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//使能USART1，GPIOA时钟
#elif	ANO_USART==2
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART2|RCC_APB2Periph_GPIOA, ENABLE);	//使能USART2，GPIOA时钟
#elif ANO_USART==3
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART3|RCC_APB2Periph_GPIOB, ENABLE);	//使能USART3，GPIOB时钟
#endif

#if ANO_USART==1
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.9
   
  //USART1_RX	  GPIOA.10初始化
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.10 
#elif ANO_USART==2
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA2
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA2
   
  //USART1_RX	  GPIOA.3初始化
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PA3
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.3
#elif ANO_USART==3
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(GPIOB, &GPIO_InitStructure);//
   
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIOB
#endif
 
 
 

  //Usart1 NVIC 配置

	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
  
   //USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
#if ANO_USART==1
  USART_Init(USART1, &USART_InitStructure); //初始化串口1
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接受中断
  USART_Cmd(USART1, ENABLE);  
#elif	ANO_USART==2
  USART_Init(USART2, &USART_InitStructure); //初始化串口1
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启串口接受中断
  USART_Cmd(USART2, ENABLE);  
#elif	ANO_USART==3
  USART_Init(USART3, &USART_InitStructure); //初始化串口1
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启串口接受中断
  USART_Cmd(USART3, ENABLE);  
#endif

	ANO_InfoStrInit();
}

#endif


#endif 


////////////////////////////////////////////////////////
//函数功能：初始化ANO_INFO结构体
//入口参数：无
//返回值:无
//说明：
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
//函数功能：数据帧接收函数
//函数说明：该函数在移植的时候为必须实现函数
//入口参数：无
//返回值：无
///////////////////////////////////
void ANO_USART_Handler(void)
{
	uint8_t uByte;  //字节接收暂存
	uint8_t i = 0;  //
	uint8_t checkSum = 0; //
	static uint8_t uCnt = 0;
#if ANO_USART==1															//如果使用串口1
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  
	{
		uByte =USART_ReceiveData(USART1);
#elif ANO_USART==2														//如果使用串口2
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  
	{
		uByte =USART_ReceiveData(USART2);
#elif	ANO_USART==3														//如果使用串口3
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
						
						ANO_INFO.RecBuff[uCnt++] = uByte;  //帧起始判断
					}
					else uCnt = 0X00;
			}break;
			case 0X01:
			{
				if(0XAF == uByte)	
				{
					ANO_INFO.RecBuff[uCnt++] = uByte;//帧起始判断
					
				}
				else uCnt = 0X00;
			}break;
			case 0X02:ANO_INFO.RecBuff[uCnt++] = uByte; break;  //功能码
			case 0X03:ANO_INFO.RecBuff[uCnt++] = uByte; break; //数据长度，除去功能码以及起始帧与检验和
			default:if(uCnt < (ANO_INFO.RecBuff[3] + 0X05)) ANO_INFO.RecBuff[uCnt++] = uByte;break;//接收数据
		}

		if(uCnt == (ANO_INFO.RecBuff[3] + 0X05))  //已经接收完整个数据帧
		{
			uCnt = 0;
			
			for(i = 0;i < ANO_INFO.RecBuff[3] + 0X04; i++) checkSum += ANO_INFO.RecBuff[i]; //计算检验和
			if((checkSum&0xff) != ANO_INFO.RecBuff[ANO_INFO.RecBuff[3] + 0X04]) ANO_INFO.REC_FLAG = 0; //接收错误
			else 
			{
				LED1 = ~LED1;
				ANO_INFO.REC_FLAG = ANO_INFO.RecBuff[2];		//数据检验无误，保存功能码
		#if ANO_DATA_PRECESS_ON==1  //选择是否接收完一帧数据后自动处理
				ANO_DataProcess();
		#endif
			}
		}
		
  } 
}
	
#if ANO_DRIVER_BY_USER==0
////////////////////////////////////////
//函数功能：通过串口发送一个字节的数据
//函数说明：该函数在移植的时候为必须实现函数
//入口参数：byte，要发送的8位数据
//返回值：无
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
//函数功能：匿名上位机数据上传
//入口参数：buff,要发送的数据数组，funCode，功能码;len数据长度，最大不超过27个字节
//返回值：0，数据长度超过限制，1，功能码错误，2，发送成功
///////////////////////////////////////////////////////////////////////////////////
uint8_t ANO_DataUpload(uint8_t *buff, uint8_t funCode, uint8_t len)
{
	uint8_t SendBuff[32];
	uint8_t i;
	if(len>27)return 0;//数据长度超过限制
	if(funCode>0xff) return 1;//功能码错误
		
	SendBuff[0] = 0XAA; //帧起始
	SendBuff[1] = 0XAA; //帧起始
	SendBuff[2] = funCode;  //功能码
	SendBuff[3] = len;   //数据长度，除去起始码和功能码以及长度
	
	//将要发送的数据复制到发送区
	for(i = 0; i < len; i++) SendBuff[i + 4] = buff[i];
	//计算校验和
	for(i = 0; i< len + 4; i++) SendBuff[len + 4] += buff[i];
	//循环发送数据
	for(i = 0; i< len + 5; i++) 
#if ANO_DRIVER_BY_USER==1
		ANO_INFO.ano_sendchar(SendBuff[i]);
#else
		ANO_SendChar(SendBuff[i]);
#endif
	return 2;
}

///////////////////////////////////////////////////////////////////////////////////
//函数功能：飞控状态上传
//入口参数：roll,pitch,yaw,alt_use,锁定状态
//返回值：无
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
//函数功能：飞控姿态上传
//入口参数：aacx,X轴加速度,aacy,aacz;gyrox，X轴角速度,gyroy,gyroz;mag_x,磁传感器数据
//返回值：无
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
//函数功能：上传第一组PID参数
//入口参数：无，所有要上传的数据都在ANO_INFO结构体中
//返回值：无
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
//函数功能：上传第二组PID参数
//入口参数：无
//返回值：无
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
//函数功能：上传第三组PID参数
//入口参数：无
//返回值：无
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
//函数功能：上传第四组PID参数
//入口参数：无
//返回值：无
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
//函数功能：上传第五组PID参数
//入口参数：无
//返回值：无
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
//函数功能：上传第六组PID参数
//入口参数：无
//返回值：无
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
//函数功能：回复上位机校验码
//入口参数：无
//返回值：无
//说明：在收到上位机的数据请求命令后，应调用该函数回应上位机，表示数据接收无误，该函数由数据处理程序自动调用，
////////////////////////////////////////////////
void ANO_Response(void)
{
	uint8_t SendBuff[8];
	uint8_t i;
	
	SendBuff[0] = 0XAA;//帧起始
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
//函数功能：接收第一组PID参数
//入口参数：无
//返回值：无
//////////////////////////////////////////////////////////////////////////////////
void ANO_PID1Download(void)
{
	//取前后两个8位的数据合并成一个16位的数据，并强制转换成一个float型的数据
	//转换完成后除以相应的传输因子
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
//函数功能：接收第二组PID参数
//入口参数：无
//返回值：无
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
//函数功能：接收第三组PID参数
//入口参数：无
//返回值：无
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
//函数功能：接收第四组PID参数
//入口参数：无
//返回值：无
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
//函数功能：接收第五组PID参数
//入口参数：无
//返回值：无
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
//函数功能：接收第六组PID参数
//入口参数：无
//返回值：无
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

//函数功能：接收从OPENMV上发送过来的线的位置
//入口参数：无
//返回值：无
//说明：无
void ANO_LinePosDownload(void)
{
	ANO_INFO.linePos = (float)(int16_t)(((ANO_INFO.RecBuff[4]&0xff)<<8)|(ANO_INFO.RecBuff[5]&0xff))/100;
}

//函数功能：从OPENMV接收小球坐标
//入口参数：无
//返回值：无
//说明：无
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
//函数功能：上传电机PWM
//入口参数：四个电机的PWM，但是上位机无法显示负的PWM数值
//返回值：无
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
//函数功能：数据处理函数，
//入口参数：无
//返回值：无
//////////////////////////////////////////////////////////////////////////////////
void ANO_DataProcess(void)
{
	switch(ANO_INFO.REC_FLAG)
	{
		case ADJ_COMMAND:break;
		case REQUEST_PID: //PID参数请求命令
		{
			ANO_PID1Upload();  //上传第一组PID参数
			ANO_PID2Upload();  //
			ANO_PID3Upload();	 //
			ANO_PID4Upload();  //
			ANO_PID5Upload();  //
			ANO_PID6Upload();	 //
		}break;
		case ADJ_SENSER:break;  //调整传感器命令
		case ADJ_PID1:ANO_PID1Download();	ANO_Response(); break; //接收第一组PID参数并回应
		case ADJ_PID2:ANO_PID2Download();	ANO_Response(); break; //
		case ADJ_PID3:ANO_PID3Download();	ANO_Response(); break; //
		case ADJ_PID4:ANO_PID4Download();	ANO_Response(); break; //
		case ADJ_PID5:ANO_PID5Download();	ANO_Response(); break; //
		case ADJ_PID6:ANO_PID6Download();	ANO_Response(); break; //
		case OPENMV_LINE_POS:ANO_LinePosDownload();break;
		case OPENMV_BLOBS1_POS:ANO_BlobsPosDownload();break;
		case ADJ_OFFSET:break;                                //调整零偏命令
		case BOOTMODE:break;                                  //进入IAP下载模式命令
	}
	ANO_INFO.REC_FLAG = 0;     //处理完数据后接收标志复位
}


