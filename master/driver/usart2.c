#include "usart2.h"
#include "imu.h"
#include "parse_packet.h"
#include "pid.h"
#include "systick.h"
#include "led.h"
#include "flash.h"

extern uint32_t pidResetFlag;
extern AllPid allPid;
extern PlaneData plane;

//������λ�����ڳ�ʼ��
void Usart2Init(u32 bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);		//ʹ�ܴ��ڸ���ʱ��

  //  TX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
  //  RX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = bound;		//���ò����ʣ������ɲ�������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     //�ֳ�Ϊ8bit
	USART_InitStructure.USART_StopBits = USART_StopBits_1;                          //ֹͣλ����������1��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;                             //��У��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //��Ӳ��������
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;                 //ģʽѡ��TX��RX
  USART_Init(USART2, &USART_InitStructure);

	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);		//RX�ж�ʹ��

	USART_Cmd(USART2, ENABLE);		//ʹ�ܴ���2
}

/* ����2��DMAͨ������                */
/* �洢�������贫�䷽��              */
/* DMA_CHx:         DMA����ͨ��x     */
/* peripheral_addr: �����ַ         */
/* memory_addr:     �ڴ��ַ         */
/* data_length:     ��������ݳ���   */
static void Usart2DmaConfig(DMA_Channel_TypeDef* DMA_CHx,u32 peripheral_addr,u32 memory_addr,u16 data_length)
{
    DMA_InitTypeDef DMA_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);		//ʱ��ʹ��

    DMA_DeInit(DMA_CHx);		//��λ

    DMA_InitStructure.DMA_PeripheralBaseAddr = peripheral_addr;                 //�����ַ
    DMA_InitStructure.DMA_MemoryBaseAddr =memory_addr;                          //�ڴ��ַ
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;                          //������Ϊ�����Ŀ�ĵ�
    DMA_InitStructure.DMA_BufferSize = data_length;                             //���ݻ����С
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;            //�����ַ������
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                     //�ڴ��ַ����
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;     //�������ݿ���8λ
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;             //�ڴ����ݿ���8λ
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                               //����ģʽ
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                     //�����ȼ�
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                                //���ڴ浽�ڴ洫��
    DMA_Init(DMA_CHx, &DMA_InitStructure);
}

/* ����DMA���ݷ��� */
static void UsartDMASendData(u32 SendBuff,u16 len)
{
	DMA_Cmd(DMA1_Channel7, DISABLE); //�ر�
	Usart2DmaConfig(DMA1_Channel7,(u32)&USART2->DR,(u32)SendBuff,len);
	DMA_SetCurrDataCounter(DMA1_Channel7,len);
	USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);		//ʹ�ܴ���DMA����
  DMA_Cmd(DMA1_Channel7, ENABLE);		//ʹ��DMA����
}

/* ������һ���ֽڵ����ݲ�ֳɶ���ֽڷ��� */
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)    ) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

/* ��������λ��������̬�ǣ�����״̬ */
void ANOSendStatus(void)
{
	u8 cnt=0;
	vs16 temp;
	vs32 temp2;
	u8 sum = 0;
	u8 i;
	u8 dataToSend[50];

	dataToSend[cnt++]=0xAA;		//֡ͷ��AAAA
	dataToSend[cnt++]=0xAA;		//
	dataToSend[cnt++]=0x01;		//�����֣�0xFnֻ�������ݣ�����ʾͼ��0x0n��ʾ���ݺ�ͼ��0x01��ʾ���͵���STATUS
	dataToSend[cnt++]=0;				//��Ҫ�������ݵ��ֽ������ݸ�0

	temp = (int)(att.rol*100);				//�����
	dataToSend[cnt++]=BYTE1(temp);		//���ֽ�
	dataToSend[cnt++]=BYTE0(temp);		//���ֽ�

	temp = (int)(att.pit*100);         //������
	dataToSend[cnt++]=BYTE1(temp);
	dataToSend[cnt++]=BYTE0(temp);

	temp = (int)(att.yaw*100);         //ƫ����
	dataToSend[cnt++]=BYTE1(temp);
	dataToSend[cnt++]=BYTE0(temp);

	temp2 = (int32_t)(0);   //�߶�
	dataToSend[cnt++]=BYTE3(temp2);
	dataToSend[cnt++]=BYTE2(temp2);
	dataToSend[cnt++]=BYTE1(temp2);
	dataToSend[cnt++]=BYTE0(temp2);

  dataToSend[cnt++]=0x01;		//����ģʽ    01����̬  02������  03������
  dataToSend[cnt++]= plane.lock;		//����״̬  0����   1����

	dataToSend[3] = cnt-4;		//�����ֽ���

	sum = 0;
	for(i=0;i<cnt;i++)		//����У��λ
		sum += dataToSend[i];
	dataToSend[cnt++]=sum;

  UsartDMASendData((u32)(dataToSend),cnt); //DMA����
}

/* ң����ͨ������ */
void ANO_DT_Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6)
{
    u8 cnt=0;
    u8 i=0;
    u8 sum = 0;
		u8 dataToSend[50];
    dataToSend[cnt++]=0xAA;
    dataToSend[cnt++]=0xAA;
    dataToSend[cnt++]=0x03;
    dataToSend[cnt++]=0;

    dataToSend[cnt++]=BYTE1(thr);
    dataToSend[cnt++]=BYTE0(thr);

    dataToSend[cnt++]=BYTE1(yaw);
    dataToSend[cnt++]=BYTE0(yaw);

    dataToSend[cnt++]=BYTE1(rol);
    dataToSend[cnt++]=BYTE0(rol);

    dataToSend[cnt++]=BYTE1(pit);
    dataToSend[cnt++]=BYTE0(pit);

    dataToSend[cnt++]=BYTE1(aux1);
    dataToSend[cnt++]=BYTE0(aux1);

    dataToSend[cnt++]=BYTE1(aux2);
    dataToSend[cnt++]=BYTE0(aux2);

    dataToSend[cnt++]=BYTE1(aux3);
    dataToSend[cnt++]=BYTE0(aux3);

    dataToSend[cnt++]=BYTE1(aux4);
    dataToSend[cnt++]=BYTE0(aux4);

    dataToSend[cnt++]=BYTE1(aux5);
    dataToSend[cnt++]=BYTE0(aux5);

    dataToSend[cnt++]=BYTE1(aux6);
    dataToSend[cnt++]=BYTE0(aux6);

    dataToSend[3] = cnt-4;

    sum = 0;
    for(i=0;i<cnt;i++)
        sum += dataToSend[i];

    dataToSend[cnt++]=sum;

    UsartDMASendData((u32)(dataToSend),cnt);
}

//pid���ݷ��͵�������λ��
static void PidDataSend(u16 *sendBuff,u8 funcByte,u8 dataLen)
{
	u8 cnt = 0;
	vs16 temp;
	u8 sum = 0;
	u8 i;
	u8 dataToSend[50];

	dataToSend[cnt++] = 0xAA;		//֡ͷ��AAAA
	dataToSend[cnt++] = 0xAA;		//
	dataToSend[cnt++] = funcByte;		//�����֣�0xFnֻ�������ݣ�����ʾͼ��0x0n��ʾ���ݺ�ͼ��0x01��ʾ���͵���STATUS
	dataToSend[cnt++] = 0;			//��Ҫ�������ݵ��ֽ������ݸ�0

	for(i = 0; i < dataLen; i++){//���ݴ������������
		temp = (int)(sendBuff[i]);
	  dataToSend[cnt++] = BYTE1(temp);
	  dataToSend[cnt++] = BYTE0(temp);
	}

	dataToSend[3] = cnt-4;		//�����ֽ���

	sum = 0;
	for(i = 0; i < cnt; i++){		//����У��λ
		sum += dataToSend[i];
	}
	dataToSend[cnt++] = sum;		//֡β����У��λ

	Usart2Send(dataToSend,cnt);		//���ڷ���
}

//pid�������������ڷ���
static void DateTransfer(void)
{
	u8 arrLen = 0;
	u16 arrTemp[10] = {0};

	arrTemp[arrLen++] = allPid.rolAngle.kp*100;
	arrTemp[arrLen++] = allPid.rolAngle.ki*100;
	arrTemp[arrLen++] = allPid.rolAngle.kd*100;
	arrTemp[arrLen++] = allPid.pitAngle.kp*100;
	arrTemp[arrLen++] = allPid.pitAngle.ki*100;
	arrTemp[arrLen++] = allPid.pitAngle.kd*100;
	arrTemp[arrLen++] = allPid.yawAngle.kp*100;
	arrTemp[arrLen++] = allPid.yawAngle.ki*100;
	arrTemp[arrLen++] = allPid.yawAngle.kd*100;
	PidDataSend(arrTemp,0x10,arrLen);
	delay_ms(2);//������һ֡���ݺ�����ʱ�����ٴη���

	arrLen = 0;
	arrTemp[arrLen++] = allPid.rolGyro.kp*100;
	arrTemp[arrLen++] = allPid.rolGyro.ki*10000;
	arrTemp[arrLen++] = allPid.rolGyro.kd*1000;
	arrTemp[arrLen++] = allPid.pitGyro.kp*100;
	arrTemp[arrLen++] = allPid.pitGyro.ki*10000;
	arrTemp[arrLen++] = allPid.pitGyro.kd*1000;
	arrTemp[arrLen++] = allPid.yawGyro.kp*100;
	arrTemp[arrLen++] = allPid.yawGyro.ki*10000;
	arrTemp[arrLen++] = allPid.yawGyro.kd*1000;
	PidDataSend(arrTemp,0x11,arrLen);

}

static void Usart2Send(const u8 *data,u8 len)
{
  u8 i;
	for(i=0;i<len;i++){
		while(USART_GetFlagStatus(USART2,USART_FLAG_TC) != SET){		//�ȴ��������

		}
			USART2->DR = *(data+i);		//�򴮿�1��������
		//USART_SendData(USART1,data);
	}
	USART_ClearFlag(USART2,USART_FLAG_TC);		//��ձ�־λ
}

//ժ����������վ
//����У������֡
static void ANO_DT_Send_Check(u8 head, u8 check_sum)
{
	u8 dataToSend[10];
	dataToSend[0]=0xAA;
	dataToSend[1]=0xAA;
	dataToSend[2]=0xEF;
	dataToSend[3]=2;
	dataToSend[4]=head;
	dataToSend[5]=check_sum;

	u8 sum = 0;
	for(u8 i=0;i<6;i++)
		sum += dataToSend[i];
	dataToSend[6]=sum;

	Usart2Send(dataToSend, 7);
}

//ժ����������վ
//��������֡�����ö�Ӧ����
static void ANODataReceiveAnalysis(u8 *dataBuffer,u8 num)
{
	u8 sum = 0;

	for(u8 i=0;i<(num-1);i++){
		sum += *(dataBuffer+i);
	}

	if(!(sum==*(dataBuffer+num-1))){		//�ж�sum
		return;
	}

	if(!(*(dataBuffer)==0xAA && *(dataBuffer+1)==0xAF)){		//�ж�֡ͷ
		return;
	}

	if(*(dataBuffer+2)==0X01)		//�����1
	{
//		if(*(dataBuffer+4)==0X01)		//ACCУ׼
//			//mpu6050.Acc_CALIBRATE = 1;
//		if(*(dataBuffer+4)==0X02)		//GYROУ׼
//			//mpu6050.Gyro_CALIBRATE = 1;
//		if(*(dataBuffer+4)==0X03)		//
//		{
//			//mpu6050.Acc_CALIBRATE = 1;
//			//mpu6050.Gyro_CALIBRATE = 1;
//		}
	}

	if(*(dataBuffer+2)==0X02){		//�����2
		if(*(dataBuffer+4)==0X01){		//����pid����
			DateTransfer();		//����pid����
		}
		if(*(dataBuffer+4)==0X02){		//��ȡ����ģʽ��������

		}
		if(*(dataBuffer+4)==0XA0){		//��ȡ��λ���汾��Ϣ

		}
		if(*(dataBuffer+4)==0XA1){		//�ָ�Ĭ�ϲ���
			pidResetFlag = 1;		//��־λ��1
			AllPidInit();		//���³�ʼ��pid����
			DateTransfer();		//����pid����
		}
	}

	if(*(dataBuffer+2)==0X10){		//д��PID1
			allPid.rolAngle.kp = 0.01*( (vs16)(*(dataBuffer+4)<<8)|*(dataBuffer+5) );
			allPid.rolAngle.ki = 0.01*( (vs16)(*(dataBuffer+6)<<8)|*(dataBuffer+7) );
			allPid.rolAngle.kd = 0.01*( (vs16)(*(dataBuffer+8)<<8)|*(dataBuffer+9) );
			allPid.pitAngle.kp = 0.01*( (vs16)(*(dataBuffer+10)<<8)|*(dataBuffer+11) );
			allPid.pitAngle.ki = 0.01*( (vs16)(*(dataBuffer+12)<<8)|*(dataBuffer+13) );
			allPid.pitAngle.kd = 0.01*( (vs16)(*(dataBuffer+14)<<8)|*(dataBuffer+15) );
			allPid.yawAngle.kp = 0.01*( (vs16)(*(dataBuffer+16)<<8)|*(dataBuffer+17) );
			allPid.yawAngle.ki = 0.01*( (vs16)(*(dataBuffer+18)<<8)|*(dataBuffer+19) );
			allPid.yawAngle.kd = 0.01*( (vs16)(*(dataBuffer+20)<<8)|*(dataBuffer+21) );

			ANO_DT_Send_Check(*(dataBuffer+2),sum);
			PidDataWriteToFlash(PID_WRITE_ADDRESS,&allPid);
	}
	 if(*(dataBuffer+2)==0X11){		//PID2
			allPid.rolGyro.kp = 0.01*( (vs16)(*(dataBuffer+4)<<8)|*(dataBuffer+5) );
			allPid.rolGyro.ki = 0.0001*( (vs16)(*(dataBuffer+6)<<8)|*(dataBuffer+7) );
			allPid.rolGyro.kd = 0.001*( (vs16)(*(dataBuffer+8)<<8)|*(dataBuffer+9) );
			allPid.pitGyro.kp = 0.01*( (vs16)(*(dataBuffer+10)<<8)|*(dataBuffer+11) );
			allPid.pitGyro.ki	= 0.0001*( (vs16)(*(dataBuffer+12)<<8)|*(dataBuffer+13) );
			allPid.pitGyro.kd	= 0.001*( (vs16)(*(dataBuffer+14)<<8)|*(dataBuffer+15) );
			allPid.yawGyro.kp = 0.01*( (vs16)(*(dataBuffer+16)<<8)|*(dataBuffer+17) );
			allPid.yawGyro.ki	= 0.0001*( (vs16)(*(dataBuffer+18)<<8)|*(dataBuffer+19) );
			allPid.yawGyro.kd	= 0.001*( (vs16)(*(dataBuffer+20)<<8)|*(dataBuffer+21) );
			ANO_DT_Send_Check(*(dataBuffer+2),sum);
			PidDataWriteToFlash(PID_WRITE_ADDRESS,&allPid);
		}
	if(*(dataBuffer+2)==0X12){		//PID3
		 ANO_DT_Send_Check(*(dataBuffer+2),sum);
	}
	if(*(dataBuffer+2)==0X13){		//PID4
		ANO_DT_Send_Check(*(dataBuffer+2),sum);
	}
	if(*(dataBuffer+2)==0X14){		//PID5
		ANO_DT_Send_Check(*(dataBuffer+2),sum);
	}
	if(*(dataBuffer+2)==0X15){		//PID6
		ANO_DT_Send_Check(*(dataBuffer+2),sum);
	}

}

//ժ����������
//����λ��������������֡���н���
static void ANODTDataReceivePrepare(u8 data)
{
	static u8 receiveBuffer[50];
	static u8 dataLen = 0,cnt = 0;
	static u8 state = 0;

	if(state == 0 && data == 0xAA){
		state = 1;
		receiveBuffer[0]=data;
	}
	else if(state == 1 && data == 0xAF){
		state = 2;
		receiveBuffer[1] = data;
	}
	else if(state == 2 && data < 0XF1){
		state = 3;
		receiveBuffer[2] = data;
	}
	else if(state ==3 && data < 50){
		state = 4;
		receiveBuffer[3] = data;
		dataLen = data;
		cnt = 0;
	}
	else if(state == 4 && dataLen > 0){
		dataLen--;
		receiveBuffer[4+cnt++] = data;
		if(dataLen==0)
			state = 5;
	}
	else if(state==5){
		state = 0;
		receiveBuffer[4+cnt] = data;
		ANODataReceiveAnalysis(receiveBuffer,cnt+5);
	}
	else{
		state = 0;
	}

}

//����2�жϷ������
void USART2_IRQHandler(void)
{
	u8 data = 0;

	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET){  //�����ж�
		data = USART_ReceiveData(USART2);//(USART1->DR);	//��ȡ���յ�������
		ANODTDataReceivePrepare(data);

	}

}
