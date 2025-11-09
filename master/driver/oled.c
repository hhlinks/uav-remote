#include "oled.h"
#include "OledFont.h"
#include "timer.h"
#include "iic.h"
#include "led.h"

/*******************************************************************************
* �� �� ��         : OLED_Init()
* ��������		     : OLED��ʼ��
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void OLED_Init(void)
{
	IIC_Init();
	delay_ms(200);
	OLED_WR_Byte(0xAE,OLED_CMD);//--display off
	OLED_WR_Byte(0x00,OLED_CMD);//---set low column address0
	OLED_WR_Byte(0x10,OLED_CMD);//---set high column address16
	OLED_WR_Byte(0x40,OLED_CMD);//--set start line address
	OLED_WR_Byte(0xB0,OLED_CMD);//--set page address176
	OLED_WR_Byte(0x81,OLED_CMD); // contract control
	OLED_WR_Byte(0x08,OLED_CMD);//--128
	OLED_WR_Byte(0xA1,OLED_CMD);//set segment remap
	OLED_WR_Byte(0xA6,OLED_CMD);//--normal / reverse
	OLED_WR_Byte(0xA8,OLED_CMD);//--set multiplex ratio(1 to 64)
	OLED_WR_Byte(0x3F,OLED_CMD);//--1/32 duty
	OLED_WR_Byte(0xC8,OLED_CMD);//Com scan direction
	OLED_WR_Byte(0xD3,OLED_CMD);//-set display offset
	OLED_WR_Byte(0x00,OLED_CMD);//

	OLED_WR_Byte(0xD5,OLED_CMD);//set osc division
	OLED_WR_Byte(0x80,OLED_CMD);//


	OLED_WR_Byte(0xD8,OLED_CMD);//set area color mode off
	OLED_WR_Byte(0x05,OLED_CMD);//

	OLED_WR_Byte(0xD9,OLED_CMD);//Set Pre-Charge Period
	OLED_WR_Byte(0xF1,OLED_CMD);//

	OLED_WR_Byte(0xDA,OLED_CMD);//set com pin configuartion
	OLED_WR_Byte(0x12,OLED_CMD);//

	OLED_WR_Byte(0xDB,OLED_CMD);//set Vcomh
	OLED_WR_Byte(0x30,OLED_CMD);//

	OLED_WR_Byte(0x8D,OLED_CMD);//set charge pump enable
	OLED_WR_Byte(0x14,OLED_CMD);//

	OLED_WR_Byte(0xAF,OLED_CMD);//--turn on oled panel
}

/*******************************************************************************
* �� �� ��         : Write_OLED_Command()
* ��������		     : OLEDд����
* ��    ��         : OLED_Command(����)
* ��    ��         : ��
*******************************************************************************/
void Write_OLED_Command(unsigned char OLED_Command)
{
  IIC_Start();
  IIC_Send_Byte(0x78);       //Slave address,SA0=0
	IIC_Wait_Ack();
  IIC_Send_Byte(0x00);			//D/C#=0;д����
	IIC_Wait_Ack();
  IIC_Send_Byte(OLED_Command);
	IIC_Wait_Ack();
  IIC_Stop();
}
//
/*******************************************************************************
* �� �� ��         : Write_OLED_Data()
* ��������		     : OLEDд����
* ��    ��         : OLED_Data(����)
* ��    ��         : ��
*******************************************************************************/
void Write_OLED_Data(unsigned char OLED_Data)
{
  IIC_Start();
  IIC_Send_Byte(0x78);			// R/W#=0;��д��ַ��ģʽѡ������SA=0��R/W#=0��дģʽ��
	IIC_Wait_Ack();
  IIC_Send_Byte(0x40);			//D/C#=1;д����
	IIC_Wait_Ack();
  IIC_Send_Byte(OLED_Data);//д������
	IIC_Wait_Ack();
  IIC_Stop();
}


/*******************************************************************************
* �� �� ��         : OLED_WR_Byte()
* ��������		     : OLED����/����ģʽѡ��
* ��    ��         : Data(����)
										 DataType(�������ͣ�����/����)
* ��    ��         : ��
*******************************************************************************/
void OLED_WR_Byte(unsigned Data,unsigned DataType)
{
	if(DataType)
	{
   Write_OLED_Data(Data);//д���ݣ�cmd=0

	}
	else
	{
   Write_OLED_Command(Data);	//д���cmd=1
	}
}

/*******************************************************************************
* �� �� ��         : OLED_Display_On()
* ��������		     : ����OLED��ʾ
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void OLED_Display_On(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC����
	OLED_WR_Byte(0X14,OLED_CMD);  //DCDC ON
	OLED_WR_Byte(0XAF,OLED_CMD);  //DISPLAY ON
}

/*******************************************************************************
* �� �� ��         : OLED_Display_Off()
* ��������		     : �ر�OLED��ʾ
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void OLED_Display_Off(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC����
	OLED_WR_Byte(0X10,OLED_CMD);  //DCDC OFF
	OLED_WR_Byte(0XAE,OLED_CMD);  //DISPLAY OFF
}

/*******************************************************************************
* �� �� ��         : OLED_Clear()
* ��������		     : ��������,������,������Ļ�Ǻ�ɫ��!��û����һ��!!!
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void OLED_Clear(void)
{
	u8 i,n;
	for(i=0;i<8;i++)
	{
		OLED_WR_Byte (0xb0+i,OLED_CMD);    //����ҳ��ַ��0~7��
		OLED_WR_Byte (0x00,OLED_CMD);      //������ʾλ�á��е͵�ַ
		OLED_WR_Byte (0x10,OLED_CMD);      //������ʾλ�á��иߵ�ַ
		for(n=0;n<128;n++)OLED_WR_Byte(0,OLED_DATA);
	} //������ʾ
}

/*******************************************************************************
* �� �� ��         : OLED_On()
* ��������		     : oled������ʾ
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void OLED_On(void)
{
	u8 i,n;
	for(i=0;i<8;i++)
	{
		OLED_WR_Byte (0xb0+i,OLED_CMD);    //����ҳ��ַ��0~7��
		OLED_WR_Byte (0x00,OLED_CMD);      //������ʾλ�á��е͵�ַ
		OLED_WR_Byte (0x10,OLED_CMD);      //������ʾλ�á��иߵ�ַ
		for(n=0;n<128;n++)OLED_WR_Byte(0xff,OLED_DATA);
	} //������ʾ
}

/*******************************************************************************
* �� �� ��         : Oled_Char_Display()
* ��������		     : �ַ�����ʾ
* ��    ��         : raw(0~3)
										 column(0~7)
										 str[]
* ��    ��         : ��
*******************************************************************************/
void Oled_Char_Display(u8 raw,u8 column,u8 str[])
{
	u8 i,n,j,c,strA[16]={0},k;
		j=0,c=0;
	for(k=0;str[k]!='\0';k++)//���ַ����𿪣���˳�����strA[]������
	{
		strA[k]=str[k]-' ';//F8X16[][16]����������' '��ʼ�ģ�����Ҫ��ȥ���ƫ����
	}


	for(i=raw;i<2+raw;i++)//������ʾ
	{
		OLED_WR_Byte (0xb0+i,OLED_CMD);    //����ҳ��ַ��0~7��
		OLED_WR_Byte (0x00,OLED_CMD);      //������ʾλ�á��е͵�ַ
		OLED_WR_Byte (0x10+column,OLED_CMD);      //������ʾλ�á��иߵ�ַ

	j=0;
		for(n=0;n<k*8;n++)//����������
		{


			if(i==raw)//������ʾ��һҳ��һ���ַ����ϰ벿�֣�
			{
			//GPIO_SetBits(Led_Port,Led_Pin);
				OLED_WR_Byte(F8X16[strA[j]][c],OLED_DATA);
				c++;
				if(c==8&&j<16)//һ���ַ���ռ������Ϊ8����ʾ��һ���ַ��󣬻�����һ���ַ�
				{
					j++;
					c=0;
				}


			}



			if(i==raw+1)//������ʾ�ڶ�ҳ��һ���ַ����°ಿ�֣�
			{
				//GPIO_ResetBits(Led_Port,Led_Pin);
				OLED_WR_Byte(F8X16[strA[j]][c],OLED_DATA);
				c++;
					if(c==16)
				{
					j++;
					c=8;
				}

			}

		}

		if(j==k)//��һҳ��ʾ��󣬻����ڶ�ҳ����ʾ�ַ����°벿��
		c=8;
	}
}
