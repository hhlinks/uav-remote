#include "nrf24l01.h"
#include "spi.h"
#include "systick.h"
#include "led.h"
#include "imath.h"
#include "pair_freq.h"

const u8 TX_ADDRESS[TX_ADR_WIDTH]={0x1F,0x2E,0x3D,0x4C,0x5B};
const u8 RX_ADDRESS[RX_ADR_WIDTH]={0x1F,0x2E,0x3D,0x4C,0x5B};

void NRF24L01Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOA, ENABLE);

	//CE
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	//IRQ
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	NRF_CE_L;
	SPI_CSN_H;
}

//无线是否在位检测
u8 NRF24L01_Check(void)
{
	u8 buf[5]={0X18,0X18,0X18,0X18,0X18};
	u8 i;

	SPI_Write_Buf(NRF_WRITE_REG+TX_ADDR,buf,5);
	SPI_Read_Buf(TX_ADDR,buf,5);
	for(i=0;i<5;i++){
        if(buf[i]!=0X18)
            break;
			}
	if(i!=5)
        return 1;
	return 0;
}

//向寄存器写入值
u8 SPI_Write_Reg(u8 reg,u8 value)
{
	u8 status;

   	SPI_CSN_L;

  	status = Spi_RW_Byte(reg);
  	Spi_RW_Byte(value);

  	SPI_CSN_H;

  	return(status);
}

//读取寄存器值
u8 SPI_Read_Reg(u8 reg)
{
	u8 reg_val;

 	SPI_CSN_L;

  	Spi_RW_Byte(reg);
  	reg_val = Spi_RW_Byte(0XFF);

  	SPI_CSN_H;

  	return(reg_val);
}

//读出寄存器中连续len个字节长度的值
u8 SPI_Read_Buf(u8 reg,u8 *pBuf,u8 len)
{
	u8 status,u8_ctr;

  	SPI_CSN_L;

  	status = Spi_RW_Byte(reg);
 	for(u8_ctr=0;u8_ctr<len;u8_ctr++)
        pBuf[u8_ctr]=Spi_RW_Byte(0XFF);

  	SPI_CSN_H;

  	return status;
}

//向寄存器写入连续len个字节的值
u8 SPI_Write_Buf(u8 reg, u8 *pBuf, u8 len)
{
	u8 status,u8_ctr;

 	SPI_CSN_L;

  	status = Spi_RW_Byte(reg);
  	for(u8_ctr=0; u8_ctr<len; u8_ctr++)
		Spi_RW_Byte(*pBuf++);

  	SPI_CSN_H;

  	return status;
}

//接收模式
void NRF24L01ReceiveMode(void)
{
		NRF_CE_L;

		SPI_Write_Reg(SETUP_AW, 0x03); // 设置地址宽度为 5bytes

    SPI_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(u8*)pair.addr,RX_ADR_WIDTH);//设置接收地址（RX）

		SPI_Write_Reg( NRF_WRITE_REG+FEATURE, 0x06 );//使能动态负载长度及ACK应答
		SPI_Write_Reg(NRF_WRITE_REG+DYNPD, 0x01); //使能接收管道0动态负载长度

  	SPI_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);      //使能通道0的自动应答
  	SPI_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01);	 //使能通道0的接收地址
  	SPI_Write_Reg(NRF_WRITE_REG+RF_CH,pair.freq_channel);	 //设置频点（RF通道）
	SPI_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);	 //设置接收数据通道0有效数据宽度为11
  	SPI_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x07);				//设置射频数据率为1MHZ，发射功率为7dBm
  	SPI_Write_Reg(NRF_WRITE_REG+CONFIG, 0x0f);		//配置基本工作模式的参数;开启CRC，配置为接收模式,开启所有中断

  	NRF_CE_H;
}


//接收数据包
u8 NRF24L01_RxPacket(u8 *rxbuf)
{
	u8 sta;

	sta = SPI_Read_Reg(NRF_READ_REG+STATUS); 	 					    //状态标志位
	SPI_Write_Reg(NRF_WRITE_REG+STATUS,sta);
	if(sta&RX_OK)											//接收成功
	{
		SPI_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);
		SPI_Write_Reg(FLUSH_RX,0xff);
		return 0;
	}
	return 1;
}



//该函数初始化NRF24L01到TX模式
//设置TX地址,写TX数据宽度,设置RX自动应答的地址,填充TX发送数据,选择RF频道,波特率和LNA HCURR
//PWR_UP,CRC使能
//当CE变高后,即进入RX模式,并可以接收数据了
//CE为高大于10us,则启动发送.
void NRF24L01_TX_Mode(void)
{
	NRF_CE_L;

	SPI_Write_Reg(SETUP_AW, 0x03); // 设置地址宽度为 5bytes

	SPI_Write_Buf(NRF_WRITE_REG+TX_ADDR,(uint8_t*)pair.addr,TX_ADR_WIDTH);    //写TX节点地址
	SPI_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(uint8_t*)pair.addr,RX_ADR_WIDTH); //设置TX节点地址,主要为了接收ACK

	//NRF24L01_Write_Reg(NRF_WRITE_REG+FEATURE, 0x02 );//使能动态负载长度及带负载的ACK应答
	//NRF24L01_Write_Reg(NRF_WRITE_REG+DYNPD, 0x01); //使能接收管道0动态负载长度

	SPI_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);               //使能通道0的自动应答
	SPI_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01);           //使能通道0的接收地址
	SPI_Write_Reg(NRF_WRITE_REG+RF_CH,pair.freq_channel);  //设置RF通道
	SPI_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0x1a);          //设置自动重发间隔时间:500us;最大自动重发次数:10次
	SPI_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x07);						//设置射频数据率为1MHZ，发射功率为7dBm
	SPI_Write_Reg(NRF_WRITE_REG+CONFIG,0x0e);              //配置基本工作模式的参数;开启CRC，配置为发射模式,开启所有中断

	NRF_CE_H;                                          //CE为高,10us后启动发送
}


//启动NRF24L01发送一次数据
//sendBuff:待发送数据首地址
//返回值:发送完成状况
uint8_t NRF24L01_TxPacket(uint8_t *sendBuff)
{
  uint8_t state;
	NRF_CE_L;

  //NRF24L01_Write_Buf(SPI_WRITE_REG+RX_ADDR_P0,(uint8_t*)pair.addr,RX_ADR_WIDTH);
	SPI_Write_Buf(WR_TX_PLOAD,sendBuff,TX_PLOAD_WIDTH);

 	NRF_CE_H;		//启动发送

	while(NRF_IRQ!=0);		//等待发送完成

	state=SPI_Read_Reg(NRF_WRITE_REG+STATUS);		//读取状态寄存器的值
	SPI_Write_Reg(NRF_WRITE_REG+STATUS,state);		//清除TX_DS或MAX_RT中断标志

	if(state&MAX_TX){		//达到最大重发次数
    SPI_Write_Reg(FLUSH_TX,0xff);		//清除TX FIFO寄存器
		return MAX_TX;
	}
	if(state&TX_OK){		//发送完成
		return TX_OK;
	}
	return 0xff;		//其他原因发送失败
}
