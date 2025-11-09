#ifndef _usart2_h_
#define _usart2_h_

#include "stm32f10x.h"
#include "stdio.h"

void Usart2Init(u32 bound);
static void Usart2DmaConfig(DMA_Channel_TypeDef* DMA_CHx,u32 peripheral_addr,u32 memory_addr,u16 data_length);
static void UsartDMASendData(u32 SendBuff,u16 len);
void ANOSendStatus(void);
static void PidDataSend(u16 sendBuff[],u8 funcByte,u8 dataLen);
static void DateTransfer(void);
static void Usart2Send(const u8 *data,u8 len);
static void ANODataReceiveAnalysis(u8 *dataBuffer,u8 num);
static void ANODTDataReceivePrepare(u8 data);

#endif
