#include "parse_packet.h"
#include "nrf24l01.h"
#include "imath.h"
#include "controller.h"
#include "pair_freq.h"

u8 Rx_packet[RX_PLOAD_WIDTH] = {0};
u8 txPacket[TX_PLOAD_WIDTH] = {0};

PlaneData plane = {0};

//数据包解析
void nrf_parse_packet(void)
{
	if(NRF24L01_RxPacket(Rx_packet)==0){		//成功接收到数据
       if((Rx_packet[RX_PLOAD_WIDTH-1]!=0x8B)){
				 return;
			 }
        			 
		plane.signalLostCount = 0;
        
		plane.thr = Rx_packet[2]<<8|Rx_packet[1];		//油门
		plane.pit = -(float)(((int8_t)Rx_packet[3]) - 50);		//俯仰舵向
		plane.rol = -(float)(((int8_t)Rx_packet[4]) - 50);		//横滚舵向
    plane.yaw =  (float)(((int8_t)Rx_packet[5]) - 50);		//偏航舵向
    plane.key_l = Rx_packet[6];		//左功能按键
    plane.key_r = Rx_packet[7];		//右功能按键
        
		plane.thr = ThrottleLimit(plane.thr,0,950);		//油门限制范围
    plane.pit = direction_to_zero(plane.pit,-5,5);		//俯仰舵向范围
    plane.rol = direction_to_zero(plane.rol,-5,5);		//横滚舵向范围
    plane.yaw = direction_to_zero(plane.yaw,-5,5);		//偏航舵向范围

    plane.thr_zone = plane.thr;//油门，用于定高控制
			 
	}
	else{
		if(plane.signalLostCount<200){		//信号丢失计数
				plane.signalLostCount++;  
				plane.signal = SIGNAL_NORMAL;
    }
		
		if(plane.signalLostCount==200){	
			plane.signal = SIGNAL_LOST;		//信号丢失
      if(plane.thr>10){	
        plane.thr -= 1;		//信号丢失减速处理                
        }
		}
		plane.thr = ThrottleLimit(plane.thr,0,1000);
	}
}


//向遥控器发送飞机数据
void NrfACKPacket(void)
{
	txPacket[0] = 0xAA;
	txPacket[1] = plane.pair;
	txPacket[2] = plane.signal;
	txPacket[3] = plane.lock;
	txPacket[4] = plane.power;
	txPacket[TX_PLOAD_WIDTH - 1] = 0xAC;
	SPI_Write_Buf(W_ACK_PLOAD,txPacket,TX_PLOAD_WIDTH);
}
