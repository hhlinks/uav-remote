#include "pair_freq.h"
#include "nrf24l01.h"
#include "spi.h"
#include "parse_packet.h"

PairInfo pair = {{0x1F,0x2E,0x3D,0x4C,0x5B},5};		//初始接收地址；初始接收频点

extern PlaneData plane;

//对频
void wait_pairing(void)
{
    uint8_t sta;

		if(plane.pair == PAIR_NOT){
			sta = SPI_Read_Reg(STATUS);
			SPI_Write_Reg(NRF_WRITE_REG+STATUS,sta);
			if(sta&RX_OK ){
				SPI_Read_Buf(RD_RX_PLOAD,Rx_packet,RX_PLOAD_WIDTH);
				SPI_Write_Reg(FLUSH_RX,0xff);
				if(Rx_packet[10]==0X8B)	//检查帧尾是否正确
				{
					pair.addr[0] = Rx_packet[1];		 	//5个字节地址
					pair.addr[1] = Rx_packet[2];	    //
					pair.addr[2] = Rx_packet[3];			//
					pair.addr[3] = Rx_packet[4];			//
					pair.addr[4] = Rx_packet[5];			//

					pair.freq_channel = Rx_packet[6];	//通信频点

					plane.pair = PAIR_NORMAL;
				}
			}

			if(plane.pair == PAIR_NORMAL){
				NRF_CE_L;
				//重新写入通信地址
				SPI_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(u8*)pair.addr,RX_ADR_WIDTH);
				SPI_Write_Reg(NRF_WRITE_REG+RF_CH,pair.freq_channel);
				NRF_CE_H;
			}
		}
}
