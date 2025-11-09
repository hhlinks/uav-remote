#ifndef _parse_packet_h_
#define _parse_packet_h_

#include "nrf24l01.h"
#include "stm32f10x.h"

typedef enum{
	LOCK = 0,
	UNLOCK,
}Lock;//��״̬

typedef enum{
		SIGNAL_LOST = 0,
    SIGNAL_NORMAL,
}Signal;//�ź�״̬

typedef enum{
		POWER_NORMAL = 0,
		POWER_LOWER,
}Power;//����״̬

typedef enum{
		PAIR_NOT = 0,
		PAIR_NORMAL,
}Pair;//���״̬

typedef struct
{
	uint16_t thr;		//����
  uint16_t thr_zone;

	float pit;		//��������
	float rol;		//�������
	float yaw;		//ƫ������
  uint8_t key_l;		//���ܰ���
  uint8_t key_r;		//�ҹ��ܰ���

  uint32_t key_l_cnt;
  uint32_t key_r_cnt;

  uint8_t key_l_flag  : 1;
  uint8_t key_r_flag  : 1;
	uint8_t mode        : 2;
	uint8_t high_flag   : 1;		//���߱�־λ

	uint8_t signalLostCount;		//�ź�ʧ����

	Lock lock;
	Signal signal;
	Power power;
	Pair pair;

	int16_t unlockCount ;
  uint16_t lockCount ;

	float voltage;

}PlaneData;

extern u8 Rx_packet[RX_PLOAD_WIDTH];

void nrf_parse_packet(void);
void parse_key_info(void);
void NrfACKPacket(void);

#endif
