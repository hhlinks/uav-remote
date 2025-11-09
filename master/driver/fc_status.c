#include "fc_status.h"
#include "parse_packet.h"

extern PlaneData plane;

//检查锁定状态
void PlaneLockStatus(void)
{
	  //解锁
    if(plane.thr <= 10 && plane.yaw <= -25 && plane.pit >= 25 && plane.rol <= -25){
        plane.unlockCount++;
		}
    if(plane.thr <= 10 && plane.yaw <= -25 && plane.pit >= 25 && plane.rol <= -25 && plane.unlockCount >= 100){
        plane.unlockCount = 0;
        plane.lock = UNLOCK;		//解锁标志置位，指示灯使用该标志
    }

    //上锁
    if(plane.thr <= 10){
        plane.lockCount++;
		}
    if((plane.thr <= 10 && plane.lockCount >= 4000) || (plane.thr <= 10 && plane.yaw >= 25 && plane.lockCount >= 500)){
        plane.lockCount = 0;
        plane.lock = LOCK;		//上锁标志置位，指示灯使用该标志
    }

		if(plane.thr > 10){
			plane.lockCount = 0;		//油门大于10要清空计数
		}
}

uint32_t  chip_id[3] = {0};

//读取芯片ID
void get_chip_id(void)
{
    chip_id[0] = *(__IO u32 *)(0X1FFFF7F0); // 高字节
    chip_id[1] = *(__IO u32 *)(0X1FFFF7EC); //
    chip_id[2] = *(__IO u32 *)(0X1FFFF7E8); // 低字节
}
