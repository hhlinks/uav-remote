#include "stm32f10x.h"
#include "systick.h"
#include "led.h"
#include "pwm.h"
#include "iic.h"
#include "mpu6050.h"
#include "spi.h"
#include "adc.h"
#include "acc_cal.h"
#include "pid.h"
#include "timer.h"
#include "nvic.h"
#include "usart2.h"

int main(void)
{
    SystemInit();			//系统初始化
	SystickInit();			//系统滴答定时器初始化

    LedInit();				//状态灯初始化

	Usart2Init(115200);		//串口2初始化

	PwmInit();				//pwm初始

	delay_ms(100);			//延时处理，以便开机进行零偏采集，防止瞬间抖动
	I2cInit();				///I2C初始化
	mpu6050_init();			//mpu6050初始化
    while(get_mpu_id()!=0x68){		//mpu6050在位检测
      	delay_ms(10);
		TopLedBlink(RED);		//故障则红色闪烁
    }

	delay_ms(100);
    SpiInit();
	NRF24L01Init();				//nrf24l01初始化
	while(NRF24L01_Check()){	//nrf24l01在位检测
		delay_ms(30);
		TopLedBlink(GREEN);		//故障则绿色闪烁
	}
    NRF24L01ReceiveMode();		//接收模式

	get_iir_factor(&Mpu.att_acc_factor,0.005f,15);   	//姿态解算时加速度低通系数
	get_iir_factor(&Mpu.fix_acc_factor,0.005f,2);		//高度融合时加速度低通系数

	AdcInit();
	AllPidInit();		//pid参数初始化
	read_cal_dat();		//flash校准数据读取
	delay_ms(100);

	TimerInit();		//定时器初始化
    NVIC_config();		//中断配置初始化

    while(1){
		VoltageDetect();				//低电压检测
		LeftButtomLedBlinkSet();		//灯闪烁
    }
}
