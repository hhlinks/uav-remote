#ifndef _pwm_h_
#define _pwm_h_

#include "stm32f10x.h"

void PwmInit(void);

void PwmOut(uint16_t pwm1,uint16_t pwm2,uint16_t pwm3,uint16_t pwm4);

#endif
