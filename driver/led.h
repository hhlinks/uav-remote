#ifndef _led_h_
#define _led_h_

#include "stm32f10x.h"

#define	RGB_RED			GPIOB->BRR = GPIO_Pin_7
#define	RGB_GREEN		GPIOB->BRR = GPIO_Pin_8
#define	RGB_BLUE		GPIOB->BRR = GPIO_Pin_9

typedef enum{
	RED = 1,	//��		
	GREEN,		//��
	BLUE,			//��
	YELLOW,		//��
	PURPLE,		//��
	CYAN,			//��
	WHITE,		//��
}LedColor;


/*   LED2          LED1   */ 
    /** *   /|\   * * *
         *   |   *
          *  |  *
           * | *
            * *
             *
            * *
           *   *
          *     *
         *       *
    * * *         * * */
/*  LED3            LED4   */ 


void LedInit(void);
static void LedStatusOff(void);
void LedColorSet(const uint8_t LedColor);
void LedBlink(uint8_t ledColor);
#endif
