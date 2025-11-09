#ifndef _led_h_
#define _led_h_

#include "stm32f10x.h"
#include "parse_packet.h"

#define	TOP_RGB_RED			GPIOA->BRR = GPIO_Pin_12
#define	TOP_RGB_GREEN		GPIOB->BRR = GPIO_Pin_14
#define	TOP_RGB_BLUE		GPIOB->BRR = GPIO_Pin_13

#define	BUTTOM_RGB_RED		GPIOB->BRR = GPIO_Pin_9;
#define	BUTTOM_RGB_GREEN	GPIOB->BRR = GPIO_Pin_8;
#define	BUTTOM_RGB_BLUE		GPIOB->BRR = GPIO_Pin_7;


typedef enum{
	RED = 1,	//��
	GREEN,		//��
	BLUE,			//��
	YELLOW,		//��
	PURPLE,		//��
	CYAN,			//��
	WHITE,		//��
}LedColor;


void LedInit(void);

//#define led1 1
//#define led2 2
//#define led3 3
//#define led4 4

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


void LeftButtomLedBlinkSet(void);
void TopLedColorSet(uint8_t LedColor);
void LedStatusOff(void);
void TopLedStatus(PlaneData plane);
void TopLedBlink(uint8_t ledColor);
void ButtomLedColorSet(const uint8_t LedColor);
#endif
