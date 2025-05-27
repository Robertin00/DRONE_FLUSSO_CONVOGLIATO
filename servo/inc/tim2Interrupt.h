/*
 * tim2Interrupt.h
 *
 *  Created on: Mar 19, 2025
 *      Author: ricca
 */

#ifndef TIM2INTERRUPT_H_
#define TIM2INTERRUPT_H_
#include <stdint.h>

extern volatile uint8_t timer_10mS_flag;
extern volatile uint8_t timer_50mS_flag;
extern volatile uint8_t timer_100mS_flag;


/*
struct timerClocks
{
	uint8_t timer_1mS;
	uint8_t timer_5mS;
	uint8_t timer_10mS;
	uint8_t timer_20mS;
	uint8_t timer_50mS;
	uint8_t timer_100mS;
	uint8_t timer_500mS;
	uint8_t timer_1000mS;
	uint8_t timer_2000mS;
	uint8_t timer_4000mS;
};
*/




void TIM2_IRQHandler(void); // funzione per isr equivalente di
unsigned int get_ms();
void delay_ms(unsigned int t);
void tim2Interrupt_startcount();



#endif /* TIM2INTERRUPT_H_ */
