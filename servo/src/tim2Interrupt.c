/*
 * tim2Interrupt.c
 *
 *  Created on: Mar 19, 2025
 *      Author: ricca
 *
 *
 *
 *
 */

#include "stm32h7xx_hal.h"
#include "tim2Interrupt.h"
#include <stdint.h>
extern TIM_HandleTypeDef htim2;



volatile uint8_t timer_10mS_flag = 0;
volatile uint8_t timer_50mS_flag = 0;
volatile uint8_t timer_100mS_flag = 0;

unsigned int general_timer_mS =0;

// struct timerClocks timers;

unsigned int get_ms(){
	return general_timer_mS;
}

void delay_ms(unsigned int t){
	unsigned int last = general_timer_mS;
	while(general_timer_mS-last<t);
}


// funzione isr
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        static uint8_t counter10 = 0;
        static uint8_t counter50 = 0;
        static uint8_t counter100 = 0;

        counter10++;
        counter50++;
        counter100++;

        if (counter10 >= 10)
        {
            timer_10mS_flag = 1;
            counter10 = 0;
        }
        if (counter50 >= 50)
        {
            timer_50mS_flag = 1;
            counter50 = 0;
        }
        if (counter100 >= 100)
        {
            timer_100mS_flag = 1;
            counter100 = 0;
        }
    }
}


void tim2Interrupt_startcount(){
	  __HAL_TIM_SET_COUNTER(&htim2, 0);  // Resetto il contatore per sicurezza

	    if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
	    {
	        Error_Handler();
	    }
}
// ISR TIM2
/* non implementata
 *
 * void TIM2_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) != RESET)
    {
        if (__HAL_TIM_GET_IT_SOURCE(&htim2, TIM_IT_UPDATE) != RESET)
        {
            __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);

            general_timer_mS++;
            timers.timer_1mS = 1;
            if (!(general_timer_mS % 5))
            {
                timers.timer_5mS = 1;
                if (!(general_timer_mS % 10))
                {
                    timers.timer_10mS = 1;
                    if (!(general_timer_mS % 20))
                    {
                        timers.timer_20mS = 1;
                    }
                    if (!(general_timer_mS % 50))
                    {
                        timers.timer_50mS = 1;
                        if (!(general_timer_mS % 100))
                        {
                            timers.timer_100mS = 1;
                            if (!(general_timer_mS % 500))
                            {
                                timers.timer_500mS = 1;
                                if (!(general_timer_mS % 1000))
                                {
                                    timers.timer_1000mS = 1;
                                    if (!(general_timer_mS % 2000))
                                    {
                                        timers.timer_2000mS = 1;
                                        general_timer_mS = 0;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
*/
