/*
 * ServoTim3.h
 *
 *  Created on: Mar 10, 2025
 *      Author: gruppo servo
 */

#ifndef SRC_SERVOTIM3_H_
#define SRC_SERVOTIM3_H_

#define PCLK_FRQ 48000000/16 /* System Peripheral Clock (PCLK) Frequency in HZ (48 MHz) /16 to use PCLK/16 prescaler */
#define BASE_PWM_FRQ 330 /* Desired PWM Frequency in Hz (Interrupt Frequency of the PWM) (330Hz) */
#define PWM_PERIOD_US 1000000/BASE_PWM_FRQ /* Desired PWM Period in Microseconds (3030 us) */

#define MAX_DUTY     82	/* il valore massimo del duty cycle in percentuale è 82 = 2500us di duty con una frequenza di 330Hz  */
#define MIN_DUTY     17	/*il valore minimo del duty cycle in percentuale è 17 = 500us di duty con una frequenza di 330Hz */


void servo_init(void);
void StartCount_TIM3(void);
void StopCount_TIM3(void);

uint16_t GetARR_TIM3(void);
void SetARR_TIM3(uint16_t arr_val);
void SetCCR1_TIM3(uint16_t ccr_val);
void SetCCR2_TIM3(uint16_t ccr_val);


#endif /* SRC_SERVOTIM3_H_ */
