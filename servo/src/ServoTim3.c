/*
 * ServoTim3.c
 *
 *  Created on: Mar 10, 2025
 *      Author: gruppo servo.
 *
 *      EQUIVALENTE DI MTU4
 */

#include "stm32h7xx_hal.h"
#include "ServoTim3.h"
extern TIM_HandleTypeDef htim3;

/**
 * Imposta il valore massimo del duty cycle per il PWM.
 * Il valore massimo del duty cycle viene calcolato come percentuale del periodo totale del timer
 * e impostato nei registri di confronto (CCR) dei canali PWM 1 e 2.
 * Note:
 * - MAX_DUTY è un valore percentuale (0-100).
 * - htim3.Init.Period rappresenta il valore massimo del contatore del timer (ARR).
 */
void Servo_SetMaxDuty(void) {
    uint32_t max_pulse = ((htim3.Init.Period + 1) * MAX_DUTY) / 100;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, max_pulse);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, max_pulse);
}

/**
 * Imposta il valore minimo del duty cycle per il PWM.
 * Il valore minimo del duty cycle viene calcolato come percentuale del periodo totale del timer
 * e impostato nei registri di confronto (CCR) dei canali PWM 1 e 2.
 * Note:
 * - MIN_DUTY è un valore percentuale (0-100).
 * - htim3.Init.Period rappresenta il valore massimo del contatore del timer (ARR).
 */
void Servo_SetMinDuty(void) {
    uint32_t min_pulse = ((htim3.Init.Period + 1) * MIN_DUTY) / 100;  // Calcola il valore di CCR per il minimo duty cycle
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, min_pulse);  // Imposta il valore CCR per il canale 1
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, min_pulse);  // Imposta il valore CCR per il canale 2
}

/*
 * funzione di inizializzazione dei servo
 *  HAL_TIM_PWM_Start() avvia i canali PWM TIM3_CH1 (PA6) e TIM3_CH2 (PA7).
 *
 */
//equivalente di Initialize_MTU_C4
void Initialize_TIM3(){
	 HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
}

/*
 *  reset e avvio tim3
 */
void StartCount_TIM3() {
	 __HAL_TIM_SET_COUNTER(&htim3, 0);  // Resetta e starta il contatore di TIM3
	    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);  // Avvia PWM su TIM3_CH1
	    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);  // Avvia PWM su TIM3_CH2
}

/*
 * stop timer tim3
 */
void StopCount_TIM3(void){
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);  // Ferma PWM su TIM3_CH1
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);  // Ferma PWM su TIM3_CH2
}

// è l equivalente di GetTGR_A_MTU_C4(), restituisce il valore di ARR
uint16_t GetARR_TIM3(void) {
    return TIM3->ARR;
}

/*
 * set valore arr, equivalente di SetTGR_A_MTU_C4 e  SetTGR_C_MTU_C4 (inutilizzato) arr_val==tgr_a_val==tgr_c_val
 */
void SetARR_TIM3(uint16_t arr_val) {
    __HAL_TIM_DISABLE(&htim3);   // Ferma il timer
    __HAL_TIM_SET_AUTORELOAD(&htim3, arr_val);  // Imposta il nuovo ARR
    __HAL_TIM_ENABLE(&htim3);    // Riavvia il timer
    __HAL_TIM_GenerateEvent(&htim3, TIM_EVENTSOURCE_UPDATE); // Aggiorna subito il timer
}

void SetCCR1_TIM3(uint16_t ccr1_val) {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ccr1_val);  // Imposta CCR1 ccr1_val=tgr_b_val
}

void SetCCR2_TIM3(uint16_t ccr2_val) {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ccr2_val);  // Imposta CCR2 ccr2_val=tgr_d_val
}


