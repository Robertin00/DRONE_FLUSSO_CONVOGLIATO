/*
 * servo.c
 *
 *  Created on: Mar 6, 2025
 *      Author: gruppo servo
 */


#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_tim.h"
extern TIM_HandleTypeDef htim3;

#include "servo.h"
#include "ServoTim3.h"





void Servos_Init(){
	Initialize_TIM3();
	StartCount_TIM3();
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

	Servo_Write_deg(SERVO_PITCH, START);
	Servo_Write_deg(SERVO_ROLL, START);
}



/*
 *
 */
void Servo_Write_PWM(int channel, float value){
	/* Channel value must be 1 or 2 */
	if(channel < 1 || channel > 2)	return;
	/* Check if value is a percentage. If not in range 1-99 return */
	if(value<=1 || value>=99)	return;

	uint16_t arr_val, ccr1_val, ccr2_val;
	arr_val=GetARR_TIM3();
	/* ARR register contains PWM period in counts */

	switch(channel){
	case 1:
		ccr1_val=(arr_val*value)/100.0;
		SetCCR1_TIM3(ccr1_val);
		break;
	case 2:
		ccr2_val=(arr_val*value)/100.0;
		SetCCR2_TIM3(ccr2_val);
		break;

	}
}


/*
 * all'interno della funzione entrano canale e us (?)
 * la funzione controlla se si tratta del canale 1 o 2 altrimenti esce
 * passa poi a servo_Write_PWM il canale e Un valore percentuale di us
 *  rispetto al periodo PWM totale.
 */
void  Servo_Write_us(int channel, float us){
	if(channel < 1 || channel > 2)	return;
	Servo_Write_PWM(channel , ((us*100)/(PWM_PERIOD_US)) );
}

/*
 * i valori di channel si riferiscono ai servomotori, uno è dedicato al roll
 * un altro al pitch (hanno pin pa6 e pa7) BISOGNA ANCORA DISCRIMINARE QUALE
 * SIA UNO E QUALE SIA L ALTRO
 *
 * la funzione di occupa di verificare che i servomotori non eccedano mai gli angoli
 * limite e se cosi fosse, vengono impostati all angolo limite
 *
 *  switch serve a correggere la posizione del servomotore
	   applicando un offset di trim specifico per il servo Pitch
	   e per il servo Roll.

	   viene poi chiamata la funzione Servo_Write_us alla quale si passa
	   il canale, deg, SERVO_MIN_DEG(0),
		SERVO_MAX_DEG(180), SERVO_MIN_US(500), SERVO_MAX_US(2500) (servo max e min us
		corrispondo alla posizione minima e massima dei servo in microsecondi)
 */

void Servo_Write_deg(int channel, float deg){
	if(channel < 1 || channel > 2)	return;

	deg=deg+OFFSET;

	/* Make sure degree is between physical angle limits */
	if(deg < SERVO_MIN_BOUND_DEG)
		deg = SERVO_MIN_BOUND_DEG;
	else if(deg > SERVO_MAX_BOUND_DEG)
		deg = SERVO_MAX_BOUND_DEG;

	switch(channel){
		case SERVO_PITCH:
			deg += SERVO_PITCH_TRIM;
			break;
		case SERVO_ROLL:
			deg += SERVO_ROLL_TRIM;
			break;
		}
Servo_Write_us(channel, range_conversion(deg, SERVO_MIN_DEG, SERVO_MAX_DEG, SERVO_MIN_US, SERVO_MAX_US));
}

/*******************************************************
  Maps a value from a range to another

  FROM_SRC = Minimum value of the source range
  TO_SCR = Maximum value of the source range
  FROM_DST = Minimum value of the destination range
  TO_DST = Maximum value of the destination range
  *****************************************************/

/*
 Questa funzione mappa un valore da un intervallo di
 origine a un intervallo di destinazione.
 Prende un valore (VAL) che appartiene a un certo intervallo [FROM_SRC, TO_SRC]
 Lo converte in un valore equivalente in un nuovo intervallo [FROM_DST, TO_DST]
 Mantiene la proporzionalità tra i due intervalli

 Utile per:

 Conversioni di unità (es. da gradi a microsecondi per PWM)
 Scalare un valore in un range differente
 */
float range_conversion (float VAL, float FROM_SRC, float TO_SRC, float FROM_DST, float TO_DST){

	return (((TO_DST-FROM_DST)/(TO_SRC-FROM_SRC))*(VAL-FROM_SRC)) + FROM_DST ; /* è us */
}
	/*
	 *  us è il valore del segnale PWM in microsecondi che controlla la posizione del servo.
 Deriva dalla conversione dell'angolo (deg) in un segnale PWM corrispondente.
 Viene passato a Servo_Write_PWM() per impostare il duty cycle del PWM.
	 */

  void set_input(SimulatedAHRS* simulated, float *roll_deg, float *pitch_deg){
	  *roll_deg = simulated->RollDeg;
	   *pitch_deg = simulated->PitchDeg;
	//*roll_deg=AHRS->ahrs_data.RollDeg;
	//*pitch_deg=AHRS->ahrs_data.PitchDeg;

};







