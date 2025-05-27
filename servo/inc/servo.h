/*
 * servo.h
 *
 *  Created on: Mar 6, 2025
 *      Author: gruppo servo
 */

#ifndef SRC_SERVO_H_
#define SRC_SERVO_H_

#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_tim.h"
#include "SetupAHRS.h"

/* costanti servo */
#define SERVO_ROLL 2 			// Roll = pa
#define SERVO_PITCH 1			// Pitch = pa
#define SERVO_MIN_US 500 		// Min position of the servo in micro-seconds
#define SERVO_MAX_US 2500 		// Max position of the servo in micro-seconds
#define SERVO_MIN_DEG 0 		// Min position of the servo in degrees
#define SERVO_MAX_DEG 180 		// Max position of the servo in degrees
#define SERVO_MIN_BOUND_DEG 60 	// Servo minimum physical limit in degrees
#define SERVO_MAX_BOUND_DEG 120	// Servo maximum physical limit in degrees
#define SERVO_PITCH_TRIM 0 	// Pitch servo offset in degrees
#define SERVO_ROLL_TRIM 0		// Roll servo offset in degrees
#define OFFSET 90				// Offset per angolo servomotori, range di movimento +/-30° dalla posizione di offset
#define START 0					// Posizione iniziale servomotori, flap perpendicolari al terreno

void Servos_Init();


/* Convertion from percentage value to counts.*/
void Servo_Write_PWM(int channel, float value);

/* Convertion from microseconds to percentage value */
void Servo_Write_us(int channel, float us);

/* Convertion from degrees value to microseconds */
void Servo_Write_deg(int channel, float deg);

/*******************************************************
  Maps a value from a range to another

  FROM_SRC = Minimum value of the source range
  TO_SCR = Maximum value of the source range
  FROM_DST = Minimum value of the destination range
  TO_DST = Maximum value of the destination range
  *****************************************************/
float range_conversion (float VAL, float FROM_SRC, float TO_SRC, float FROM_DST, float TO_DST);

/* set roll's value and pitch's value in degree */

void set_input(SimulatedAHRS* simulated ,float * ,float *);









































































/*
#define SERVO_PITCH 1
#define SERVO_ROLL 2

#define START 90  // Posizione iniziale del servo

#define SERVO_MIN_DEG 0.0f
#define SERVO_MAX_DEG 180.0f
#define SERVO_MIN_US 1000.0f
#define SERVO_MAX_US 2000.0f
#define PWM_PERIOD_US 20000.0f // Periodo PWM tipico di 20ms

#define OFFSET 0.0f
#define SERVO_MIN_BOUND_DEG 0.0f
#define SERVO_MAX_BOUND_DEG 180.0f
#define SERVO_PITCH_TRIM 0.0f
#define SERVO_ROLL_TRIM 0.0f

typedef struct {
    struct {
        float RollDeg;
        float PitchDeg;
    } ahrs_data;
} AHRS_out;

void Servos_Init();
void Servo_Write_PWM(int channel, float value);
void Servo_Write_us(int channel, float us);
void Servo_Write_deg(int channel, float deg);
float range_conversion(float VAL, float FROM_SRC, float TO_SRC, float FROM_DST, float TO_DST);
void set_input(AHRS_out* ahrs, float* roll_deg, float* pitch_deg);
void Servo_Update(float current_roll, float current_pitch, float setpoint_roll, float setpoint_pitch);


*/

#endif /* SRC_SERVO_H_ */
