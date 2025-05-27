/*
 * AHRS.h
 *
 *  Created on: Mar 15, 2025
 *      Author: ricca
 */

#ifndef SRC_AHRS_H_
#define SRC_AHRS_H_

#include <math.h>

#include <stdio.h>


//simulazione ahrs
typedef struct {
    float RollDeg; //angolo di rollio simulato in gradi
    float PitchDeg; //angolo di beccheggio simulato in gradi
} SimulatedAHRS;

void initialize_ahrs_state(SimulatedAHRS* simulated);
void update_simulated_angles(SimulatedAHRS* simulated, float servo_cmd_roll, float servo_cmd_pitch, float* next_roll_deg_output, float* next_pith_deg_output);

//simulazione lineare, sinusoidale e step
//void simulate_input_step(SimulatedAHRS* simulated, float *rollDeg, float *PitchDeg);
//void simulate_input_gradual(SimulatedAHRS* simulated, float *roll_deg, float *pitch_deg);
//void simulate_input_linear(SimulatedAHRS* simulated, float *roll_deg, float *pitch_deg);
#endif /* SRC_AHRS_H_ */
