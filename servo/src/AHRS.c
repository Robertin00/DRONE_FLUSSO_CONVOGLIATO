/*
 * AHRS.c
 *
 *  Created on: Mar 15, 2025
 *      Author: ricca
 */


#include "AHRS.h"
// simulated_ahrs.c
#include <stdlib.h>

#include <math.h>

// questi 6 define erano utili per le funzioni lineari sinusoidali e step
#define SIMULATION_STEP 0.1f  // tempo virtuale che avanza ogni chiamata
#define SIMULATION_FREQ 0.01f  // frequenza del movimento (Hz)
#define SIMULATION_AMPLITUDE 5.0f // ampiezza massima -5°
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


// --- Costanti per la Simulazione Dinamica ---
// Disturbo iniziale: l'angolo parte da questo valore.
#define INITIAL_DISTURBANCE_MAGNITUDE 5.0f
// Ritardo prima che il disturbo sia applicato (in secondi). 1s = 1000ms
#define DISTURBANCE_DELAY_TIME 1.0f
// Passo temporale della simulazione (deve corrispondere a dt_servo, es. 0.01s = 10ms)
#define SIMULATION_TIME_STEP 0.01f



// FATTORE DI CORREZIONE DEL SERVO (CRUCIALE!)
// Indica quanto un grado di comando del servo modifica l'angolo del "drone" simulato ad ogni passo.
// Esempio: se servo_cmd_roll è -5° e questo fattore è 0.02,
// l'angolo RollDeg cambierà di (-5 * 0.02) = -0.1 gradi in questo passo di simulazione.
// DOVRAI TARARE QUESTO VALORE! Inizia con un valore piccolo.
#define SERVO_CORRECTION_FACTOR 0.02f


// FATTORE DI SMORZAMENTO NATURALE (Opzionale)
// Rappresenta la tendenza del "drone" simulato a tornare naturalmente verso 0 gradi,
// o a resistere al movimento. Aiuta a stabilizzare la simulazione.
// Se 0, non c'è smorzamento naturale. Prova con un valore piccolo.
#define NATURAL_DAMPING_FACTOR 0.005f


// Variabili statiche globali al file AHRS.c per gestire l'applicazione del disturbo
static float time_elapsed_for_disturbance_s = 0.0f;
static int initial_disturbance_applied_flag = 0; // 0 = non applicato, 1 = applicato



/**
 * @brief Inizializza lo stato del AHRS simulato a 0 e resetta i flag del disturbo.
 * @param simulated Puntatore alla struttura SimulatedAHRS da inizializzare.
 */

void initialize_ahrs_state(SimulatedAHRS* simulated){
	if(simulated!=NULL){
		simulated->RollDeg = 0.0f;
		simulated->PitchDeg = 0.0f;
	}
	time_elapsed_for_disturbance_s=0.0f;
	initial_disturbance_applied_flag=0;
}

/**
 * @brief Aggiorna gli angoli simulati del AHRS in base ai comandi dei servi.
 * Questa funzione implementa un modello dinamico molto semplice.
 * Introduce anche un disturbo iniziale dopo un certo ritardo.
 * @param simulated Puntatore alla struttura SimulatedAHRS che tiene traccia dello stato.
 * @param servo_cmd_roll Comando del servo per il rollio (output del PID, es. -5 gradi).
 * @param servo_cmd_pitch Comando del servo per il beccheggio.
 * @param next_roll_deg_output Puntatore dove verrà scritto il nuovo angolo di rollio calcolato.
 * @param next_pitch_deg_output Puntatore dove verrà scritto il nuovo angolo di beccheggio calcolato.
 */


 void update_simulated_angles(SimulatedAHRS* simulated, float servo_cmd_roll, float servo_cmd_pitch, float* next_roll_deg_output, float* next_pitch_deg_output){
	 if(simulated== NULL){
		 if(next_roll_deg_output != NULL) *next_roll_deg_output =0.0f;
		 if(next_pitch_deg_output != NULL) *next_pitch_deg_output =0.0f;
		 return;
	 }
	 // Applica il disturbo iniziale una sola volta dopo DISTURBANCE_DELAY_TIME
	 if(!initial_disturbance_applied_flag){
		 time_elapsed_for_disturbance_s += SIMULATION_TIME_STEP;
		         if (time_elapsed_for_disturbance_s >= DISTURBANCE_DELAY_TIME) {
		             simulated->RollDeg = INITIAL_DISTURBANCE_MAGNITUDE;
		             simulated->PitchDeg = INITIAL_DISTURBANCE_MAGNITUDE; // Applica anche al pitch per simmetria
		             initial_disturbance_applied_flag = 1;
		         }
	 }
	 else{
		 // Il disturbo è stato applicato, ora il sistema risponde ai comandi del servo.
		         // Modello dinamico semplificato:
		         // L'angolo cambia in proporzione al comando del servo.
		 float roll_change_from_servo = SERVO_CORRECTION_FACTOR * servo_cmd_roll;
		         simulated->RollDeg += roll_change_from_servo;

		         float pitch_change_from_servo = SERVO_CORRECTION_FACTOR * servo_cmd_pitch;
		         simulated->PitchDeg += pitch_change_from_servo;

		         // Applica uno smorzamento naturale (se abilitato)
		         if (NATURAL_DAMPING_FACTOR > 0.0f) {
		             simulated->RollDeg -= NATURAL_DAMPING_FACTOR * simulated->RollDeg; // Tende a tornare a 0
		             simulated->PitchDeg -= NATURAL_DAMPING_FACTOR * simulated->PitchDeg; // Tende a tornare a 0
		         }
	 }

	 // Fornisci gli angoli aggiornati come output della funzione
	     if (next_roll_deg_output != NULL) {
	         *next_roll_deg_output = simulated->RollDeg;
	     }
	     if (next_pitch_deg_output != NULL) {
	         *next_pitch_deg_output = simulated->PitchDeg;
	     }

 }













































































/*void simulate_input_step(SimulatedAHRS* simulated, float *roll_deg, float *pitch_deg){
	static float t=0.0f;
	t = t + SIMULATION_STEP;
if(t>=1.0f){
	// SIMULAZIONE DEL GRADINO DI AMPIEZZA -5
	simulated->RollDeg = SIMULATION_AMPLITUDE;
	simulated->PitchDeg = SIMULATION_AMPLITUDE;
}	else{
	//RITORNO DEI VALORI
	simulated->RollDeg = 0.0f;
	simulated->PitchDeg = 0.0f;
}
*roll_deg= simulated->RollDeg;
*pitch_deg= simulated->PitchDeg;
}
*/
/*void simulate_input_gradual(SimulatedAHRS* simulated, float *roll_deg, float *pitch_deg) {
    static float t = 0.0f;

    // Simula un movimento sinusoidale continuo
    simulated->RollDeg = SIMULATION_AMPLITUDE * sinf(2 * M_PI * SIMULATION_FREQ * t);
    simulated->PitchDeg = SIMULATION_AMPLITUDE * cosf(2 * M_PI * SIMULATION_FREQ * t);

    // Ritorna i valori
    *roll_deg = simulated->RollDeg;
    *pitch_deg = simulated->PitchDeg;

    // Avanza il tempo virtuale
    t += SIMULATION_STEP;
}
*/


/*void simulate_input_linear(SimulatedAHRS* simulated, float *roll_deg, float *pitch_deg) {
    static float value = -30.0f;
    static float step = 1.0f;  // quanto cambia a ogni chiamata

    // Aggiorna il valore lineare
    value += step;

    // Inverti direzione se superi i limiti
    if (value > 30.0f || value < -30.0f) {
        step = -step;
        value += step; // per non restare fuori intervallo
    }

    simulated->RollDeg = value;
    simulated->PitchDeg = value;

    *roll_deg = simulated->RollDeg;
    *pitch_deg = simulated->PitchDeg;
}
*/
