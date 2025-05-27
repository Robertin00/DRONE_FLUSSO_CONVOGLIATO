/*************************************************
 * File Name  : PID.h
 * Description: Definizione della struttura e funzioni PID
 * Version    : 3.0
 * Author     : Adattato per STM32
 *************************************************/



#ifndef SRC_PID_H_
#define SRC_PID_H_


#include <stdint.h>

/**
 * @brief Struttura per la configurazione del PID
 */
typedef struct
{
    float kp;      ///< Guadagno proporzionale
    float ki;      ///< Guadagno integrale
    float kd;      ///< Guadagno derivativo
    float dt;      ///< Tempo di campionamento
    float lastError; ///< Errore precedente (per calcolo derivativo)
    float ITerm;   ///< Componente integrale accumulata
    float outMax;  ///< Valore massimo dell'output
    float outMin;	///< Valore minimo dell'output
} PID_config;

/**
 * @brief Inizializza la struttura PID
 * @param conf Puntatore alla struttura PID_config
 * @param kp Guadagno proporzionale
 * @param ki Guadagno integrale
 * @param kd Guadagno derivativo
 * @param dt Tempo di campionamento
 * @param outMin Valore minimo dell'output
 * @param outMax Valore massimo dell'output
 */


void PID_Init(PID_config* conf, float kp, float ki, float kd, float dt, float outMin, float outMax);

/**
 * @brief Calcola l'output del PID
 * @param input Valore attuale (feedback del sistema)
 * @param setPoint Valore desiderato
 * @param conf Puntatore alla struttura PID_config
 * @return Valore corretto da applicare al sistema
 */


float PID_Compute(float input, float setPoint, PID_config* conf);

#endif /* SRC_PID_H_ */

