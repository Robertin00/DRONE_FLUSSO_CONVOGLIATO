
/*************************************************
 * File Name  : PID.c
 * Description: Implementazione del controllo PID per STM32
 * Version    : 3.0
 * Author     : Adattato per STM32
 *************************************************/

#include "PID.h"

/**
 * @brief  Inizializza la struttura PID con i parametri forniti.
 * @param  conf: Puntatore alla struttura PID_config
 * @param  kp: Guadagno proporzionale
 * @param  ki: Guadagno integrale
 * @param  kd: Guadagno derivativo
 * @param  dt: Tempo di campionamento
 * @param  outMin: Valore minimo dell'output
 * @param  outMax: Valore massimo dell'output
 */
void PID_Init(PID_config* conf, float kp, float ki, float kd, float dt, float outMin, float outMax)
{
    conf->kp = kp;
    conf->ki = ki;
    conf->kd = kd;
    conf->dt = dt;
    conf->ITerm = 0.0f;
    conf->lastError = 0.0f;
    conf->outMax = outMax;
    conf->outMin = outMin;
}

/**
 * @brief  Calcola il valore di output del PID.
 * @param  input: Valore attuale (feedback del sensore)
 * @param  setPoint: Valore desiderato (target)
 * @param  conf: Puntatore alla struttura PID_config
 * @retval Valore corretto da applicare al sistema
 */
float PID_Compute(float input, float setPoint, PID_config* conf)
{
    // Calcola l'errore attuale
    float error = setPoint - input;

    // Componente integrale
    conf->ITerm += (error * conf->dt) * conf->ki;
    if (conf->ITerm > conf->outMax) conf->ITerm = conf->outMax;
    else if (conf->ITerm < conf->outMin) conf->ITerm = conf->outMin;

    // Componente derivativa
    float dInput = (error - conf->lastError) / conf->dt;

    // Calcola l'output del PID
    float output = (conf->kp * error) + (conf->ITerm) + (conf->kd * dInput);

    // Limita l'output nei limiti definiti
    if (output > conf->outMax) output = conf->outMax;
    else if (output < conf->outMin) output = conf->outMin;

    // Aggiorna la memoria per il prossimo ciclo
    conf->lastError = error;

    return output;
}
