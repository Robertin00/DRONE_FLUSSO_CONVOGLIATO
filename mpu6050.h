/***********************************************************************
 * VERSIONE DI PROVA DEL FILE
 * dichiarazione e spiefazione delle funzioni utilizzate per la gestione 
 * dell'MPU6050 IvenSense
 * Prima stesura : 07/03/2025
 * Autore : Robert Laurentiu Mincu
 * 
 */


#include <defANDdata.h>
#include <stdio.h>
#include <string.h>


/**
 * La funzione inizializza l'intero MPU6050.
 * Verifica il corretto funzionamento della comunicazione seriale sotto protocollo I2C. Se presente
 * all'occorenza disattiva la modalità "Sleep" del sensore ponendolo in modalità "Cycle", imposta
 * il "SampleRate" del accelerometro e giroscopio e configura la sensibilità di misurazione dell'
 * accelerometro e del giroscopio. Di default la funzione imposta la sensibilità dell'
 * accelerometro a (+-2g) e la sensibilità del giroscopio a (+-250°)
 * 
 */
void MPU6050_Init(void);


/**
 * La funzione legge il valore dell'accelerazione grezza, acquisita dal registro interno del
 * dispositivo e la elabora(versione 1.0 della funzione, in cui elabora i dati solamente se il
 * dispositivo ha sensibilità di accelerometro impostata (+-2g))
 */
void MPU6050_LettoreAccelerazione(void);

/**
 * La funzione legge il valore della velocità angolare grezza, acquisita dal registro interno 
 * del dispositivo e la elabora(versione 1.0 della funzione, in cui elabora i dati solamente se 
 * il dispositivo ha sensibilità di giroscopio impostata a (+-250°/s))
 */
void MPU6050_LettoreVelAngolare(void);


