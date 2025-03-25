/*************************************************************************************************
 * @file MPU6050_platform_user_data
 * @author Robert Laurentiu Mincu
 * @brief il file contiene le dichiarazione delle strutture dati utilizzate per la gestione del 
 *        sensore MPU6050 IvenSense
 *  
 * @version 0.1
 * @date 24/03/2025
 *************************************************************************************************/
#include "stdint.h"
#include <stdio.h>

/*------- Definizioni -------*/

/**
 * @brief 
 * 
 */
#define MPU6050_DEVICE_ADDR 0x68


/*------- Enumerazioni -------*/


/**
 * @brief Enumerazione delle tipologie di "Full scale range" dell'accelerometro
 * 
 */
typedef enum{
    FULL_SCALE_RANGE_2g,
    FULL_SCALE_RANGE_4g,
    FULL_SCALE_RANGE_8g,
    FULL_SCALE_RANGE_16g
} MPU6050_intervalloMisurazioneAccelerometro;

/**
 * @brief Enumerazione delle tipologie di "Full scale range" del giroscopio
 * 
 */
typedef enum{
    FULL_SCALE_RANGE_250_gr_sec,
    FULL_SCALE_RANGE_500_gr_sec,
    FULL_SCALE_RANGE_1000_gr_sec,
    FULL_SCALE_RANGE_2000_gr_sec,
} MPU6050_intervalloMisurazioneGiroscopio;

/**
 * @brief Enumerazione delle modalità di funzionamento del sensore
 * 
 */
typedef enum{
    SLEEP_MODE,
    CYCLE_MODE,
} MPU6050_modFunzionamento;


/*--------Strutture Dati per gestione della misurazione--------*/


/*----- Accelerometro -----*/

/**
 * @brief Struttura dati contenente il dato grezzo ascquisito direttamente dai registri dell'accelerometro per ogni asse
 * 
 */
typedef struct{
    uint16_t accelerazioneGrezza_X;
    uint16_t accelerazioneGrezza_Y;
    uint16_t accelerazioneGrezza_Z;
} MPU6050_datoGrezzoAccelerometro_treAssi;

/**
 * @brief 
 * 
 */
typedef struct {
    float accelerazioneInG_X;
    float accelerazioneInG_Y;
    float accelerazioneInG_Z;
}  MPU6050_datoInGAccelerometro_treAssi;
/**
 * @brief 
 * 
 */
typedef struct{
    float accelerazioneMisuraInternazionale_X;
    float accelerazioneMisuraInternazionale_Y;
    float accelerazioneMisuraInternazionale_Z;
} MPU6050_datoMisuraInternazionaleAccelerometro_treAssi;


/*----- Giroscopio -----*/

/**
 * @brief 
 * 
 */
typedef struct{
    uint16_t velocitaaAngolare_X;
    uint16_t velocitaaAngolare_Y;
    uint16_t velocitaaAngolare_Z;
} MPU6050_datoGrezzoGiroscopio_treAssi;

/**
 * @brief 
 * 
 */
typedef struct{
    float posizioneAngolare_X;
    float posizioneAngolare_Y;
    float posizioneAngolare_Z;
} MPU6050_datoPosizioneAngolareGiroscopio_treAssi;

/**
 * @brief 
 * 
 */
typedef struct{
    float accelerazioneAngolare_X;
    float accelerazioneAngolare_Y;
    float accelerazioneAngolare_Z;
} MPU6050_datoAccelerazioneAngolareGiroscopio_treAssi;


/*----- Struttura dati delle tipologie di misure sui tre assi -----*/

/**
 * @brief 
 * 
 */
typedef struct{
   MPU6050_datoGrezzoAccelerometro_treAssi datoGrezzoAcc;
   MPU6050_datoInGAccelerometro_treAssi datoInGAcc;
   MPU6050_datoMisuraInternazionaleAccelerometro_treAssi datoMisIntAcc;

} MPU6050_StrutturaDatiAccelerometro_treAssi;

/**
 * @brief 
 * 
 */
typedef struct{
    MPU6050_datoGrezzoGiroscopio_treAssi datoGrezzoGiro;
    MPU6050_datoPosizioneAngolareGiroscopio_treAssi datoPosAngGiro;
    MPU6050_datoAccelerazioneAngolareGiroscopio_treAssi datoAccAngGiro;
} MPU6050_StrutturaDatiGiroscopio_treAssi;




/*------- Struttura dati per il pilotaggio del dispositivo -------*/

/**
 * @brief 
 * 
 */
typedef struct {
    uint8_t i2c_slave_address;
    MPU6050_StrutturaDatiAccelerometro_treAssi datiAccelerometro;
    MPU6050_intervalloMisurazioneAccelerometro intAccelerometro;
    MPU6050_StrutturaDatiGiroscopio_treAssi datiGiroscopio;
    MPU6050_intervalloMisurazioneGiroscopio intGiroscopio;
    uint16_t datoTemperatura;
    MPU6050_modFunzionamento modFunzionamento;

} MPU6050_dispositivo_t;

typedef MPU6050_dispositivo_t* MPU6050_disp;
