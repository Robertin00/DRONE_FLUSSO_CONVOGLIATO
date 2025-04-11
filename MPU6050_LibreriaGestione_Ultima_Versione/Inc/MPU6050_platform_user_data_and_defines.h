/*************************************************************************************************
 * @file MPU6050_platform_user_data
 * @author Robert Laurentiu Mincu
 * @brief il file contiene le dichiarazione delle strutture dati utilizzate per la gestione del 
 *        sensore MPU6050 IvenSense
 * @version 0.1
 * @date 11/04/2025
 *************************************************************************************************/
#include "stdint.h"
#include <stdio.h>
#include "stdbool.h"
#include "stm32h7xx_hal.h"

/*------- Definizioni -------*/

/**
 * @brief 
 * 
 */
#define MPU6050_DEVICE_ADDR_7BIT												0x68

#define MPU6050_DEVICE_ADDR_8BIT				    							0xD0

#define NUMERO_TENTATIVI_STANDARD 												10

#define STANDARD_TIMEOUT_MILLISEC 												1000

#define STANDARD_ERROR_MESSAGE_LENGTH 											100

#define MEM_SIZE_1_BYTE 														1

#define MEM_SIZE_6_BYTE															6

#define RESET_MPU6050 															0x80

#define TEMPO_ATTESA_RESET_MPU6050_MILLISECONDI									200

#define MPU6050_MOD_SLEEP														0x40

#define MPU6050_MOD_CYCLE 														0x20

#define MPU6050_MOD_ACQUISIZIONE_CONTINUA										0x00

#define VALORE_MASSIMO_FREQUENZA_CAMPIONAMENTO 									255

#define BIT_VERIFICA_DATI_ACQUISITI												0x01

#define TIMEOUT_CORTO															100

#define AMPIEZZA_BUFFER_DATI_ACC_GIRO_BYTE										6

#define INVERTI_MISURA															-1

#define FREQUENZA_AGGIORNAMENTO_GIROSCOPIO_DLPF_OFF								8000

#define FREQUENZA_AGGIORNAMENTO_GIROSCOPIO_DLPF_ON								1000

#define ACC_LSB_SENSITIVITY_FOR_FSR_2g											16384.0

#define ACC_LSB_SENSITIVITY_FOR_FSR_4g											8192.0

#define ACC_LSB_SENSITIVITY_FOR_FSR_8g											4096.0

#define ACC_LSB_SENSITIVITY_FOR_FSR_16g											2048.0

#define ACCELERAZIONE_GRAVITAA_MISURA_INTERNAZIONALE							9.81

#define GIRO_LSB_SENSITIVITY_FOR_250_g_s										131.0

#define ESCURSIONE_GIRO_PER_250_g_s												500

#define VALORE_MAX_VEL_ANG_MISURABILE_CON_RUMORE_PER_250_g_s					250.15 //arrotondato per eccesso

#define GIRO_LSB_SENSITIVITY_FOR_500_g_s										65.5

#define ESCURSIONE_GIRO_PER_500_g_s												1000

#define VALORE_MAX_VEL_ANG_MISURABILE_CON_RUMORE_PER_500_g_s					500.6 //arrotondato per eccesso

#define GIRO_LSB_SENSITIVITY_FOR_1000_g_s										32.8

#define ESCURSIONE_GIRO_PER_1000_g_s											2000

#define VALORE_MAX_VEL_ANG_MISURABILE_CON_RUMORE_PER_1000_g_s					1000.9//valori teorici

#define GIRO_LSB_SENSITIVITY_FOR_2000_g_s										16.4

#define VALORE_MAX_VEL_ANG_MISURABILE_CON_RUMORE_PER_2000_g_s					2002 //valori teorici

#define ESCURSIONE_GIRO_2000_g_s												4000

#define PI_GRECO_RADIANTI_IN_GRADI												180.0

#define PI_GRECO																3.14159265358979323846

#define ANGOLO_360																360

#define ANGOLO_0																0

#define MASSIMA_ESCURSIONE_RUMORE_BIANCO_ASSE_X_EMPIRICO						0.320611

#define MASSIMA_ESCURSIONE_RUMORE_BIANCO_ASSE_Y_EMPIRICO						0.290111

#define MASSIMA_ESCURSIONE_RUMORE_BIANCO_ASSE_Z_EMPIRICO						0.2214

#define AMPLIFICATORE_EMPIRICO_INTEGRALE_ASSE_X									8.00

#define AMPLIFICATORE_EMPIRICO_INTEGRALE_ASSE_Y									11.7356

#define AMPLIFICATORE_EMPIRICO_INTEGRALE_ASSE_Z									6.95


/*------- Enumerazioni -------*/

typedef enum{
	DLPF_CFG_0 = 0x00,
	DLPF_CFG_1 = 0x01,
	DLPF_CFG_2 = 0x02,
	DLPF_CFG_3 = 0x03,
	DLPF_CFG_4 = 0x04,
	DLPF_CFG_5 = 0x05,
	DLPF_CFG_6 = 0x06
}MPU6050_valoreDLPF;


/**
 * @brief Enumerazione delle tipologie di "Full scale range" dell'accelerometro
 * 
 */
typedef enum{
    FULL_SCALE_RANGE_2g  = 0x00,
    FULL_SCALE_RANGE_4g  = 0x08,
    FULL_SCALE_RANGE_8g  = 0x10,
    FULL_SCALE_RANGE_16g = 0x18,

} MPU6050_intervalloMisurazioneAccelerometro;

/**
 * @brief Enumerazione delle tipologie di "Full scale range" del giroscopio
 * 
 */
typedef enum{
    FULL_SCALE_RANGE_250_gr_sec  = 0x00,
    FULL_SCALE_RANGE_500_gr_sec  = 0x08,
    FULL_SCALE_RANGE_1000_gr_sec = 0x10,
    FULL_SCALE_RANGE_2000_gr_sec = 0x18,

} MPU6050_intervalloMisurazioneGiroscopio;

/**
 * @brief Enumerazione delle modalità di funzionamento del sensore
 * 
 */
typedef enum{

	ACQUISIZIONE_CONTINUA,
    SLEEP,
    CYCLE,
	SOFT_RESET

} MPU6050_modFunzionamento;




/*--------Strutture Dati per gestione della misurazione--------*/


/*----- Accelerometro -----*/

/**
 * @brief Struttura dati contenente il dato grezzo ascquisito direttamente dai registri dell'accelerometro per ogni asse
 * 
 */
typedef struct{
    int16_t accelerazioneGrezza_X;
    int16_t accelerazioneGrezza_Y;
    int16_t accelerazioneGrezza_Z;
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
    uint16_t DatoGrezzoGiroscopio_X;
    uint16_t DatoGrezzoGiroscopio_Y;
    uint16_t DatoGrezzoGiroscopio_Z;
} MPU6050_datoGrezzoGiroscopio_treAssi;

/**
 * @brief 
 * 
 */
typedef struct{

    float posizioneAngolare_X;
    float vecchia_posizioneAngolare_X_val_integrale;

    float posizioneAngolare_Y;
    float vecchia_posizioneAngolare_Y_val_integrale;

    float posizioneAngolare_Z;
    float vecchia_posizioneAngolare_Z_val_integrale;

} MPU6050_datoPosizioneAngolareGiroscopio_treAssi;

/**
 * @brief 
 * 
 */
typedef struct{
    float velocitaaAngolare_X;
    float velocitaaAngolare_Y;
    float velocitaaAngolare_Z;

} MPU6050_datoVelocitaaAngolareGiroscopio_treAssi;


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
    MPU6050_datoVelocitaaAngolareGiroscopio_treAssi datoVelAngGiro;
} MPU6050_StrutturaDatiGiroscopio_treAssi;

/*------- Strutture dati per l'assistenza a funzioni -------*/

typedef struct{
	float vecchio_DatoX;
	float attuale_DatoX;

	float vecchio_DatoY;
	float attuale_DatoY;

	float vecchio_DatoZ;
	float attuale_DatoZ;

} assistenza_MPU6050_PosizioneAngolareDispositivo;



/*------- Struttura dati per il pilotaggio del dispositivo -------*/

/**
 * @brief 
 *
 *
 * 	-default, periferica hi2c1 e huart3 per NUCLEO-H745ZI-Q
 */
typedef struct {
	I2C_HandleTypeDef perifericaComunicazioneI2c;
	UART_HandleTypeDef perifericaComunicazioneHost;
    uint8_t i2c_slave_address;
    MPU6050_StrutturaDatiAccelerometro_treAssi datiAccelerometro;
    MPU6050_intervalloMisurazioneAccelerometro intAccelerometro;
    MPU6050_StrutturaDatiGiroscopio_treAssi datiGiroscopio;
    MPU6050_intervalloMisurazioneGiroscopio intGiroscopio;
    float datoTemperatura;
    MPU6050_modFunzionamento modFunzionamento;

} MPU6050_dispositivo_t;

typedef MPU6050_dispositivo_t* MPU6050_disp;
