/*************************************************************************************************
 * @file MPU6050_platform.h
 * @author Robert Laurentiu Mincu
 * @brief Il file contiene le dichiarazioni delle funzioni create appositamente per poter 
 *        interfacciare il sensore a NUCLEO-H745ZI-Q utilizzando STCubeIDE
 * @version 0.1
 * @date 11/04/2025
 *************************************************************************************************/
#include "MPU6050_platform_user_data_and_defines.h"
#include "MPU6050_register_map.h"


/**
 * @brief la funzione inizializza la struttura dati di controllo e gestione del dispositivo con valori di default
 */
void MPU6050_PlatformInit(MPU6050_disp disp);

HAL_StatusTypeDef MPU6050_VerificaComunicazioni(MPU6050_disp disp);

HAL_StatusTypeDef MPU6050_ImpostaModFunzionamento(MPU6050_disp disp, MPU6050_modFunzionamento modFunzionamento);

HAL_StatusTypeDef MPU6050_ImpostaDLPF(MPU6050_disp disp, MPU6050_valoreDLPF valore_DLPF);

HAL_StatusTypeDef MPU6050_OttieniDLPF(MPU6050_disp disp, uint8_t* buffer_valore_DLPF);

HAL_StatusTypeDef MPI6050_ImpostaDivisoreFrequenzaCampionamento(MPU6050_disp disp, uint8_t divisore);

HAL_StatusTypeDef MPU6050_OttieniDivisoreFrequenzaCampionamento(MPU6050_disp disp, uint8_t* buffer_valore_SMPLR);

HAL_StatusTypeDef MPU6050_ImpostaFullScaleRangeGiroscopio(MPU6050_disp disp, MPU6050_intervalloMisurazioneGiroscopio valore_FSR_giroscopio);

HAL_StatusTypeDef MPU6050_OttieniFullScaleRangeGiroscopio(MPU6050_disp disp, uint8_t* buffer);

HAL_StatusTypeDef MPU6050_ImpostaFullScaleRangeAccelerometro(MPU6050_disp disp, MPU6050_intervalloMisurazioneAccelerometro valore_FSR_accelerometro);

HAL_StatusTypeDef MPU6050_OttieniFullScaleRangeAccelerometro(MPU6050_disp disp, uint8_t* buffer);

HAL_StatusTypeDef MPU6050_InizializzazioneDispositivo(MPU6050_disp disp, MPU6050_modFunzionamento modFunzionamento, MPU6050_valoreDLPF valore_DLPF, uint8_t divisore, MPU6050_intervalloMisurazioneGiroscopio valore_FSR_giroscopio, MPU6050_intervalloMisurazioneAccelerometro valore_FSR_accelerometro);

bool MPU6050_AspettaMisure(MPU6050_disp disp);

HAL_StatusTypeDef MPU6050_LetturaDatoGrezzoAccelerometro__TrasformazioneDati(MPU6050_disp disp);

HAL_StatusTypeDef MPU6050_LetturaDatiGiroscopio__TrasformazioneDati(MPU6050_disp disp);

HAL_StatusTypeDef MPU6050_LetturaDatoGrezzoGiroscopio(MPU6050_disp disp);
