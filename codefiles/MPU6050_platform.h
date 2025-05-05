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
 * @brief La funzione inizializza la struttura dati di controllo e gestione del dispositivo con valori di default
 * 
 * @param disp Puntatore alla struttura dati MPU6050_dispositivo_t
 * @return void
 */
void MPU6050_PlatformInit(MPU6050_disp disp);

 /**
  * @brief La funzione verifica lo stato di funzionamento della periferica di comunicazione I2C scelta per il 
  *        controllo del dispositivo inoltre verifica l’indirizzo utile alla comunicazione I2C del dispositivo.
  * 
  * @param disp Puntatore alla struttura dati MPU6050_dispositivo_t
  * @return HAL_StatusTypeDef Tipo di dato enumerato definito nella HAL, STCubeIDE, rappresenta lo stato di ritorno
  *         delle funzioni HAL
  */
HAL_StatusTypeDef MPU6050_VerificaComunicazioni(MPU6050_disp disp);

/**
 * @brief La funzione imposta la modalità di funzionamento del dispositivo. Offre inoltre la possibilità di
 *         eaeguire il "soft reset" del dispositivo, se scelta l'opzione, dopo il "soft reset" la modalità
 *         di funzionamento viene impostata ad"ACQUISIZIONE_CONTINUA"
 * 
 * @param disp Puntatore alla struttura dati MPU6050_dispositivo_t
 * @param modFunzionamento Tipo dato enumerato definito in rappresentanten le modalità di funzionamento 
 *                         del dispositivo utilizzate in questo progetto
 * @return HAL_StatusTypeDef Tipo di dato enumerato definito nella HAL, STCubeIDE, rappresenta lo stato di ritorno
 *                           delle funzioni HAL
 */
HAL_StatusTypeDef MPU6050_ImpostaModFunzionamento(MPU6050_disp disp, MPU6050_modFunzionamento modFunzionamento);

/**
 * @brief La funzione imposta il valore del "Digital Low-Pass Filter" del dispositivo tra quelli consentiti
 * 
 * @param disp Puntatore alla struttura dati MPU6050_dispositivo_t
 * @param valore_DLPF Tipo di dato enumerato rappresentante i valori impostabili del "DLPF"
 * @return HAL_StatusTypeDef Tipo di dato enumerato definito nella HAL, STCubeIDE, rappresenta lo stato di ritorno
 *                           delle funzioni HAL
 */
HAL_StatusTypeDef MPU6050_ImpostaDLPF(MPU6050_disp disp, MPU6050_valoreDLPF valore_DLPF);

/**
 * @brief La funzione restituisce l'attuale valore del "Digital Low-Pass Filter" attualmente impostato
 * 
 * @param disp Puntatore alla struttura dati MPU6050_dispositivo_t
 * @param buffer_valore_DLPF Puntatore ad un tipo di dato uint8_t, in cui il valore attuale del "DLPF" 
 *                           verrà salvato
 * @return HAL_StatusTypeDef Tipo di dato enumerato definito nella HAL, STCubeIDE, rappresenta lo stato di ritorno
 *                           delle funzioni HAL
 */
HAL_StatusTypeDef MPU6050_OttieniDLPF(MPU6050_disp disp, uint8_t* buffer_valore_DLPF);

/**
 * @brief La funzione imposta il valore del divisore della frequenza di acquisizione del giroscopio usato per 
 *        generare il "Sample Rate" del dispositivo
 * 
 * @param disp Puntatore alla struttura dati MPU6050_dispositivo_t
 * @param divisore tipo di dato uint8_t, il cui valore sarà il divisore della frequenza di campionamento
 * @return HAL_StatusTypeDef Tipo di dato enumerato definito nella HAL, STCubeIDE, rappresenta lo stato di ritorno
 *                           delle funzioni HAL
 * @note Per un corretto funzionamento del dispositivo gestito con questa libreria, si raccomanda il valore del divisore
 *       pari a 1 o 2. Per valori superiori è consigliato eseguire una calibrazione sostituendo i valori di :
 *       STANDARD_TIMEOUT_MILLISEC, AMPLIFICATORE_EMPIRICO_INTEGRALE_ASSE_X, AMPLIFICATORE_EMPIRICO_INTEGRALE_ASSE_Y,
 *       e AMPLIFICATORE_EMPIRICO_INTEGRALE_ASSE_Z 
 */
HAL_StatusTypeDef MPI6050_ImpostaDivisoreFrequenzaCampionamento(MPU6050_disp disp, uint8_t divisore);

/**
 * @brief La funzione restituisce l'attuale valore del divisore della frequenza di acquisizione del giroscopio
 * 
 * @param disp Puntatore alla struttura dati MPU6050_dispositivo_t
 * @param buffer_valore_SMPLR Puntatore ad un tipo di dato uint8_t, in cui il valore attuale del "Divisore" 
 *                            verrà salvato
 * @return HAL_StatusTypeDef Tipo di dato enumerato definito nella HAL, STCubeIDE, rappresenta lo stato di ritorno
 *                           delle funzioni HAL
 */
HAL_StatusTypeDef MPU6050_OttieniDivisoreFrequenzaCampionamento(MPU6050_disp disp, uint8_t* buffer_valore_SMPLR);

/**
 * @brief La funzione imposta il valore dell'intervallo a piena scala del giroscopio
 * 
 * @param disp Puntatore alla struttura dati MPU6050_dispositivo_t
 * @param valore_FSR_giroscopio Tipo di dato enumerato rappresentante il valore dell'intervallo a piena scala da 
 *                              impostare
 * @return HAL_StatusTypeDef Tipo di dato enumerato definito nella HAL, STCubeIDE, rappresenta lo stato di ritorno
 *                           delle funzioni HAL
 * @note Per il proggetto del drone a flusso convogliato è consigliato utilizzare @ref FULL_SCALE_RANGE_250_gr_sec, 
 *       essendo l'impostazione con più accuratezza. Per diversi valori del piano 
 */
HAL_StatusTypeDef MPU6050_ImpostaFullScaleRangeGiroscopio(MPU6050_disp disp, MPU6050_intervalloMisurazioneGiroscopio valore_FSR_giroscopio);

/**
 * @brief  La funzione restituisce l'attuale valore del intervallo a piena scala del giroscopio
 * 
 * @param disp Puntatore alla struttura dati MPU6050_dispositivo_t
 * @param buffer Puntatore ad un tipo di dato uint8_t, in cui il valore attuale del "Full Scale Range"
 *               del giroscopio verrà salvato
 * @return HAL_StatusTypeDef Tipo di dato enumerato definito nella HAL, STCubeIDE, rappresenta lo stato di ritorno
 *                           delle funzioni HAL
 */
HAL_StatusTypeDef MPU6050_OttieniFullScaleRangeGiroscopio(MPU6050_disp disp, uint8_t* buffer);

/**
 * @brief La funzione imposta il valore dell'intervallo a piena scala dell'accelerometro
 * 
 * @param disp Puntatore alla struttura dati MPU6050_dispositivo_t
 * @param valore_FSR_accelerometro Tipo di dato enumerato rappresentante il valore dell'intervallo a piena scala da 
 *                              impostare
 * @return HAL_StatusTypeDef Tipo di dato enumerato definito nella HAL, STCubeIDE, rappresenta lo stato di ritorno
 *                           delle funzioni HAL
 * @note Per il progetto del drone a flusso convogliato è consigliato utilizzare @ref FULL_SCALE_RANGE_2g 
 */
HAL_StatusTypeDef MPU6050_ImpostaFullScaleRangeAccelerometro(MPU6050_disp disp, MPU6050_intervalloMisurazioneAccelerometro valore_FSR_accelerometro);

/**
 * @brief La funzione restituisce l'attuale valore del l'intervallo a piena scala dell'accelerometro
 * 
 * @param disp  Puntatore alla struttura dati MPU6050_dispositivo_t
 * @param buffer Puntatore ad un tipo di dato uint8_t, il cui valore attuale del "Full Scale Range"
 *               dell'accelerometro verrrà salvato
 * @return HAL_StatusTypeDef Tipo di dato enumerato definito nella HAL, STCubeIDE, rappresenta lo stato di ritorno
 *                           delle funzioni HAL
 */
HAL_StatusTypeDef MPU6050_OttieniFullScaleRangeAccelerometro(MPU6050_disp disp, uint8_t* buffer);

/**
 * @brief La funzione inizializza il dispositivo impostando : la 
 *        modalità di funzionamento del sensore, il valore del 
 *        "Digital Low-Pass Filter" (DLPF), il valore del 
 *        divisore della frequenza di acquisisione del giroscopio, 
 *        l'intervallo a piena scala del giroscopio e 
 *        dell'accelerometro "Full Scale Range".
 * 
 * @param disp Puntatore alla struttura dati MPU6050_dispositivo_t
 * @param modFunzionamento Tipo dato enumerato definito in 
 *                        rappresentante le modalità di funzionamento
 *                         del dispositivo utilizzate in questo 
 *                         progetto
 * @param valore_DLPF  Tipo di dato enumerato rappresentante i valori
 *                     impostabili del "DLPF"
 * @param divisore  Tipo di dato uint8_t, il cui valore sarà il 
 *                  divisore della frequenza di campionamento
 * @param valore_FSR_giroscopio Tipo di dato enumerato rappresentante
 *                              il valore dell'intervallo a piena 
 *                              scala da impostare per il giroscopio
 * @param valore_FSR_accelerometro Tipo di dato enumerato 
 *                                 rappresentante il valore 
 *                                  dell'intervallo a piena scala da 
 *                                 impostare per l'accelerometro 
 * @return HAL_StatusTypeDef Tipo di dato enumerato definito nella 
 *                           HAL, STCubeIDE, rappresenta lo stato di
 *                           ritorno delle funzioni HAL
 */
HAL_StatusTypeDef MPU6050_InizializzazioneDispositivo(MPU6050_disp disp, MPU6050_modFunzionamento modFunzionamento, MPU6050_valoreDLPF valore_DLPF, uint8_t divisore, MPU6050_intervalloMisurazioneGiroscopio valore_FSR_giroscopio, MPU6050_intervalloMisurazioneAccelerometro valore_FSR_accelerometro);

/**
 * @brief La funzione è incaricata di dettare il tempo di acquisione dell'informazione, proveniente dall'accelerometro 
 *        e dal giroscopio, da parte del sistema.
 *        Solo quando un nuovo dato è pronto per la lettura la funzione restituisce "true". Il fine è evitare letture 
 *        ridondanti.
 * 
 * @param disp Puntatore alla struttura dati MPU6050_dispositivo_t
 * @return true Un nuovo dato acquisito dall'accelerometro e/o dal giroscopio è pronto per la lettura
 * @return false Non ci sono nuovi dati acquisiti dall'accelerometro e/o dal giroscopio pronti per la lettura
 */
bool MPU6050_AspettaMisure(MPU6050_disp disp);

/**
 * @brief La funzione accede ai registri interni del sensore per leggere il dato di accelerazione rilevato lungo i tre assi, successivamente a seconda del 
 * 		  "Full Scale Range" dell'accelerometro, i dati verrano manipolati ottenendo l'accelerazione lungo le tre dimensioni in due unità di misura
 * 		  differenti, in [g] e in [m/s^2]
 * 
 * @param disp Puntatore alla struttura dati MPU6050_dispositivo_t
 * @return HAL_StatusTypeDef Tipo di dato enumerato definito nella HAL, STCubeIDE, rappresenta lo stato di ritorno
 *                           delle funzioni HAL
 */
HAL_StatusTypeDef MPU6050_LetturaDatoGrezzoAccelerometro__TrasformazioneDati(MPU6050_disp disp);

/**
 * @brief La funzione utilizza @ref MPU6050_LetturaDatoGrezzoGiroscopio per ottenere il dato della velocità 
 *        angolare. Questo viene poi manipolato in modo da ottenere una stima della posizione angolare del 
 *        dispositivo
 * 
 * 
 * @param disp Puntatore alla struttura dati MPU6050_dispositivo_t
 * @return HAL_StatusTypeDef Tipo di dato enumerato definito nella HAL, STCubeIDE, rappresenta lo stato di ritorno
 *                           delle funzioni HAL
 * @note Per la corretta operatività della funzione a seconda della frequenza di acquisizione del giroscopio 
 *       devono essere ricalcolati : @ref AMPLIFICATORE_EMPIRICO_INTEGRALE_ASSE_X
 *                                   @ref AMPLIFICATORE_EMPIRICO_INTEGRALE_ASSE_Y
 *                                   @ref AMPLIFICATORE_EMPIRICO_INTEGRALE_ASSE_Z
 */
HAL_StatusTypeDef MPU6050_LetturaDatiGiroscopio__TrasformazioneDati(MPU6050_disp disp);

/**
 * @brief La funzione accede ai registri interni del sensore per leggere il dato grezzo acquisito dal giroscopio. 
 *        Questo verrà poi manipolato, in funzione del "Full Scale Range" del giroscopioper ottenere la velocità 
 *        angolare lungo i tre assi
 * 
 * @param disp Puntatore alla struttura dati MPU6050_dispositivo_t
 * @return HAL_StatusTypeDef Tipo di dato enumerato definito nella HAL, STCubeIDE, rappresenta lo stato di ritorno
 *                           delle funzioni HAL
 */
HAL_StatusTypeDef MPU6050_LetturaDatoGrezzoGiroscopio(MPU6050_disp disp);



//per evitare errori
void MPU6050_PlatformInit(MPU6050_disp disp);

HAL_StatusTypeDef MPU6050_VerificaComunicazioni(MPU6050_disp disp);

HAL_StatusTypeDef MPU6050_ImpostaModFunzionamento(MPU6050_disp disp, MPU6050_modFunzionamento modFunzionamento);

HAL_StatusTypeDef MPU6050_ImpostaDLPF(MPU6050_disp disp, MPU6050_valoreDLPF valore_DLPF);

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


