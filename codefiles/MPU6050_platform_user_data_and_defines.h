/*************************************************************************************************
 * @file MPU6050_platform_user_data
 * @author Robert Laurentiu Mincu
 * @brief il file contiene le dichiarazione delle strutture dati utilizzate per la gestione del 
 *        sensore MPU6050 IvenSense
 * @version 0.1
 * @date 17/04/2025
 *************************************************************************************************/
#include "stdint.h"
#include "stdio.h"
#include "stdbool.h"
#include "stdlib.h"
#include "stm32h7xx_hal.h"

/*------- Definizioni -------*/

/**
 * @brief Questa direttiva del preprocessore serve a specificare l'indirizzo del dispositivo nel 
 *        caso in cui il protocollo di comunicazione I2C utilizzi indirizzi a 7 bit
 * 
 */
#define MPU6050_DEVICE_ADDR_7BIT												0x68

/**
 * @brief Questa direttiva del preprocessore serve a specificare l'indirizzo del dispositivo nel 
 *        caso in cui il protocollo di comunicazione I2C utilizzi indirizzi a 8 bit
 * 
 */
#define MPU6050_DEVICE_ADDR_8BIT				    							0xD0

/**
 * @brief Questa direttiva del preprocessore specifica il numero di tentativi di "comunicazione",
 *        effettuati dalla funzione @ref HAL_I2C_IsDeviceReady, (nella funzione 
 *        @ref MPU6050_VerificaComunicazioni) prima di considerare il dispositivo
 *         "non pronto"
 */
#define NUMERO_TENTATIVI_STANDARD 												10

/**
 * @brief La  seguente direttiva di preprocessore specifica il valore di attesa standard da 
 *        rispettare, prima che la venga indicato un "Timeout"
 * 
 */
#define STANDARD_TIMEOUT_MILLISEC 												1000

/**
 * @brief La seguente direttiva di preprocessore indica la lunghezza standard dei messagi di 
 *        errore
 * 
 */
#define STANDARD_ERROR_MESSAGE_LENGTH 											100

/**
 * @brief La seguente direttiva di preprocessore specifica che deve essere letto o scritto 
 *        1 [byte] sul registro interessato
 * 
 */
#define MEM_SIZE_1_BYTE 														1

/**
 * @brief La seguente direttiva di proprocessore specifica che deve essere letti o scritti
 *        6 [byte] sul registro interessato
 *
 */
#define MEM_SIZE_6_BYTE															6

/**
 * @brief La seguente direttiva di preprocessore specifica il valore, che deve essere scritto nel
 *        registro @ref  PWR_MGMT_1 , per effettuare il reset del dispositivo
 * 
 */
#define RESET_MPU6050 															0x80

/**
 * @brief Questa direttiva di preprocessore specifica il valore di attesa,in millisecondi, da
 *        rispettare al fine di ottenere un corretto reset del dispositivo
 * 
 */
#define TEMPO_ATTESA_RESET_MPU6050_MILLISECONDI									200

/**
 * @brief La seguente direttiva di preprocessore specifica il valore da scrivere nel
 *        registro @ref  PWR_MGMT_1 , al fine di configurare il dispositivo per funzionare in 
 *        modalità "Sleep"
 */
#define MPU6050_MOD_SLEEP														0x40

/**
 * @brief La seguente direttiva di preprocessore specifica il valore da scrivere nel
 *        registro @ref  PWR_MGMT_1 , al fine di configurare il dispositivo per funzionare in 
 *        modalità "Cycle"
 * 
 */
#define MPU6050_MOD_CYCLE 														0x20

/**
 * @brief La seguente direttiva di preprocessore specifica il valore, che deve essere scritto nel
 *        registro @ref  PWR_MGMT_1 , al fine di configurare il dispositivo per funzionare nella
 *        modalità standard. La modalità nella quale il sensore acquisisce continuamente dati
 * 
 */
#define MPU6050_MOD_ACQUISIZIONE_CONTINUA										0x00

/**
 * @brief la seguente direttiva di preprocessore specifica il valore massimo permesso per il 
 *        "Divisore della frequenza di acquisizione" del giroscopio
 * 
 */
#define VALORE_MASSIMO_DIVISORE_REQUENZA_ACQUISIZIONE_GIRO						255

/**
 * @brief La seguente direttiva di preprocessore specifica il valore di comparazione, della
 *        variabile @ref statoDati, nella funzione @ref MPU6050_AspettaMisure, per verificare
 *        la presenza o meno, di un nuovo dato da leggere
 * 
 */
#define BIT_VERIFICA_DATI_ACQUISITI												0x01

/**
 * @brief La seguente direttiva di preprocessore specifica il valore di "Timeout" corto in 
 *        millisecondi
 * 
 */
#define TIMEOUT_CORTO															100

/**
 * @brief La seguente direttiva di preprocessore specifica il numero di byte che devono essere
 *        letti dalla serie di registri che contengono l'informazione di accelerazione lungo i
 *        tre assi.
 * 
 */
#define AMPIEZZA_BUFFER_DATI_ACC_GIRO_BYTE										6

/**
 * @brief La seguente direttiva di preprocessore è stata creata con l'unico fine di aumentare
 *        la leggibilità del codice. La presenza o meno della direttiva nel codice "core" 
 *        dall'orientamento con cui viene installato il modulo GY-86 
 * 
 */
#define INVERTI_MISURA															-1

/**
 * @brief La seguente direttiva di preprocessore specifica la frequenza di acquisizione del
 *        giroscopio quando non è attivato il "Digital Low-Pass Filter"
 * 
 */
#define FREQUENZA_AGGIORNAMENTO_GIROSCOPIO_DLPF_OFF								8000

/**
 * @brief La seguente direttiva di preprocessore specifica la frequenza di acquisizione del
 *        giroscopio quando è attivato il "Digital Low-Pass Filter"
 * 
 */
#define FREQUENZA_AGGIORNAMENTO_GIROSCOPIO_DLPF_ON								1000

/**
 * @brief Le seguenti quattro direttive di preprocessore specificano il valore, per cui deve
 *        essere diviso il segnale ricavato dalla misurazione, per ottenere il risultato 
 *        con effettivo significato fisico. Ogni direttiva specifica il valore del divisore 
 *        per ogni "Full Scale Range" dell'accelerometro
 * 
 */
#define ACC_LSB_SENSITIVITY_FOR_FSR_2g											16384.0

#define ACC_LSB_SENSITIVITY_FOR_FSR_4g											8192.0

#define ACC_LSB_SENSITIVITY_FOR_FSR_8g											4096.0

#define ACC_LSB_SENSITIVITY_FOR_FSR_16g											2048.0

/**
 * @brief La seguente direttiva di preprocessore è stata creata con l'unico fine di aumentare
 *        la leggibilità del codice. Viene utilizzata nella manipolazione dell'informazione
 *        estrapolata dall'accelerometro
 * 
 */
#define ACCELERAZIONE_GRAVITAA_MISURA_INTERNAZIONALE							9.81


/**
 * @brief Le seguenti quattro direttive di preprocessore specificano il valore, per cui deve
 *        essere diviso il segnale ricavato dalla misurazione, per ottenere il risultato
 *        con effettivo significato fisico. Ogni direttiva specifica il valore del divisore
 *        per ogni "Full Scale Range" del giroscopio
 * 
 */
#define GIRO_LSB_SENSITIVITY_FOR_250_g_s										131.0

#define GIRO_LSB_SENSITIVITY_FOR_500_g_s								      	65.5

#define GIRO_LSB_SENSITIVITY_FOR_1000_g_s										32.8

#define GIRO_LSB_SENSITIVITY_FOR_2000_g_s										16.4

/**
 * @brief La seguenti direttive di preprocessore specificano l'escursione di misura della
 *        velocità angolare massima per ogni valore di "Full Scale Range" del giroscopio. 
 *        Questi valori sono utili per la distinzione del senso di rotazione del dispotivo
 */
#define ESCURSIONE_GIRO_PER_250_g_s												500

#define ESCURSIONE_GIRO_PER_500_g_s												1000

#define ESCURSIONE_GITO_PER_1000_g_s                                            2000

#define ESCURSIONE_GIRO_PER_2000_g_s                                            4000

/**
 * @brief Le seguenti direttive di preprocessore specificano il valore massimo di
 *        velocità angolare misurabile lungo i tre assi, per ogni 
 *        "Full Scale Range" del giroscopio
 * 
 */
#define VALORE_MAX_VEL_ANG_MISURABILE_CON_RUMORE_PER_250_g_s					250.15 

#define VALORE_MAX_VEL_ANG_MISURABILE_CON_RUMORE_PER_500_g_s					500.6 

#define VALORE_MAX_VEL_ANG_MISURABILE_CON_RUMORE_PER_1000_g_s					1000.9

#define VALORE_MAX_VEL_ANG_MISURABILE_CON_RUMORE_PER_2000_g_s					2002 

/**
 * @brief La seguente direttiva di preprocessore è stata creata con il fine di aumentare la 
 *        leggibilità del codice. Specifica il valore di pigreco
 * 
 */
#define PI_GRECO																3.14159265358979323846

/**
 * @brief La seguente direttiva di preprocessore è stata creata con il fine di aumentare la
 *        leggibilità del codice. Specifica il valore in gradi di un giro completo
 * 
 */
#define ANGOLO_360																360

/**
 * @brief La seguente direttiva di preprocessore è stata creata con il fine di aumentare la
 *        leggibilità del codice. Specifica il valore in gradi dell'angolo nullo
 * 
 */
#define ANGOLO_0																0

/**
 * @brief Le seguenti tre direttive di preprocessore specificano i valori di offset, 
 *        registrati dal giroscopio lungo i tre assi, nonostante la completa assenza di
 *        movimento. I valori sottostanti sono stati ricavati tramite manipolazione dei 
 *        dati ottenuti con osservazioni empiriche
 * 
 */
#define MASSIMA_ESCURSIONE_RUMORE_BIANCO_ASSE_X_EMPIRICO						0.320611

#define MASSIMA_ESCURSIONE_RUMORE_BIANCO_ASSE_Y_EMPIRICO						0.290111

#define MASSIMA_ESCURSIONE_RUMORE_BIANCO_ASSE_Z_EMPIRICO						0.2214

/**
 * @brief Le seguenti tre direttive di preprocessore specificano l'amplificazione apportata,
 *        ai risultati dell'integrazione sulla velocità angolare lungo i tre assi, per ottenere
 *        il valore corretto di angolo spaziato. I valori che seguono sono stati determinati 
 *        tramite osservazioni empiriche e calcoli matematici. I valori devono essere 
 *        rideterminati al momento dell'installazione del modulo GY-86 nel sistema e 
 *        a seconda del "Full Scale Range" attuale del giroscopio 
 * 
 * @note  I valori specifici sottostanti sono stati determinati nelle seguenti condizioni: 
 *        FSR : +-250°/s, angolo spaziato : 90° su superficie piana per ogni asse
 */
#define AMPLIFICATORE_EMPIRICO_INTEGRALE_ASSE_X									8.00

#define AMPLIFICATORE_EMPIRICO_INTEGRALE_ASSE_Y									11.7356

#define AMPLIFICATORE_EMPIRICO_INTEGRALE_ASSE_Z									6.95


/*------- Enumerazioni -------*/

/**
 * @brief L'enumerazione specifica le configurazioni possibili per il 
 *        "Digital Low-Pass Filter". Il valore assegnato ad ogni dato, 
 *        rappresenta è quello che deve essere scritto nel registro 
 *        @ref CONFIG per attivare la configurazione desiderata.
 * 
 */
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
 * @brief L'enumerazione specifica le configurazioni possibili per il
 *        "Full Scale Range" dell'accelerometro. Il valore assegnato 
 *        ad ogni dato, è quello che deve essere scritto nel registro
 *        @ref ACCEL_CONFIG per impostare l'FSR desiderato
 * 
 */
typedef enum{
    FULL_SCALE_RANGE_2g  = 0x00,
    FULL_SCALE_RANGE_4g  = 0x08,
    FULL_SCALE_RANGE_8g  = 0x10,
    FULL_SCALE_RANGE_16g = 0x18,

} MPU6050_intervalloMisurazioneAccelerometro;

/**
 * @brief L'enumerazione specifica le configurazioni possibili per il
 *        "Full Scale Range" del giroscopio. Il valore assegnato ad 
 *        ogni dato, è quello che deve essere scritto nel registro 
 *        @ref GYRO_CONFIG per impostare l'FSR desiderato
 * 
 */
typedef enum{
    FULL_SCALE_RANGE_250_gr_sec  = 0x00,
    FULL_SCALE_RANGE_500_gr_sec  = 0x08,
    FULL_SCALE_RANGE_1000_gr_sec = 0x10,
    FULL_SCALE_RANGE_2000_gr_sec = 0x18,

} MPU6050_intervalloMisurazioneGiroscopio;

/**
 * @brief L'enumerazione specifica le modalità di funzionamento del 
 *        dispostivo
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
 * @brief  La struttura dati contiene il dato grezzo ascquisito 
 *        direttamente dai registri dell'accelerometro per ogni asse.
 * 
 */
typedef struct{
    int16_t accelerazioneGrezza_X;
    int16_t accelerazioneGrezza_Y;
    int16_t accelerazioneGrezza_Z;

} MPU6050_datoGrezzoAccelerometro_treAssi;

/**
 * @brief La struttura dati contiene il dato dell'accelerazione 
 *        espresso in [g], per ogni asse cartesiano.
 * 
 */
typedef struct {
    float accelerazioneInG_X;
    float accelerazioneInG_Y;
    float accelerazioneInG_Z;

}  MPU6050_datoInGAccelerometro_treAssi;

/**
 * @brief La struttura dati contiene il dato dell'accelerazione 
 *        espresso in [m/s^2], per ogni asse cartesiano.
 * 
 */
typedef struct{
    float accelerazioneMisuraInternazionale_X;
    float accelerazioneMisuraInternazionale_Y;
    float accelerazioneMisuraInternazionale_Z;

} MPU6050_datoMisuraInternazionaleAccelerometro_treAssi;


/*----- Giroscopio -----*/

/**
 * @brief La struttura dati contiene il dato grezzo acquisito 
 *        direttamente dai registri del giroscopio per ogni asse
 * 
 */
typedef struct{
    uint16_t DatoGrezzoGiroscopio_X;
    uint16_t DatoGrezzoGiroscopio_Y;
    uint16_t DatoGrezzoGiroscopio_Z;
} MPU6050_datoGrezzoGiroscopio_treAssi;

/**
 * @brief La struttura dati contiene due diversi dati per ogni asse.
 *        Il dato dell'attuale posizione angolare e quello  della 
 *        posizione angolare in un istante di tempo precedente. La 
 *        seguente impostazione della struttura è necessaria per il
 *        corretto funzionamento della funzione 
 *        @ref  MPU6050_LetturaDatiGiroscopio__TrasformazioneDati
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
 * @brief La struttura dati contiene il dato della velocità angolare
 *        attorno ai tre assi cartesiani, espressa in [°/s]
 * 
 */
typedef struct{
    float velocitaaAngolare_X;
    float velocitaaAngolare_Y;
    float velocitaaAngolare_Z;

} MPU6050_datoVelocitaaAngolareGiroscopio_treAssi;


/*----- Struttura dati delle diverse misure sui tre assi -----*/

/**
 * @brief Struttura dati agreggata che contiene le possibili 
 *       espressioni dell'accelerazione lungo i tre assi cartesiani
 * 
 */
typedef struct{
   MPU6050_datoGrezzoAccelerometro_treAssi datoGrezzoAcc;
   MPU6050_datoInGAccelerometro_treAssi datoInGAcc;
   MPU6050_datoMisuraInternazionaleAccelerometro_treAssi datoMisIntAcc;

} MPU6050_StrutturaDatiAccelerometro_treAssi;

/**
 * @brief Struttura dati aggregata che contiene le possibili 
 *        espressioni della velocità angolare e della posizione 
 *        angolare lungo i tre assi cartesiani  
 * 
 */
typedef struct{
    MPU6050_datoGrezzoGiroscopio_treAssi datoGrezzoGiro;
    MPU6050_datoPosizioneAngolareGiroscopio_treAssi datoPosAngGiro;
    MPU6050_datoVelocitaaAngolareGiroscopio_treAssi datoVelAngGiro;
} MPU6050_StrutturaDatiGiroscopio_treAssi;

/*------- Strutture dati per assistenza alle funzioni -------*/

/**
 * @brief Struttura dati di supporto alla funzione 
 *        @ref MPU6050_LetturaDatiGiroscopio__TrasformazioneDati
 * 
 */
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
 * @brief Struttura dati aggregata per la gestione del firmware. 
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

/**
 * @brief Puntatore alla struttura dati aggregata di gestione del 
 *        firmware @ref MPU6050_dispositivo_t
 * 
 */
typedef MPU6050_dispositivo_t* MPU6050_disp;
