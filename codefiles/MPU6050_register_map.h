/*****************************************************************************
 * @file MPU6050_register_map.h
 * @brief Il file contiene le definizioni di macro reppresentanti i principali registri utilizzati per la gestionedel sensore MPU6050 di IvenSense tramite NUCLEO-H745ZI-Q
 * @author Robert Laurentiu Mincu
 * @version 0.1
 * @date 11/04/2025
 *****************************************************************************/

/**
 * @brief DA IMPLEMENTARE, MIGLIORAMENTE DELLA LIBRERIA
 * 		  I registri 13-16 sono utilizzati per i "self-test" dell'accelerometro e del giroscopio, che permettono all'utente di verificare il corretto
 *        funzionamento delle componenti meccaniche ed elettriche del giroscopio e dell'accelerometro 
 *        -tipo : R/W
 *        -i2c_size_byte : 1
 */
#define SELF_TEST_X                                         0x000D 

#define SELF_TEST_Y                                         0x000E

#define SELF_TEST_Z                                         0x000F

#define SELF_TEST_A                                         0x0010

#define SMPLRT_DIV											0x0019

#define CONFIG												0x001A


/**
 * @brief Il registro è usato per innescare il self-test del giroscopio e configurare l'intervallo di misurazione
 *        -Innesco del self-test :  impostare il bit alto causa il self-test : bit7,bit6,bit5 -> (XG_ST, YG_ST, ZG_ST)
 *                                 
 *        -FS_SEL[1:0] : 2-bit(bit4, bit3) 
 *              - (0,0) : +- 250  °/s
 *              - (0,1) : +- 500  °/s
 *              - (1,0) : +- 1000 °/s
 *              - (1,1) : +- 2000 °/s
 *        -tipo : R/W
 *        -i2c_size_byte : 1
 */
#define GYRO_CONFIG                                         0x001B

/**
 * @brief Il registro è usato per innescare il self test dell'accelerometro 
 *        e configurare l'intervallo di misurazione. Configura 
 *        anche il DHPF.
 *        - Innesco del self-test : impostare il bit alto causa
 *          il self test : bit7, bit6, bit5 ->(XA_ST, YA_ST, ZA_ST)
 *                                     
 *        - AFS_SEL[1:0] : 2-bit(bit4, bit3)
 *              -(0,0) : +- 2g
 *              -(0,1) : +- 4g
 *              -(1,0) : +- 8g
 *              -(1,1) : +- 16g 
 *         -tipo : R/W
 *         -i2c_size_byte : 1
 * 
 */
#define ACCEL_CONFIG                                        0x001C

/**
 * @brief Il registro determina quali misure del sensore verrano caricate nel buffer FIFO.
 *        -tipo : R/W
 *        -i2c_size_byte : 1
 */
#define FIFO_EN                                             0x0023

/**
 * @brief Il registro mostra lo stato delle interrupt per ogni sorgente di interrupt. Ogni bit dopo la lettura viene azzerato dopo la lettura.
 * 	      lo stato del bit0 rispecchia la presenza di un nuovo dato acquisito dall'accelerometro o dal giroscopio.
 *
 * 	      -tipo : R
 * 	      -i2c_size_byte : 1
 */
#define INT_STATUS											0x003A

/**
 * @brief Questi registri contengono le più recenti misurazioni dell'accelerometro, la misura per ogni asse e divisa in due registri, HIGH e LOW
 *        dunque il dato finale per ogni asse è composto da 16 bit. Il dato grezzo letto direttamente dai registri deve essere diviso per "LSB sensitivity"
 *        -AFS_SEL = 00 -> /16384
 *        -AFS_SEL = 01 -> /8192
 *        -AFS_SEL = 10 -> /4096
 *        -AFS_SEL = 11 -> /2048
 *        -tipo : R
 *        -i2c_size_byte : 1 per ogni registro 
 * 
 */
#define ACCEL_XOUT_H                                        0x003B

#define ACCEL_XOUT_L                                        0x003C

#define ACCEL_YOUT_H                                        0x003D

#define ACCEL_YOUT_L                                        0x003E

#define ACCEL_ZOUT_H                                        0x003F

#define ACCEL_ZOUT_L                                        0x0040

/**
 * @brief Questi registri contengono le più recenti misurazioni della temperatura
 *        -tipo : R
 *        -i2c_size_byte : 1 per ogni registro 
 * 
 */
#define TEMP_OUT_H                                          0x0041

#define TEMP_OUT_L                                          0x0042

/**
 * @brief Questi registri contengono le più recenti misurazioni di velocità angolare intorno ad un asse, stessa logia dei registri di misurazione
 *        dell'accelerazione. Il dato grezzo letto direttamente dai registri deve essere diviso per "LSB Sensitivity"
 *        -FS_SEL = 00 -> /131
 *        -FS_SEL = 01 -> /65,5
 *        -FS_SEL = 10 -> /32,8
 *        -FS_SEL = 11 -> /16,4
 *        -tipo : R
 *        -i2c_size_byte : 1
 *         
 */
#define GYRO_XOUT_H                                         0x0043

#define GYRO_XOUT_L                                         0x0044

#define GYRO_YOUT_H                                         0x0045

#define GYRO_YOUT_L                                         0x0046

#define GYRO_ZOUT_H                                         0x0047

#define GYRO_ZOUT_L                                         0x0048

/**
 * @brief Il registro consente all'utente di impostare la "power mode" e la sorgente di clock. Inoltre provvede un bit per il
 *        reset dell'intero dispositivo e un bit per disabilitare il sensore di temperatura
 *        bit7 : DEVICE_RESET -> quando impostato ad 1, il sensore si resetta, tutti i registri conterrano i valori di default.
 *                               inoltre il bit torna a 0 una volta che il reset è stato completato.
 *        bit6 : SLEEP        -> quando impostato ad 1, il sensore entra in "sleep-mode"
 *        bit5 : CYCLE        -> quando impostato ad 1 e SLEEP impostato a 0, il sensore alterna tra "sleep-mode" e "waking-up" per prelevare un singolo 
 *                               campione di dati dai sensori attivi con una frequenza determinata dal valore nel registro : decimale 108
 *        bit3 : TEMP_DIS     -> se impostato ad 1, disattiva il sensore di temperatura 
 * bit2, bit1, bit0 : CLSKSEL -> specifica la sorgente id clock del dispositivo 
 *        -tipo : R/W
 *        -i2c_size_byte : 1
 * 
 */
#define PWR_MGMT_1                                          0x006B

/**
 * @brief Il registro è utilizzato per verificare l'identità del dispositivo, contiene l'indirizzo "i2c_slave" e può essere usato per testare
 *        il corretto funzionamento della comunicazione I2C
 * 
 *        Indirizzo I2C: 7bit = 0x68, 8bit = 0xD0
 *        -tipo : R
 *        -i2c_size_byte : 1
 * 
 */
#define WHO_AM_I                                            0x0075
