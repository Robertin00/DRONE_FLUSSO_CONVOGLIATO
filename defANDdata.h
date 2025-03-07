/**********************************************************************
 * VERSIONE DI PROVA
 * Il file contiene le strutture dati e le definizioni usate nel codice di gestione dell'MPU6050
 * Prima stesura : 07/03/2025
 * Autore : Robert Laurentiu Mincu
 * 
 */

#define MPU6050_ADDR     0xD0
#define SMPLRT_DIV_REG   0x19
#define ACCEL_CONFIG_REG 0x1C
#define GYRO_CONFIG_REG  0x1B
#define ACCEL_XOUT_H_REG 0x3B 
#define GYRO_XOUT_H_REG  0x43
#define TEMP_OUT_H_REG   0x41
#define PWR_MGMT_1_REG   0x6B
#define WHO_AM_I_REG     0x75
#define VALORE_RITORNO_WHO_I_AM_REG 104 

/*dati e strutture, DA MODIFICARE*/

typedef struct{
    int16_t accelerazioneGrezzaX = 0;
    int16_t accelerazioneGrezzaY = 0;
    int16_t accelerazioneGrezzaZ = 0;
} accelerazioneGrezzaXYZ;

typedef struct{
    float accelerazioneG_X = 0;
    float accelerazioneG_Y = 0;
    float accelerazioneG_Z = 0;
}accelerazioneG_XYZ;

typedef struct{
    int16_t gyroGrezzaX = 0;
    int16_t gyroGrezzaY = 0;
    int16_t gyroGrezzaZ = 0;
}veloAngGrezzaXYZ;

typedef struct{
    float velAng_g_s_X = 0;
    float velAng_g_s_Y = 0;
    float velAng_g_s_Z = 0;
}veloAng_g_s_XYZ;

typedef struct{
    uint8_t recData1[6];
    uint8_t recData2[6];
}recData;
