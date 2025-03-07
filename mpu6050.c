#include <mpu6050.h>
#include <stdio.h>
#include <string.h>
#include <main.h>


/**
 * sicuramente da modificare i registri usando un #define
 */
void MPU6050_Init(void){
    uint8_t check = 0;
    uint8_t data;
    uint8_t dataSR;
    uint8_t dataAC; 
    uint8_t dataGC;

    HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, HAL_MAX_DELAY);
    
    if(check == VALORE_RITORNO_WHO_I_AM_REG){

        data = 0;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, 1000);

        dataSR = 0x07;
        HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &dataSR, 1, 1000);

        dataAC = 0x00;
        HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &dataAC, 1, 1000);

        dataGC = 0x00;
        HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &dataGC, 1, 1000);

    }
    /**
     * IMPLMENTAZIONE CON STRUCT DA PROVARE 
     */
    void MPU6050_LettoreAccelerazione(void){
        
        HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, recData->recData1, 6, 1000);

        accelerazioneGrezzaXYZ->accelerazioneGrezzaX = (int16_t)(recData->recData1[0]<<8 | recData->recData1[1]);
        accelerazioneGrezzaXYZ->accelerazioneGrezzaY = (int16_t)(recData->recData1[2]<<8 | recData->recData1[3]);
        accelerazioneGrezzaXYZ->accelerazioneGrezzaZ = (int16_t)(recData->recData1[4]<<8 | recData->recData1[5]);

        accelerazioneG_XYZ->accelerazioneG_X = accelerazioneGrezzaXYZ->accelerazioneGrezzaX/16384.0;
        accelerazioneG_XYZ->accelerazioneG_Y = accelerazioneGrezzaXYZ->accelerazioneGrezzaY/16384.0;
        accelerazioneG_XYZ->accelerazioneG_Z = accelerazioneGrezzaXYZ->accelerazioneGrezzaZ/16384.0;
    }
    /**
     * IMPLEMENTAZIONE CON STRUCT DA PROVARE
     */
    void MPU6050_LettoreVelAngolare(void){

        HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, recData->recData2, 6, 1000);

        veloAngGrezzaXYZ->gyroGrezzaX = (int16_t)(recData->recData2[0]<<8 | recData->recData2[1]);
        veloAngGrezzaXYZ->gyroGrezzaY = (int16_t)(recData->recData2[2]<<8 | recData->recData2[3]);
        veloAngGrezzaXYZ->gyroGrezzaZ = (int16_t)(recData->recData2[4]<<8 | recData->recData2[5]);

        veloAng_g_s_XYZ->velAng_g_s_X = veloAngGrezzaXYZ->gyroGrezzaX/131.0;
        veloAng_g_s_XYZ->velAng_g_s_Y = veloAngGrezzaXYZ->gyroGrezzaY/131.0;
        veloAng_g_s_XYZ->velAng_g_s_Z = veloAngGrezzaXYZ->gyroGrezzaZ/131.0;
    }
}