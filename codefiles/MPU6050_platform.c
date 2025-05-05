/**
 * @file MPU6050_platform.c
 * @author Robert Laurentiu Mincu
 * @brief Il file contiene le definizioni delle funzioni create appositamente per poter 
 *        interfacciare il sensore a NUCLEO-H745ZI-Q utilizzando STCubeIDE
 * @version 0.1
 * @date 11/04/2025
 */

/*-------DIRETTIVE DI INCLUSIONE E VARIABILI ESTERNE--------*/
#include "MPU6050_platform.h"

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart3;

/*-------INIZIALIZZAZIONE STRUTTURA DI GESTIONE DEL DISPOSITIVO -------*/

void MPU6050_PlatformInit(MPU6050_disp disp){

	disp->i2c_slave_address = MPU6050_DEVICE_ADDR_8BIT;
	disp->perifericaComunicazioneI2c = hi2c1;
	disp->perifericaComunicazioneHost = huart3;
	disp->intAccelerometro =  FULL_SCALE_RANGE_250_gr_sec;
	disp->intGiroscopio = FULL_SCALE_RANGE_2g;
	disp->datoTemperatura = -274;

	disp->datiAccelerometro.datoGrezzoAcc.accelerazioneGrezza_X = 0;
	disp->datiAccelerometro.datoGrezzoAcc.accelerazioneGrezza_Y = 0;
	disp->datiAccelerometro.datoGrezzoAcc.accelerazioneGrezza_Z = 0;

	disp->datiAccelerometro.datoInGAcc.accelerazioneInG_X = 0;
	disp->datiAccelerometro.datoInGAcc.accelerazioneInG_Y = 0;
	disp->datiAccelerometro.datoInGAcc.accelerazioneInG_Z = 0;

	disp->datiAccelerometro.datoMisIntAcc.accelerazioneMisuraInternazionale_X = 0;
	disp->datiAccelerometro.datoMisIntAcc.accelerazioneMisuraInternazionale_Y = 0;
	disp->datiAccelerometro.datoMisIntAcc.accelerazioneMisuraInternazionale_Z = 0;

	disp->datiGiroscopio.datoGrezzoGiro.DatoGrezzoGiroscopio_X = 0;
	disp->datiGiroscopio.datoGrezzoGiro.DatoGrezzoGiroscopio_Y = 0;
	disp->datiGiroscopio.datoGrezzoGiro.DatoGrezzoGiroscopio_Z = 0;

	disp->datiGiroscopio.datoPosAngGiro.posizioneAngolare_X = 0;
	disp->datiGiroscopio.datoPosAngGiro.vecchia_posizioneAngolare_X_val_integrale = 0;
	disp->datiGiroscopio.datoPosAngGiro.posizioneAngolare_Y = 0;
	disp->datiGiroscopio.datoPosAngGiro.vecchia_posizioneAngolare_Y_val_integrale = 0;
	disp->datiGiroscopio.datoPosAngGiro.posizioneAngolare_Z = 0;
	disp->datiGiroscopio.datoPosAngGiro.vecchia_posizioneAngolare_Z_val_integrale = 0;

	disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_X = 0;
	disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_Y = 0;
	disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_Z = 0;

	disp->modFunzionamento = -1;

}
/*--------COMUNICAZIONI-------*/

HAL_StatusTypeDef MPU6050_VerificaComunicazioni(MPU6050_disp disp){

	uint32_t inizioTimer;

	uint32_t erroreComunicazioneI2C = HAL_I2C_ERROR_NONE;

	HAL_I2C_StateTypeDef status = HAL_I2C_STATE_RESET;

	HAL_StatusTypeDef status_ComUART;
	HAL_StatusTypeDef statoComunicazioneDispositivo = HAL_OK;

	inizioTimer = HAL_GetTick();

	while(status != HAL_I2C_STATE_READY){

		status = HAL_I2C_GetState(&(disp->perifericaComunicazioneI2c));

		if((HAL_GetTick()-inizioTimer) >= STANDARD_TIMEOUT_MILLISEC){

			erroreComunicazioneI2C = HAL_I2C_GetError(&(disp->perifericaComunicazioneI2c));

char msg_errPer_MPU6050_VerificaComunicazioni[STANDARD_ERROR_MESSAGE_LENGTH];

sprintf(msg_errPer_MPU6050_VerificaComunicazioni,"MPU6050_VerificaComunicazione ERRORE::: errore periferica di comunicazione:::timeout:::%d\r\n",erroreComunicazioneI2C);

status_ComUART = HAL_UART_Transmit(&(disp->perifericaComunicazioneHost), 
					(uint8_t*)msg_errPer_MPU6050_VerificaComunicazioni, 
					strlen(msg_errPer_MPU6050_VerificaComunicazioni),
					 HAL_MAX_DELAY);
			break;

		}
	}

	if(status == HAL_I2C_STATE_READY){

		statoComunicazioneDispositivo = HAL_I2C_IsDeviceReady( &(disp->perifericaComunicazioneI2c), disp->i2c_slave_address, NUMERO_TENTATIVI_STANDARD, STANDARD_TIMEOUT_MILLISEC);

		if(statoComunicazioneDispositivo != HAL_OK){

char msg_errCom_MPU6050_VerificaComunicazione[STANDARD_ERROR_MESSAGE_LENGTH];
sprintf(msg_errCom_MPU6050_VerificaComunicazione, "MPUU6050_VerificaComunicazione ERRORE:::errore nella comunicaizone con il dispositivo\r\n");
status_ComUART = HAL_UART_Transmit( &(disp->perifericaComunicazioneHost), (uint8_t*)msg_errCom_MPU6050_VerificaComunicazione, strlen(msg_errCom_MPU6050_VerificaComunicazione), HAL_MAX_DELAY);

			return HAL_ERROR;

		}else{

			return statoComunicazioneDispositivo;
		}

	}

	return HAL_ERROR;
}

/*-------- FUNZIONI DI GESTIONE DELLE IMPOSTAZIONI DEL DISPOSITIVO  --------*/

HAL_StatusTypeDef MPU6050_ImpostaModFunzionamento(MPU6050_disp disp, MPU6050_modFunzionamento modFunzionamento){

	HAL_StatusTypeDef status, status_ComUART;

	if(modFunzionamento > SOFT_RESET){

		char msg_errVal_MPU6050_ImpostaModFunzionamento[STANDARD_ERROR_MESSAGE_LENGTH];
		sprintf(msg_errVal_MPU6050_ImpostaModFunzionamento, "MPU6050_ImpostaModFunzionamento ERRORE ::: modalità di funzionamento errata!");
		status_ComUART = HAL_UART_Transmit(&(disp->perifericaComunicazioneHost), (uint8_t*)msg_errVal_MPU6050_ImpostaModFunzionamento, strlen(msg_errVal_MPU6050_ImpostaModFunzionamento), HAL_MAX_DELAY);

	}else{

		switch(modFunzionamento){

			case ACQUISIZIONE_CONTINUA :

				uint8_t valore_acquisizione_continua = MPU6050_MOD_ACQUISIZIONE_CONTINUA;

				status = HAL_I2C_Mem_Write(&(disp->perifericaComunicazioneI2c), disp->i2c_slave_address, PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &valore_acquisizione_continua, MEM_SIZE_1_BYTE, STANDARD_TIMEOUT_MILLISEC);

				disp->modFunzionamento = modFunzionamento;

				if(status != HAL_OK){

					char msg_errScr_MPU6050_ImpostaModFunzionamento[STANDARD_ERROR_MESSAGE_LENGTH];
					sprintf(msg_errScr_MPU6050_ImpostaModFunzionamento, "MPU6050_ImpostaModFunzionamento ERRORE ::: errore nella scrittura : %d",status);
					status_ComUART = HAL_UART_Transmit(&(disp->perifericaComunicazioneHost), (uint8_t*)msg_errScr_MPU6050_ImpostaModFunzionamento, strlen(msg_errScr_MPU6050_ImpostaModFunzionamento), HAL_MAX_DELAY);

				}

				return status;

				break;

			case SLEEP :

				uint8_t valore_sleep = MPU6050_MOD_SLEEP;

				status = HAL_I2C_Mem_Write(&(disp->perifericaComunicazioneI2c), disp->i2c_slave_address, PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &valore_sleep, MEM_SIZE_1_BYTE, STANDARD_TIMEOUT_MILLISEC);

				disp->modFunzionamento = modFunzionamento;

				if(status != HAL_OK){

					char msg_errScr_MPU6050_ImpostaModFunzionamento[STANDARD_ERROR_MESSAGE_LENGTH];
					sprintf(msg_errScr_MPU6050_ImpostaModFunzionamento, "MPU6050_ImpostaModFunzionamento ERRORE ::: errore nella scrittura : %d", status);
					status_ComUART = HAL_UART_Transmit(&(disp->perifericaComunicazioneHost), (uint8_t*)msg_errScr_MPU6050_ImpostaModFunzionamento, strlen(msg_errScr_MPU6050_ImpostaModFunzionamento), HAL_MAX_DELAY);

				}

				return status;

				break;

			case CYCLE :

				uint8_t valore_cycle = MPU6050_MOD_CYCLE;

				status = HAL_I2C_Mem_Write(&(disp->perifericaComunicazioneI2c), disp->i2c_slave_address, PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &valore_cycle, MEM_SIZE_1_BYTE, STANDARD_TIMEOUT_MILLISEC);

				disp->modFunzionamento = modFunzionamento;

				if(status != HAL_OK){

					char msg_errScr_MPU6050_ImpostaModFunzionamento[STANDARD_ERROR_MESSAGE_LENGTH];
					sprintf(msg_errScr_MPU6050_ImpostaModFunzionamento, "MPU6050_ImpostaModFunzionamento ERRORE ::: errore nella scrittura : %d", status);
					status_ComUART = HAL_UART_Transmit(&(disp->perifericaComunicazioneHost), (uint8_t*)msg_errScr_MPU6050_ImpostaModFunzionamento, strlen(msg_errScr_MPU6050_ImpostaModFunzionamento), HAL_MAX_DELAY);

				}

				return status;

				break;

			case SOFT_RESET :

				uint8_t valore_soft_reset = RESET_MPU6050;

				status = HAL_I2C_Mem_Write(&(disp->perifericaComunicazioneI2c), disp->i2c_slave_address, PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &valore_soft_reset, MEM_SIZE_1_BYTE, STANDARD_TIMEOUT_MILLISEC);

				if(status != HAL_OK){

					char msg_errScr_MPU6050_ImpostaModFunzionamento[STANDARD_ERROR_MESSAGE_LENGTH];
					sprintf(msg_errScr_MPU6050_ImpostaModFunzionamento, "MPU6050_ImpostaModFunzionamento ERRORE ::: errore nella scrittura : %d", status);
					status_ComUART = HAL_UART_Transmit(&(disp->perifericaComunicazioneHost), (uint8_t*)msg_errScr_MPU6050_ImpostaModFunzionamento, strlen(msg_errScr_MPU6050_ImpostaModFunzionamento), HAL_MAX_DELAY);
				}

				HAL_Delay(TEMPO_ATTESA_RESET_MPU6050_MILLISECONDI);

				status = MPU6050_ImpostaModFunzionamento(disp, ACQUISIZIONE_CONTINUA);

				return status;

				break;

			default :

				char msg_errSwitchCase_MPU6050_IMF[STANDARD_ERROR_MESSAGE_LENGTH];
				sprintf(msg_errSwitchCase_MPU6050_IMF, "MPU6050_ImpostaModFunzionamento ERRORE ::: qualcosa è andato storto : %d",status);
				status_ComUART = HAL_UART_Transmit(&(disp->perifericaComunicazioneHost), (uint8_t*)msg_errSwitchCase_MPU6050_IMF, strlen(msg_errSwitchCase_MPU6050_IMF), HAL_MAX_DELAY);

				return status;

				break;

		}

	}

}


HAL_StatusTypeDef MPU6050_ImpostaDLPF(MPU6050_disp disp, MPU6050_valoreDLPF valore_DLPF){

		uint8_t pdata = valore_DLPF;

		HAL_StatusTypeDef status, status_ComUART;

		if(pdata <= DLPF_CFG_6){

		   status = HAL_I2C_Mem_Write(&(disp->perifericaComunicazioneI2c),disp->i2c_slave_address,  CONFIG, I2C_MEMADD_SIZE_8BIT, &pdata, MEM_SIZE_1_BYTE, STANDARD_TIMEOUT_MILLISEC);

		   if(status != HAL_OK){

			 char msg_errCom_MPU6050_ImpostaDLPF[STANDARD_ERROR_MESSAGE_LENGTH];
			 sprintf(msg_errCom_MPU6050_ImpostaDLPF,"\n\rMPU6050_ImpostaDLPF ERRORE:::errore di comunicazione\r\n");
			 status_ComUART = HAL_UART_Transmit(&(disp->perifericaComunicazioneHost), (uint8_t*)msg_errCom_MPU6050_ImpostaDLPF, strlen(msg_errCom_MPU6050_ImpostaDLPF), HAL_MAX_DELAY);

			 return status;

		   }else{

			 return status;
		   }

		}else{

		status = HAL_ERROR;

		char msg_errVal_MPU6050_ImpostaDLPF[STANDARD_ERROR_MESSAGE_LENGTH];
		sprintf(msg_errVal_MPU6050_ImpostaDLPF, "\n\rMPU6050_ImpostaDLPF ERRORE:::valore inserito DLPF non corretto\r\n");
		status_ComUART = HAL_UART_Transmit(&(disp->perifericaComunicazioneHost), (uint8_t*)msg_errVal_MPU6050_ImpostaDLPF, strlen(msg_errVal_MPU6050_ImpostaDLPF), HAL_MAX_DELAY);

		return status;

		}
}

HAL_StatusTypeDef MPU6050_OttieniDLPF(MPU6050_disp disp, uint8_t* buffer_valore_DLPF){

	HAL_StatusTypeDef status, status_ComUART;

	status = HAL_I2C_Mem_Read(&(disp->perifericaComunicazioneI2c), disp->i2c_slave_address, CONFIG, I2C_MEMADD_SIZE_8BIT, buffer_valore_DLPF, MEM_SIZE_1_BYTE, STANDARD_TIMEOUT_MILLISEC);

	if(status != HAL_OK){

		char msg_errLett_MPU6050_OttieniDLPF[STANDARD_ERROR_MESSAGE_LENGTH];
		sprintf(msg_errLett_MPU6050_OttieniDLPF, "\r\nMPU6050_OttieniDLPF ERRORE::: errore nella lettura : %d", status);
		status_ComUART = HAL_UART_Transmit(&(disp->perifericaComunicazioneHost), (uint8_t*)msg_errLett_MPU6050_OttieniDLPF, strlen(msg_errLett_MPU6050_OttieniDLPF), HAL_MAX_DELAY);

		return status;

	}

	return status;
}

HAL_StatusTypeDef MPU6050_ImpostaDivisoreFrequenzaCampionamento(MPU6050_disp disp, uint8_t divisore){

		HAL_StatusTypeDef status, status_ComUART;

		if(divisore <= VALORE_MASSIMO_DIVISORE_REQUENZA_ACQUISIZIONE_GIRO	){

		   status = HAL_I2C_Mem_Write(&(disp->perifericaComunicazioneI2c), disp->i2c_slave_address, SMPLRT_DIV, I2C_MEMADD_SIZE_8BIT, &divisore, MEM_SIZE_1_BYTE, STANDARD_TIMEOUT_MILLISEC);

		 	 if(status != HAL_OK){

		 		 char msg_errScr_MPU6050_ImpostaFrequenzaCampionamento[STANDARD_ERROR_MESSAGE_LENGTH];
		 		 sprintf(msg_errScr_MPU6050_ImpostaFrequenzaCampionamento,"\r\nMPU6050_ImpostaDivisoreFrequenzaCampionamento ERRORE:::errore nella scrittura:::%d\r\n",status);
		 		 status_ComUART = HAL_UART_Transmit(&(disp->perifericaComunicazioneHost), (uint8_t*)msg_errScr_MPU6050_ImpostaFrequenzaCampionamento, strlen(msg_errScr_MPU6050_ImpostaFrequenzaCampionamento), HAL_MAX_DELAY);

		 		 return status;

		 	 }else{

		 		 return status;

		 	 }
		}else{

			char msg_errVal_MPU6050_ImpostaDFC[STANDARD_ERROR_MESSAGE_LENGTH];
			sprintf(msg_errVal_MPU6050_ImpostaDFC, "\r\nMPU6050_ImpostaDivisoreFrequenzaCampionamento ERRORE::: valore inserito DFC non corretto [0:255]");
			status_ComUART = HAL_UART_Transmit(&(disp->perifericaComunicazioneHost), (uint8_t*)msg_errVal_MPU6050_ImpostaDFC, strlen(msg_errVal_MPU6050_ImpostaDFC), HAL_MAX_DELAY);

			return HAL_ERROR;
		}
}

HAL_StatusTypeDef MPU6050_OttieniDivisoreFrequenzaCampionamento(MPU6050_disp disp, uint8_t* buffer_valore_SMPLR){

	HAL_StatusTypeDef status, status_ComUART;
	
	status = HAL_I2C_Mem_Read(&(disp->perifericaComunicazioneI2c), disp->i2c_slave_address, SMPLRT_DIV,  I2C_MEMADD_SIZE_8BIT, buffer_valore_SMPLR, MEM_SIZE_1_BYTE, STANDARD_TIMEOUT_MILLISEC);

	if(status != HAL_OK){

		char msg_errLett_MPU6050_OttieniD_F_C[STANDARD_ERROR_MESSAGE_LENGTH];
		sprintf(msg_errLett_MPU6050_OttieniD_F_C, "\r\nMPU6050_OttieniDivisoreFrequenzaCampionamento ERRORE::: errore nella lettura : %d", status);
		status_ComUART = HAL_UART_Transmit(&(disp->perifericaComunicazioneHost), (uint8_t*)msg_errLett_MPU6050_OttieniD_F_C, strlen(msg_errLett_MPU6050_OttieniD_F_C), HAL_MAX_DELAY);

		return status;
	}

	return status;
}

HAL_StatusTypeDef MPU6050_ImpostaFullScaleRangeGiroscopio(MPU6050_disp disp, MPU6050_intervalloMisurazioneGiroscopio valore_FSR_giroscopio){

	uint8_t valore_escursione_giroscopio = valore_FSR_giroscopio;
	HAL_StatusTypeDef status_ComUART;

	if(valore_escursione_giroscopio <= FULL_SCALE_RANGE_2000_gr_sec){

	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&(disp->perifericaComunicazioneI2c),disp->i2c_slave_address, GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT, &valore_escursione_giroscopio, MEM_SIZE_1_BYTE, STANDARD_TIMEOUT_MILLISEC);

		if(status != HAL_OK){

			char msg_errScr_MPU6050_ImpostaFSLGiroscopio[STANDARD_ERROR_MESSAGE_LENGTH];
			sprintf(msg_errScr_MPU6050_ImpostaFSLGiroscopio,"\r\nMPU6050_ImpostaFSLGiroscopio ERRORE:::errore nella scrittura:::%d\r\n",status);
			status_ComUART = HAL_UART_Transmit(&(disp->perifericaComunicazioneHost), (uint8_t*)msg_errScr_MPU6050_ImpostaFSLGiroscopio, strlen(msg_errScr_MPU6050_ImpostaFSLGiroscopio), HAL_MAX_DELAY);

			return status;

		}else{

			return status;
		}

	}else{

		char msg_errVal_MPU6050_ImpostaFSLGiroscopio[STANDARD_ERROR_MESSAGE_LENGTH];
		sprintf(msg_errVal_MPU6050_ImpostaFSLGiroscopio, "\r\nMPU6050_ImpostaFSLGiroscopio ERRORE:::valore inserito FSR Giroscopio non corretto");
		status_ComUART = HAL_UART_Transmit(&(disp->perifericaComunicazioneHost), (uint8_t*)msg_errVal_MPU6050_ImpostaFSLGiroscopio, strlen(msg_errVal_MPU6050_ImpostaFSLGiroscopio), HAL_MAX_DELAY);

		return HAL_ERROR;

	}
}

HAL_StatusTypeDef MPU6050_OttieniFullScaleRangeGiroscopio(MPU6050_disp disp, uint8_t* buffer){

	HAL_StatusTypeDef status_ComUART;

	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&(disp->perifericaComunicazioneI2c),disp->i2c_slave_address,GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT, buffer, MEM_SIZE_1_BYTE, STANDARD_TIMEOUT_MILLISEC);

	if(status != HAL_OK){

		char msg_errLett_MPU6050_OttieniFSLGiroscopio[STANDARD_ERROR_MESSAGE_LENGTH];
		sprintf(msg_errLett_MPU6050_OttieniFSLGiroscopio,"\r\nMPU6050_OttieniFSRGiroscopio ERRORE::: errore nella lettura\r\n");
		status_ComUART = HAL_UART_Transmit(&(disp->perifericaComunicazioneHost), (uint8_t*)msg_errLett_MPU6050_OttieniFSLGiroscopio, strlen(msg_errLett_MPU6050_OttieniFSLGiroscopio), HAL_MAX_DELAY);

		return status;

	}else{

		return status;

	}
}

HAL_StatusTypeDef MPU6050_ImpostaFullScaleRangeAccelerometro(MPU6050_disp disp, MPU6050_intervalloMisurazioneAccelerometro valore_FSR_accelerometro){

	uint8_t valore_escursione_accelerometro = valore_FSR_accelerometro;
	HAL_StatusTypeDef status_ComUART;

	if(valore_escursione_accelerometro <= FULL_SCALE_RANGE_16g){

	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&(disp->perifericaComunicazioneI2c), disp->i2c_slave_address, ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, &valore_escursione_accelerometro, MEM_SIZE_1_BYTE, STANDARD_TIMEOUT_MILLISEC);

		if(status != HAL_OK){

			char msg_errScr_MPU6050_ImpostaFSLAccelerometro[STANDARD_ERROR_MESSAGE_LENGTH];
			sprintf(msg_errScr_MPU6050_ImpostaFSLAccelerometro,"\r\nMPU6050_ImpostaFSLAccelerometro ERRORE::: erroe nella scrittura:::%d\r\n",status);
			status_ComUART = HAL_UART_Transmit(&(disp->perifericaComunicazioneHost), (uint8_t*)msg_errScr_MPU6050_ImpostaFSLAccelerometro, strlen(msg_errScr_MPU6050_ImpostaFSLAccelerometro), HAL_MAX_DELAY);

			return status;

		}else{

			return status;
		}

	}else{

		char msg_errVal_MPU6050_ImpostaFSLAccelerometro[STANDARD_ERROR_MESSAGE_LENGTH];
		sprintf(msg_errVal_MPU6050_ImpostaFSLAccelerometro, "\r\nMPU6050_ImpostaFLSAccelerometro ERRORE::: valore FSR accelerometro non corretto\r\n");
		status_ComUART = HAL_UART_Transmit(&(disp->perifericaComunicazioneHost), (uint8_t*)msg_errVal_MPU6050_ImpostaFSLAccelerometro, strlen(msg_errVal_MPU6050_ImpostaFSLAccelerometro), HAL_MAX_DELAY);

		return HAL_ERROR;
	}
}

HAL_StatusTypeDef MPU6050_OttieniFullScaleRangeAccelerometro(MPU6050_disp disp, uint8_t* buffer){

	HAL_StatusTypeDef status_ComUART;
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&(disp->perifericaComunicazioneI2c), disp->i2c_slave_address, ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, buffer,  MEM_SIZE_1_BYTE, STANDARD_TIMEOUT_MILLISEC);

	if(status != HAL_OK){

			char msg_errLett_MPU6050_OttieniFSLAccelerometro[STANDARD_ERROR_MESSAGE_LENGTH];
			sprintf(msg_errLett_MPU6050_OttieniFSLAccelerometro,"\r\nMPU6050_ImpostaDivisoreFrequenzaCampionamento ERRORE::: errore nella lettura\r\n");
			status_ComUART = HAL_UART_Transmit(&(disp->perifericaComunicazioneHost), (uint8_t*)msg_errLett_MPU6050_OttieniFSLAccelerometro, strlen(msg_errLett_MPU6050_OttieniFSLAccelerometro), HAL_MAX_DELAY);

			return status;

	}else{
		    return status;

	}
}

HAL_StatusTypeDef MPU6050_InizializzazioneDispositivo(MPU6050_disp disp, MPU6050_modFunzionamento modFunzionamento, MPU6050_valoreDLPF valore_DLPF, uint8_t divisore, MPU6050_intervalloMisurazioneGiroscopio valore_FSR_giroscopio, MPU6050_intervalloMisurazioneAccelerometro valore_FSR_accelerometro){

	HAL_StatusTypeDef status;

	MPU6050_PlatformInit(disp);

	status = MPU6050_ImpostaModFunzionamento(disp, modFunzionamento);

	if(status == HAL_OK){

		status = MPU6050_ImpostaDLPF(disp, valore_DLPF);

		if(status == HAL_OK){

			status = MPU6050_ImpostaDivisoreFrequenzaCampionamento(disp, divisore);

			if(status == HAL_OK){

				status = MPU6050_ImpostaFullScaleRangeGiroscopio(disp, valore_FSR_giroscopio);

				if(status == HAL_OK){

					status = MPU6050_ImpostaFullScaleRangeAccelerometro(disp, valore_FSR_accelerometro);
				}
			}
		}
	}

	return status;
}

/*------- FUNZIONI DI TIMING -------*/

bool MPU6050_AspettaMisure(MPU6050_disp disp){

	uint32_t inizioTimer;
	uint8_t statoDati = RESET_MPU6050;
	uint8_t buffer;
	HAL_StatusTypeDef status_ComUART,status;
	bool statoLogico = false;
	inizioTimer = HAL_GetTick();

	while(!(statoDati & BIT_VERIFICA_DATI_ACQUISITI)){

		status = HAL_I2C_Mem_Read(&(disp->perifericaComunicazioneI2c), disp->i2c_slave_address, INT_STATUS, I2C_MEMADD_SIZE_8BIT, &buffer, MEM_SIZE_1_BYTE, STANDARD_TIMEOUT_MILLISEC);
		statoDati = buffer;
		statoLogico = true;

		if((HAL_GetTick()-inizioTimer) >= TIMEOUT_CORTO){

			char msg_errTempo_MPU6050_A_M[STANDARD_ERROR_MESSAGE_LENGTH];
			sprintf(msg_errTempo_MPU6050_A_M, "\r\nMPU6050_AspettaMisure ERRORE::: timeout o errore nella lettura : %d\r\n",status);
			status_ComUART = HAL_UART_Transmit(&(disp->perifericaComunicazioneHost), (uint8_t*)msg_errTempo_MPU6050_A_M, strlen(msg_errTempo_MPU6050_A_M), HAL_MAX_DELAY);
			statoLogico = false;
			break;
		}
	}

	return statoLogico;
}

/*------- FUNZIONI DI LETTURA E MANIPOLAZIONE DEI DATI -------*/

HAL_StatusTypeDef MPU6050_LetturaDatoGrezzoAccelerometro__TrasformazioneDati(MPU6050_disp disp){

	uint8_t bufferDatiGrezziAccelerometro[AMPIEZZA_BUFFER_DATI_ACC_GIRO_BYTE];
	HAL_StatusTypeDef status, status_ComUART, status_OttieniFSRAcc;
	uint8_t fullScaleRangeAccelerometroAttuale;

	status = HAL_I2C_Mem_Read(&(disp->perifericaComunicazioneI2c), disp->i2c_slave_address, ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, bufferDatiGrezziAccelerometro, MEM_SIZE_6_BYTE, STANDARD_TIMEOUT_MILLISEC);

	if(status != HAL_OK){

		char msg_errLett_MPU6050_LetturaDatoGrezzoAccelerazione[STANDARD_ERROR_MESSAGE_LENGTH];
		sprintf(msg_errLett_MPU6050_LetturaDatoGrezzoAccelerazione, "\r\nMPU6050_LetturaDatoGrezzoAccelerazione ERRORE::: errore nella lettura : %d\r\n",status);
		status_ComUART = HAL_UART_Transmit(&(disp->perifericaComunicazioneHost), (uint8_t*)msg_errLett_MPU6050_LetturaDatoGrezzoAccelerazione, strlen(msg_errLett_MPU6050_LetturaDatoGrezzoAccelerazione), HAL_MAX_DELAY);

		return status;

	}else{
		
		disp->datiAccelerometro.datoGrezzoAcc.accelerazioneGrezza_X = (int16_t)(bufferDatiGrezziAccelerometro[0]<<8 | bufferDatiGrezziAccelerometro[1]);
		disp->datiAccelerometro.datoGrezzoAcc.accelerazioneGrezza_Y = (int16_t)(bufferDatiGrezziAccelerometro[2]<<8 | bufferDatiGrezziAccelerometro[3]);
		disp->datiAccelerometro.datoGrezzoAcc.accelerazioneGrezza_Z = (int16_t)(bufferDatiGrezziAccelerometro[4]<<8 | bufferDatiGrezziAccelerometro[5]);

		status_OttieniFSRAcc = MPU6050_OttieniFullScaleRangeAccelerometro(disp, &fullScaleRangeAccelerometroAttuale);

		switch(fullScaleRangeAccelerometroAttuale){

			case FULL_SCALE_RANGE_2g :

					disp->datiAccelerometro.datoInGAcc.accelerazioneInG_X = ((disp->datiAccelerometro.datoGrezzoAcc.accelerazioneGrezza_X)/ACC_LSB_SENSITIVITY_FOR_FSR_2g)*(INVERTI_MISURA);
					disp->datiAccelerometro.datoInGAcc.accelerazioneInG_Y = ((disp->datiAccelerometro.datoGrezzoAcc.accelerazioneGrezza_Y)/ACC_LSB_SENSITIVITY_FOR_FSR_2g)*(INVERTI_MISURA);
					disp->datiAccelerometro.datoInGAcc.accelerazioneInG_Z = ((disp->datiAccelerometro.datoGrezzoAcc.accelerazioneGrezza_Z)/ACC_LSB_SENSITIVITY_FOR_FSR_2g)*(INVERTI_MISURA);


					disp->datiAccelerometro.datoMisIntAcc.accelerazioneMisuraInternazionale_X = (disp->datiAccelerometro.datoInGAcc.accelerazioneInG_X)*ACCELERAZIONE_GRAVITAA_MISURA_INTERNAZIONALE;
					disp->datiAccelerometro.datoMisIntAcc.accelerazioneMisuraInternazionale_Y = (disp->datiAccelerometro.datoInGAcc.accelerazioneInG_Y)*ACCELERAZIONE_GRAVITAA_MISURA_INTERNAZIONALE;
					disp->datiAccelerometro.datoMisIntAcc.accelerazioneMisuraInternazionale_Z = (disp->datiAccelerometro.datoInGAcc.accelerazioneInG_Z)*ACCELERAZIONE_GRAVITAA_MISURA_INTERNAZIONALE;

					break;

			case FULL_SCALE_RANGE_4g :

					disp->datiAccelerometro.datoInGAcc.accelerazioneInG_X = ((disp->datiAccelerometro.datoGrezzoAcc.accelerazioneGrezza_X)/ACC_LSB_SENSITIVITY_FOR_FSR_4g)*(INVERTI_MISURA);
					disp->datiAccelerometro.datoInGAcc.accelerazioneInG_Y = ((disp->datiAccelerometro.datoGrezzoAcc.accelerazioneGrezza_Y)/ACC_LSB_SENSITIVITY_FOR_FSR_4g)*(INVERTI_MISURA);
					disp->datiAccelerometro.datoInGAcc.accelerazioneInG_Z = ((disp->datiAccelerometro.datoGrezzoAcc.accelerazioneGrezza_Z)/ACC_LSB_SENSITIVITY_FOR_FSR_4g)*(INVERTI_MISURA);

					

					disp->datiAccelerometro.datoMisIntAcc.accelerazioneMisuraInternazionale_X = (disp->datiAccelerometro.datoInGAcc.accelerazioneInG_X)*ACCELERAZIONE_GRAVITAA_MISURA_INTERNAZIONALE;
					disp->datiAccelerometro.datoMisIntAcc.accelerazioneMisuraInternazionale_Y = (disp->datiAccelerometro.datoInGAcc.accelerazioneInG_Y)*ACCELERAZIONE_GRAVITAA_MISURA_INTERNAZIONALE;
					disp->datiAccelerometro.datoMisIntAcc.accelerazioneMisuraInternazionale_Z = (disp->datiAccelerometro.datoInGAcc.accelerazioneInG_Z)*ACCELERAZIONE_GRAVITAA_MISURA_INTERNAZIONALE;

					break;

			case FULL_SCALE_RANGE_8g :

					disp->datiAccelerometro.datoInGAcc.accelerazioneInG_X = ((disp->datiAccelerometro.datoGrezzoAcc.accelerazioneGrezza_X)/ACC_LSB_SENSITIVITY_FOR_FSR_8g)*(INVERTI_MISURA);
					disp->datiAccelerometro.datoInGAcc.accelerazioneInG_Y = ((disp->datiAccelerometro.datoGrezzoAcc.accelerazioneGrezza_Y)/ACC_LSB_SENSITIVITY_FOR_FSR_8g)*(INVERTI_MISURA);
					disp->datiAccelerometro.datoInGAcc.accelerazioneInG_Z = ((disp->datiAccelerometro.datoGrezzoAcc.accelerazioneGrezza_Z)/ACC_LSB_SENSITIVITY_FOR_FSR_8g)*(INVERTI_MISURA);

					

					disp->datiAccelerometro.datoMisIntAcc.accelerazioneMisuraInternazionale_X = (disp->datiAccelerometro.datoInGAcc.accelerazioneInG_X)*ACCELERAZIONE_GRAVITAA_MISURA_INTERNAZIONALE;
					disp->datiAccelerometro.datoMisIntAcc.accelerazioneMisuraInternazionale_Y = (disp->datiAccelerometro.datoInGAcc.accelerazioneInG_Y)*ACCELERAZIONE_GRAVITAA_MISURA_INTERNAZIONALE;
					disp->datiAccelerometro.datoMisIntAcc.accelerazioneMisuraInternazionale_Z = (disp->datiAccelerometro.datoInGAcc.accelerazioneInG_Z)*ACCELERAZIONE_GRAVITAA_MISURA_INTERNAZIONALE;

					break;

			case FULL_SCALE_RANGE_16g :

					disp->datiAccelerometro.datoInGAcc.accelerazioneInG_X = ((disp->datiAccelerometro.datoGrezzoAcc.accelerazioneGrezza_X)/ACC_LSB_SENSITIVITY_FOR_FSR_16g)*(INVERTI_MISURA);
					disp->datiAccelerometro.datoInGAcc.accelerazioneInG_Y = ((disp->datiAccelerometro.datoGrezzoAcc.accelerazioneGrezza_Y)/ACC_LSB_SENSITIVITY_FOR_FSR_16g)*(INVERTI_MISURA);
					disp->datiAccelerometro.datoInGAcc.accelerazioneInG_Z = ((disp->datiAccelerometro.datoGrezzoAcc.accelerazioneGrezza_Z)/ACC_LSB_SENSITIVITY_FOR_FSR_16g)*(INVERTI_MISURA);

					

					disp->datiAccelerometro.datoMisIntAcc.accelerazioneMisuraInternazionale_X = (disp->datiAccelerometro.datoInGAcc.accelerazioneInG_X)*ACCELERAZIONE_GRAVITAA_MISURA_INTERNAZIONALE;
					disp->datiAccelerometro.datoMisIntAcc.accelerazioneMisuraInternazionale_Y = (disp->datiAccelerometro.datoInGAcc.accelerazioneInG_Y)*ACCELERAZIONE_GRAVITAA_MISURA_INTERNAZIONALE;
					disp->datiAccelerometro.datoMisIntAcc.accelerazioneMisuraInternazionale_Z = (disp->datiAccelerometro.datoInGAcc.accelerazioneInG_Z)*ACCELERAZIONE_GRAVITAA_MISURA_INTERNAZIONALE;

					break;

			default :

					char msg_errSwitchCaseFSRAcc[STANDARD_ERROR_MESSAGE_LENGTH];
					sprintf(msg_errSwitchCaseFSRAcc, "\r\nMPU6050_LetturaDatoGrezzoAccelerometro ERRORE::: errore del valore o nella luttura del full scale range accelerometro : %d\r\n",status_OttieniFSRAcc);
					status_ComUART = HAL_UART_Transmit(&(disp->perifericaComunicazioneHost), (uint8_t*)msg_errSwitchCaseFSRAcc, strlen(msg_errSwitchCaseFSRAcc), HAL_MAX_DELAY);

					break;

			}

		return status;
	}
}

HAL_StatusTypeDef MPU6050_LetturaDatiGiroscopio__TrasformazioneDati(MPU6050_disp disp){

	HAL_StatusTypeDef status = HAL_OK, status_ODFC, status_ODLPF,status_vecchio_dato, status_nuovo_dato;

	uint8_t valore_DFC, valore_DLPF;

	float frequenza_Di_Campionamento_Hz;
	float intervallo_Tempo_Discreto_sec;

	assistenza_MPU6050_PosizioneAngolareDispositivo controllore_dato;

	status_ODFC = MPU6050_OttieniDivisoreFrequenzaCampionamento(disp, &valore_DFC);
	status_ODLPF = MPU6050_OttieniDLPF(disp, &valore_DLPF);

	if(status_ODFC != HAL_OK && status_ODLPF != HAL_OK){

		status = HAL_ERROR;
		return status;

	}else{

		if(valore_DLPF > DLPF_CFG_0 && valore_DLPF <= DLPF_CFG_6){

			frequenza_Di_Campionamento_Hz = (FREQUENZA_AGGIORNAMENTO_GIROSCOPIO_DLPF_ON/(1+valore_DFC));

		}else{

			frequenza_Di_Campionamento_Hz = (FREQUENZA_AGGIORNAMENTO_GIROSCOPIO_DLPF_OFF/(1+valore_DFC));

		}

		intervallo_Tempo_Discreto_sec = (1/frequenza_Di_Campionamento_Hz);

		if(MPU6050_AspettaMisure(disp)){

			status_vecchio_dato = MPU6050_LetturaDatoGrezzoGiroscopio(disp);

			controllore_dato.vecchio_DatoX = disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_X;
			controllore_dato.vecchio_DatoY = disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_Y;
			controllore_dato.vecchio_DatoZ = disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_Z;

		}

			if(MPU6050_AspettaMisure(disp)){

				status_nuovo_dato = MPU6050_LetturaDatoGrezzoGiroscopio(disp);

				controllore_dato.attuale_DatoX = disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_X;
				controllore_dato.attuale_DatoY = disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_Y;
				controllore_dato.attuale_DatoZ = disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_Z;



			if( controllore_dato.attuale_DatoX - controllore_dato.vecchio_DatoX >= MASSIMA_ESCURSIONE_RUMORE_BIANCO_ASSE_X_EMPIRICO){

				disp->datiGiroscopio.datoPosAngGiro.posizioneAngolare_X = disp->datiGiroscopio.datoPosAngGiro.vecchia_posizioneAngolare_X_val_integrale + (intervallo_Tempo_Discreto_sec*( AMPLIFICATORE_EMPIRICO_INTEGRALE_ASSE_X*(controllore_dato.vecchio_DatoX + controllore_dato.attuale_DatoX)/2.0f));
				disp->datiGiroscopio.datoPosAngGiro.vecchia_posizioneAngolare_X_val_integrale = disp->datiGiroscopio.datoPosAngGiro.posizioneAngolare_X;
			}

				if(disp->datiGiroscopio.datoPosAngGiro.posizioneAngolare_X < ANGOLO_0){

					disp->datiGiroscopio.datoPosAngGiro.posizioneAngolare_X = disp->datiGiroscopio.datoPosAngGiro.posizioneAngolare_X +ANGOLO_360;

				}

				if(disp->datiGiroscopio.datoPosAngGiro.posizioneAngolare_X > ANGOLO_360){

					disp->datiGiroscopio.datoPosAngGiro.posizioneAngolare_X = disp->datiGiroscopio.datoPosAngGiro.posizioneAngolare_X -ANGOLO_360;

				}



			if(controllore_dato.attuale_DatoY - controllore_dato.vecchio_DatoY >= MASSIMA_ESCURSIONE_RUMORE_BIANCO_ASSE_Y_EMPIRICO){

				disp->datiGiroscopio.datoPosAngGiro.posizioneAngolare_Y = disp->datiGiroscopio.datoPosAngGiro.vecchia_posizioneAngolare_Y_val_integrale + (intervallo_Tempo_Discreto_sec*(AMPLIFICATORE_EMPIRICO_INTEGRALE_ASSE_Y*(controllore_dato.vecchio_DatoY + controllore_dato.attuale_DatoY)/2.0f));
				disp->datiGiroscopio.datoPosAngGiro.vecchia_posizioneAngolare_Y_val_integrale = disp->datiGiroscopio.datoPosAngGiro.posizioneAngolare_Y;
			}

				if(disp->datiGiroscopio.datoPosAngGiro.posizioneAngolare_Y < ANGOLO_0){

					disp->datiGiroscopio.datoPosAngGiro.posizioneAngolare_Y = disp->datiGiroscopio.datoPosAngGiro.posizioneAngolare_Y +ANGOLO_360;

				}

				if(disp->datiGiroscopio.datoPosAngGiro.posizioneAngolare_Y > ANGOLO_360){

					disp->datiGiroscopio.datoPosAngGiro.posizioneAngolare_Y = disp->datiGiroscopio.datoPosAngGiro.posizioneAngolare_Y - ANGOLO_360;
				}


			if(controllore_dato.attuale_DatoZ - controllore_dato.vecchio_DatoZ >= MASSIMA_ESCURSIONE_RUMORE_BIANCO_ASSE_Z_EMPIRICO){

				disp->datiGiroscopio.datoPosAngGiro.posizioneAngolare_Z = disp->datiGiroscopio.datoPosAngGiro.vecchia_posizioneAngolare_Z_val_integrale + (intervallo_Tempo_Discreto_sec*(AMPLIFICATORE_EMPIRICO_INTEGRALE_ASSE_Z*(controllore_dato.vecchio_DatoZ + controllore_dato.attuale_DatoZ)/2.0f));
				disp->datiGiroscopio.datoPosAngGiro.vecchia_posizioneAngolare_Z_val_integrale = disp->datiGiroscopio.datoPosAngGiro.posizioneAngolare_Z;
			}

				if(disp->datiGiroscopio.datoPosAngGiro.posizioneAngolare_Z < ANGOLO_0){

					disp->datiGiroscopio.datoPosAngGiro.posizioneAngolare_Z = disp->datiGiroscopio.datoPosAngGiro.posizioneAngolare_Z +ANGOLO_360;

				}

				if(disp->datiGiroscopio.datoPosAngGiro.posizioneAngolare_Z > ANGOLO_360){

					disp->datiGiroscopio.datoPosAngGiro.posizioneAngolare_Z = disp->datiGiroscopio.datoPosAngGiro.posizioneAngolare_Z - ANGOLO_360;

				}
			}

		return status;
	}
}

HAL_StatusTypeDef MPU6050_LetturaDatoGrezzoGiroscopio(MPU6050_disp disp){

	uint8_t bufferDatiGrezziGiroscopio[AMPIEZZA_BUFFER_DATI_ACC_GIRO_BYTE];

	HAL_StatusTypeDef status, status_ComUART, status_OttieniFSRGiro, status_PAD;

	uint8_t fullScaleRangeGiroscopio;

	status = HAL_I2C_Mem_Read(&(disp->perifericaComunicazioneI2c), disp->i2c_slave_address, GYRO_XOUT_H, I2C_MEMADD_SIZE_8BIT, bufferDatiGrezziGiroscopio, MEM_SIZE_6_BYTE, STANDARD_TIMEOUT_MILLISEC);

	if(status != HAL_OK){

		char msg_errLett_MPU6050_LetturaDatoGrezzoGiroscopio[STANDARD_ERROR_MESSAGE_LENGTH];
		sprintf(msg_errLett_MPU6050_LetturaDatoGrezzoGiroscopio, "MPU6050_LetturaDatoGrezzoGiroscopio ERRORE::: errore nella lettura : %d", status);
		status_ComUART = HAL_UART_Transmit(&(disp->perifericaComunicazioneHost), (uint8_t*)msg_errLett_MPU6050_LetturaDatoGrezzoGiroscopio, strlen(msg_errLett_MPU6050_LetturaDatoGrezzoGiroscopio), HAL_MAX_DELAY);

		return status;

	}else{

		disp->datiGiroscopio.datoGrezzoGiro.DatoGrezzoGiroscopio_X = (int16_t)(bufferDatiGrezziGiroscopio[0]<<8 | bufferDatiGrezziGiroscopio[1]);
		disp->datiGiroscopio.datoGrezzoGiro.DatoGrezzoGiroscopio_Y = (int16_t)(bufferDatiGrezziGiroscopio[2]<<8 | bufferDatiGrezziGiroscopio[3]);
		disp->datiGiroscopio.datoGrezzoGiro.DatoGrezzoGiroscopio_Z = (int16_t)(bufferDatiGrezziGiroscopio[4]<<8 | bufferDatiGrezziGiroscopio[5]);

		status_OttieniFSRGiro = MPU6050_OttieniFullScaleRangeGiroscopio(disp, &fullScaleRangeGiroscopio);

		switch(fullScaleRangeGiroscopio){

		case FULL_SCALE_RANGE_250_gr_sec :

				disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_X = (disp->datiGiroscopio.datoGrezzoGiro.DatoGrezzoGiroscopio_X)/GIRO_LSB_SENSITIVITY_FOR_250_g_s;
				disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_Y = (disp->datiGiroscopio.datoGrezzoGiro.DatoGrezzoGiroscopio_Y)/GIRO_LSB_SENSITIVITY_FOR_250_g_s;
				disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_Z = (disp->datiGiroscopio.datoGrezzoGiro.DatoGrezzoGiroscopio_Z)/GIRO_LSB_SENSITIVITY_FOR_250_g_s;

				if((disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_X) > VALORE_MAX_VEL_ANG_MISURABILE_CON_RUMORE_PER_250_g_s){

					disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_X = (disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_X-ESCURSIONE_GIRO_PER_250_g_s);
				}

				if((disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_Y) > VALORE_MAX_VEL_ANG_MISURABILE_CON_RUMORE_PER_250_g_s){

					disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_Y = (disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_Y-ESCURSIONE_GIRO_PER_250_g_s);

				}

				if((disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_Z) > VALORE_MAX_VEL_ANG_MISURABILE_CON_RUMORE_PER_250_g_s){

					disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_Z = (disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_Z-ESCURSIONE_GIRO_PER_250_g_s);
				}


				break;

		case FULL_SCALE_RANGE_500_gr_sec :

				disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_X = (disp->datiGiroscopio.datoGrezzoGiro.DatoGrezzoGiroscopio_X)/GIRO_LSB_SENSITIVITY_FOR_500_g_s;
				disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_Y = (disp->datiGiroscopio.datoGrezzoGiro.DatoGrezzoGiroscopio_Y)/GIRO_LSB_SENSITIVITY_FOR_500_g_s;
				disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_Z = (disp->datiGiroscopio.datoGrezzoGiro.DatoGrezzoGiroscopio_Z)/GIRO_LSB_SENSITIVITY_FOR_500_g_s;

				if((disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_X) > VALORE_MAX_VEL_ANG_MISURABILE_CON_RUMORE_PER_500_g_s){

					disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_X = (disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_X-ESCURSIONE_GIRO_PER_500_g_s);

				}

				if((disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_Y) > VALORE_MAX_VEL_ANG_MISURABILE_CON_RUMORE_PER_500_g_s){

					disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_Y = (disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_Y-ESCURSIONE_GIRO_PER_500_g_s);

				}

				if((disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_Z) > VALORE_MAX_VEL_ANG_MISURABILE_CON_RUMORE_PER_500_g_s){

					disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_Z = (disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_Z-ESCURSIONE_GIRO_PER_500_g_s);

				}

				break;

		case FULL_SCALE_RANGE_1000_gr_sec :

				disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_X = (disp->datiGiroscopio.datoGrezzoGiro.DatoGrezzoGiroscopio_X)/GIRO_LSB_SENSITIVITY_FOR_1000_g_s;
				disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_Y = (disp->datiGiroscopio.datoGrezzoGiro.DatoGrezzoGiroscopio_Y)/GIRO_LSB_SENSITIVITY_FOR_1000_g_s;
				disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_Z = (disp->datiGiroscopio.datoGrezzoGiro.DatoGrezzoGiroscopio_Z)/GIRO_LSB_SENSITIVITY_FOR_1000_g_s;

				if((disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_X) > VALORE_MAX_VEL_ANG_MISURABILE_CON_RUMORE_PER_1000_g_s){

					disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_X = (disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_X-ESCURSIONE_GIRO_PER_1000_g_s);

				}

				if((disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_Y) > VALORE_MAX_VEL_ANG_MISURABILE_CON_RUMORE_PER_1000_g_s){

					disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_Y = (disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_Y-ESCURSIONE_GIRO_PER_1000_g_s);

				}

				if((disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_Z) > VALORE_MAX_VEL_ANG_MISURABILE_CON_RUMORE_PER_1000_g_s){

					disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_Z = (disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_Z-ESCURSIONE_GIRO_PER_1000_g_s);

				}

				break;

		case FULL_SCALE_RANGE_2000_gr_sec :

				disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_X = (disp->datiGiroscopio.datoGrezzoGiro.DatoGrezzoGiroscopio_X)/GIRO_LSB_SENSITIVITY_FOR_2000_g_s;
				disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_Y = (disp->datiGiroscopio.datoGrezzoGiro.DatoGrezzoGiroscopio_Y)/GIRO_LSB_SENSITIVITY_FOR_2000_g_s;
				disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_Z = (disp->datiGiroscopio.datoGrezzoGiro.DatoGrezzoGiroscopio_Z)/GIRO_LSB_SENSITIVITY_FOR_2000_g_s;

				if((disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_X) > VALORE_MAX_VEL_ANG_MISURABILE_CON_RUMORE_PER_2000_g_s){

					disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_X = (disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_X-ESCURSIONE_GIRO_PER_2000_g_s);

				}

				if((disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_Y) > VALORE_MAX_VEL_ANG_MISURABILE_CON_RUMORE_PER_2000_g_s){

					disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_Y = (disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_Y-ESCURSIONE_GIRO_PER_2000_g_s);
				}

				if((disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_Z) > VALORE_MAX_VEL_ANG_MISURABILE_CON_RUMORE_PER_2000_g_s){

					disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_Z = (disp->datiGiroscopio.datoVelAngGiro.velocitaaAngolare_Z-ESCURSIONE_GIRO_PER_2000_g_s);
				}

				break;

		default :

			char msg_errSwitchCaseFSRGiro[STANDARD_ERROR_MESSAGE_LENGTH];
			sprintf(msg_errSwitchCaseFSRGiro, "MPU6050_LetturaDatoGrezzoGiroscopio__TrasformazioneDati ERRORE::: errore del valore o nella luttura del full scale range accelerometro : %d", status_OttieniFSRGiro);
			status_ComUART = HAL_UART_Transmit(&(disp->perifericaComunicazioneHost), (uint8_t*)msg_errSwitchCaseFSRGiro, strlen(msg_errSwitchCaseFSRGiro), HAL_MAX_DELAY);

			break;

		}

		return status;
	}
}

/**
 * Per esplicare l'operatività della funzione MPU6050_AspettaMisure
 * 
 * 
 */

if(MPU6050_AspettaMisure(disp)){
	MPU6050_LetturaDatiGiroscopio__TrasformazioneDati(disp)
}



