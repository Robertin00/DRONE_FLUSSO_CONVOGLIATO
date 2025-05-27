/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "PID.h"
#include "servo.h"
#include "tim2Interrupt.h"
#include "AHRS.h"
#include <stdarg.h>  // Per va_start, va_end, va_list, vsnprintf
#include <string.h>  // Per strlen
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

//variabili locali


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

//prototipi per funzioni locali

void Setup();
void initialize();


void Callback_10ms();
void Callback_50ms();
void Callback_100ms();
void Fallback();
void display_results (uint16_t);

void sw1_callback(void); //accensione sensori
void sw2_callback(void); //accensione motori a velocità minima
void sw3_callback(void); //avvio completo dei motori per raggiungimento quota impostata
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//variabili globali



float conv;

//struttura per definire i punti del piano
struct axis
{
	float x;
	float y;
	float z;
};

struct dynamic
{
	struct axis acc;
	struct axis vel;
	struct axis pos;
};

struct physicalState
{
	struct dynamic accel;
	struct dynamic gyro;
	struct dynamic magn;
	struct dynamic abs;
	struct dynamic Kalman;
	float avg_motor_us;
	float motor_diff_us;
	float x_servo_deg;
	float y_servo_deg;
};

union // vive una struttura alla volta
{
	struct physicalState key;
	float index[sizeof(struct physicalState)];
} desiredState;	//state variables you want to reach
union
{
	struct physicalState key;
	float index[sizeof(struct physicalState)];
} currentState;	//current state variables of the DuctedFan

// Structure containing timer flags
extern struct timerClocks timers;

extern UART_HandleTypeDef huart3;

/* Create PID structure used for PID properties */
PID_config z_axis_PID;
PID_config Pitch_PID;
PID_config Roll_PID;

/* Used to store value of altitude need to be reached (meters) */
float altitudeValue = 0.17;

SimulatedAHRS simulatedData;

/* Time in seconds every which PID control is made */
float dt = 0.05;		// per i motori
float dt_servo=0.01f;	// per i flap

// variabili altimetro
// vl53l1x sensore; i2c
// vl53l1x* temp; i2c
float distanza;
float distanza_metri;

//variabili servomotore
float angolo_roll=0;  	// uscita del pid
float roll_deg;		  	// lettura imu
float setPoint_roll=0.0f;	// set point pid di assetto
float angolo_pitch=0.0f; 	// uscita del pid
float pitch_deg;	 	// lettura imu
float setPoint_pitch=0; // set point pid di assetto

//variabili per regolare accensione motori e sensori
int motore=0;
int dist=1;
int sens_on=1;

/*Dichiarazioni strutture dati utilizzate.*/

 // AHRS_out ahrs;

int cont=0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /*HW semaphore Clock enable*/
  __HAL_RCC_HSEM_CLK_ENABLE();
  /* Activate HSEM notification for Cortex-M4*/
  HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));
  /*
  Domain D2 goes to STOP mode (Cortex-M4 in deep-sleep) waiting for Cortex-M7 to
  perform system initialization (system clock config, external memory configuration.. )
  */
  HAL_PWREx_ClearPendingEvent();
  HAL_PWREx_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE, PWR_D2_DOMAIN);
  /* Clear HSEM flag */
  __HAL_HSEM_CLEAR_FLAG(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));

/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

 //inizializzazione servomotori
  Servos_Init();

  //Inizializzazione PID servo
  PID_Init(&Pitch_PID, -1.0, 0.0, 0.0, dt_servo, -30, 30); //Valori da scegliere kp, ki, kd
  PID_Init(&Roll_PID, -1.0, 0.0, 0.0, dt_servo, -30, 30); //Valori da scegliere kp, ki, kd

  // --- INIZIALIZZA LO STATO DEL AHRS SIMULATO ---
    initialize_ahrs_state(&simulatedData);

  Setup();



/*  initialize(); //inizializzazione sensore di altezza (non implementata)

  while(!timers.timer_2000mS)
  	{
  		// time necessary to arm motor1 and motor2
  	}
  	timers.timer_2000mS = 0;

	Setup_MARG(&ahrs); //funzione di setup accelerometro, giroscopio, magnetometro e calibrazione magnetometro.(non implementata)

  	while(sens_on);
	lcd_clear(); // Clear LCD
*/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (timer_10mS_flag)
	    {
	        timer_10mS_flag = 0;
	        Callback_10ms();  // PID per i flap
	    }

	    if (timer_50mS_flag)
	    {
	        timer_50mS_flag = 0;
	        Callback_50ms();  // PID altezza (motori) non implementata
	    }

	    if (timer_100mS_flag)
	    {
	        timer_100mS_flag = 0;
	        Callback_100ms(); // Stampa su display non implementata

	    }

  }
  /* USER CODE END 3 */
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7499;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 75-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 3030;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Setup(){

	/* Setup Compare Match Timer/GLOBAL TIMER */
	tim2Interrupt_startcount();
}
void Callback_10ms(){
	uint32_t timestamp_ms = HAL_GetTick(); // Per il logging
	// 1. LEGGI STATO CORRENTE DAL SIMULATORE AHRS
	    //    Questi sono gli angoli risultanti dal passo di simulazione precedente.
	    //    All'inizio, saranno 0.0, poi scatteranno al valore del disturbo,
	    //    poi il PID inizierà a correggerli.
	    float attuale_roll_simulato = simulatedData.RollDeg;
	    float attuale_pitch_simulato = simulatedData.PitchDeg;

	    // 2. CALCOLA COMANDI SERVO CON IL PID
	    //    Il PID usa gli angoli simulati attuali come "misura" e cerca di portarli al setpoint (0.0).
	    float comando_servo_roll = -(PID_Compute(attuale_roll_simulato, setPoint_roll, &Roll_PID));
	    float comando_servo_pitch = -(PID_Compute(attuale_pitch_simulato, setPoint_pitch, &Pitch_PID));

	    // 3. AGGIORNA GLI ANGOLI SIMULATI PER IL PROSSIMO CICLO
	    //    L'output del PID (comando_servo_roll/pitch) ora influenza lo stato simulato.
	    //    Le variabili locali 'prossimo_roll_calcolato' e 'prossimo_pitch_calcolato'
	    //    ricevono i nuovi valori, che sono anche aggiornati dentro 'simulatedData'.
	    float prossimo_roll_calcolato, prossimo_pitch_calcolato;
	    update_simulated_angles(&simulatedData, comando_servo_roll, comando_servo_pitch, &prossimo_roll_calcolato, &prossimo_pitch_calcolato);
	    // Ora simulatedData.RollDeg e simulatedData.PitchDeg contengono i valori per l'inizio del prossimo ciclo.

	    // 4. COMANDA I SERVO FISICI (se li avessi collegati e volessi muoverli)
	    //    Per ora, questo serve principalmente a vedere l'output del PID.
	    Servo_Write_deg(SERVO_ROLL, comando_servo_roll);
	    Servo_Write_deg(SERVO_PITCH, comando_servo_pitch); // Decommenta se vuoi controllare anche il pitch

	    // 5. STAMPA I DATI VIA UART PER IL DEBUG/LOGGING
	    //    È utile vedere l'angolo di input del PID, il comando servo risultante,
	    //    e l'angolo che ne deriva per il ciclo successivo.


	   /* uart_printf("T:%lu, Roll_Attuale:%.2f, Cmd_Roll:%.2f, Roll_Prossimo:%.2f\r\n",
	                timestamp_ms,
	                attuale_roll_simulato,
	                comando_servo_roll,
	                simulatedData.RollDeg); // o prossimo_roll_calcolato, sono uguali qui
*/
	    //log roll
	    uart_printf("T:%lu --- RA: %.2f\r\n", timestamp_ms, attuale_roll_simulato);
	    uart_printf("CmdR: %.2f\r\n", comando_servo_roll);
	    uart_printf("RP: %.2f\r\n\r\n", simulatedData.RollDeg); // Stampa Roll_Prossimo
	    //log pitch
	    uart_printf("T:%lu --- PA: %.2f\r\n", timestamp_ms, attuale_pitch_simulato);
	    uart_printf("CmdP: %.2f\r\n", comando_servo_pitch);
	    uart_printf("PP: %.2f\r\n\r\n", simulatedData.PitchDeg);
	    // uart_printf("T:%lu, Pitch_Attuale:%.2f, Cmd_Pitch:%.2f, Pitch_Prossimo:%.2f\r\n",
	    //             timestamp_ms,
	    //             attuale_pitch_simulato,
	    //             comando_servo_pitch,
	    //             simulatedData.PitchDeg); // o prossimo_pitch_calcolato

	/* uint32_t t = HAL_GetTick(); // tempo in millisecondi

  //simulate_input_gradual(&simulatedData, &roll_deg, &pitch_deg);
// simulate_input_linear(&simulatedData, &roll_deg, &pitch_deg);

simulate_input_step(&simulatedData, &roll_deg, &pitch_deg);

 //verifica arrivo dei valori
  uart_printf("t:[%lu]\r\n",t);
  uart_printf("R:%.2f\r\n", roll_deg);
  uart_printf("P:%.2f\r\n", pitch_deg);

    angolo_roll = -(PID_Compute(roll_deg, setPoint_roll, &Roll_PID));
    angolo_pitch = -(PID_Compute(pitch_deg, setPoint_pitch, &Pitch_PID));

    //verifica PID
    uart_printf("Set_R:%.2f\r\n", setPoint_roll);
     uart_printf("Out_R:%.2f\r\n",angolo_roll);
     uart_printf("Set_P:%.2f\r\n", setPoint_pitch);
         uart_printf("Out_P:%.2f\r\n",angolo_pitch);



       //comando ai servo
    Servo_Write_deg(SERVO_ROLL, angolo_roll);
    Servo_Write_deg(SERVO_PITCH, angolo_pitch);
*/
};
void Callback_50ms(){};
void Callback_100ms(){};

void uart_printf(const char *fmt, ...) {
    char buffer[200];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
