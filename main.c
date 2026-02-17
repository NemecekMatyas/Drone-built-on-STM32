/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_host.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ibus.h"

//MPU6050
#include "mpu6050.h"

#include "pid.h"
#include <math.h>



//#include "pid.c"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//IBUS
uint16_t ibus_data[IBUS_USER_CHANNELS];

//IBUS CONTROL
uint16_t MappedData = 0;

uint16_t PLData = 0;
uint16_t PPData = 0;
uint16_t ZLData = 0;
uint16_t ZPData = 0;


uint16_t dataLevaPrava = 0;
uint16_t dataDopreduDozadu = 0;
uint16_t dataRotace = 0;

uint16_t data_Leva =0;
uint16_t data_Prava =0;
uint16_t data_RotaceDoprava = 0;
uint16_t data_RotaceDoleva = 0;


uint16_t PLDataManSniz = 0;
uint16_t PPDataManSniz = 0;
uint16_t ZLDataManSniz = 0;
uint16_t ZPDataManSniz = 0;

//ADXL345 ACCELOMETER
int16_t x, y, z;

//mpu6050
MPU6050_t MPU6050;

double roll = 0;
double pitch = 0;

double stabilizationRollPL = 0;
double stabilizationRollPP = 0;
double stabilizationRollZL = 0;
double stabilizationRollZP = 0;


double stabilizationPitchPL = 0;
double stabilizationPitchPP = 0;
double stabilizationPitchZL = 0;
double stabilizationPitchZP = 0;

double stabilizationPitch = 0;
double stabilizationRoll = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
//IBUS

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//IBUS CONTROL
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min + 1) / (in_max - in_min + 1) + out_min;
}


//PID CONTROL
// Constants for control
#define KP 1.5

// Function to constrain values
float constrain(float value, float min, float max) {
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

// Function for P-only control
float p_control(float setpoint, float measured) {
    float error = setpoint - measured;
    return KP * error;
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */



  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USB_HOST_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  //SERVO
  	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  	//IBUS CONTROL
  	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	//IBUS
	ibus_init();

	//mpu6050
	while (MPU6050_Init(&hi2c1) == 1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  //IBUS
	  ibus_read(ibus_data);
	  ibus_soft_failsafe(ibus_data, 10); // if ibus is not updated, clear ibus data.
	  HAL_Delay(10);


    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */
    MPU6050_Read_All(&hi2c1, &MPU6050);



  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */


//IBUS
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == IBUS_UART)
    {
        ibus_reset_failsafe();

        // Read MPU6050 sensor data
        MPU6050_Read_All(&hi2c1, &MPU6050);

        // Get pitch and roll values from Kalman filter
        pitch = MPU6050.KalmanAngleX;
        roll = MPU6050.KalmanAngleY;

        // Apply P-control for stabilization
        stabilizationPitch = p_control(0, pitch);
        stabilizationRoll = p_control(0, roll);

        stabilizationPitchPL = stabilizationPitch;
        stabilizationPitchPP = stabilizationPitch;
        stabilizationPitchZL = stabilizationPitch;
        stabilizationPitchZP = stabilizationPitch;

        stabilizationRollPL = stabilizationRoll;
        stabilizationRollPP = stabilizationRoll;
        stabilizationRollZL = stabilizationRoll;
        stabilizationRollZP = stabilizationRoll;

        // Read user inputs from IBUS
        MappedData = map(ibus_data[2], 1000, 2000, 50, 100);
        dataLevaPrava = map(ibus_data[0], 1000, 2000, 50, 100);
        dataDopreduDozadu = map(ibus_data[1], 1000, 2000, 50, 100);
        dataRotace = map(ibus_data[3], 1000, 2000, 50, 100);

        // Default motor values
        PLDataManSniz = MappedData;
        PPDataManSniz = MappedData;
        ZLDataManSniz = MappedData;
        ZPDataManSniz = MappedData;



        // Apply manual reduction due to user input
    if (dataLevaPrava < 75) { // Doleva
        uint16_t snizeni = 75 - dataLevaPrava;
        snizeni = snizeni > 15 ? 15 : snizeni;
        ZLDataManSniz -= snizeni;
        PLDataManSniz -= snizeni;
    } else if (dataLevaPrava > 75) { // Doprava
        uint16_t snizeni = dataLevaPrava - 75;
        snizeni = snizeni > 15 ? 15 : snizeni;
        ZPDataManSniz -= snizeni;
        PPDataManSniz -= snizeni;
    }

    if (dataDopreduDozadu < 75) { // Dopředu
        uint16_t snizeni = 75 - dataDopreduDozadu;
        snizeni = snizeni > 15 ? 15 : snizeni;
        ZLDataManSniz -= snizeni;
        ZPDataManSniz -= snizeni;
    } else if (dataDopreduDozadu > 75) { // Dozadu
        uint16_t snizeni = dataDopreduDozadu - 75;
        snizeni = snizeni > 15 ? 15 : snizeni;
        PLDataManSniz -= snizeni;
        PPDataManSniz -= snizeni;
    }

    if (dataRotace < 75) { // Rotace doleva
        int16_t snizeni = 75 - dataRotace;
        snizeni = snizeni > 15 ? 15 : snizeni;
        PLDataManSniz -= snizeni;
        ZPDataManSniz -= snizeni;
    } else if (dataRotace > 75) { // Rotace doprava
        uint16_t snizeni = dataRotace - 75;
        snizeni = snizeni > 15 ? 15 : snizeni;
        ZLDataManSniz -= snizeni;
        PPDataManSniz -= snizeni;
    }
        PLDataManSniz = (PLDataManSniz - stabilizationPitchPL + stabilizationRollPL);
        PPDataManSniz = (PPDataManSniz - stabilizationPitchPP - stabilizationRollPP);
        ZLDataManSniz = (ZLDataManSniz + stabilizationPitchZL + stabilizationRollZL);
        ZPDataManSniz = (ZPDataManSniz + stabilizationPitchZP - stabilizationRollZP);

        // Constrain values
        PLDataManSniz = constrain(PLDataManSniz, 50, 100);
        PPDataManSniz = constrain(PPDataManSniz, 50, 100);
        ZLDataManSniz = constrain(ZLDataManSniz, 50, 100);
        ZPDataManSniz = constrain(ZPDataManSniz, 50, 100);

        // Aktualizace PWM pro motory
        TIM1->CCR1 = PLDataManSniz; // Přední levý motor
        TIM1->CCR2 = ZPDataManSniz; // Zadní pravý motor
        TIM1->CCR3 = ZLDataManSniz; // Zadní levý motor
        TIM1->CCR4 = PPDataManSniz; // Přední pravý motor

        // Debug output
        //printf("PL: %d, PP: %d, ZL: %d, ZP: %d\n", PLDataManSniz, PPDataManSniz, ZLDataManSniz, ZPDataManSniz);
    }
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
