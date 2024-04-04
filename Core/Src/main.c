/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "adc.h"
#include "i2c.h"
#include "iwdg.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "func.h"
#include "INA219.h"
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
INA219_t ina219;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


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
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  // Uruchomienie układu

  for (int i = 0; i < 10; i++) {
    HAL_GPIO_TogglePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin);
    HAL_Delay(50);
  }

  while(!INA219_Init(&ina219, &hi2c1, INA219_ADDRESS))
  {

  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // ZMIENNE STANU
  time_of_day pora_dnia = undefined;
  stan_dzialania stan;
  mode tryb = automatic;
  stan_dzialania lst_stan;
  bool button_flag = 0;


  //DELAY
  unsigned long lst_time_pomiar = HAL_GetTick();
  unsigned long lst_time_manual = HAL_GetTick();
  unsigned long lst_time_motor = HAL_GetTick();
  unsigned long lst_time_pomiar_INA = HAL_GetTick();
  unsigned long lst_time_input = HAL_GetTick();

  //DEBOUNCING INPUTS
  int counter_otw = 0;
  int counter_zam = 0;
  int counter_button = 0;


  //ZABEZPIECZNIA
  int work_time = 0;
  int max_current_counter = 0;
  int incorrect_voltage_counter = 0;


  //USTALENIE STANU POCZĄTKOWEGO
  if(get_state(kran_zam)) stan = zamkniete;
  else if(get_state(kran_otw)) stan = otwarte;
  else stan = pomiedzy;
  lst_stan = stan;

  //ADC
  HAL_ADCEx_Calibration_Start(&hadc1);


  //PWM
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);


  //INA 219
  uint16_t vbus, current;

  vbus = INA219_ReadBusVoltage(&ina219);
  current = INA219_ReadCurrent(&ina219);


  while (1)
  {
	  // SPRAWDZANIE WEJŚĆ
	  if(HAL_GetTick() - lst_time_input > 2)
	  {

		  HAL_IWDG_Refresh(&hiwdg);

		  lst_time_input = HAL_GetTick();

		  if(get_state(kran_zam) && (stan == zamykanie || stan == pomiedzy)) counter_zam ++;
		  else counter_zam = 0;

		  if(get_state(kran_otw) && (stan == otwieranie || stan == pomiedzy)) counter_otw ++;
		  else counter_otw = 0;

		  if(get_state(button) && !button_flag) counter_button++;
		  if(!get_state(button) && counter_button < 49) counter_button = 0;
	  }


	  //USTAWIENIE STANÓW WEJŚĆ
	  if(counter_zam > 30)
	  {
		  work_time = 0;
		  counter_zam = 0;
		  motor(zamkniete);
		  stan = zamkniete;
	  }
	  if(counter_otw > 30)
	  {
		  work_time = 0;
		  counter_otw = 0;
		  motor(otwarte);
		  stan = otwarte;
	  }

	  //!get_state  ponieważ wciśnięcie przycisku uznane dopiero po jego puszczeniu
	  if(counter_button > 50 && !get_state(button))
	  {
		  counter_button = 0;
		  button_flag = 1;
		  work_time = 0;

	  }


// TRYB AUTOMATYCZNY

	  if(tryb == automatic)
	  {
		  //POMIAR Z FOTO-REZYSTORA
		  if(stan != otwieranie && stan != zamykanie && ((HAL_GetTick() - lst_time_pomiar) > 100))
		  {
			  lst_time_pomiar = HAL_GetTick();
			  pomiar_foto(&pora_dnia);

		  }

		  // ROZPOCZYNANIE ZAMYKANIA LUB OTWIERANIA
		  if((stan == otwarte || stan == pomiedzy) && pora_dnia == noc)
		  {
			  motor(zamykanie);
			  stan = zamykanie;
		  }
		  if((stan == zamkniete || stan == pomiedzy) && pora_dnia == dzien)
		  {
			  motor(otwieranie);
			  stan = otwieranie;
		  }
	  }




// TRYB MANUALNY

	  if(tryb == manual && (HAL_GetTick() - lst_time_manual >  150000))   // 2.5 min : czas działania trybu manualnego
	  {
		  tryb = automatic;
		  HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, GPIO_PIN_SET);
	  }



	  if(button_flag)  //debouncing
	  {
		  lst_time_manual = HAL_GetTick();
		  tryb = manual;
		  HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, GPIO_PIN_RESET);

		  if(stan == otwieranie)
		  {
			  motor(pomiedzy);
			  stan = pomiedzy;
			  lst_stan = otwieranie;
		  }

		  else if(stan == zamykanie)
		  {
			  motor(pomiedzy);
			  stan = pomiedzy;
			  lst_stan = zamykanie;
		  }

		  else if(stan == zamkniete)
		  {
			  stan = otwieranie;
		  }

		  else if(stan == otwarte)
		  {
			  stan = zamykanie;
		  }

		  else if(stan == pomiedzy && (lst_stan == otwieranie || lst_stan == pomiedzy))
		  {
			  stan = zamykanie;
		  }

		  else if(stan == pomiedzy && lst_stan == zamykanie)
		  {
			  stan = otwieranie;
		  }

		  button_flag = 0;
	  }




// OBS�?UGA SILNIKA
	  if((HAL_GetTick() - lst_time_motor > 15) && (stan == otwieranie || stan == zamykanie))
	  {
		  lst_time_motor = HAL_GetTick();
		  motor(stan);
		  work_time ++;
	  }






// ZABEZPIECZENIA
	  if(stan != otwieranie && stan != zamykanie) work_time = 0;

	  //pomiar prądu i napięcia
	  if(HAL_GetTick() - lst_time_pomiar_INA > 5)
	  {
		  lst_time_pomiar_INA = HAL_GetTick();
		  vbus = INA219_ReadBusVoltage(&ina219);
		  current = INA219_ReadCurrent(&ina219);

		  if(work_time > 40 && current > 2700)  //jeżeli po 200 ms prąd > 2500 mA
		  {
			  max_current_counter ++;
		  }
		  else max_current_counter = 0;


		  if(vbus > 13500 || vbus < 8000)
		  {
			  incorrect_voltage_counter ++;
		  }
		  else incorrect_voltage_counter = 0;

	  }



	  if(work_time > 11000)   // czas pracy w sekundach    (work_time * 15)/1000
	  {
		  error_led(to_long_run_time);
		  motor(pomiedzy);
		  work_time = 0;
		  stan = pomiedzy;
	  }




	  if(max_current_counter > 10)
	  {
		  if(stan == zamykanie)
		  {
			  work_time = 0;
			  counter_zam = 0;
			  max_current_counter = 0;
			  motor(zamkniete);
			  stan = zamkniete;
		  }
		  else{
			  error_led(to_high_current);
			  motor(pomiedzy);
			  work_time = 0;
			  stan = pomiedzy;
			  max_current_counter = 0;
		  }
	  }


	  if(incorrect_voltage_counter > 100)        // nieodpowiedznie napięcie [mV]
	  {
		  error_led(incorrect_voltage);
		  motor(pomiedzy);
		  work_time = 0;
		  stan = pomiedzy;
		  incorrect_voltage_counter = 0;
	  }


	  //if(work_time > 20 && current < 100)    //zbyt niski prąd silnia (silnik nie działa lub się odłączył)
	  //{
		  //error_led(to_low_work_current);
		  //motor(pomiedzy);
		  //work_time = 0;
		  //stan = pomiedzy;
	  //}

  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
