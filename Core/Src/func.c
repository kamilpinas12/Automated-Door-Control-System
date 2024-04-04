/*
 * func.c
 *
 *  Created on: Oct 21, 2023
 *      Author: kamil
 */
#include "func.h"


bool get_state(pin pin)
{
	if (pin == kran_otw)
	{
		if(HAL_GPIO_ReadPin(KRAN_OTW_GPIO_Port, KRAN_OTW_Pin) == GPIO_PIN_RESET)
		{
			return 0;
		}
		else
		{
			return 1;
		}
	}


	if (pin == kran_zam)
	{
		if(HAL_GPIO_ReadPin(KRAN_ZAM_GPIO_Port, KRAN_ZAM_Pin) == GPIO_PIN_RESET)
		{
			return 0;
		}
		else
		{
			return 1;
		}
	}

	if (pin == button)
		{
			if(HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == GPIO_PIN_RESET)
			{
				return 1;
			}
			else
			{
				return 0;
			}
		}

	return 0;
}


void motor(stan_dzialania command)
{
	static int counter;

	if(command == zamkniete || command == otwarte || command == pomiedzy)
	{
		counter = 0;
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);

	}
	else if(command == otwieranie)
	{
		if(counter == 0)counter = motor_starting_value;

		if(counter < 990)
		{
			counter += 10;
		}
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, counter);

	}

	else if (command == zamykanie)
	{
		if(counter == 0) counter = motor_starting_value;
		if(counter < 990)
		{
			counter += 10;
		}
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, counter);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
	}



}



void pomiar_foto(time_of_day* state)
{
	//pomiar ADC
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	uint32_t val = HAL_ADC_GetValue(&hadc1);


	static uint8_t counter_noc;
	static uint8_t counter_dzien;

	if(val > granica_dzien && (*state == noc || *state == undefined))
	{
		counter_dzien ++;
		counter_noc = 0;
	}

	else if(val < granica_noc && (*state == dzien || *state == undefined))
	{
		counter_noc ++;
		counter_dzien = 0;
	}
	else
	{
		counter_noc = 0;
		counter_dzien = 0;
	}


	if(counter_dzien > 150)
	{
		counter_dzien = 0;
		*state = dzien;
	}

	if(counter_noc > 150)
	{
		counter_noc = 0;
		*state = noc;
	}

}



void error_led(error_code error)
{
	motor(pomiedzy);

	while(!get_state(button))
	{
		HAL_IWDG_Refresh(&hiwdg);

		for(int i = 0; (i < error) && !get_state(button); i++)
		{
			HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, GPIO_PIN_RESET);
			HAL_Delay(150);
			HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, GPIO_PIN_SET);
			HAL_Delay(150);
		}
		HAL_Delay(700);
	}
}










