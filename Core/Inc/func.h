/*
 * func.h
 *
 *  Created on: Oct 21, 2023
 *      Author: kamil
 */

#ifndef INC_FUNC_H_
#define INC_FUNC_H_

#include "stdbool.h"


#include "gpio.h"
#include "adc.h"
#include "tim.h"
#include "iwdg.h"


#define kran1 1
#define kran2 2

#define granica_dzien 1000
#define granica_noc 60

#define motor_starting_value 300




typedef enum
{
	noc = 0U,
	dzien,
	undefined
}time_of_day;


typedef enum
{
	pomiedzy = 0U,
	zamkniete,
	otwarte,
	otwieranie,
	zamykanie
}stan_dzialania;




typedef enum
{
	manual = 0U,
	automatic
}mode;

typedef enum
{
	kran_zam = 0U,
	kran_otw,
	button

}pin;

typedef enum
{
	to_high_current = 1,
	incorrect_voltage,
	to_long_run_time,
	to_low_work_current

}error_code;


bool get_state(pin pin);

void motor(stan_dzialania command);

void pomiar_foto(time_of_day* state);

void error_led(error_code error);

#endif /* INC_FUNC_H_ */
