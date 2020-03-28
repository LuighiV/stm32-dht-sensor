/*
 * delay.h
 *
 *  Created on: Mar 26, 2020
 *      Author: luighi
 *
 *  Reference:
 *  https://controllerstech.com/create-1-microsecond-delay-stm32/
 */

#ifndef INC_DELAY_H_
#define INC_DELAY_H_

#include "main.h"
#include "dht_sensor_conf.h"

extern TIM_HandleTypeDef TIM_HANDLER;

void delay_us(uint16_t us);

void resetTick_us();
uint32_t getTick_us ();

#endif /* INC_DELAY_H_ */
