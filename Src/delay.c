/*
 * delay.c
 *
 *  Created on: Mar 27, 2020
 *      Author: luighi
 */
#include "main.h"
#include "delay.h"
#include "dht_sensor_conf.h"

void delay_us(uint16_t us){

  resetTick_us();  // set the counter value a 0

  while (__HAL_TIM_GET_COUNTER(&TIM_HANDLER) < us);  // wait for the counter to reach the us input in the parameter
}

void resetTick_us(){
	__HAL_TIM_SET_COUNTER(&TIM_HANDLER,0);
}
uint32_t getTick_us(){
	return __HAL_TIM_GET_COUNTER(&TIM_HANDLER);
}
