/*
 * dht_sensor_conf.h
 *
 *  Created on: Mar 27, 2020
 *      Author: luighi
 */

#ifndef INC_DHT_SENSOR_CONF_H_
#define INC_DHT_SENSOR_CONF_H_

#include "main.h"

#define DHT_PIN_conf GPIO_PIN_7
#define DHT_GPIO_conf GPIOD
#define DHT_Type_conf DHT21

#define TIM_HANDLER htim2


/*Debuger Options*/
//#define DHT_DEBUG

//#define DEBUG_TYPE USB_OTG

#define DEBUG_HANDLER	huart3

#endif /* INC_DHT_SENSOR_CONF_H_ */
