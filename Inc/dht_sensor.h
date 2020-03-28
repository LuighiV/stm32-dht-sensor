/*
 * dht_sensor.h
 *
 *  Created on: Mar 26, 2020
 *      Author: luighi
 *  Reference:
 *  https://github.com/adafruit/DHT-sensor-library/blob/master/DHT.h
 */

#ifndef INC_DHT_SENSOR_H_
#define INC_DHT_SENSOR_H_

#include "main.h"
#include "string.h"
#include "dht_sensor_conf.h"

#include "delay.h"
#include "stdio.h"

#if defined(DHT_DEBUG) && (DEBUG_TYPE == USB_OTG)
#include "usb_device.h"
#endif


#define DATA_BYTE_LENGTH 5

#define countof(a) (sizeof(a) / sizeof(*(a)))


#define UART 		0
#define USB_OTG 	1

#define DEBUG_BUFFER_LENGTH 100

#ifndef DEBUG_TYPE
#define DEBUG_TYPE UART
#endif

#ifdef DHT_DEBUG

#if DEBUG_TYPE == USB_OTG
#define DEBUGGER(...)	CDC_Transmit_FS(__VA_ARGS__);
#else
#define DEBUGGER(...)	HAL_UART_Transmit(&DEBUG_HANDLER, __VA_ARGS__, 0xFFFFFFFF);

#endif

#define DEBUG_PRINT(MSG) 	DEBUGGER((uint8_t*)&MSG, countof(MSG)-1)
#define DEBUG_PRINTF(...)	memset(buff,0,countof(buff)); \
							sprintf(buff, __VA_ARGS__);\
							DEBUG_PRINT(buff)

#else
#define DEBUGGER(...) {}
#define DEBUG_PRINT(MSG) {}
#define DEBUG_PRINTF(...) {}
#endif

/**
 * Private variables for debugger
 */
#ifdef DHT_DEBUG
extern UART_HandleTypeDef DEBUG_HANDLER;
char buff[DEBUG_BUFFER_LENGTH];
#endif

#if defined(DHT_DEBUG) && (DEBUG_TYPE == USB_OTG)
extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
#endif

typedef enum DHT_Type{
	DHT11 = 0U,
	DHT12,
	DHT21,
	DHT22
} DHT_TypeDef;

typedef struct
{
	float temperature;
	float humidity;
} DHT_Values;

typedef enum
{
	READ_OK = 0U,
	EARLY_READ,
	TIMEOUT_LOW,
	TIMEOUT_HIGH,
	TIMEOUT_CYCLES,
	CHECKSUM_FAILED
} DHT_ReadStatus;

typedef struct
{
  uint8_t data[DATA_BYTE_LENGTH];      /* 8Bits word*/
  DHT_Values values;
  GPIO_TypeDef* DHT_GPIO;
  uint32_t DHT_PIN;
  DHT_TypeDef DHT_Type;
  uint32_t lastReadTime;
  DHT_ReadStatus status;

} DHT_HandleTypeDef;

void DHT_Sensor_Init(DHT_HandleTypeDef *hdht);

DHT_ReadStatus DHT_Sensor_Read(DHT_HandleTypeDef *hdht);

uint32_t waitForPulse(uint8_t level);

#endif /* INC_DHT_SENSOR_H_ */
