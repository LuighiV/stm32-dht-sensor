/*
 * dht_sensor.c
 *
 *  Created on: Mar 27, 2020
 *      Author: luighi
 */

#include "main.h"
#include "dht_sensor.h"

#define MIN_DELAY_INTERVAL 2000 /**< minimum delay between reads */
#define WAIT_SIGNAL_US 55
#define TIMEOUT -1
#define MAX_WAIT_TIME_US 1000
/**
 * Reference: https://stackoverflow.com/a/60891394/5107192
 */
#define SINLINE static inline __attribute__((always_inline))

SINLINE void  DHT_CLK_ENABLE(GPIO_TypeDef * const gpio)
{
    switch((uint32_t)gpio)
    {
        case (uint32_t)GPIOA:
            __HAL_RCC_GPIOA_CLK_ENABLE();
            break;
        case (uint32_t)GPIOB:
            __HAL_RCC_GPIOB_CLK_ENABLE();
            break;
        case (uint32_t)GPIOC:
            __HAL_RCC_GPIOC_CLK_ENABLE();
            break;
        case (uint32_t)GPIOD:
			__HAL_RCC_GPIOD_CLK_ENABLE();
			break;
        case (uint32_t)GPIOE:
			__HAL_RCC_GPIOE_CLK_ENABLE();
			break;
        case (uint32_t)GPIOF:
            __HAL_RCC_GPIOF_CLK_ENABLE();
            break;
        case (uint32_t)GPIOG:
			__HAL_RCC_GPIOG_CLK_ENABLE();
			break;
        case (uint32_t)GPIOH:
			__HAL_RCC_GPIOH_CLK_ENABLE();
			break;
    }
}

float evalHumidity(uint8_t* data, uint8_t len, DHT_TypeDef type);
float evalTemperature(uint8_t* data, uint8_t len, DHT_TypeDef type);


void DHT_Sensor_Init(DHT_HandleTypeDef *hdht){

  GPIO_InitTypeDef  gpio_init_structure;

  DHT_CLK_ENABLE(DHT_GPIO_conf);

  gpio_init_structure.Pin   = DHT_PIN_conf;
  gpio_init_structure.Mode  = GPIO_MODE_OUTPUT_OD;
  gpio_init_structure.Pull  = GPIO_PULLUP;
  gpio_init_structure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  HAL_GPIO_Init(DHT_GPIO_conf, &gpio_init_structure);
  HAL_GPIO_WritePin(DHT_GPIO_conf, DHT_PIN_conf, GPIO_PIN_RESET);

  hdht->DHT_GPIO = DHT_GPIO_conf;
  hdht->DHT_PIN = DHT_PIN_conf;
  hdht->DHT_Type = DHT_Type_conf;
  hdht->lastReadTime = HAL_GetTick() - MIN_DELAY_INTERVAL;

}

DHT_ReadStatus DHT_Sensor_Read(DHT_HandleTypeDef *hdht){

	uint32_t currentTime = HAL_GetTick();
	if ((currentTime - hdht->lastReadTime) < MIN_DELAY_INTERVAL) {
		DEBUG_PRINT("Early read, time not passed, returned last read\n\r");
		hdht->status = EARLY_READ;
	    return hdht->status; // return last correct measurement
	}

	hdht->lastReadTime = currentTime;

	for(int i; i<DATA_BYTE_LENGTH;i++){
		hdht->data[i] = 0;
	}

	HAL_GPIO_WritePin(DHT_GPIO_conf, DHT_PIN_conf, GPIO_PIN_SET);
	HAL_Delay(1);

	HAL_GPIO_WritePin(DHT_GPIO_conf, DHT_PIN_conf, GPIO_PIN_RESET);

	switch (hdht->DHT_Type) {
	case DHT22:
	case DHT21:
		delay_us(1100); // data sheet says "at least 1ms"
		break;
	case DHT11:
	default:
		HAL_Delay(20); // data sheet says at least 18ms, 20ms just to be safe
	break;
	}

	uint32_t cycles[80];

	{
	    // End the start signal by setting data line high
		HAL_GPIO_WritePin(DHT_GPIO_conf, DHT_PIN_conf, GPIO_PIN_SET);

	    // Delay a moment to let sensor pull data line low.
	    delay_us(WAIT_SIGNAL_US);

	    // Now start reading the data line to get the value from the DHT sensor.

	    // First expect a low signal for ~80 microseconds followed by a high signal
	    // for ~80 microseconds again.
	    if (waitForPulse(0) == TIMEOUT) {
	      DEBUG_PRINT("DHT timeout waiting for start signal low pulse.\n\r");
	      hdht->status = TIMEOUT_LOW;
	      return hdht->status;
	    }
	    if (waitForPulse(1) == TIMEOUT) {
	      DEBUG_PRINT("DHT timeout waiting for start signal high pulse.\n\r");
	      hdht->status = TIMEOUT_HIGH;
	      return hdht->status;
	    }

	    // Now read the 40 bits sent by the sensor.  Each bit is sent as a 50
	    // microsecond low pulse followed by a variable length high pulse.  If the
	    // high pulse is ~28 microseconds then it's a 0 and if it's ~70 microseconds
	    // then it's a 1.  We measure the cycle count of the initial 50us low pulse
	    // and use that to compare to the cycle count of the high pulse to determine
	    // if the bit is a 0 (high state cycle count < low state cycle count), or a
	    // 1 (high state cycle count > low state cycle count). Note that for speed
	    // all the pulses are read into a array and then examined in a later step.
	    for (int i = 0; i < 80; i += 2) {
	      cycles[i] = waitForPulse(0);
	      cycles[i + 1] = waitForPulse(1);
	    }
	  } // Timing critical code is now complete.

	  // Inspect pulses and determine which ones are 0 (high state cycle count < low
	  // state cycle count), or 1 (high state cycle count > low state cycle count).
	  for (int i = 0; i < 40; ++i) {
	    uint32_t lowCycle = cycles[2 * i];
	    uint32_t highCycle = cycles[2 * i + 1];
	    if ((lowCycle == TIMEOUT) || (highCycle == TIMEOUT)) {
	      DEBUG_PRINT("DHT timeout waiting for pulse.\n\r");
	      hdht->status = TIMEOUT_CYCLES;
	      return hdht->status;
	    }

	    hdht->data[i / 8] <<= 1;
	    // Now compare the low and high cycle times to see if the bit is a 0 or 1.
	    if (highCycle > lowCycle) {
	      // High cycles are greater than 50us low cycle count, must be a 1.
	    	hdht->data[i / 8] |= 1;
	    }
	    // Else high cycles are less than (or equal to, a weird case) the 50us low
	    // cycle count so this must be a zero.  Nothing needs to be changed in the
	    // stored data.
	  }

	  uint8_t parity;
	  parity = (hdht->data[0] + hdht->data[1] + hdht->data[2] + hdht->data[3]) & 0xFF;
	  DEBUG_PRINT("Received from DHT:\n\r");
	  DEBUG_PRINTF("Hum: %x,%x\n\rTemp: %x,%x\n\rParity: %x =? %x\n\r",
			  hdht->data[0],
			  hdht->data[1],
			  hdht->data[2],
			  hdht->data[3],
			  hdht->data[4],
			  parity
	  	  	  );

	  if(hdht->data[4]!=parity){
		  DEBUG_PRINT("DHT checksum failure!\n\r");
		  hdht->status = CHECKSUM_FAILED;
		  return hdht->status;
	  }

	  /*Evaluating the humidity and temperature*/
	  DHT_Values dht_val;

	  dht_val.temperature = evalTemperature(hdht->data,countof(hdht->data),hdht->DHT_Type);
	  dht_val.humidity = evalHumidity(hdht->data,countof(hdht->data),hdht->DHT_Type);

	  DEBUG_PRINTF("Data converted:\n\rHumidity: %.1f\n\rTemperature: %.1f\n\r",
			  dht_val.humidity,
			  dht_val.temperature
	  );

	  hdht->values = dht_val;

	  hdht->status = READ_OK;
	  return hdht->status;
}

float evalHumidity(uint8_t* data, uint8_t len, DHT_TypeDef type)
{
	float f = NAN;
	uint8_t _data[len];
	memcpy(_data,data,len);

	if(len > 2){
		switch (type) {
		case DHT11:
		case DHT12:
		  f = _data[0] + _data[1] * 0.1;
		  break;
		case DHT22:
		case DHT21:
		  f = ((uint16_t)_data[0]) << 8 | _data[1];
		  f *= 0.1;
		  break;
		}
	}

	return f;
}


float evalTemperature(uint8_t* data, uint8_t len, DHT_TypeDef type)
{
	 float f = NAN;
	 uint8_t _data[len];
	 memcpy(_data,data,len);

	  if (len > 4) {
	    switch (type) {
	    case DHT11:
	      f = _data[2];
	      if (_data[3] & 0x80) {
	        f = -1 - f;
	      }
	      f += (_data[3] & 0x0f) * 0.1;

	      break;
	    case DHT12:
	      f = _data[2];
	      f += (_data[3] & 0x0f) * 0.1;
	      if (_data[2] & 0x80) {
	        f *= -1;
	      }

	      break;
	    case DHT22:
	    case DHT21:
	      f = ((uint16_t)(_data[2] & 0x7F)) << 8 | _data[3];
	      f *= 0.1;
	      if (_data[2] & 0x80) {
	        f *= -1;
	      }

	      break;
	    }
	  }
	  return f;
}



uint32_t waitForPulse(uint8_t level){

	resetTick_us();

	while (HAL_GPIO_ReadPin(DHT_GPIO_conf, DHT_PIN_conf) == level) {
	    if (getTick_us() >= MAX_WAIT_TIME_US) {
	      return TIMEOUT; // Exceeded timeout, fail.
	    }
	}
	return getTick_us();
}


