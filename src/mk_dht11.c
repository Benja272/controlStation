/**
7 *  @file mk_dht11.c
 *	@brief DHT11 Library
 *  @date Created on: Oct 4, 2019
 *  @author Author: mesut.kilic
 *	@version 1.0.0
 */

#include "mk_dht11.h"

/**
 * @brief configure dht11 struct with given parameter
 * @param htim TIMER for calculate delays ex:&htim2
 * @param port GPIO port ex:GPIOA
 * @param pin GPIO pin ex:GPIO_PIN_2
 * @param dht struct to configure ex:&dht
 */
void init_dht11(dht11_t *dht, TIM_HandleTypeDef *htim, GPIO_TypeDef* port, uint16_t pin){
	dht->htim = htim;
	dht->port = port;
	dht->pin = pin;
}

/**
 * @brief set DHT pin direction with given parameter
 * @param dht struct for dht
 * @param pMode GPIO Mode ex:INPUT or OUTPUT
 */
void set_dht11_gpio_mode(dht11_t *dht, uint8_t pMode)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	if(pMode == OUTPUT)
	{
	  GPIO_InitStruct.Pin = dht->pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	  HAL_GPIO_Init(dht->port, &GPIO_InitStruct);
	}else if(pMode == INPUT)
	{
	  GPIO_InitStruct.Pin = dht->pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	  HAL_GPIO_Init(dht->port, &GPIO_InitStruct);
	}
}

/**
 * @brief reads dht11 value
 * @param dht struct for dht11
 * @return 1 if read ok 0 if something wrong in read
 */
uint8_t control500ms(dht11_t *dht, uint8_t compare){
	BSP_TIM3_SetCounter();
	//check dht answer
	while(HAL_GPIO_ReadPin(dht->port, dht->pin) == compare){
		uint32_t a = BSP_TIM3_GetCounter();
		if(a > 500){
			return 1;
		}
	}
	return 0;
}

uint8_t readDHT11(dht11_t *dht)
{
	uint32_t mTime1 = 0, mTime2 = 0, mBit = 0;
	uint8_t humVal = 0, tempVal = 0, parityVal = 0, genParity = 0;
	uint8_t mData[40];
	uint32_t mtimes[40];
	//start comm
	set_dht11_gpio_mode(dht, OUTPUT);			//set pin direction as input
	HAL_GPIO_WritePin(dht->port, dht->pin, GPIO_PIN_RESET);
	HAL_Delay(18);					//wait 18 ms in Low state
	//__disable_irq();	//disable all interupts to do only read dht otherwise miss timer
	set_dht11_gpio_mode(dht, INPUT);
	//check dht answer
	if(control500ms(dht, GPIO_PIN_SET)) return 0;
	if(control500ms(dht, GPIO_PIN_RESET)) return 0;
	mTime1 = BSP_TIM3_GetCounter();
	if(control500ms(dht, GPIO_PIN_SET)) return 0;
	mTime2 =  BSP_TIM3_GetCounter();

	//if answer is wrong return
	if(mTime1 < 75 && mTime1 > 85 && mTime2 < 75 && mTime2 > 85)
	{
		return 0;
	}

//	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	for(int j = 0; j < 40; j++)
	{
		if(control500ms(dht, GPIO_PIN_RESET)) return 0;
		if(control500ms(dht, GPIO_PIN_SET)) return 0;
		mTime1 = BSP_TIM3_GetCounter();

		//check pass time in high state
		//if pass time 25uS set as LOW
		if(mTime1 < 9)
		{
			mBit = 0;
		}
		else if(mTime1 > 12 && mTime1 < 25) //if pass time 70 uS set as HIGH
		{
			 mBit = 1;
		}
		mtimes[j] = mTime1;
		//set i th data in data buffer
		mData[j] = mBit;

	}

	//get hum value from data buffer
	for(int i = 0; i < 8; i++)
	{
		humVal += mData[i];
		humVal = humVal << 1;
	}

	//get temp value from data buffer
	for(int i = 16; i < 24; i++)
	{
		tempVal += mData[i];
		tempVal = tempVal << 1;
	}

	//get parity value from data buffer
	for(int i = 32; i < 40; i++)
	{
		parityVal += mData[i];
		parityVal = parityVal << 1;
	}

	parityVal = parityVal >> 1;
	humVal = humVal >> 1;
	tempVal = tempVal >> 1;

	genParity = humVal + tempVal;

//	if(genParity == parityVal)

	dht->temperature = tempVal;
	dht->humidty = humVal;


	return 1;
}
