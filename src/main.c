#include "bsp.h"

/* Borrar luego */
	#include "stm32f411e_discovery.h"
	#include "stdio.h"
	void HAL_UART_RxCpltCallback ( UART_HandleTypeDef *huart);
	uint8_t rx_buffer[100], rx_data;
	uint8_t counter = 0;
	extern UART_HandleTypeDef huart2;
/* Borrar luego */



int main(void)
{
	BSP_Init();
	uint8_t *dht11_measures;
	float 	 temperatura_board;
	float    temperatura_dht11;
	float 	 humedad_suelo;
	float    humedad_dht11;

	/* Borrar luego */
	uint8_t command[4];
	sprintf((char *)command, "AT\r\n");
	HAL_UART_Receive_IT(&huart2, &rx_data, 1);
	HAL_UART_Transmit(&huart2, command, 4, 100);
	/* Borrar luego */

	for(;;){
		if (!BSP_LUZ_GetState()) {
			BSP_LED_Toggle(LED_BLUE);
			//BSP_Delay(200);
		}

		if (BSP_PB_GetState(BUTTON_KEY)){
			BSP_LED_Toggle(LED_GREEN);
			//BSP_Delay(200);
		}
		/* Borrar luego */
		if (counter == 1){
			BSP_LED_On(LED_RED);
			HAL_Delay(10);
		}
		/* Borrar luego */
		temperatura_board = BSP_BOARD_GetTemp();
		humedad_suelo     = BSP_SUELO_GetHum();
		dht11_measures    = BSP_DHT11_Read();
		temperatura_dht11 = dht11_measures[0];
		humedad_dht11     = dht11_measures[1];
	}
}

/* Borrar luego */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART2){
		uint8_t i;
		for(i=99; i>0; i--){
			rx_buffer[i] = rx_buffer[i-1];
		}
		rx_buffer[0] = rx_data;
		if(rx_data == 75){
			counter++;
		}
		HAL_UART_Receive_IT(&huart2, &rx_data, 1);
	}
}
/* Borrar luego */
