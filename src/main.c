#include "bsp.h"

int main(void)
{
	BSP_Init();
	uint8_t *dht11_measures;
	float 	 temperatura_board;
	float 	 humedad_suelo;
	float    temperatura_dht11;
	float    humedad_dht11;

	for(;;){
		if (!BSP_LUZ_GetState()) {
			BSP_LED_Toggle(LED_BLUE);
			BSP_Delay(200);
		}

		if (BSP_PB_GetState(BUTTON_KEY)){
			BSP_LED_Toggle(LED_GREEN);
			BSP_Delay(200);
		}

		temperatura_board = BSP_BOARD_GetTemp();
		humedad_suelo     = BSP_SUELO_GetHum();
		dht11_measures    = BSP_DHT11_Read();
		temperatura_dht11 = dht11_measures[0];
		humedad_dht11     = dht11_measures[1];

	}
}



