#include "bsp.h"

// extern TIM_HandleTypeDef htim3;

int main(void)
{
	/*HAL_Init();
	SystemClock_Config();
	MX_USART1_UART_Init();
	MX_TIM3_Init();
	dht11_t dht;
	init_dht11(&dht, &htim3, GPIOA, GPIO_PIN_15);
	readDHT11(&dht);
	uint8_t i = 0;
	i++;
	BSP_LED_Init(LED4);
	*/
	BSP_Init();
	uint32_t estado_luz;
	float temp;
	float humedad;
	for(;;){
		estado_luz = BSP_LUZ_GetState();
		if (estado_luz) {
			BSP_LED_Toggle(LED_BLUE);
			BSP_Delay(500);
		}
		temp    = BSP_BOARD_GetTemp();
		humedad = BSP_SUELO_GetHum();
	}
}



