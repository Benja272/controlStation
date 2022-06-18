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
	for(;;){
		BSP_LED_Toggle(LED_BLUE);
		BSP_Delay(500);
	}
}



