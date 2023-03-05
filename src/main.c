#include "bsp.h"
#include "string.h"

extern uint8_t init_wifi;

void create_msg(float * sensor_inputs, char * msg){
	char temp[50];
	char inputs[4][6] = {"\0"};
	for (uint8_t i=0; i<4; i++){
		gcvt(sensor_inputs[i],5, &inputs[i][0]);
	}
	sprintf(msg, "Temperatura de la placa %s °C, ", &inputs[0]);
	sprintf(temp, "La Humedad del suelo es del %s %%, ", &inputs[1]);
	strcat(msg, temp);
	sprintf(temp, "La Temperatura Ambiente es de %s °C, ", &inputs[2]);
	strcat(msg, temp);
	sprintf(temp, "La Humedad Ambiente es del %s %%.", &inputs[3]);
	strcat(msg, temp);
}

int main(void)
{



	BSP_Init();
	uint8_t sended=0;
	uint8_t *dht11_measures;
	float 	 temperatura_board;
	float    temperatura_dht11;
	float 	 humedad_suelo;
	float    humedad_dht11;
	char msg[150];
	BSP_WIFI_Init();
	for(;;){
		//BSP_WIFI_Init();
		BSP_WIFI_connect();
		if (init_wifi == 0){
			BSP_LED_Toggle(LED_BLUE);
		}
		if (!BSP_LUZ_GetState()) {
			BSP_LED_Toggle(LED_BLUE);
			BSP_Delay(50);
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
		float sensor_inputs[] = {temperatura_board, humedad_suelo,temperatura_dht11, humedad_dht11};
		create_msg(sensor_inputs, msg);
		if(BSP_WIFI_status() && sended){
			sended=0;
			char msg[20];
			strcpy(msg, "ABCDEFGHIJKL");
			BSP_WIFI_send_msg(msg, strlen(msg));
		}
	}
}


