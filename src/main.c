#include "bsp.h"
#include "string.h"
#include "FreeRTOS.h"

/* Variables de sensores */
volatile uint8_t *dht11_measures;
volatile float 	  temperatura_board;
volatile float    temperatura_dht11;
volatile float 	  humedad_suelo;
volatile float    humedad_dht11;
volatile float 	  sensor_inputs[4];

/* Semaforos */
SemaphoreHandle_t semaforo_1  = xSemaphoreCreateBinary();
SemaphoreHandle_t semaforo_it = xSemaphoreCreateBinary();

/**************************************** TASKS ****************************************/

void vConectarWifi( void *pvParameters ){
	for(;;)
	{
		/* Conectamos el modulo wifi a la red */
		BSP_WIFI_connect();
		/* Esperamos a que se conecte el modulo */
		vTaskDelay(pdMS_TO_TICKS(1000));
		break;
	}
	/* Borramos la task ya que cumplio su proposito */
	vTaskDelete(NULL);
}

void vLeerDatos( void *pvParameters ){
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t xPeriod = pdMS_TO_TICKS( 30*1000 );

	for(;;)
	{
		temperatura_board = BSP_BOARD_GetTemp();
		humedad_suelo     = BSP_SUELO_GetHum();
		dht11_measures    = BSP_DHT11_Read();
		temperatura_dht11 = dht11_measures[0];
		humedad_dht11     = dht11_measures[1];

		/* Permitimos al modulo wifi enviar los datos */
		xSemaphoreGive(semaforo_1);

		/* Establecemos periodicidad */
		vTaskDelayUntil(&xLastWakeTime, xPeriod);

	}
	vTaskDelete(NULL);
}

void vEnviarDatos( void *pvParameters )
{
	float sensor_inputs[4];
	char msg[150];
	for(;;){
		/* Tomamos el semaforo cedido en la otra task */
		xSemaphoreTake(semaforo_1, portMAX_DELAY);

		/* Creamos el mensaje */
		sensor_inputs[0] = temperatura_board;
		sensor_inputs[1] = humedad_suelo;
		sensor_inputs[2] = temperatura_dht11;
		sensor_inputs[3] = humedad_dht11;
		create_msg(sensor_inputs, msg);

		/* Enviamos el mensaje */
		BSP_WIFI_send_msg(msg, strlen(msg));
	}
	vTaskDelete(NULL);
}

/**************************************** AUXILIARES ****************************************/

void create_msg(float * sensor_inputs, char * msg){
	char temp[50];
	char inputs[4][6] = {"\0"};
	for (uint8_t i=0; i<4; i++){
		gcvt(sensor_inputs[i],5, &inputs[i][0]);
	}
	sprintf(msg, "Temperatura de la placa %s �C, ", &inputs[0]);
	sprintf(temp, "La Humedad del suelo es del %s %%, ", &inputs[1]);
	strcat(msg, temp);
	sprintf(temp, "La Temperatura Ambiente es de %s �C, ", &inputs[2]);
	strcat(msg, temp);
	sprintf(temp, "La Humedad Ambiente es del %s %%.", &inputs[3]);
	strcat(msg, temp);
}


/*******************************************************************************************/

int main(void)
{
	/*Inicializacion de los recursos*/
	BSP_Init();

	/* Instanciacion de tasks */
	xTaskCreate(vConectarWifi, "Task conectar wifi"   , 500, NULL, 1, NULL);
	XTaskCreate(vLeerDatos   , "Task lectura de datos", 500, NULL, 1, NULL);
	xTaskCraete(vEnviarDatos , "Task envio de datos"  , 500, NULL, 2, NULL);

	for(;;){

	}
}

/*

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

*/
