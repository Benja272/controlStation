/* Includes ------------------------------------------------------------------*/
#include "stm32f411e_discovery.h"
#include "mk_dht11.h"
#include "bsp.h"

/* Estructuras que facilitan el manejo de los LEDS */

GPIO_TypeDef* GPIO_PORT[LEDn] = {LED4_GPIO_PORT, 
                                 LED3_GPIO_PORT, 
                                 LED5_GPIO_PORT,
                                 LED6_GPIO_PORT};

const uint16_t GPIO_PIN[LEDn] = {LED4_PIN, 
                                 LED3_PIN, 
                                 LED5_PIN,
                                 LED6_PIN};






/* Facilitan el uso de los botones de la board */

typedef enum
 {
   BUTTON_MODE_GPIO = 0,
   BUTTON_MODE_EXTI = 1
 }ButtonMode_TypeDef;

GPIO_TypeDef* 	BUTTON_PORT[BUTTONn] = {KEY_BUTTON_GPIO_PORT};
const uint16_t 	BUTTON_PIN[BUTTONn]  = {KEY_BUTTON_PIN};
const uint8_t 	BUTTON_IRQn[BUTTONn] = {KEY_BUTTON_EXTI_IRQn};



/* Definiciones del modulo */
void 		SystemClock_Config(void);
void 		Error_Handler(void);
void    	BSP_LUZ_Init(void);
void    	ADC1_Init(void);
void 		BSP_LED_Init(Led_TypeDef Led);
void 		BSP_DHT11_Init(void);
void 		BSP_TIM3_Init(void);
void 		BSP_USART1_Init(void);
void 		BSP_PB_Init(Button_TypeDef 	   Button,
						ButtonMode_TypeDef ButtonMode);


/* Handlers necesarios */
ADC_HandleTypeDef 	hadc1;
TIM_HandleTypeDef 	htim3;
UART_HandleTypeDef 	huart1;
dht11_t 			dht;
uint32_t 			counter = -1;


/******************************************************************************
 * 				     	     MANIPULACION DE LEDS 					      	  *
 *****************************************************************************/

/**
  * @brief  Enciende el LED seleccionado.
  * @param  Led: Especifica el Led a encender.
  *   Este parametro debe ser alguno de la siguiente lista:
  *     @arg LED_GREEN
  *     @arg LED_ORANGE
  *     @arg LED_RED
  *     @arg LED_BLUE
  */
void BSP_LED_On(Led_TypeDef Led)
{
  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_SET); 
}

/**
  * @brief  Apaga el LED seleccionado.
  * @param  Led: Especifica el LED a apagar.
  *   Este parametro debe ser alguno de la siguiente lista:
  *     @arg LED_GREEN
  *     @arg LED_ORANGE
  *     @arg LED_RED
  *     @arg LED_BLUE
  */
void BSP_LED_Off(Led_TypeDef Led)
{
  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET); 
}


/**
  * @brief  'Togglea' el LED seleccionado.
  * @param  Led: Especifica el LED a 'togglear'.
  *   Este parametro debe ser alguno de la siguietne lista:
  *     @arg LED_GREEN
  *     @arg LED_ORANGE
  *     @arg LED_RED
  *     @arg LED_BLUE
  */
void BSP_LED_Toggle(Led_TypeDef Led)
{
  HAL_GPIO_TogglePin(GPIO_PORT[Led], GPIO_PIN[Led]);
}



/******************************************************************************
 * 				     	   MANIPULACION DE SENSORES 					      *
 *****************************************************************************/

/**
 * @brief	Obtiene una lectura del sensor de temperatura de la placa
 * @retval	Temp: Temperatura en Celsius de la placas
 */
float BSP_BOARD_GetTemp(void){
	ADC_ChannelConfTypeDef sConfig = {0};
	uint32_t ADCValue;
	float Vsense, Temp;

	sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
	sConfig.Rank    = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
			Error_Handler();
	}

	HAL_ADC_Start(&hadc1);

	if(HAL_ADC_PollForConversion(&hadc1, 100) != HAL_OK){
		return 0;
	}

	ADCValue = HAL_ADC_GetValue(&hadc1);
	Vsense   = (float)ADCValue * 3000 / ((1<<12) - 1);
	Temp     = (Vsense - 760) / 2.5 + 25;
	return Temp;
}

/**
 * @brief	Obtiene una lectura del sensor de humedad del suelo,
 * 			este sensor debe conectarse al pin PA1 (ADC_CHANNEL_1)
 * @retval	Hum: Devuelve la humedad del suelo medida.
 */
float BSP_SUELO_GetHum(void){
	ADC_ChannelConfTypeDef sConfig = {0};
	float ADCValue, Hum;
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank    = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;

	if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK){
		Error_Handler();
	}

	HAL_ADC_Start(&hadc1);
	if(HAL_ADC_PollForConversion(&hadc1, 100) != HAL_OK){
			return 0;
	}
	ADCValue = HAL_ADC_GetValue(&hadc1);
	Hum = 1 - ADCValue/4095;
	return Hum;
}


uint8_t res[2];
/**
 * @brief	Obtiene una lectura del sensor DHT11.
 * @retval	res[0]: Temperatura medida con el sensor.
 * @retval  res[1]: Humedad del ambiente medida con el sensor.
 */
uint8_t *BSP_DHT11_Read(){
	readDHT11(&dht);
	res[0] = dht.temperature;
	res[1] = dht.humidty;
	return res;
}

void BSP_TIM3_Init(){
	TIM3_Init();
	__HAL_TIM_SET_COUNTER(&htim3, 0);
	HAL_TIM_Base_Start_IT(&htim3);
}

void BSP_TIM3_SetCounter(){
	counter = 0;
}

uint32_t BSP_TIM3_GetCounter(){
	return counter;
}

void BSP_TIM3_IncCounter(){
	if(counter >= 0){
		counter++;
	}
}



/******************************************************************************
 * 				     	FUNCIONES DE MISCELANEAS   					          *
 *****************************************************************************/

/**
  * @brief  Devuelve el estado del botón seleccionado.
  * @param  Button: Especifica el botón cuyo estado es de interes.
  *   Este parametro debe ser: BUTTON_KEY
  * @retval El estado del boton.
  */
uint32_t BSP_PB_GetState(Button_TypeDef Button)
{
  return HAL_GPIO_ReadPin(BUTTON_PORT[Button], BUTTON_PIN[Button]);
}


/**
 * @brief	Delay bloqueante
 * @param	ms: Indica la cantidad en ms del delay
 */
void BSP_Delay(uint32_t ms){
	HAL_Delay(ms);
}


/******************************************************************************
 * 				     	FUNCIONES DE INICIALIZACION 					      *
 *****************************************************************************/

/* Inicializacion de la placa */
void BSP_Init(){
	/* Inicializacion de la libreria HAL */
	HAL_Init();
	/* Configuracion de los clocks */
	SystemClock_Config();

	/* Inicializacion de los LEDS */
	BSP_LED_Init(LED_RED);
	BSP_LED_Init(LED_GREEN);
	BSP_LED_Init(LED_ORANGE);
	BSP_LED_Init(LED_BLUE);

	/* Inicializamos el sensor de luz */
	BSP_LUZ_Init();
	/* Inicializamos el conversor ADC */
	ADC1_Init();
	/* Inicializamos el timer 3 */
	BSP_TIM3_Init();
	/* Inicializamos usart */
	BSP_USART1_Init();
	/* Inicializamos el sensor de temperatura y humedad DHT11 */
	BSP_DHT11_Init();

	BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_GPIO);
}

void BSP_DHT11_Init(){
	init_dht11(&dht, &htim3, DHT11_USART_PORT, DHT11_USART_Tx_PIN);
}

void BSP_LUZ_Init(){
	/* Inicializamos el clock del puerto del sensor */
	 __HAL_RCC_GPIOC_CLK_ENABLE();
	 __HAL_RCC_GPIOA_CLK_ENABLE();
	 GPIO_InitTypeDef GPIO_InitStruct = {0};
	 /* Configuracion GPIO del sensor */
	 GPIO_InitStruct.Pin = SENSOR_LUZ_PIN;
	 GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	 GPIO_InitStruct.Pull = GPIO_NOPULL;
	 HAL_GPIO_Init(SENSOR_LUZ_PORT, &GPIO_InitStruct);
}


/**
  * @brief  Configures LED GPIO.
  * @param  Led: Specifies the Led to be configured.
  *   This parameter can be one of following parameters:
  *     @arg LED_GREEN
  *     @arg LED_ORANGE
  *     @arg LED_RED
  *     @arg LED_BLUE
  */
void BSP_LED_Init(Led_TypeDef Led)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  /* Enable the GPIO_LED Clock */
  LEDx_GPIO_CLK_ENABLE(Led);

  /* Configure the GPIO_LED pin */
  GPIO_InitStruct.Pin = GPIO_PIN[Led];
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;

  HAL_GPIO_Init(GPIO_PORT[Led], &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET);
}


/**
  * @brief  Configures Button GPIO and EXTI Line.
  * @param  Button: Specifies the Button to be configured.
  *   This parameter should be: BUTTON_KEY
  * @param  ButtonMode: Specifies Button mode.
  *   This parameter can be one of following parameters:
  *     @arg BUTTON_MODE_GPIO: Button will be used as simple IO
  *     @arg BUTTON_MODE_EXTI: Button will be connected to EXTI line with interrupt
  *                            generation capability
  */
void BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* Enable the BUTTON Clock */
  BUTTONx_GPIO_CLK_ENABLE(Button);

  if(ButtonMode == BUTTON_MODE_GPIO)
  {
    /* Configure Button pin as input */
    GPIO_InitStruct.Pin = BUTTON_PIN[Button];
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    HAL_GPIO_Init(BUTTON_PORT[Button], &GPIO_InitStruct);
  }

  if(ButtonMode == BUTTON_MODE_EXTI)
  {
    /* Configure Button pin as input with External interrupt */
    GPIO_InitStruct.Pin = BUTTON_PIN[Button];
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    HAL_GPIO_Init(BUTTON_PORT[Button], &GPIO_InitStruct);

    /* Enable and set Button EXTI Interrupt to the lowest priority */
    HAL_NVIC_SetPriority((IRQn_Type)(BUTTON_IRQn[Button]), 0x0F, 0x00);
    HAL_NVIC_EnableIRQ((IRQn_Type)(BUTTON_IRQn[Button]));
  }
}

void ADC1_Init(){
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}
}


void BSP_TIM3_Init(){
	  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
	  TIM_MasterConfigTypeDef sMasterConfig = {0};

	  htim3.Instance = TIM3;
	  htim3.Init.Prescaler = 48;
	  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	  htim3.Init.Period = 65535;
	  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
	  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
	  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  HAL_TIM_Base_MspInit(&htim3);
	  HAL_TIM_Base_Start_IT(&htim3);
}


void BSP_USART1_Init(){
	  huart1.Instance = USART1;
	  huart1.Init.BaudRate = 115200;
	  huart1.Init.WordLength = UART_WORDLENGTH_8B;
	  huart1.Init.StopBits = UART_STOPBITS_1;
	  huart1.Init.Parity = UART_PARITY_NONE;
	  huart1.Init.Mode = UART_MODE_TX_RX;
	  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	  if (HAL_UART_Init(&huart1) != HAL_OK)
	  {
	    Error_Handler();
	  }
}


/******************************************************************************
 * 				    FUNCIONES DE INICIALIZACION (MSP) 					      *
 *****************************************************************************/


void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(adcHandle->Instance==ADC1) {
    /* ADC1 clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /*
     * ADC1 GPIO Configuration
     * PA1 ------> ADC1_IN1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
}


void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle){
  if(adcHandle->Instance==ADC1){
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();
    /*
     * ADC1 GPIO Configuration
     * PA1 ------> ADC1_IN1
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1);
  }
}


void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{
  if(tim_baseHandle->Instance==TIM3)
  {
    /* TIM3 clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();

    /* TIM3 interrupt Init */
    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{
  if(tim_baseHandle->Instance==TIM3)
  {
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();

    /* TIM3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM3_IRQn);
  }
}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /*
    USART1 GPIO Configuration
    PA15     ------> USART1_TX
    PB7     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle) {
  if(uartHandle->Instance==USART1) {
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();
    /*
    USART1 GPIO Configuration
    PA15     ------> USART1_TX
    PB7     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_15);
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);
  }
}


/******************************************************************************
 * 				  Configuracion de clocks y error handler 					  *
 *****************************************************************************/

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ =  8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
    Error_Handler();
  }
}

void Error_Handler()
{
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
}
