#ifndef BSP_H_
#define BSP_H_

#include "stdint.h"

/* LEDS */
typedef enum
{
  LED_GREEN  = 0,
  LED_ORANGE = 1,
  LED_RED    = 2,
  LED_BLUE   = 3
} Led_TypeDef;

/* USER BUTTON */
typedef enum
{
  BUTTON_KEY = 0,
} Button_TypeDef;


float 		BSP_BOARD_GetTemp(void);
void		BSP_Delay(uint32_t ms);
uint8_t*	BSP_DHT11_Read(void);
void 		BSP_Init(void);
void     	BSP_LED_On(Led_TypeDef Led);
void     	BSP_LED_Off(Led_TypeDef Led);
void     	BSP_LED_Toggle(Led_TypeDef Led);
uint32_t    BSP_LUZ_GetState(void);
uint32_t 	BSP_PB_GetState(Button_TypeDef Button);
uint32_t    BSP_SUELO_GetHum(void);
void 		BSP_WIFI_Init(void);

#endif /* BSP_H_ */
