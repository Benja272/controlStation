#ifndef BSP_H_
#define BSP_H_

#include "stdint.h"

typedef enum
{
  LED_GREEN = 0,
  LED_ORANGE = 1,
  LED_RED = 2,
  LED_BLUE = 3
} Led_TypeDef;

typedef enum
{
  BUTTON_KEY = 0,
} Button_TypeDef;

typedef enum
 {
   BUTTON_MODE_GPIO = 0,
   BUTTON_MODE_EXTI = 1
 }ButtonMode_TypeDef;


void		BSP_Delay(uint32_t ms);
void 		BSP_Init(void);
void     	BSP_LED_On(Led_TypeDef Led);
void     	BSP_LED_Off(Led_TypeDef Led);
void     	BSP_LED_Toggle(Led_TypeDef Led);
uint32_t 	BSP_PB_GetState(Button_TypeDef Button);
uint32_t    BSP_LUZ_GetState(void);
float 		BSP_BOARD_GetTemp(void);
float       BSP_SUELO_GetHum(void);
void 		Error_Handler(void);
//dht11
void BSP_DHT11_Init(void);
uint8_t *BSP_DHT11_Read(void);
//tim
void BSP_TIM3_Init(void);
void BSP_TIM3_SetCounter(void);
uint32_t BSP_TIM3_GetCounter(void);
void BSP_TIM3_IncCounter(void);
//usart
void BSP_USART1_Init(void);

#endif /* BSP_H_ */
