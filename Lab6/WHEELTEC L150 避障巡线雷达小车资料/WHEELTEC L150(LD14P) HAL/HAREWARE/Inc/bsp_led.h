#ifndef __LED_H
#define	__LED_H



#include "Header.h"


#define LED1_OFF()  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET)
#define LED1_ON()   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET)
#define LED1_TOGGLE()  HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_2)


#define LED2_OFF  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET)
#define LED2_ON   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET)
#define LED2_TOGGLE  HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_3)


void LED_Flash(u16 count);
#endif /* __LED_H */
