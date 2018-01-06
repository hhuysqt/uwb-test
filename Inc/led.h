/*
 * LED trigger functions
 */
#ifndef MY_LED_H
#define MY_LED_H

#include <main.h>
#include <stm32f0xx_hal.h>

#define LED_ON(NR)  HAL_GPIO_WritePin(LED##NR##_GPIO_Port, LED##NR##_Pin, 1)
#define LED_OFF(NR) HAL_GPIO_WritePin(LED##NR##_GPIO_Port, LED##NR##_Pin, 0)

#define IS_LED_ON(NR) (HAL_GPIO_ReadPin(LED##NR##_GPIO_Port, LED##NR##_Pin) == GPIO_PIN_SET)

#define LED_TUGGLE(NR)\
        if (IS_LED_ON(NR)) LED_OFF(NR); \
        else LED_ON(NR);

#endif

