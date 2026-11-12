#ifndef __UTIL_H__
#define __UTIL_H__

#include <stdint.h>
#include "main.h"



#if TARGET_BOARD == BOARD_STM32_NUCELO_F411
#define LED1_GPIO_PORT                  GPIOA
#define LED1_GPIO_PIN                   GPIO_PIN_5

#define DBG_LED_OFF()       HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_GPIO_PIN, GPIO_PIN_RESET)
#define DBG_LED_ON()        HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_GPIO_PIN, GPIO_PIN_SET)
#define DBG_LED_TOGGLE()    HAL_GPIO_TogglePin(LED1_GPIO_PORT, LED1_GPIO_PIN)


#define BTN1_READ()         HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)

#define BTN1_DOWN()         ( HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET?1:0 )
#define DBG_USE_CALDATA()   ( HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == GPIO_PIN_SET?1:0 )           // H is active



#else
#define LED1_GPIO_PORT                  GPIOB
#define LED1_GPIO_PIN                   GPIO_PIN_5

#endif

void Clock_Wait(uint32_t i);
void hal_delay(uint32_t i);
void hexdump(char *ptitle, uint8_t *pdata, uint32_t len);
void led_blink(uint8_t cnt);


int host_eeprom_write(uint32_t offset, void *pdata, uint16_t len);
int host_eeprom_read(uint32_t offset, void *pdata, uint16_t len);

#endif


