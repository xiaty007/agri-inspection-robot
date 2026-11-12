#ifndef __BOARD_CONFIG_H__
#define __BOARD_CONFIG_H__


#include "stm32f4xx_hal.h"
#include "util.h"

#define     BOARD_STM32_NUCELO_F411         1
#define     BOARD_PNI_13860                 2
#define     BOARD_STM32_L073                3


#define TARGET_BOARD                        BOARD_STM32_NUCELO_F411  //BOARD_PNI_13860


#if TARGET_BOARD == BOARD_PNI_13860
#define set_testpin1(x)     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, x); // low red
#define set_testpin2(x)     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, x); // low green
#define set_testpin3(x)     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, x); // low blue


#elif TARGET_BOARD == BOARD_STM32_NUCELO_F411
#define set_testpin1(x)     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, x);

#define USE_HOST_EXT_EEPROM

#endif  //TARGET_BOARD


#endif // _DEF
