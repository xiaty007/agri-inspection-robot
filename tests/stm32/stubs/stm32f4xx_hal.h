#ifndef TEST_HAL_H
#define TEST_HAL_H
#include <stdint.h>
#include <stddef.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { int unused; } I2C_HandleTypeDef;
typedef struct { unsigned gState; } UART_HandleTypeDef;
typedef struct { uint32_t counter; } TIM_HandleTypeDef;
typedef struct { unsigned Pin, Mode, Pull, Speed; } GPIO_InitTypeDef;
#define HAL_UART_STATE_READY 0U
#define HAL_UART_STATE_BUSY_TX 1U
#define GPIO_PIN_0 1U
#define GPIO_PIN_6 64U
#define GPIO_PIN_7 128U
#define GPIO_PIN_RESET 0U
#define GPIO_PIN_SET 1U
#define GPIO_MODE_INPUT 0U
#define GPIO_MODE_OUTPUT_OD 1U
#define GPIO_PULLUP 1U
#define GPIO_SPEED_FREQ_VERY_HIGH 3U
#define GPIOA ((void *)1)
#define GPIOB ((void *)2)
#define I2C_MEMADD_SIZE_8BIT 1U
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
#define __HAL_TIM_SET_COUNTER(t, x) ((t)->counter = (x))
#define __HAL_TIM_GET_COUNTER(t) ((t)->counter++)
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim4;
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t ms);
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *, uint16_t, unsigned, unsigned);
void HAL_I2C_DeInit(I2C_HandleTypeDef *);
unsigned HAL_I2C_GetState(I2C_HandleTypeDef *);
unsigned long HAL_I2C_GetError(I2C_HandleTypeDef *);
void HAL_GPIO_Init(void *, GPIO_InitTypeDef *);
unsigned HAL_GPIO_ReadPin(void *, unsigned);
void HAL_GPIO_WritePin(void *, unsigned, unsigned);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
void MX_I2C1_Init(void);
void delay_us(uint32_t);
#endif
