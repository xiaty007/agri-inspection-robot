#ifndef __HAL_I2C_H__
#define __HAL_I2C_H__

#include "main.h"
#include "stm32f4xx_hal_i2c.h"
#include <stdint.h>

extern I2C_HandleTypeDef hi2c1;

int8_t hal_i2c_init(void);


int hal_i2c_write(int handle,uint8_t saddr, uint16_t reg, uint8_t reg_sz,  uint8_t *pdat, uint16_t dat_sz );

int hal_i2c_read(int handle, uint8_t saddr, uint16_t reg, uint8_t reg_sz, uint8_t * pdat, uint16_t dat_sz);

#endif
