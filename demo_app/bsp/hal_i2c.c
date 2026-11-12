#include "hal_i2c.h"
#include "main.h"
#include "stm32f4xx_hal_i2c.h"


#define I2C_WAIT_TIMEOUT                 1000


I2C_HandleTypeDef hi2c1;


static void _hal_i2c_gpio_init(void)
{
    // PB8 - SCL 
    // PB9 -SDA
    GPIO_InitTypeDef GPIO_InitStruct;
  
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE(); 

    GPIO_InitStruct.Pin       = GPIO_PIN_8;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
      
    /* I2C RX GPIO pin configuration  */
    GPIO_InitStruct.Pin       = GPIO_PIN_9;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}



int8_t hal_i2c_init(void)
{
    _hal_i2c_gpio_init();

    hi2c1.Instance              = I2C1;
    hi2c1.Init.ClockSpeed       = 400*1000;
    hi2c1.Init.DutyCycle        = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1      = 0x00;
    hi2c1.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2      = 0x00;
    hi2c1.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;  
  
    if(HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
    /* Initialization Error */
    return 1;
    }
  
    
    return 0;
}


//handle = ch
int  hal_i2c_write(int handle,uint8_t saddr, uint16_t reg, uint8_t reg_sz,  uint8_t *pdat, uint16_t dat_sz )
{
    // force i2c1
    reg_sz = (reg_sz == 1) ? I2C_MEMADD_SIZE_8BIT:I2C_MEMADD_SIZE_16BIT;
    
    return HAL_I2C_Mem_Write(&hi2c1, saddr, reg, reg_sz, pdat, dat_sz, I2C_WAIT_TIMEOUT);

}


int hal_i2c_read(int handle, uint8_t saddr, uint16_t reg, uint8_t reg_sz, uint8_t * pdat, uint16_t dat_sz)
{
    reg_sz = (reg_sz == 1) ? I2C_MEMADD_SIZE_8BIT:I2C_MEMADD_SIZE_16BIT;
    
    return HAL_I2C_Mem_Read(&hi2c1, saddr, reg, reg_sz, pdat, dat_sz, I2C_WAIT_TIMEOUT);
}
