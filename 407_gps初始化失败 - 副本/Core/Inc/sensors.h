#include "stm32f4xx.h"                  // Device header
#include "usart.h"
#include "tim.h"
// DS18B20 引脚 PA0
#define DQ_H   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET)
#define DQ_L   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET)
#define DQ_RD  HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)

// DS18B20 指令
#define DS18B20_SKIP_ROM          0xCC
#define DS18B20_CONVERT_T         0x44
#define DS18B20_READ_SCRATCHPAD   0xBE
void delay_us(uint32_t us);
uint8_t DS18B20_Init(void);
void DS18B20_WriteByte(uint8_t dat);
uint8_t DS18B20_ReadByte(void);
float DS18B20_GetTemp(void);
void DS18B20_ReportStartup(void);
void DS18B20_UpdateTask(void);
void Sensor_UploadTask(void);
void delay_ms(uint32_t ms);
