#include "sensors.h"
#include "pni.h"
#include <stdio.h>

static float ds18b20_latest_temperature = 85.0f;
static uint32_t ds18b20_convert_start = 0U;
static uint8_t ds18b20_conversion_pending = 0U;
static uint8_t ds18b20_valid = 0U;
static uint32_t ds18b20_last_good_tick = 0U;

#define SENSOR_UPLOAD_INTERVAL_MS 50U

static void DS18B20_UploadToRaspberryPi(float temperature)
{
  static char tx_buf[512];
  char temperature_text[16];
  int len;
  uint32_t now;
  PNI_OrientationTypeDef orientation;
  PNI_AccelTypeDef accel;
  PNI_StatusTypeDef pni_status;
  PNI_StatusTypeDef accel_status;
  uint8_t pni_valid;
  uint8_t accel_valid;

  if(huart2.gState != HAL_UART_STATE_READY) return;
  pni_valid = PNI_GetLatestOrientation(&orientation, &pni_status);
  accel_valid = PNI_GetLatestAccel(&accel, &accel_status);
  now = HAL_GetTick();
  if(ds18b20_valid != 0U && now - ds18b20_last_good_tick <= 2000U)
    snprintf(temperature_text, sizeof(temperature_text), "%.2f", temperature);
  else
    snprintf(temperature_text, sizeof(temperature_text), "null");

  len = snprintf(tx_buf, sizeof(tx_buf),
                 "{\"tick\":%lu,\"temperature\":%s,\"unit\":\"C\",\"pni\":{\"valid\":%s,\"heading\":%.2f,\"pitch\":%.2f,\"roll\":%.2f,\"status\":%u,\"update_tick\":%lu},\"accel\":{\"valid\":%s,\"x\":%.3f,\"y\":%.3f,\"z\":%.3f,\"status\":%u,\"update_tick\":%lu}}\r\n",
                 now,
                 temperature_text,
                 (pni_valid != 0U) ? "true" : "false",
                 (pni_valid != 0U) ? orientation.heading : 0.0f,
                 (pni_valid != 0U) ? orientation.pitch : 0.0f,
                 (pni_valid != 0U) ? orientation.roll : 0.0f,
                 (pni_valid != 0U) ? orientation.status : (uint8_t)pni_status,
                 (pni_valid != 0U) ? orientation.update_tick : 0UL,
                 (accel_valid != 0U) ? "true" : "false",
                 (accel_valid != 0U) ? accel.x : 0.0f,
                 (accel_valid != 0U) ? accel.y : 0.0f,
                 (accel_valid != 0U) ? accel.z : 0.0f,
                 (accel_valid != 0U) ? accel.status : (uint8_t)accel_status,
                 (accel_valid != 0U) ? accel.update_tick : 0UL);

  if(len <= 0)
  {
    return;
  }

  if(len >= (int)sizeof(tx_buf))
  {
    return;
  }

  (void)HAL_UART_Transmit_IT(&huart2, (uint8_t *)tx_buf, (uint16_t)len);
}

void delay_us(uint32_t us)
{
    /* TIM4: APB1 timer clock 84 MHz / (83 + 1) = 1 MHz. */
    while(us > 0U)
    {
      uint32_t ticks = (us > 60000U) ? 60000U : us;
      __HAL_TIM_SET_COUNTER(&htim4, 0);
      while(__HAL_TIM_GET_COUNTER(&htim4) < ticks) {}
      us -= ticks;
    }
}
void delay_ms(uint32_t ms)
{while(ms)
	{
delay_us(1000);
		ms--;
  }
}
// DS18B20 初始化
uint8_t DS18B20_Init(void)
{
  uint8_t ack;
  DQ_L;
  delay_us(480);
  DQ_H;
  delay_us(60);
  ack = DQ_RD;
  delay_us(420);
  return ack;
}

// 写一个字节
void DS18B20_WriteByte(uint8_t dat)
{
  uint8_t i;
  for(i=0; i<8; i++)
  {
    DQ_L;
    delay_us(2);
    if(dat & 0x01)
      DQ_H;
    else
      DQ_L;
    delay_us(60);
    DQ_H;
    delay_us(2);
    dat >>= 1;
  }
}

// 读一个字节
uint8_t DS18B20_ReadByte(void)
{
  uint8_t i, dat=0;
  for(i=0; i<8; i++)
  {
    DQ_L;
    delay_us(2);
    DQ_H;
    delay_us(10);
    dat >>= 1;
    if(DQ_RD) dat |= 0x80;
    delay_us(50);
  }
  return dat;
}

float DS18B20_GetTemp(void)
{
  return ds18b20_latest_temperature;
}

static void DS18B20_StartConvert(void)
{
  DS18B20_WriteByte(DS18B20_SKIP_ROM);
  DS18B20_WriteByte(DS18B20_CONVERT_T);
}

static float DS18B20_ReadConvertedTemp(float fallback_temperature)
{
  uint8_t scratch[9];
  uint8_t i, bit, crc = 0U;
  int16_t temp;

  ds18b20_valid = 0U;
  if(DS18B20_Init() != 0)
  {
    return fallback_temperature;
  }

  DS18B20_WriteByte(DS18B20_SKIP_ROM);
  DS18B20_WriteByte(DS18B20_READ_SCRATCHPAD);

  for(i = 0U; i < 9U; i++) scratch[i] = DS18B20_ReadByte();
  for(i = 0U; i < 8U; i++)
  {
    uint8_t value = scratch[i];
    for(bit = 0U; bit < 8U; bit++)
    {
      uint8_t mix = (uint8_t)((crc ^ value) & 1U);
      crc >>= 1U;
      if(mix != 0U) crc ^= 0x8CU;
      value >>= 1U;
    }
  }
  /* Reject a stuck-low bus even though an all-zero frame has zero CRC. */
  if(crc != scratch[8] || scratch[5] != 0xFFU || scratch[7] != 0x10U)
    return fallback_temperature;
  temp = (int16_t)((uint16_t)scratch[0] | ((uint16_t)scratch[1] << 8U));
  if(temp < -880 || temp > 2000) return fallback_temperature;
  ds18b20_valid = 1U;
  ds18b20_last_good_tick = HAL_GetTick();
  return (float)temp * 0.0625f;
}

void DS18B20_ReportStartup(void)
{
  if(DS18B20_Init() == 0)
  {
    printf("DS18B20 detected on PA0\r\n");
  }
  else
  {
    printf("DS18B20 not detected on PA0\r\n");
  }
}

void DS18B20_UpdateTask(void)
{
  static uint32_t next_probe = 0U;
  uint32_t now = HAL_GetTick();

  if(ds18b20_conversion_pending == 0U)
  {
    if((int32_t)(now - next_probe) < 0) return;
    if(DS18B20_Init() == 0)
    {
      DS18B20_StartConvert();
      ds18b20_convert_start = HAL_GetTick();
      ds18b20_conversion_pending = 1U;
    }
    else
    {
      ds18b20_valid = 0U;
      next_probe = now + 1000U;
    }
  }
  else if(now - ds18b20_convert_start >= 800U)
  {
    ds18b20_latest_temperature = DS18B20_ReadConvertedTemp(ds18b20_latest_temperature);
    ds18b20_conversion_pending = 0U;
    next_probe = HAL_GetTick();
  }
}

void Sensor_UploadTask(void)
{
  static uint32_t last_upload = 0U;
  uint32_t now = HAL_GetTick();

  if(now - last_upload >= SENSOR_UPLOAD_INTERVAL_MS && huart2.gState == HAL_UART_STATE_READY)
  {
    if(last_upload == 0U)
    {
      last_upload = now;
    }
    else
    {
      last_upload += SENSOR_UPLOAD_INTERVAL_MS;
      if(now - last_upload >= SENSOR_UPLOAD_INTERVAL_MS)
      {
        last_upload = now;
      }
    }
    DS18B20_UploadToRaspberryPi(ds18b20_latest_temperature);
  }
}
