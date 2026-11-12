#include "pni.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#define PNI_REG_DATA_TRANSFER_AREA        0x00U
#define PNI_REG_CHIP_CONTROL              0x34U
#define PNI_REG_INT_STATUS                0x36U
#define PNI_REG_CHIP_STATUS               0x37U
#define PNI_REG_BYTES_REMAINING           0x38U
#define PNI_REG_ERROR_STATUS              0x50U
#define PNI_REG_HOST_INTERFACE_CONTROL    0x55U
#define PNI_REG_PARAM_ACK                 0x3AU
#define PNI_REG_PARAM_SAVE                0x3BU
#define PNI_REG_PARAM_PAGE_SELECT         0x54U
#define PNI_REG_PARAM_LOAD                0x5CU
#define PNI_REG_PARAM_REQUEST             0x64U
#define PNI_REG_ROM_VERSION               0x70U
#define PNI_REG_RAM_VERSION               0x72U
#define PNI_REG_PRODUCT_ID                0x90U
#define PNI_REG_RESET_REQUEST             0x9BU

#define PNI_CHIP_CONTROL_CPU_RUN_REQUEST  0x01U
#define PNI_META_SID                      254U
#define PNI_META_EVENT_INITIALIZED        16U
#define PNI_CHIP_STATUS_FW_LOADED         0x03U
#define PNI_CHIP_STATUS_FW_UPLOAD_ERR     0x04U
#define PNI_CHIP_STATUS_FW_HALT           0x08U
#define PNI_CHIP_STATUS_NO_EEPROM         0x10U
#define PNI_HOST_IF_ABORT_TRANSFER        0x02U
#define PNI_HOST_IF_UPDATE_TRANSFER_COUNT 0x04U
#define PNI_PARAM_WRITE_BIT               0x80U
#define PNI_FIFO_MAX_READ_LEN             50U
#define PNI_FIFO_PARSE_BUFFER_LEN         512U
#define PNI_FIFO_SNAPSHOT_LEN             4096U
#define PNI_I2C_TIMEOUT_MS                100U
#define PNI_PARAM_ACK_TIMEOUT_MS          1000U

static I2C_HandleTypeDef *pni_i2c = &hi2c1;
static uint8_t pni_addr_7bit = PNI_DEFAULT_ADDR_7BIT;
static PNI_OrientationTypeDef pni_latest_orientation;
static PNI_StatusTypeDef pni_latest_status = PNI_NO_DATA;
static uint8_t pni_latest_valid = 0U;
static PNI_AccelTypeDef pni_latest_accel;
static PNI_StatusTypeDef pni_latest_accel_status = PNI_NO_DATA;
static uint8_t pni_latest_accel_valid = 0U;
static uint8_t pni_started = 0U;
static const char *pni_parse_reason = "NONE";
static uint8_t pni_stream[64];
static uint16_t pni_stream_len;
static uint8_t pni_stream_synced;
static uint32_t pni_stream_tick;
static uint32_t pni_stream_dropped;

#if PNI_FIFO_TEST_ENABLE
static uint8_t pni_test_fifo[PNI_FIFO_SNAPSHOT_LEN];
static uint16_t pni_test_len;
static uint16_t pni_test_remaining;
static uint16_t pni_test_bad_offset;
static uint32_t pni_test_tick;
static PNI_StatusTypeDef pni_test_status;
static const char *pni_test_reason = "NONE";
static uint16_t pni_test_carry;
static uint32_t pni_test_dropped;
static uint8_t pni_test_pending;
#endif

static uint16_t PNI_DeviceAddress(void)
{
  return (uint16_t)(pni_addr_7bit << 1);
}

static PNI_StatusTypeDef PNI_FromHal(HAL_StatusTypeDef status)
{
  if (status == HAL_OK)
  {
    return PNI_OK;
  }

  if (status == HAL_TIMEOUT)
  {
    return PNI_TIMEOUT;
  }

  return PNI_ERROR;
}

static int16_t PNI_ReadInt16LE(const uint8_t *data)
{
  return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static float PNI_RawAngleToDegree(int16_t raw)
{
  return (float)raw * 360.0f / 32768.0f;
}

const char *PNI_StatusString(PNI_StatusTypeDef status)
{
  switch (status)
  {
    case PNI_OK:
      return "OK";
    case PNI_ERROR:
      return "ERROR";
    case PNI_TIMEOUT:
      return "TIMEOUT";
    case PNI_NOT_FOUND:
      return "NOT_FOUND";
    case PNI_BAD_ID:
      return "BAD_ID";
    case PNI_NO_DATA:
      return "NO_DATA";
    case PNI_FW_UPLOAD_ERROR:
      return "FW_UPLOAD_ERROR";
    case PNI_FW_HALT:
      return "FW_HALT";
    case PNI_NO_EEPROM:
      return "NO_EEPROM";
    case PNI_FW_NOT_READY:
      return "FW_NOT_READY";
    case PNI_META_NOT_READY:
      return "META_NOT_READY";
    default:
      return "UNKNOWN";
  }
}

static PNI_StatusTypeDef PNI_ParseOrientationPacket(const uint8_t *packet,
                                                    uint16_t len,
                                                    PNI_OrientationTypeDef *orientation)
{
  uint16_t i;

  if ((packet == NULL) || (orientation == NULL) || (len < 8U))
  {
    return PNI_ERROR;
  }

  for (i = 0U; i <= (uint16_t)(len - 8U); i++)
  {
    if (packet[i] == PNI_ORIENTATION_SID)
    {
      orientation->heading_raw = PNI_ReadInt16LE(&packet[i + 1U]);
      orientation->pitch_raw = PNI_ReadInt16LE(&packet[i + 3U]);
      orientation->roll_raw = PNI_ReadInt16LE(&packet[i + 5U]);
      orientation->heading = PNI_RawAngleToDegree(orientation->heading_raw);
      orientation->pitch = PNI_RawAngleToDegree(orientation->pitch_raw);
      orientation->roll = PNI_RawAngleToDegree(orientation->roll_raw);
      orientation->status = packet[i + 7U];
      return PNI_OK;
    }
  }

  return PNI_NO_DATA;
}

static PNI_StatusTypeDef PNI_ParseAccelPacket(const uint8_t *packet,
                                             uint16_t len,
                                             PNI_AccelTypeDef *accel)
{
  uint16_t i;
  float scale = 9.8f / 8192.0f;

  if ((packet == NULL) || (accel == NULL) || (len < 8U))
  {
    return PNI_ERROR;
  }

  for (i = 0U; i <= (uint16_t)(len - 8U); i++)
  {
    if (packet[i] == PNI_ACCELEROMETER_SID)
    {
      accel->x_raw = PNI_ReadInt16LE(&packet[i + 1U]);
      accel->y_raw = PNI_ReadInt16LE(&packet[i + 3U]);
      accel->z_raw = PNI_ReadInt16LE(&packet[i + 5U]);
      accel->x = (float)accel->x_raw * scale;
      accel->y = (float)accel->y_raw * scale;
      accel->z = (float)accel->z_raw * scale;
      accel->status = packet[i + 7U];
      return PNI_OK;
    }
  }

  return PNI_NO_DATA;
}
static PNI_StatusTypeDef PNI_ParseRotationVectorPacket(const uint8_t *packet,
                                                       uint16_t len,
                                                       PNI_RotationVectorTypeDef *rotation)
{
  uint16_t i;

  if ((packet == NULL) || (rotation == NULL) || (len < 11U))
  {
    return PNI_ERROR;
  }

  for (i = 0U; i <= (uint16_t)(len - 11U); i++)
  {
    if (packet[i] == PNI_ROTATION_VECTOR_SID)
    {
      rotation->x_raw = PNI_ReadInt16LE(&packet[i + 1U]);
      rotation->y_raw = PNI_ReadInt16LE(&packet[i + 3U]);
      rotation->z_raw = PNI_ReadInt16LE(&packet[i + 5U]);
      rotation->w_raw = PNI_ReadInt16LE(&packet[i + 7U]);
      rotation->accuracy_raw = PNI_ReadInt16LE(&packet[i + 9U]);
      rotation->x = (float)rotation->x_raw / 16384.0f;
      rotation->y = (float)rotation->y_raw / 16384.0f;
      rotation->z = (float)rotation->z_raw / 16384.0f;
      rotation->w = (float)rotation->w_raw / 16384.0f;
      rotation->accuracy = (float)rotation->accuracy_raw / 16384.0f;
      rotation->status = 0U;
      return PNI_OK;
    }
  }

  return PNI_NO_DATA;
}

void PNI_SetI2C(I2C_HandleTypeDef *hi2c)
{
  if (hi2c != NULL)
  {
    pni_i2c = hi2c;
  }
}

void PNI_SetAddress(uint8_t addr_7bit)
{
  pni_addr_7bit = (uint8_t)(addr_7bit & 0x7FU);
}

uint8_t PNI_GetAddress(void)
{
  return pni_addr_7bit;
}

void PNI_I2C_PinModeProbe(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  HAL_I2C_DeInit(pni_i2c);
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  printf("GPIO input pullup: PB6/SCL=%d, PB7/SDA=%d\r\n",
         HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6),
         HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7));

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);
  HAL_Delay(1U);

  printf("GPIO open-drain release: PB6/SCL=%d, PB7/SDA=%d\r\n",
         HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6),
         HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7));

  MX_I2C1_Init();

  printf("I2C AF mode: PB6/SCL=%d, PB7/SDA=%d, state=%d, error=0x%08lX\r\n",
         HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6),
         HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7),
         HAL_I2C_GetState(pni_i2c),
         HAL_I2C_GetError(pni_i2c));
}

void PNI_I2C_BusRecover(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  uint8_t i;

  HAL_I2C_DeInit(pni_i2c);
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);
  delay_us(10);

  for (i = 0U; i < 9U; i++)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    delay_us(5);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    delay_us(5);
  }

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
  delay_us(5);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
  delay_us(5);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
  delay_us(5);

  MX_I2C1_Init();
}

PNI_StatusTypeDef PNI_WriteRegister(uint8_t reg, uint8_t value)
{
  return PNI_FromHal(HAL_I2C_Mem_Write(pni_i2c,
                                       PNI_DeviceAddress(),
                                       reg,
                                       I2C_MEMADD_SIZE_8BIT,
                                       &value,
                                       1U,
                                       PNI_I2C_TIMEOUT_MS));
}

PNI_StatusTypeDef PNI_ReadRegister(uint8_t reg, uint8_t *value)
{
  if (value == NULL)
  {
    return PNI_ERROR;
  }

  return PNI_FromHal(HAL_I2C_Mem_Read(pni_i2c,
                                      PNI_DeviceAddress(),
                                      reg,
                                      I2C_MEMADD_SIZE_8BIT,
                                      value,
                                      1U,
                                      PNI_I2C_TIMEOUT_MS));
}

PNI_StatusTypeDef PNI_ReadRegisters(uint8_t reg, uint8_t *data, uint16_t len)
{
  if ((data == NULL) || (len == 0U))
  {
    return PNI_ERROR;
  }

  return PNI_FromHal(HAL_I2C_Mem_Read(pni_i2c,
                                      PNI_DeviceAddress(),
                                      reg,
                                      I2C_MEMADD_SIZE_8BIT,
                                      data,
                                      len,
                                      PNI_I2C_TIMEOUT_MS));
}

PNI_StatusTypeDef PNI_Scan(uint8_t *addr_list, uint8_t list_size, uint8_t *found_count)
{
  uint8_t addr;
  uint8_t count = 0U;

  for (addr = PNI_I2C_SCAN_FIRST_ADDR; addr <= PNI_I2C_SCAN_LAST_ADDR; addr++)
  {
    if (HAL_I2C_IsDeviceReady(pni_i2c, (uint16_t)(addr << 1), 2U, 10U) == HAL_OK)
    {
      if ((addr_list != NULL) && (count < list_size))
      {
        addr_list[count] = addr;
      }
      count++;
    }
  }

  if (found_count != NULL)
  {
    *found_count = count;
  }

  if (count == 0U)
  {
    return PNI_NOT_FOUND;
  }

  return PNI_OK;
}

PNI_StatusTypeDef PNI_FindDevice(uint8_t *addr_7bit)
{
  uint8_t addr;
  uint8_t old_addr = pni_addr_7bit;

  for (addr = PNI_I2C_SCAN_FIRST_ADDR; addr <= PNI_I2C_SCAN_LAST_ADDR; addr++)
  {
    if (HAL_I2C_IsDeviceReady(pni_i2c, (uint16_t)(addr << 1), 2U, 10U) == HAL_OK)
    {
      PNI_SetAddress(addr);
      if (PNI_CheckId() == PNI_OK)
      {
        if (addr_7bit != NULL)
        {
          *addr_7bit = addr;
        }
        return PNI_OK;
      }
    }
  }

  PNI_SetAddress(old_addr);
  return PNI_NOT_FOUND;
}

PNI_StatusTypeDef PNI_ReadId(PNI_IdTypeDef *id)
{
  uint8_t rom[2];
  uint8_t product[2];
  PNI_StatusTypeDef status;

  if (id == NULL)
  {
    return PNI_ERROR;
  }

  status = PNI_ReadRegisters(PNI_REG_ROM_VERSION, rom, 2U);
  if (status != PNI_OK)
  {
    return status;
  }

  status = PNI_ReadRegisters(PNI_REG_PRODUCT_ID, product, 2U);
  if (status != PNI_OK)
  {
    return status;
  }

  id->rom_version_l = rom[0];
  id->rom_version_h = rom[1];
  id->product_id_l = product[0];
  id->product_id_h = product[1];

  return PNI_OK;
}

PNI_StatusTypeDef PNI_CheckId(void)
{
  PNI_IdTypeDef id;
  PNI_StatusTypeDef status = PNI_ReadId(&id);

  if (status != PNI_OK)
  {
    return status;
  }

  if ((id.rom_version_l != PNI_ROM_VERSION_L_EXPECTED) ||
      (id.rom_version_h != PNI_ROM_VERSION_H_EXPECTED) ||
      (id.product_id_l != PNI_PRODUCT_ID_L_EXPECTED) ||
      (id.product_id_h != PNI_PRODUCT_ID_H_EXPECTED))
  {
    return PNI_BAD_ID;
  }

  return PNI_OK;
}

PNI_StatusTypeDef PNI_ReadIntStatus(uint8_t *status_value)
{
  return PNI_ReadRegister(PNI_REG_INT_STATUS, status_value);
}

PNI_StatusTypeDef PNI_ReadChipStatus(uint8_t *status_value)
{
  return PNI_ReadRegister(PNI_REG_CHIP_STATUS, status_value);
}

PNI_StatusTypeDef PNI_ReadErrorStatus(uint8_t *status_value)
{
  return PNI_ReadRegister(PNI_REG_ERROR_STATUS, status_value);
}

PNI_StatusTypeDef PNI_ReadRamVersion(uint16_t *version)
{
  uint8_t data[2];
  PNI_StatusTypeDef status;

  if (version == NULL)
  {
    return PNI_ERROR;
  }

  status = PNI_ReadRegisters(PNI_REG_RAM_VERSION, data, 2U);
  if (status != PNI_OK)
  {
    return status;
  }

  *version = (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
  return PNI_OK;
}

PNI_StatusTypeDef PNI_WaitFirmwareLoaded(uint32_t timeout_ms)
{
  uint8_t chip_status;
  PNI_StatusTypeDef status;
  uint32_t start_tick;

  HAL_Delay(1000U);
  start_tick = HAL_GetTick();

  do
  {
    status = PNI_ReadChipStatus(&chip_status);
    if (status != PNI_OK)
    {
      return status;
    }

    if (chip_status == PNI_CHIP_STATUS_FW_LOADED)
    {
      return PNI_OK;
    }

    if ((chip_status & PNI_CHIP_STATUS_FW_UPLOAD_ERR) != 0U)
    {
      return PNI_FW_UPLOAD_ERROR;
    }

    if ((chip_status & PNI_CHIP_STATUS_FW_HALT) != 0U)
    {
      return PNI_FW_HALT;
    }

    if ((chip_status & PNI_CHIP_STATUS_NO_EEPROM) != 0U)
    {
      return PNI_NO_EEPROM;
    }

    HAL_Delay(200U);
  } while ((HAL_GetTick() - start_tick) < timeout_ms);

  return PNI_FW_NOT_READY;
}

PNI_StatusTypeDef PNI_WaitInitializedMetaEvent(uint32_t timeout_ms, uint16_t *ram_version)
{
  uint8_t fifo[PNI_FIFO_MAX_READ_LEN];
  uint16_t bytes_read;
  uint16_t offset;
  PNI_StatusTypeDef status;
  uint32_t start_tick = HAL_GetTick();

  do
  {
    status = PNI_ReadFifo(fifo, sizeof(fifo), &bytes_read);
    if (status == PNI_NO_DATA)
    {
      HAL_Delay(20U);
      continue;
    }
    if (status != PNI_OK)
    {
      return status;
    }

    offset = 0U;
    while (offset < bytes_read)
    {
      if ((fifo[offset] == PNI_META_SID) && ((uint16_t)(offset + 3U) < bytes_read))
      {
        if (fifo[offset + 1U] == PNI_META_EVENT_INITIALIZED)
        {
          if (ram_version != NULL)
          {
            *ram_version = (uint16_t)fifo[offset + 2U] | ((uint16_t)fifo[offset + 3U] << 8U);
          }
          return PNI_OK;
        }
        offset = (uint16_t)(offset + 4U);
      }
      else if ((fifo[offset] == 252U) || (fifo[offset] == 253U))
      {
        offset = (uint16_t)(offset + 3U);
      }
      else if (fifo[offset] == 0U)
      {
        offset = (uint16_t)(offset + 1U);
      }
      else
      {
        offset = (uint16_t)(offset + 1U);
      }
    }
  } while ((HAL_GetTick() - start_tick) < timeout_ms);

  return PNI_META_NOT_READY;
}

PNI_StatusTypeDef PNI_Reset(void)
{
  return PNI_ResetAndDelay(100U);
}

PNI_StatusTypeDef PNI_ResetAndDelay(uint32_t delay_ms)
{
  PNI_StatusTypeDef status = PNI_WriteRegister(PNI_REG_RESET_REQUEST, 0x01U);

  HAL_Delay(delay_ms);
  return status;
}

PNI_StatusTypeDef PNI_Run(void)
{
  PNI_StatusTypeDef status = PNI_WriteRegister(PNI_REG_CHIP_CONTROL,
                                               PNI_CHIP_CONTROL_CPU_RUN_REQUEST);

  HAL_Delay(10U);
  return status;
}

PNI_StatusTypeDef PNI_WriteParam(uint8_t page, uint8_t param, const uint8_t *data, uint8_t len)
{
  uint32_t start_tick;
  uint8_t ack;
  PNI_StatusTypeDef status;

  if ((data == NULL) || (len == 0U) || (len > 8U) || (page > 0x0FU) || (param > 0x7FU))
  {
    return PNI_ERROR;
  }

  status = PNI_WriteRegister(PNI_REG_PARAM_PAGE_SELECT, (uint8_t)((len << 4U) | (page & 0x0FU)));
  if (status != PNI_OK)
  {
    return status;
  }

  status = PNI_FromHal(HAL_I2C_Mem_Write(pni_i2c,
                                         PNI_DeviceAddress(),
                                         PNI_REG_PARAM_LOAD,
                                         I2C_MEMADD_SIZE_8BIT,
                                         (uint8_t *)data,
                                         len,
                                         PNI_I2C_TIMEOUT_MS));
  if (status != PNI_OK)
  {
    return status;
  }

  status = PNI_WriteRegister(PNI_REG_PARAM_REQUEST, (uint8_t)(PNI_PARAM_WRITE_BIT | param));
  if (status != PNI_OK)
  {
    return status;
  }

  start_tick = HAL_GetTick();
  do
  {
    status = PNI_ReadRegister(PNI_REG_PARAM_ACK, &ack);
    if (status != PNI_OK)
    {
      return status;
    }

    if (ack == (uint8_t)(PNI_PARAM_WRITE_BIT | param))
    {
      (void)PNI_WriteRegister(PNI_REG_PARAM_REQUEST, 0x00U);
      (void)PNI_WriteRegister(PNI_REG_PARAM_PAGE_SELECT, 0x00U);
      return PNI_OK;
    }
  } while ((HAL_GetTick() - start_tick) < PNI_PARAM_ACK_TIMEOUT_MS);

  (void)PNI_WriteRegister(PNI_REG_PARAM_REQUEST, 0x00U);
  (void)PNI_WriteRegister(PNI_REG_PARAM_PAGE_SELECT, 0x00U);
  return PNI_TIMEOUT;
}

PNI_StatusTypeDef PNI_ReadParam(uint8_t page, uint8_t param, uint8_t *data, uint8_t len)
{
  uint32_t start_tick;
  uint8_t ack;
  PNI_StatusTypeDef status;

  if ((data == NULL) || (len == 0U) || (len > 8U) || (page > 0x0FU) || (param > 0x7FU))
  {
    return PNI_ERROR;
  }

  status = PNI_WriteRegister(PNI_REG_PARAM_PAGE_SELECT, (uint8_t)((len << 4U) | (page & 0x0FU)));
  if (status != PNI_OK)
  {
    return status;
  }

  status = PNI_WriteRegister(PNI_REG_PARAM_REQUEST, param);
  if (status != PNI_OK)
  {
    return status;
  }

  start_tick = HAL_GetTick();
  do
  {
    status = PNI_ReadRegister(PNI_REG_PARAM_ACK, &ack);
    if (status != PNI_OK)
    {
      return status;
    }

    if (ack == param)
    {
      status = PNI_ReadRegisters(PNI_REG_PARAM_SAVE, data, len);
      (void)PNI_WriteRegister(PNI_REG_PARAM_REQUEST, 0x00U);
      (void)PNI_WriteRegister(PNI_REG_PARAM_PAGE_SELECT, 0x00U);
      return status;
    }
  } while ((HAL_GetTick() - start_tick) < PNI_PARAM_ACK_TIMEOUT_MS);

  (void)PNI_WriteRegister(PNI_REG_PARAM_REQUEST, 0x00U);
  (void)PNI_WriteRegister(PNI_REG_PARAM_PAGE_SELECT, 0x00U);
  return PNI_TIMEOUT;
}

PNI_StatusTypeDef PNI_EnableOrientation(uint16_t sample_rate_hz)
{
  return PNI_EnableSensor(PNI_ORIENTATION_SID, sample_rate_hz);
}

PNI_StatusTypeDef PNI_EnableSensor(uint8_t sid, uint16_t sample_rate_hz)
{
  uint8_t rate[2];

  rate[0] = (uint8_t)(sample_rate_hz & 0xFFU);
  rate[1] = (uint8_t)((sample_rate_hz >> 8U) & 0xFFU);

  return PNI_WriteParam(5U, sid, rate, 2U);
}

PNI_StatusTypeDef PNI_EnableDemoSensors(void)
{
  PNI_StatusTypeDef status;

  status = PNI_EnableSensor(PNI_PRESSURE_SID, PNI_PRESSURE_DEFAULT_RATE_HZ);
  if (status != PNI_OK)
  {
    return status;
  }

  return PNI_EnableSensor(PNI_ROTATION_VECTOR_SID, PNI_ROTATION_VECTOR_RATE_HZ);
}

PNI_StatusTypeDef PNI_EnableAllTestSensors(void)
{
  static const uint8_t sids[] = {
    PNI_ACCELEROMETER_SID,
    PNI_MAGNETIC_FIELD_SID,
    PNI_ORIENTATION_SID,
    PNI_GYROSCOPE_SID,
    PNI_LINEAR_ACCELERATION_SID,
    PNI_PRESSURE_SID,
    PNI_ROTATION_VECTOR_SID
  };
  static const uint16_t rates[] = {
    PNI_MOTION_SENSOR_RATE_HZ,
    PNI_MOTION_SENSOR_RATE_HZ,
    PNI_ORIENTATION_DEFAULT_RATE_HZ,
    PNI_MOTION_SENSOR_RATE_HZ,
    PNI_MOTION_SENSOR_RATE_HZ,
    PNI_PRESSURE_DEFAULT_RATE_HZ,
    PNI_ROTATION_VECTOR_RATE_HZ
  };
  PNI_StatusTypeDef status;
  PNI_StatusTypeDef first_error = PNI_OK;
  uint8_t enabled_count = 0U;
  uint8_t i;

  for (i = 0U; i < (uint8_t)(sizeof(sids) / sizeof(sids[0])); i++)
  {
    status = PNI_EnableSensor(sids[i], rates[i]);
    if (status == PNI_OK)
    {
      enabled_count++;
      printf("PNI enable SID %u @ %uHz ok\r\n", sids[i], rates[i]);
    }
    else
    {
      printf("PNI enable SID %u failed: %s\r\n", sids[i], PNI_StatusString(status));
      if (first_error == PNI_OK)
      {
        first_error = status;
      }
    }
  }

  if (enabled_count == 0U)
  {
    return first_error;
  }

  return PNI_OK;
}

PNI_StatusTypeDef PNI_ReadSensorRate(uint8_t sid, uint16_t *sample_rate_hz)
{
  uint8_t rate[2];
  PNI_StatusTypeDef status;

  if (sample_rate_hz == NULL)
  {
    return PNI_ERROR;
  }

  status = PNI_ReadParam(5U, sid, rate, 2U);
  if (status != PNI_OK)
  {
    return status;
  }

  *sample_rate_hz = (uint16_t)rate[0] | ((uint16_t)rate[1] << 8U);
  return PNI_OK;
}

PNI_StatusTypeDef PNI_ReadBytesRemaining(uint16_t *bytes_remaining)
{
  uint8_t data[2];
  PNI_StatusTypeDef status;

  if (bytes_remaining == NULL)
  {
    return PNI_ERROR;
  }

  status = PNI_WriteRegister(PNI_REG_HOST_INTERFACE_CONTROL,
                             PNI_HOST_IF_UPDATE_TRANSFER_COUNT);
  if (status != PNI_OK)
  {
    return status;
  }

  HAL_Delay(1U);

  status = PNI_ReadRegisters(PNI_REG_BYTES_REMAINING, data, 2U);
  (void)PNI_WriteRegister(PNI_REG_HOST_INTERFACE_CONTROL, 0x00U);
  if (status != PNI_OK)
  {
    return status;
  }

  *bytes_remaining = (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
  return PNI_OK;
}

PNI_StatusTypeDef PNI_ReadFifo(uint8_t *buffer, uint16_t buffer_size, uint16_t *bytes_read)
{
  uint16_t remaining;
  uint16_t read_len;
  PNI_StatusTypeDef status;

  if ((buffer == NULL) || (buffer_size == 0U) || (bytes_read == NULL))
  {
    return PNI_ERROR;
  }

  *bytes_read = 0U;
  status = PNI_ReadBytesRemaining(&remaining);
  if (status != PNI_OK)
  {
    return status;
  }

  if (remaining == 0U)
  {
    return PNI_NO_DATA;
  }

  read_len = remaining;
  if (read_len > buffer_size)
  {
    read_len = buffer_size;
  }
  if (read_len > PNI_FIFO_MAX_READ_LEN)
  {
    read_len = PNI_FIFO_MAX_READ_LEN;
  }

  status = PNI_ReadRegisters(PNI_REG_DATA_TRANSFER_AREA, buffer, read_len);
  if (status != PNI_OK)
  {
    return status;
  }

  *bytes_read = read_len;
  return PNI_OK;
}

static PNI_StatusTypeDef PNI_RequestTransferCountUpdate(void)
{
  PNI_StatusTypeDef status;

  status = PNI_WriteRegister(PNI_REG_HOST_INTERFACE_CONTROL,
                             PNI_HOST_IF_UPDATE_TRANSFER_COUNT);
  if (status != PNI_OK)
  {
    return status;
  }

  HAL_Delay(1U);
  return PNI_OK;
}

static PNI_StatusTypeDef PNI_AbortTransfer(void)
{
  PNI_StatusTypeDef status;

  status = PNI_WriteRegister(PNI_REG_HOST_INTERFACE_CONTROL,
                             PNI_HOST_IF_ABORT_TRANSFER);
  if (status != PNI_OK)
  {
    return status;
  }

  HAL_Delay(1U);
  (void)PNI_WriteRegister(PNI_REG_HOST_INTERFACE_CONTROL, 0x00U);
  return PNI_OK;
}

PNI_StatusTypeDef PNI_ReadOrientation(PNI_OrientationTypeDef *orientation)
{
  return PNI_ReadOrientationDebug(orientation, NULL, NULL);
}

PNI_StatusTypeDef PNI_ReadOrientationDebug(PNI_OrientationTypeDef *orientation,
                                           uint16_t *bytes_remaining_before,
                                           uint16_t *bytes_read_total)
{
  uint8_t fifo[PNI_FIFO_MAX_READ_LEN];
  uint16_t remaining;
  uint16_t bytes_read;
  uint16_t total_read = 0U;
  PNI_StatusTypeDef status;
  PNI_StatusTypeDef parse_status = PNI_NO_DATA;

  status = PNI_ReadBytesRemaining(&remaining);
  if (status != PNI_OK)
  {
    return status;
  }

  if (bytes_remaining_before != NULL)
  {
    *bytes_remaining_before = remaining;
  }

  if (remaining == 0U)
  {
    if (bytes_read_total != NULL)
    {
      *bytes_read_total = 0U;
    }
    return PNI_NO_DATA;
  }

  while (remaining > 0U)
  {
    status = PNI_ReadFifo(fifo, sizeof(fifo), &bytes_read);
    if (status != PNI_OK)
    {
      break;
    }

    total_read = (uint16_t)(total_read + bytes_read);
    parse_status = PNI_ParseOrientationPacket(fifo, bytes_read, orientation);
    if (parse_status == PNI_OK)
    {
      break;
    }

    status = PNI_ReadBytesRemaining(&remaining);
    if (status != PNI_OK)
    {
      break;
    }
  }

  if (bytes_read_total != NULL)
  {
    *bytes_read_total = total_read;
  }

  if (parse_status == PNI_OK)
  {
    return PNI_OK;
  }

  if (total_read > 0U)
  {
    return PNI_NO_DATA;
  }

  return status;
}

PNI_StatusTypeDef PNI_ReadRotationVectorDebug(PNI_RotationVectorTypeDef *rotation,
                                              uint16_t *bytes_remaining_before,
                                              uint16_t *bytes_read_total)
{
  uint8_t fifo[PNI_FIFO_MAX_READ_LEN];
  uint16_t remaining;
  uint16_t bytes_read;
  uint16_t total_read = 0U;
  PNI_StatusTypeDef status;
  PNI_StatusTypeDef parse_status = PNI_NO_DATA;

  status = PNI_ReadBytesRemaining(&remaining);
  if (status != PNI_OK)
  {
    return status;
  }

  if (bytes_remaining_before != NULL)
  {
    *bytes_remaining_before = remaining;
  }

  if (remaining == 0U)
  {
    if (bytes_read_total != NULL)
    {
      *bytes_read_total = 0U;
    }
    return PNI_NO_DATA;
  }

  while (remaining > 0U)
  {
    status = PNI_ReadFifo(fifo, sizeof(fifo), &bytes_read);
    if (status != PNI_OK)
    {
      break;
    }

    total_read = (uint16_t)(total_read + bytes_read);
    parse_status = PNI_ParseRotationVectorPacket(fifo, bytes_read, rotation);
    if (parse_status == PNI_OK)
    {
      break;
    }

    status = PNI_ReadBytesRemaining(&remaining);
    if (status != PNI_OK)
    {
      break;
    }
  }

  if (bytes_read_total != NULL)
  {
    *bytes_read_total = total_read;
  }

  if (parse_status == PNI_OK)
  {
    return PNI_OK;
  }

  if (total_read > 0U)
  {
    return PNI_NO_DATA;
  }

  return status;
}

PNI_StatusTypeDef PNI_DebugDumpFifo(uint16_t max_bytes)
{
  uint8_t fifo[PNI_FIFO_MAX_READ_LEN];
  uint16_t remaining;
  uint16_t bytes_read;
  uint16_t dumped = 0U;
  uint16_t i;
  PNI_StatusTypeDef status;

  status = PNI_ReadBytesRemaining(&remaining);
  if (status != PNI_OK)
  {
    printf("FIFO dump: read remaining failed %s\r\n", PNI_StatusString(status));
    return status;
  }

  printf("FIFO dump: remaining=%u\r\n", remaining);
  while ((remaining > 0U) && (dumped < max_bytes))
  {
    status = PNI_ReadFifo(fifo, sizeof(fifo), &bytes_read);
    if (status != PNI_OK)
    {
      printf("FIFO dump: read failed %s\r\n", PNI_StatusString(status));
      return status;
    }

    printf("FIFO bytes:");
    for (i = 0U; i < bytes_read; i++)
    {
      printf(" %02X", fifo[i]);
    }
    printf("\r\n");

    dumped = (uint16_t)(dumped + bytes_read);
    status = PNI_ReadBytesRemaining(&remaining);
    if (status != PNI_OK)
    {
      return status;
    }
  }

  return PNI_OK;
}

static uint8_t PNI_PrintOnePacket(const uint8_t *packet, uint16_t len, uint16_t *pkt_len)
{
  uint8_t sid;
  int16_t x;
  int16_t y;
  int16_t z;
  int16_t w;
  uint8_t status;
  float scale;
  float fx;

  if ((packet == NULL) || (pkt_len == NULL) || (len == 0U))
  {
    return 0U;
  }

  sid = packet[0];
  *pkt_len = 1U;

  switch (sid)
  {
    case 0U:
      *pkt_len = 1U;
      return 1U;

    case 252U:
    case 253U:
      if (len < 3U)
      {
        return 0U;
      }
      *pkt_len = 3U;
      return 1U;

    case 254U:
      if (len < 4U)
      {
        return 0U;
      }
      printf("META: event=0x%02X, data=%02X %02X\r\n", packet[1], packet[2], packet[3]);
      *pkt_len = 4U;
      return 1U;

    case PNI_ACCELEROMETER_SID:
    case PNI_LINEAR_ACCELERATION_SID:
      if (len < 8U)
      {
        return 0U;
      }
      x = PNI_ReadInt16LE(&packet[1]);
      y = PNI_ReadInt16LE(&packet[3]);
      z = PNI_ReadInt16LE(&packet[5]);
      status = packet[7];
      scale = 9.8f / 8192.0f;
      printf("ACC sid=%u: x=%.3f, y=%.3f, z=%.3f, status=%u\r\n",
             sid, (float)x * scale, (float)y * scale, (float)z * scale, status);
      *pkt_len = 8U;
      return 1U;

    case PNI_MAGNETIC_FIELD_SID:
      if (len < 8U)
      {
        return 0U;
      }
      x = PNI_ReadInt16LE(&packet[1]);
      y = PNI_ReadInt16LE(&packet[3]);
      z = PNI_ReadInt16LE(&packet[5]);
      status = packet[7];
      printf("MAG: x=%.3f, y=%.3f, z=%.3f, status=%u\r\n",
             (float)x / 75.0f, (float)y / 75.0f, (float)z / 75.0f, status);
      *pkt_len = 8U;
      return 1U;

    case PNI_ORIENTATION_SID:
      if (len < 8U)
      {
        return 0U;
      }
      x = PNI_ReadInt16LE(&packet[1]);
      y = PNI_ReadInt16LE(&packet[3]);
      z = PNI_ReadInt16LE(&packet[5]);
      status = packet[7];
      scale = 360.0f / 32768.0f;
      printf("HPR: heading=%.2f, pitch=%.2f, roll=%.2f, status=%u\r\n",
             (float)x * scale, (float)y * scale, (float)z * scale, status);
      *pkt_len = 8U;
      return 1U;

    case PNI_GYROSCOPE_SID:
      if (len < 8U)
      {
        return 0U;
      }
      x = PNI_ReadInt16LE(&packet[1]);
      y = PNI_ReadInt16LE(&packet[3]);
      z = PNI_ReadInt16LE(&packet[5]);
      status = packet[7];
      printf("GYRO: x=%.3f, y=%.3f, z=%.3f, status=%u\r\n",
             (float)x * 2000.0f / 32768.0f,
             (float)y * 2000.0f / 32768.0f,
             (float)z * 2000.0f / 32768.0f,
             status);
      *pkt_len = 8U;
      return 1U;

    case PNI_PRESSURE_SID:
      if (len < 4U)
      {
        return 0U;
      }
      {
        int32_t pressure_raw = (int32_t)((uint32_t)packet[1] |
                                         ((uint32_t)packet[2] << 8U) |
                                         ((uint32_t)packet[3] << 16U));
        if ((pressure_raw & 0x00800000L) != 0)
        {
          pressure_raw |= (int32_t)0xFF000000L;
        }
        fx = (float)pressure_raw / 12800.0f;
      }
      printf("PRESS: %.3f hPa\r\n", fx);
      *pkt_len = 4U;
      return 1U;

    case PNI_ROTATION_VECTOR_SID:
      if (len < 11U)
      {
        return 0U;
      }
      x = PNI_ReadInt16LE(&packet[1]);
      y = PNI_ReadInt16LE(&packet[3]);
      z = PNI_ReadInt16LE(&packet[5]);
      w = PNI_ReadInt16LE(&packet[7]);
      printf("QUAT: x=%.4f, y=%.4f, z=%.4f, w=%.4f, acc=%.4f\r\n",
             (float)x / 16384.0f,
             (float)y / 16384.0f,
             (float)z / 16384.0f,
             (float)w / 16384.0f,
             (float)PNI_ReadInt16LE(&packet[9]) / 16384.0f);
      *pkt_len = 11U;
      return 1U;

    default:
      printf("SID %u: unhandled\r\n", sid);
      *pkt_len = 1U;
      return 1U;
  }
}

PNI_StatusTypeDef PNI_PrintAllFifoPackets(uint16_t max_bytes)
{
  static uint8_t fifo[PNI_FIFO_PARSE_BUFFER_LEN];
  uint16_t remaining;
  uint16_t read_len;
  uint16_t read_reg;
  uint16_t offset;
  uint16_t pkt_len;
  uint16_t total_read = 0U;
  uint16_t target_read;
  PNI_StatusTypeDef status;

  status = PNI_ReadBytesRemaining(&remaining);
  if (status != PNI_OK)
  {
    return status;
  }

  if (remaining == 0U)
  {
    status = PNI_RequestTransferCountUpdate();
    if (status != PNI_OK)
    {
      return status;
    }

    status = PNI_ReadBytesRemaining(&remaining);
    (void)PNI_WriteRegister(PNI_REG_HOST_INTERFACE_CONTROL, 0x00U);
    if (status != PNI_OK)
    {
      return status;
    }

    if (remaining == 0U)
    {
      return PNI_NO_DATA;
    }
  }

  target_read = remaining;
  if ((max_bytes > 0U) && (max_bytes < target_read))
  {
    target_read = max_bytes;
  }
  if (target_read > sizeof(fifo))
  {
    target_read = sizeof(fifo);
  }

  while (total_read < target_read)
  {
    read_len = (uint16_t)(target_read - total_read);
    if (read_len > PNI_FIFO_MAX_READ_LEN)
    {
      read_len = PNI_FIFO_MAX_READ_LEN;
    }

    read_reg = (uint16_t)(PNI_REG_DATA_TRANSFER_AREA + (total_read % PNI_FIFO_MAX_READ_LEN));
    status = PNI_ReadRegisters((uint8_t)read_reg, &fifo[total_read], read_len);
    if (status != PNI_OK)
    {
      return status;
    }

    total_read = (uint16_t)(total_read + read_len);
  }

  offset = 0U;
  while (offset < total_read)
  {
    if (PNI_PrintOnePacket(&fifo[offset], (uint16_t)(total_read - offset), &pkt_len) == 0U)
    {
      break;
    }
    offset = (uint16_t)(offset + pkt_len);
  }

  if (target_read < remaining)
  {
    (void)PNI_AbortTransfer();
  }

  return PNI_OK;
}

PNI_StatusTypeDef PNI_Init(void)
{
  uint8_t addr;
  uint16_t ram_version;
  PNI_StatusTypeDef find_status;
  PNI_StatusTypeDef status;

  /* Match the vendor demo sequence: reset first, then read ROM/Product ID. */
  status = PNI_ResetAndDelay(5U);
  if (status != PNI_OK)
  {
    find_status = PNI_FindDevice(&addr);
    if (find_status != PNI_OK)
    {
      return find_status;
    }

    PNI_SetAddress(addr);
    status = PNI_ResetAndDelay(5U);
    if (status != PNI_OK)
    {
      return status;
    }
  }

  status = PNI_CheckId();
  if (status != PNI_OK)
  {
    find_status = PNI_FindDevice(&addr);
    if (find_status != PNI_OK)
    {
      return find_status;
    }

    PNI_SetAddress(addr);
    status = PNI_CheckId();
    if (status != PNI_OK)
    {
      return status;
    }
  }

  status = PNI_ResetAndDelay(1000U);
  if (status != PNI_OK)
  {
    return status;
  }

  status = PNI_Run();
  if (status != PNI_OK)
  {
    return status;
  }

  status = PNI_WaitFirmwareLoaded(2000U);
  if (status != PNI_OK)
  {
    return status;
  }

  status = PNI_ReadRamVersion(&ram_version);
  if (status != PNI_OK)
  {
    return status;
  }

  if (ram_version == 0U)
  {
    return PNI_FW_NOT_READY;
  }

  status = PNI_EnableSensor(PNI_ACCELEROMETER_SID, PNI_MOTION_SENSOR_RATE_HZ);
  if (status != PNI_OK)
  {
    return status;
  }

  return PNI_EnableOrientation(PNI_ORIENTATION_DEFAULT_RATE_HZ);
}

void PNI_ReportStartup(void)
{
  PNI_StatusTypeDef status;

  status = PNI_Init();
  pni_latest_status = status;
  pni_latest_valid = 0U;
  pni_latest_accel_status = status;
  pni_latest_accel_valid = 0U;
  pni_stream_len = 0U;
  pni_stream_synced = 0U;

  if(status == PNI_OK)
  {
    pni_started = 1U;
    printf("PNI init ok, addr=0x%02X\r\n", PNI_GetAddress());
  }
  else
  {
    pni_started = 0U;
    printf("PNI init failed: %s (%d)\r\n", PNI_StatusString(status), status);
  }
}

/* MM-Amber V3, tables 7/8: lengths include the SID byte. */
static uint8_t PNI_PacketLength(uint8_t sid)
{
  switch(sid)
  {
    case 0U: return 1U;
    case PNI_ACCELEROMETER_SID:
    case PNI_MAGNETIC_FIELD_SID:
    case PNI_ORIENTATION_SID:
    case PNI_GYROSCOPE_SID:
    case PNI_LINEAR_ACCELERATION_SID: return 8U;
    case PNI_PRESSURE_SID:
    case PNI_META_SID: return 4U;
    case PNI_ROTATION_VECTOR_SID: return 11U;
    case 252U:
    case 253U: return 3U;
    case 245U: return 14U; /* Fixed debug packet in the vendor demo. */
    default: return 0U;
  }
}

static uint8_t PNI_PacketValid(const uint8_t *packet, uint8_t packet_len)
{
  PNI_OrientationTypeDef orientation;
  if(packet_len == 8U && packet[7] > 3U) return 0U;
  if(packet[0] == PNI_ORIENTATION_SID)
  {
    (void)PNI_ParseOrientationPacket(packet, packet_len, &orientation);
    if(orientation.heading < -180.0f || orientation.heading > 180.0f ||
       orientation.pitch < -180.0f || orientation.pitch > 180.0f ||
       orientation.roll < -180.0f || orientation.roll > 180.0f) return 0U;
  }
  return 1U;
}

/* Recovery heuristic: require several complete packets, not a lone 1 or 3. */
static uint8_t PNI_ConfirmBoundary(const uint8_t *data, uint16_t len)
{
  uint16_t offset = 0U;
  uint8_t packets = 0U;
  uint8_t sensors = 0U;
  uint8_t size;
  while(offset < len)
  {
    size = PNI_PacketLength(data[offset]);
    if(size == 0U || size > len - offset) return 0U;
    if(PNI_PacketValid(&data[offset], size) == 0U) return 0U;
    if(data[offset] != 0U) packets++;
    if(data[offset] == PNI_ORIENTATION_SID || data[offset] == PNI_ACCELEROMETER_SID)
      sensors++;
    if(packets >= 3U && sensors >= 2U) return 1U;
    offset = (uint16_t)(offset + size);
  }
  return 0U;
}

static PNI_StatusTypeDef PNI_ParseSnapshot(const uint8_t *fifo, uint16_t len,
                                           uint32_t update_tick, uint16_t *bad_offset)
{
  uint16_t input_pos;
  uint16_t candidate;
  uint8_t size;
  uint8_t updated = 0U;
  uint32_t dropped_before = pni_stream_dropped;
  PNI_OrientationTypeDef orientation;
  PNI_AccelTypeDef accel;

  /* FIFO reads can end inside a packet; carry that tail into the next read. */
  if(update_tick - pni_stream_tick > PNI_DATA_TIMEOUT_MS)
  {
    pni_stream_dropped += pni_stream_len;
    pni_stream_len = 0U;
    pni_stream_synced = 0U;
  }
  pni_stream_tick = update_tick;
  *bad_offset = 0U;
  for(input_pos = 0U; input_pos < len; input_pos++)
  {
    pni_stream[pni_stream_len++] = fifo[input_pos];
    while(pni_stream_len > 0U)
    {
      if(pni_stream_synced == 0U)
      {
        for(candidate = 0U; candidate < pni_stream_len; candidate++)
        {
          if(pni_stream[candidate] != 0U &&
             PNI_ConfirmBoundary(&pni_stream[candidate], (uint16_t)(pni_stream_len - candidate)))
            break;
        }
        if(candidate == pni_stream_len) break;
        pni_stream_dropped += candidate;
        pni_stream_len = (uint16_t)(pni_stream_len - candidate);
        memmove(pni_stream, &pni_stream[candidate], pni_stream_len);
        pni_stream_synced = 1U;
      }
      size = PNI_PacketLength(pni_stream[0]);
      if(size == 0U)
      {
        *bad_offset = (input_pos + 1U >= pni_stream_len) ?
                      (uint16_t)(input_pos + 1U - pni_stream_len) : 0U;
        pni_stream_synced = 0U;
        continue;
      }
      if(size > pni_stream_len) break;
      if(PNI_PacketValid(pni_stream, size) == 0U)
      {
        *bad_offset = (input_pos + 1U >= pni_stream_len) ?
                      (uint16_t)(input_pos + 1U - pni_stream_len) : 0U;
        pni_stream_synced = 0U;
        continue;
      }
      if(pni_stream[0] == PNI_ORIENTATION_SID)
      {
        (void)PNI_ParseOrientationPacket(pni_stream, size, &orientation);
        orientation.update_tick = update_tick;
        pni_latest_orientation = orientation;
        pni_latest_valid = 1U;
        pni_latest_status = PNI_OK;
        updated = 1U;
      }
      else if(pni_stream[0] == PNI_ACCELEROMETER_SID)
      {
        (void)PNI_ParseAccelPacket(pni_stream, size, &accel);
        accel.update_tick = update_tick;
        pni_latest_accel = accel;
        pni_latest_accel_valid = 1U;
        pni_latest_accel_status = PNI_OK;
        updated = 1U;
      }
      pni_stream_len = (uint16_t)(pni_stream_len - size);
      memmove(pni_stream, &pni_stream[size], pni_stream_len);
    }
    /* Only discard unconfirmed noise, never an aligned partial packet. */
    if(pni_stream_len == sizeof(pni_stream))
    {
      pni_stream_len--;
      memmove(pni_stream, &pni_stream[1], pni_stream_len);
      pni_stream_dropped++;
    }
  }
  if(pni_stream_synced == 0U) pni_parse_reason = "WAIT_SYNC";
  else if(pni_stream_dropped != dropped_before) pni_parse_reason = "RESYNC";
  else if(pni_stream_len > 0U) pni_parse_reason = "PARTIAL";
  else pni_parse_reason = "OK";
  return (updated != 0U) ? PNI_OK : PNI_NO_DATA;
}

void PNI_UpdateTask(void)
{
  static uint32_t last_update = 0U;
  static uint32_t last_report = 0U;
  static uint32_t rejected_batches = 0U;
  static uint8_t fifo[PNI_FIFO_SNAPSHOT_LEN];
  PNI_StatusTypeDef status = PNI_NO_DATA;
  uint16_t remaining = 0U;
  uint16_t total_read = 0U;
  uint16_t read_len;
  uint16_t bad_offset = 0U;
  uint16_t i;
  const char *reason = "COUNT_READ";

  if(pni_started == 0U)
  {
#if PNI_FIFO_TEST_ENABLE
    if(pni_test_pending == 0U && HAL_GetTick() - pni_test_tick >= 1000U)
    {
      pni_test_len = 0U;
      pni_test_remaining = 0U;
      pni_test_bad_offset = 0U;
      pni_test_status = pni_latest_status;
      pni_test_reason = "INIT_FAILED";
      pni_test_carry = 0U;
      pni_test_dropped = pni_stream_dropped;
      pni_test_tick = HAL_GetTick();
      pni_test_pending = 1U;
    }
#endif
    return;
  }

  if(HAL_GetTick() - last_update < PNI_UPDATE_POLL_MS)
  {
    return;
  }

  last_update = HAL_GetTick();

  /* Keep the previously working count-based polling entry point. */
  status = PNI_ReadBytesRemaining(&remaining);
  if(status == PNI_OK && remaining == 0U)
  {
    status = PNI_NO_DATA;
    reason = "EMPTY_FIFO";
  }
  if(status == PNI_OK && remaining > sizeof(fifo))
  {
    status = PNI_ERROR;
    reason = "FIFO_TOO_LARGE";
    (void)PNI_AbortTransfer();
  }
  /* Finish this transfer without refreshing its count between 50-byte reads. */
  while(status == PNI_OK && total_read < remaining)
  {
    read_len = (uint16_t)(remaining - total_read);
    if(read_len > PNI_FIFO_MAX_READ_LEN) read_len = PNI_FIFO_MAX_READ_LEN;
    status = PNI_ReadRegisters(PNI_REG_DATA_TRANSFER_AREA, &fifo[total_read], read_len);
    if(status != PNI_OK)
    {
      reason = "FIFO_READ";
      (void)PNI_AbortTransfer();
      break;
    }
    total_read = (uint16_t)(total_read + read_len);
  }
  if(status == PNI_OK)
  {
    status = PNI_ParseSnapshot(fifo, total_read, last_update, &bad_offset);
    reason = pni_parse_reason;
  }
  if(status != PNI_OK && status != PNI_NO_DATA)
  {
    rejected_batches++;
    pni_latest_status = status;
    pni_latest_accel_status = status;
    pni_latest_valid = 0U;
    pni_latest_accel_valid = 0U;
    pni_stream_len = 0U;
    pni_stream_synced = 0U;
  }

#if PNI_FIFO_TEST_ENABLE
  /* Capture the same bytes used for parsing; never consume the FIFO twice. */
  if(pni_test_pending == 0U && last_update - pni_test_tick >= 1000U)
  {
    memcpy(pni_test_fifo, fifo, total_read);
    pni_test_len = total_read;
    pni_test_remaining = remaining;
    pni_test_bad_offset = bad_offset;
    pni_test_status = status;
    pni_test_reason = reason;
    pni_test_carry = pni_stream_len;
    pni_test_dropped = pni_stream_dropped;
    pni_test_tick = last_update;
    pni_test_pending = 1U;
  }
#endif

  if(HAL_GetTick() - last_report >= 2000U)
  {
    last_report = HAL_GetTick();
    printf("PNI update: %s, bytes=%u/%u, rejected=%lu, bad_offset=%u reason=%s\r\n",
           PNI_StatusString(status), total_read, remaining, rejected_batches, bad_offset, reason);
    if(status != PNI_OK && status != PNI_NO_DATA && total_read > 0U)
    {
      printf("PNI raw:");
      for(i = bad_offset; i < total_read && i < bad_offset + 32U; i++)
        printf(" %02X", fifo[i]);
      printf("\r\n");
    }
  }
}

void PNI_TestUploadTask(void)
{
#if PNI_FIFO_TEST_ENABLE
  static char tx[192];
  static uint16_t offset = 0U;
  static uint8_t phase = 0U;
  uint16_t end;
  uint16_t i;
  int len;

  if(pni_test_pending == 0U || huart2.gState != HAL_UART_STATE_READY) return;
  if(phase == 0U)
  {
    len = snprintf(tx, sizeof(tx),
                   "FIFO BEGIN tick=%lu bytes=%u/%u status=%s bad_offset=%u reason=%s carry=%u dropped=%lu\r\n",
                   pni_test_tick, pni_test_len, pni_test_remaining,
                   PNI_StatusString(pni_test_status), pni_test_bad_offset, pni_test_reason,
                   pni_test_carry, pni_test_dropped);
    if(HAL_UART_Transmit_IT(&huart2, (uint8_t *)tx, (uint16_t)len) == HAL_OK)
    {
      offset = 0U;
      phase = 1U;
    }
  }
  else if(offset < pni_test_len)
  {
    end = (uint16_t)(offset + 24U);
    if(end > pni_test_len) end = pni_test_len;
    len = snprintf(tx, sizeof(tx), "FIFO DATA %04X:", offset);
    for(i = offset; i < end; i++)
      len += snprintf(&tx[len], sizeof(tx) - (size_t)len, " %02X", pni_test_fifo[i]);
    len += snprintf(&tx[len], sizeof(tx) - (size_t)len, "\r\n");
    if(HAL_UART_Transmit_IT(&huart2, (uint8_t *)tx, (uint16_t)len) == HAL_OK)
      offset = end;
  }
  else
  {
    len = snprintf(tx, sizeof(tx), "FIFO END tick=%lu\r\n", pni_test_tick);
    if(HAL_UART_Transmit_IT(&huart2, (uint8_t *)tx, (uint16_t)len) == HAL_OK)
    {
      phase = 0U;
      pni_test_pending = 0U;
    }
  }
#endif
}

uint8_t PNI_GetLatestOrientation(PNI_OrientationTypeDef *orientation,
                                 PNI_StatusTypeDef *status)
{
  if(pni_latest_valid != 0U &&
     HAL_GetTick() - pni_latest_orientation.update_tick > PNI_DATA_TIMEOUT_MS)
  {
    pni_latest_valid = 0U;
    pni_latest_status = PNI_TIMEOUT;
  }
  if(status != NULL)
  {
    *status = pni_latest_status;
  }

  if((orientation != NULL) && (pni_latest_valid != 0U))
  {
    *orientation = pni_latest_orientation;
  }

  return pni_latest_valid;
}
uint8_t PNI_GetLatestAccel(PNI_AccelTypeDef *accel,
                           PNI_StatusTypeDef *status)
{
  if(pni_latest_accel_valid != 0U &&
     HAL_GetTick() - pni_latest_accel.update_tick > PNI_DATA_TIMEOUT_MS)
  {
    pni_latest_accel_valid = 0U;
    pni_latest_accel_status = PNI_TIMEOUT;
  }
  if(status != NULL)
  {
    *status = pni_latest_accel_status;
  }

  if((accel != NULL) && (pni_latest_accel_valid != 0U))
  {
    *accel = pni_latest_accel;
  }

  return pni_latest_accel_valid;
}
void PNI_DebugStartup(void)
{
  PNI_StatusTypeDef pni_status;
  uint8_t i;
  uint16_t pni_rate = 0;
  static const uint8_t pni_test_sids[] = {
    PNI_ACCELEROMETER_SID,
    PNI_MAGNETIC_FIELD_SID,
    PNI_ORIENTATION_SID,
    PNI_GYROSCOPE_SID,
    PNI_LINEAR_ACCELERATION_SID,
    PNI_PRESSURE_SID,
    PNI_ROTATION_VECTOR_SID
  };

  pni_status = PNI_Init();
  if(pni_status == PNI_OK)
  {
    printf("PNI init ok, addr=0x%02X\r\n", PNI_GetAddress());
    for(i = 0; i < (uint8_t)(sizeof(pni_test_sids) / sizeof(pni_test_sids[0])); i++)
    {
      if(PNI_ReadSensorRate(pni_test_sids[i], &pni_rate) == PNI_OK)
      {
        printf("PNI SID %u rate=%uHz\r\n", pni_test_sids[i], pni_rate);
      }
      else
      {
        printf("PNI SID %u rate read failed\r\n", pni_test_sids[i]);
      }
    }
  }
  else
  {
    printf("PNI init failed: %s (%d)\r\n", PNI_StatusString(pni_status), pni_status);
  }
}

void PNI_DebugLoopTask(void)
{
  PNI_StatusTypeDef read_status;
  static uint8_t pni_wait_count = 0;

  read_status = PNI_PrintAllFifoPackets(0);
  if(read_status == PNI_OK)
  {
    pni_wait_count = 0;
  }
  else
  {
    printf("PNI waiting data: %s\r\n", PNI_StatusString(read_status));
    if(++pni_wait_count >= 10)
    {
      uint8_t chip_status = 0;
      uint8_t int_status = 0;
      uint8_t err_status = 0;
      uint16_t bytes_remaining = 0;

      pni_wait_count = 0;
      PNI_ReadChipStatus(&chip_status);
      PNI_ReadIntStatus(&int_status);
      PNI_ReadErrorStatus(&err_status);
      PNI_ReadBytesRemaining(&bytes_remaining);
      printf("PNI regs: chip=0x%02X, int=0x%02X, err=0x%02X, bytes=%u\r\n",
             chip_status,
             int_status,
             err_status,
             bytes_remaining);
      PNI_DebugDumpFifo(50);
    }
  }
}
