#ifndef __PNI_H
#define __PNI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define PNI_DEFAULT_ADDR_7BIT             0x28U
#define PNI_I2C_SCAN_FIRST_ADDR           0x08U
#define PNI_I2C_SCAN_LAST_ADDR            0x77U

#define PNI_ROM_VERSION_L_EXPECTED        0xF2U
#define PNI_ROM_VERSION_H_EXPECTED        0x27U
#define PNI_PRODUCT_ID_L_EXPECTED         0x86U
#define PNI_PRODUCT_ID_H_EXPECTED         0x01U

#define PNI_ORIENTATION_SID               3U
#define PNI_ACCELEROMETER_SID             1U
#define PNI_MAGNETIC_FIELD_SID            2U
#define PNI_GYROSCOPE_SID                 4U
#define PNI_PRESSURE_SID                  6U
#define PNI_ROTATION_VECTOR_SID           11U
#define PNI_LINEAR_ACCELERATION_SID        10U
#define PNI_ORIENTATION_DEFAULT_RATE_HZ   10U
#define PNI_MOTION_SENSOR_RATE_HZ         10U
#define PNI_PRESSURE_DEFAULT_RATE_HZ      1U
#define PNI_ROTATION_VECTOR_RATE_HZ       10U
#define PNI_UPDATE_POLL_MS                100U
#define PNI_DATA_TIMEOUT_MS               500U
/* Temporary PC capture; set to 0 before connecting the Raspberry Pi service. */
#ifndef PNI_FIFO_TEST_ENABLE
#define PNI_FIFO_TEST_ENABLE              0U
#endif

typedef enum
{
  PNI_OK = 0,
  PNI_ERROR,
  PNI_TIMEOUT,
  PNI_NOT_FOUND,
  PNI_BAD_ID,
  PNI_NO_DATA,
  PNI_FW_UPLOAD_ERROR,
  PNI_FW_HALT,
  PNI_NO_EEPROM,
  PNI_FW_NOT_READY,
  PNI_META_NOT_READY
} PNI_StatusTypeDef;

typedef struct
{
  uint8_t rom_version_l;
  uint8_t rom_version_h;
  uint8_t product_id_l;
  uint8_t product_id_h;
} PNI_IdTypeDef;

typedef struct
{
  int16_t heading_raw;
  int16_t pitch_raw;
  int16_t roll_raw;
  float heading;
  float pitch;
  float roll;
  uint8_t status;
  uint32_t update_tick;
} PNI_OrientationTypeDef;

typedef struct
{
  int16_t x_raw;
  int16_t y_raw;
  int16_t z_raw;
  float x;
  float y;
  float z;
  uint8_t status;
  uint32_t update_tick;
} PNI_AccelTypeDef;

typedef struct
{
  int16_t x_raw;
  int16_t y_raw;
  int16_t z_raw;
  int16_t w_raw;
  float x;
  float y;
  float z;
  float w;
  int16_t accuracy_raw;
  float accuracy;
  uint8_t status;
} PNI_RotationVectorTypeDef;

const char *PNI_StatusString(PNI_StatusTypeDef status);

void PNI_SetI2C(I2C_HandleTypeDef *hi2c);
void PNI_SetAddress(uint8_t addr_7bit);
uint8_t PNI_GetAddress(void);
void PNI_I2C_PinModeProbe(void);
void PNI_I2C_BusRecover(void);

PNI_StatusTypeDef PNI_Scan(uint8_t *addr_list, uint8_t list_size, uint8_t *found_count);
PNI_StatusTypeDef PNI_FindDevice(uint8_t *addr_7bit);
PNI_StatusTypeDef PNI_ReadId(PNI_IdTypeDef *id);
PNI_StatusTypeDef PNI_CheckId(void);
PNI_StatusTypeDef PNI_ReadIntStatus(uint8_t *status_value);
PNI_StatusTypeDef PNI_ReadChipStatus(uint8_t *status_value);
PNI_StatusTypeDef PNI_ReadErrorStatus(uint8_t *status_value);
PNI_StatusTypeDef PNI_ReadRamVersion(uint16_t *version);
PNI_StatusTypeDef PNI_WaitFirmwareLoaded(uint32_t timeout_ms);
PNI_StatusTypeDef PNI_WaitInitializedMetaEvent(uint32_t timeout_ms, uint16_t *ram_version);
PNI_StatusTypeDef PNI_Reset(void);
PNI_StatusTypeDef PNI_ResetAndDelay(uint32_t delay_ms);
PNI_StatusTypeDef PNI_Run(void);
PNI_StatusTypeDef PNI_Init(void);

PNI_StatusTypeDef PNI_WriteRegister(uint8_t reg, uint8_t value);
PNI_StatusTypeDef PNI_ReadRegister(uint8_t reg, uint8_t *value);
PNI_StatusTypeDef PNI_ReadRegisters(uint8_t reg, uint8_t *data, uint16_t len);

PNI_StatusTypeDef PNI_WriteParam(uint8_t page, uint8_t param, const uint8_t *data, uint8_t len);
PNI_StatusTypeDef PNI_ReadParam(uint8_t page, uint8_t param, uint8_t *data, uint8_t len);
PNI_StatusTypeDef PNI_EnableSensor(uint8_t sid, uint16_t sample_rate_hz);
PNI_StatusTypeDef PNI_EnableOrientation(uint16_t sample_rate_hz);
PNI_StatusTypeDef PNI_EnableDemoSensors(void);
PNI_StatusTypeDef PNI_EnableAllTestSensors(void);
PNI_StatusTypeDef PNI_ReadSensorRate(uint8_t sid, uint16_t *sample_rate_hz);
PNI_StatusTypeDef PNI_ReadBytesRemaining(uint16_t *bytes_remaining);
PNI_StatusTypeDef PNI_ReadFifo(uint8_t *buffer, uint16_t buffer_size, uint16_t *bytes_read);
PNI_StatusTypeDef PNI_ReadOrientation(PNI_OrientationTypeDef *orientation);
PNI_StatusTypeDef PNI_ReadOrientationDebug(PNI_OrientationTypeDef *orientation,
                                           uint16_t *bytes_remaining_before,
                                           uint16_t *bytes_read_total);
PNI_StatusTypeDef PNI_ReadRotationVectorDebug(PNI_RotationVectorTypeDef *rotation,
                                              uint16_t *bytes_remaining_before,
                                              uint16_t *bytes_read_total);
PNI_StatusTypeDef PNI_DebugDumpFifo(uint16_t max_bytes);
PNI_StatusTypeDef PNI_PrintAllFifoPackets(uint16_t max_bytes);
void PNI_ReportStartup(void);
void PNI_UpdateTask(void);
void PNI_TestUploadTask(void);
uint8_t PNI_GetLatestOrientation(PNI_OrientationTypeDef *orientation,
                                 PNI_StatusTypeDef *status);
uint8_t PNI_GetLatestAccel(PNI_AccelTypeDef *accel,
                           PNI_StatusTypeDef *status);
void PNI_DebugStartup(void);
void PNI_DebugLoopTask(void);

#ifdef __cplusplus
}
#endif

#endif
