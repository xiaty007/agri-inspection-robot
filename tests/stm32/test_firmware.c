#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "stm32f4xx_hal.h"

/* Include the real implementation, including its private parser. */
#include "pni.c"
#include "sensors.c"

I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;
TIM_HandleTypeDef htim4;
static uint32_t clock_tick;
static uint8_t input[5000];
static uint16_t input_len, input_offset;
static unsigned read_calls, count_requests, aborts;
static unsigned int_status_reads;
static int fail_read;
static uint8_t wire_bits[80];
static unsigned wire_pos;
static char last_tx[1024];
static uint8_t *active_tx;
static uint16_t active_tx_len;

uint32_t HAL_GetTick(void) { return clock_tick; }
void HAL_Delay(uint32_t ms) { clock_tick += ms; }
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *h, uint16_t a,
    uint16_t reg, uint16_t mode, uint8_t *out, uint16_t len, uint32_t timeout)
{
  (void)h; (void)a; (void)mode; (void)timeout;
  if(reg == 0x36U) { int_status_reads++; *out = 0U; return HAL_OK; }
  if(reg == 0x38U) { out[0] = input_len & 255U; out[1] = input_len >> 8; return HAL_OK; }
  if(reg == 0U)
  {
    read_calls++;
    if(fail_read) return HAL_TIMEOUT;
    assert(len <= 50U && input_offset + len <= input_len);
    memcpy(out, &input[input_offset], len);
    input_offset += len;
    return HAL_OK;
  }
  memset(out, 0, len);
  return HAL_OK;
}
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *h, uint16_t a,
    uint16_t reg, uint16_t mode, uint8_t *data, uint16_t len, uint32_t timeout)
{
  (void)h; (void)a; (void)mode; (void)len; (void)timeout;
  if(reg == 0x55U && *data == 4U) count_requests++;
  if(reg == 0x55U && *data == 2U) aborts++;
  return HAL_OK;
}
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *h, uint16_t a, unsigned n, unsigned t)
{ (void)h; (void)a; (void)n; (void)t; return HAL_OK; }
void HAL_I2C_DeInit(I2C_HandleTypeDef *h) { (void)h; }
unsigned HAL_I2C_GetState(I2C_HandleTypeDef *h) { (void)h; return 0; }
unsigned long HAL_I2C_GetError(I2C_HandleTypeDef *h) { (void)h; return 0; }
void HAL_GPIO_Init(void *p, GPIO_InitTypeDef *g) { (void)p; (void)g; }
unsigned HAL_GPIO_ReadPin(void *p, unsigned pin)
{ (void)p; return pin == GPIO_PIN_0 ? wire_bits[wire_pos++] : 1U; }
void HAL_GPIO_WritePin(void *p, unsigned pin, unsigned value) { (void)p; (void)pin; (void)value; }
void MX_I2C1_Init(void) {}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *h, uint8_t *data, uint16_t len)
{
  if(h->gState != HAL_UART_STATE_READY) return HAL_BUSY;
  assert(len < sizeof(last_tx));
  memcpy(last_tx, data, len);
  last_tx[len] = 0;
  active_tx = data;
  active_tx_len = len;
  h->gState = HAL_UART_STATE_BUSY_TX;
  return HAL_OK;
}
static void complete_tx(void)
{
  if(huart2.gState != HAL_UART_STATE_READY)
    assert(memcmp(active_tx, last_tx, active_tx_len) == 0);
  huart2.gState = HAL_UART_STATE_READY;
}
static void packet(uint8_t sid, int16_t x, int16_t y, int16_t z, uint8_t status)
{
  int16_t axes[3] = { x, y, z };
  unsigned i;
  input[input_len++] = sid;
  for(i = 0; i < 3; i++)
  {
    input[input_len++] = (uint8_t)axes[i];
    input[input_len++] = (uint8_t)((uint16_t)axes[i] >> 8);
  }
  input[input_len++] = status;
}
static void start_input(void)
{
  input_len = input_offset = 0;
  read_calls = count_requests = aborts = 0;
  fail_read = 0;
  clock_tick += 200U;
  pni_started = 1U;
}
static void scratch_bits(const uint8_t *scratch)
{
  unsigned i, b;
  wire_pos = 0;
  wire_bits[0] = 0; /* Presence pulse. */
  for(i = 0; i < 9; i++)
    for(b = 0; b < 8; b++) wire_bits[1 + i * 8 + b] = (scratch[i] >> b) & 1U;
}
static void reset_stream(void)
{
  pni_stream_len = 0;
  pni_stream_synced = 0;
  pni_stream_tick = clock_tick;
  pni_latest_valid = pni_latest_accel_valid = 0;
}
static void test_screenshot_stream(void)
{
  /* Transcribed in 24-byte rows from the user's 132-byte FIFO capture. */
  static const uint8_t capture[] = {
    0x8E,0xFF,0x81,0x1E,0x01,0xFC,0xF1,0x96,0x01,0xEC,0x0C,0x8A,0xFF,0x73,0x1E,0x01,0xFC,0xA4,0x9A,0x03,0x4E,0x27,0x4F,0x00,
    0x2A,0x08,0x00,0xFC,0xE0,0x9B,0x01,0xDE,0x0C,0x8E,0xFF,0x79,0x1E,0x01,0x00,0x00,0x00,0x00,0x00,0x0C,0x8A,0xFF,0x78,0x1E,
    0x01,0xFC,0xFC,0xCE,0xA0,0x01,0xEB,0x0C,0x8A,0xFF,0x78,0x1E,0x01,0xFC,0x82,0xA4,0x03,0x4E,0x27,0x4E,0x00,0x2B,0x08,0x00,
    0xFC,0xBD,0xA5,0x01,0xE3,0x0C,0x86,0xFF,0x7A,0x1E,0x01,0xFC,0xAC,0xAA,0x01,0xEF,0x0C,0x8E,0xFF,0x7A,0x1E,0x01,0xFC,0x5F,
    0xAE,0x03,0x4E,0x27,0x4F,0x00,0x2C,0x08,0x00,0xFC,0x9B,0xAF,0x01,0xE6,0x0C,0x90,0xFF,0x78,0x1E,0x01,0xFC,0x8A,0xB4,0x01,
    0xE9,0x0C,0x87,0xFF,0x72,0x1E,0x01,0xFC,0x3D,0xB8,0x03,0x4E
  };
  static const uint8_t continuation[] = { 0x27,0x50,0x00,0x2D,0x08,0x00 };
  uint16_t bad, pos, count, chunk;
  assert(sizeof(capture) == 132);
  for(chunk = 1; chunk <= sizeof(capture); chunk++)
  {
    reset_stream();
    for(pos = 0; pos < sizeof(capture); pos += count)
    {
      count = (uint16_t)(sizeof(capture) - pos);
      if(count > chunk) count = chunk;
      (void)PNI_ParseSnapshot(&capture[pos], count, ++clock_tick, &bad);
    }
    assert(pni_latest_valid && pni_latest_accel_valid);
    assert(pni_latest_orientation.heading_raw == 0x274E);
    assert(pni_latest_orientation.pitch_raw == 0x004F);
    assert(pni_latest_orientation.roll_raw == 0x082C);
    assert(pni_latest_accel.x_raw == 0x0CE9 && pni_latest_accel.z_raw == 0x1E72);
    assert(pni_stream_len == 2 && pni_stream[0] == 3 && pni_stream[1] == 0x4E);
    assert(PNI_ParseSnapshot(continuation, sizeof(continuation), ++clock_tick, &bad) == PNI_OK);
    assert(pni_latest_orientation.pitch_raw == 0x0050 && pni_stream_len == 0);
  }
  puts("PASS screenshot replay at all chunk sizes 1..132, recovery and cross-read tail completion");
}
int main(void)
{
  PNI_OrientationTypeDef orientation;
  PNI_AccelTypeDef accel;
  PNI_StatusTypeDef status;
  uint16_t bad = 0;
  unsigned i, parts, hex_count;
  uint8_t scratch[9] = { 0x50, 0x05, 0x4B, 0x46, 0x7F, 0xFF, 0x0C, 0x10, 0x1C };

  start_input();
  input[input_len++] = 252; input[input_len++] = 3; input[input_len++] = 1;
  input[input_len++] = 254; input[input_len++] = 3; input[input_len++] = 1; input[input_len++] = 3;
  packet(3, 1000, -100, 200, 3);
  packet(1, 0, 0, 8192, 3);
  PNI_UpdateTask();
  assert(PNI_GetLatestOrientation(&orientation, &status));
  assert(orientation.heading_raw == 1000 && orientation.pitch_raw == -100);
  assert(PNI_GetLatestAccel(&accel, &status) && fabs(accel.z - 9.8) < 0.001);
  assert(count_requests == 1 && input_offset == input_len);
  assert(int_status_reads == 0);
  puts("PASS mixed SID boundaries: timestamp/META payloads containing 1 and 3");
  puts("PASS count-based polling reads data without requiring an interrupt flag");

  start_input();
  for(i = 0; i < 40; i++) packet(3, (int16_t)i, 0, 0, 3);
  packet(1, 0, 0, 8192, 3);
  PNI_UpdateTask();
  assert(input_offset == input_len && read_calls == 7 && count_requests == 1);
  assert(PNI_GetLatestOrientation(&orientation, &status) && orientation.heading_raw == 39);
  puts("PASS complete 328-byte transfer: 7 reads, one count request, latest value retained");

  start_input();
  packet(3, 1200, 0, 0, 3);
  packet(1, 0, 0, 8192, 141);
  PNI_UpdateTask();
  assert(PNI_GetLatestOrientation(&orientation, &status) && orientation.heading_raw == 1200);
  assert(pni_latest_accel.status == 3 && pni_stream_synced == 0);
  puts("PASS bad accelerometer packet does not erase a complete valid orientation");

  start_input();
  reset_stream();
  input[input_len++] = 99;
  packet(3, 500, 0, 0, 3);
  assert(PNI_ParseSnapshot(input, input_len, clock_tick, &bad) == PNI_NO_DATA);
  assert(!pni_latest_valid && strcmp(pni_parse_reason, "WAIT_SYNC") == 0);
  puts("PASS a single SID-like candidate after noise cannot establish framing");

  test_screenshot_stream();

  start_input();
  packet(3, 100, 0, 0, 3);
  packet(1, 0, 0, 8192, 3);
  PNI_UpdateTask();
  assert(PNI_GetLatestOrientation(&orientation, &status));
  clock_tick += 501;
  assert(!PNI_GetLatestOrientation(&orientation, &status) && status == PNI_TIMEOUT);
  assert(!PNI_GetLatestAccel(&accel, &status) && status == PNI_TIMEOUT);
  pni_latest_orientation.update_tick = UINT32_MAX - 100U;
  pni_latest_valid = 1U;
  clock_tick = 50U;
  assert(PNI_GetLatestOrientation(&orientation, &status));
  clock_tick = 600U;
  assert(!PNI_GetLatestOrientation(&orientation, &status));
  puts("PASS stale timeout including tick rollover");

  start_input();
  packet(3, 100, 0, 0, 3);
  fail_read = 1;
  PNI_UpdateTask();
  assert(aborts == 1 && !pni_latest_valid);
  start_input(); input_len = 4097;
  PNI_UpdateTask();
  assert(aborts == 1 && read_calls == 0);
  puts("PASS failed/oversized transfer is aborted and invalidated");

  scratch_bits(scratch);
  assert(fabs(DS18B20_ReadConvertedTemp(-1) - 85.0) < 0.001 && ds18b20_valid);
  scratch[0] ^= 1;
  scratch_bits(scratch);
  assert(DS18B20_ReadConvertedTemp(24) == 24 && !ds18b20_valid);
  memset(scratch, 0, sizeof(scratch)); scratch_bits(scratch);
  assert(DS18B20_ReadConvertedTemp(24) == 24 && !ds18b20_valid);
  delay_us(10); assert(htim4.counter == 11U);
  puts("PASS temperature CRC, stuck-low rejection, and 1 MHz delay");

  complete_tx();
  DS18B20_UploadToRaspberryPi(24);
  assert(strstr(last_tx, "\"temperature\":null") && strstr(last_tx, "\"update_tick\":"));
  DS18B20_UploadToRaspberryPi(99);
  complete_tx();
  puts("PASS asynchronous JSON buffer remains unchanged while UART is busy");

#if PNI_FIFO_TEST_ENABLE
  pni_test_pending = 0;
  start_input(); clock_tick += 3000;
  for(i = 0; i < 40; i++) packet(3, (int16_t)i, 0, 0, 3);
  PNI_UpdateTask();
  assert(pni_test_pending && pni_test_len == input_len);
  assert(memcmp(pni_test_fifo, input, input_len) == 0);
  parts = hex_count = 0;
  while(pni_test_pending)
  {
    PNI_TestUploadTask();
    if(strncmp(last_tx, "FIFO DATA ", 10) == 0)
    {
      char *cursor = strchr(last_tx, ':') + 1;
      unsigned value;
      int used;
      while(sscanf(cursor, " %x%n", &value, &used) == 1)
      {
        assert(hex_count < input_len && value == input[hex_count++]);
        cursor += used;
      }
    }
    PNI_TestUploadTask(); /* Busy UART must not overwrite the in-flight buffer. */
    complete_tx();
    assert(++parts < 200);
  }
  assert(hex_count == input_len && strstr(last_tx, "FIFO END"));
  puts("PASS full diagnostic dump round-trip and UART busy handling");

  start_input(); clock_tick += 2000;
  PNI_UpdateTask();
  assert(pni_test_pending && pni_test_status == PNI_NO_DATA);
  PNI_TestUploadTask();
  assert(strstr(last_tx, "reason=EMPTY_FIFO") && strstr(last_tx, "bytes=0/0"));
  complete_tx(); PNI_TestUploadTask(); complete_tx();
  assert(!pni_test_pending);
  clock_tick += 2000;
  pni_started = 0;
  pni_latest_status = PNI_NOT_FOUND;
  PNI_UpdateTask();
  PNI_TestUploadTask();
  assert(strstr(last_tx, "reason=INIT_FAILED") && strstr(last_tx, "status=NOT_FOUND"));
  complete_tx(); PNI_TestUploadTask(); complete_tx();
  puts("PASS no-data and initialization failures remain visible on USART2");
#endif
  puts("ALL STM32 HOST TESTS PASSED");
  return 0;
}
