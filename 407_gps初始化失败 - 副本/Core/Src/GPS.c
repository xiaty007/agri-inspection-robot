#include "GPS.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

nmea_msg gpsx;
uint8_t gps_buf[GPS_BUF_SIZE];
uint8_t gps_done = 0;

extern UART_HandleTypeDef huart3;

// 找第 n 个逗号位置（正点原子官方方法）
uint8_t NMEA_Comma_Pos(uint8_t *buf, uint8_t n)
{
    uint8_t *p = buf;
    while(n)
    {
        if(*p == ',' || *p == '\n')
        {
            if(*p == ',') n--;
            if(*p == '\n') break;
        }
        p++;
    }
    return p - buf;
}

// 解析 GNRMC（手册标准帧）
void NMEA_GNRMC_Analysis(uint8_t *buf)
{
    if(strstr((char*)buf, "$GNRMC") == NULL) return;

    uint8_t *p = buf;
    int pos;

    // 状态 A=有效
    pos = NMEA_Comma_Pos(p, 2);
    if(buf[pos] == 'A') gpsx.valid = 1;
    else gpsx.valid = 0;

    if(!gpsx.valid) return;

    // 纬度 ddmm.mmmm
    pos = NMEA_Comma_Pos(p, 3);
    float lat = atof((char*)&buf[pos]);
    int d1 = (int)(lat / 100);
    float m1 = lat - d1 * 100.0f;
    gpsx.latitude = d1 + m1 / 60.0f;

    // 经度 dddmm.mmmm
    pos = NMEA_Comma_Pos(p, 5);
    float lon = atof((char*)&buf[pos]);
    int d2 = (int)(lon / 100);
    float m2 = lon - d2 * 100.0f;
    gpsx.longitude = d2 + m2 / 60.0f;

    // 时间
    pos = NMEA_Comma_Pos(p, 1);
    char tim[10];
    snprintf(tim, 10, "%s", &buf[pos]);
    gpsx.utc.hour = (tim[0]-'0')*10 + (tim[1]-'0');
    gpsx.utc.min = (tim[2]-'0')*10 + (tim[3]-'0');
    gpsx.utc.sec = (tim[4]-'0')*10 + (tim[5]-'0');
}

void GPS_Parse(void)
{
    if(gps_done == 0) return;

    NMEA_GNRMC_Analysis(gps_buf);
    gps_done = 0;
}

void GPS_Init(void)
{
    memset(gps_buf, 0, GPS_BUF_SIZE);
    gps_done = 0;
    gpsx.valid = 0;
    gpsx.latitude = 0;
    gpsx.longitude = 0;

    HAL_UART_Receive_IT(&huart3, &gps_buf[0], 1);
}