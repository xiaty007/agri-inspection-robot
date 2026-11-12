#ifndef __GPS_H
#define __GPS_H

#include "main.h"
#include <stdint.h>

#define GPS_BUF_SIZE 256

typedef struct {
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
} nmea_utc_time;

typedef struct {
    nmea_utc_time utc;
    float latitude;   // 纬度
    float longitude;  // 经度
    uint8_t valid;    // 1=有效
} nmea_msg;

extern nmea_msg gpsx;
extern uint8_t gps_buf[GPS_BUF_SIZE];
extern uint8_t gps_done;

void GPS_Init(void);
void GPS_Parse(void);

#endif