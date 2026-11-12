#ifndef __APP_DEF_H__
#define __APP_DEF_H__

#include <stdint.h>
#include <string.h>
#include <time.h>

#include "board_config.h"

// options
#define MM_USING_GPIO6
#define ENABLE_PRINTF                       (1)

#define ENABLE_SENTRAL_A2                   (1)

#define _DEBUG_RTI_LOAD                     (2)                 // 1 ram.   2. wr to eeprom.


#define SID_MAXSZ                           (256)
#define MMA_RATIO                           (0.90)
#define AXIS_NUM                            (3)


// common definition.
#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif /* MAX */

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif /* MIN */

#ifndef ARRAY_SZ
#define ARRAY_SZ(arr)   (sizeof(arr) / sizeof((arr)[0]))
#endif


#ifndef TRUE
#define TRUE            (1)
#endif


#ifndef FALSE
#define FALSE           (0)
#endif

#define BE16(x)        ((((uint16_t)(x) & 0XFF00) >> 8) | (((uint16_t)(x) & 0X00FF) << 8))

#define BE32(x)        ((((uint32_t)(x) & 0XFF000000) >> 24) | \
                        (((uint32_t)(x) & 0X00FF0000) >> 8) | \
                        (((uint32_t)(x) & 0X0000FF00) << 8) | \
                        (((uint32_t)(x) & 0X000000FF) << 24))


#define SET_BITS(value, bits)               ((value) |= (bits))
#define CLEAR_BITS(value, bits)              ((value) &= ~(bits))
#define CHECK_BITS(value, bits)             (((value) & (bits)) > 0 ? 1 : 0)


#define APP_E(fmt, ...)                     printf("[APP-E %d]" fmt "\r\n", __LINE__, ## __VA_ARGS__)

#if 1
#define APP_DBG(...)                        printf(__VA_ARGS__)
#else
#define APP_DBG(...)
#endif


#define MAGIC8                              (0xBA)
#define MAGIC16                             (0xDCBA)

// user action
enum
{
    UA_JUMP_RAM_CODE = 0xDCBB,
    UA_CLEAR_WARMSTAT,
    UA_PROG_SA2_FW,

};
enum
{

    MM_FW_UPLOAD_ERR = 0x80,
    MM_FW_HALT,
    MM_NO_EEPROM,
};


typedef struct
{
    uint16_t hostevent;
    uint8_t  u8errcnt_sa2_fifo;
    uint8_t  UserAction;
    uint8_t  bUseCalDat;
    uint8_t  warmstartAction;
    int      print_skip;

} stc_app_runtime_t;


typedef struct
{
    uint16_t app_reset_reason;
    uint16_t useraction;

    uint16_t bupgrade_rti;                              // 0xDCBA valid
    uint8_t  app_reset_in_fsm;
    uint8_t  rev1;

    uint8_t  bClearWarmStartEEPROM;                     // 0xBA
    uint8_t  bdocalibration;                            //

}stc_app_ram_t;


typedef struct
{

    float dat[4];                                       // xyzw
}stc_axis_raw_t;


typedef struct
{
    float fspeed[AXIS_NUM];

    float posistion[AXIS_NUM];

    float distance;

}stc_distance_t;


typedef struct
{
    stc_axis_raw_t acc;
    stc_axis_raw_t acc_cal_min;
    stc_axis_raw_t acc_cal_max;

    stc_axis_raw_t gyro;                                // gyro raw data.
    stc_axis_raw_t gyro_cal_offset;                     //.

    stc_axis_raw_t rot;                                 // integral of gyro
    stc_axis_raw_t mag;
    stc_axis_raw_t hpr;
    stc_axis_raw_t quan;
    float          baro;

    stc_distance_t distance;

    uint32_t       ts;                                  // timestamp

    uint8_t        oriantation_status;
    int            gyro_init_cal_cnt;
    uint16_t       skipcnt[SID_MAXSZ];

}stc_mm_data_t;


typedef struct
{
    stc_app_runtime_t runtime;
    stc_app_ram_t *   pRAM;
    stc_mm_data_t     mmdat;
} stc_app_t;


// mcu
#define APP_RAM_DATA_BASE                  (0x2001F000) //app ram debug area.


typedef enum
{
    APP_RESET_BASE = 0x9030,
    APP_RESET_RTI_METAERR,
    APP_RESET_RTI_FIFO_EMPTY,
    APP_RESET_RTI_INIT,
    APP_RESET_RTI_STAT_CHK,
    APP_RESET_WDT,


}en_app_resetreason;


#pragma pack(push, 1)
typedef struct pni_park_meta_event
{
    uint8_t type;
    uint8_t data[2];
} PNI_ParkingMetaEvent;
#pragma pack(pop)


#define PARK_PKT_DATA_SIZE      (16)                    //(12)

#pragma pack(push, 1)
typedef struct pni_park_pkt
{
    uint8_t sid;
    union
    {

        PNI_ParkingMetaEvent meta_event;

        uint8_t              data[PARK_PKT_DATA_SIZE];
    };
} PNI_ParkingPacket;
#pragma pack(pop)


extern I2C_HandleTypeDef hi2c1;

#define SENTRAL_A2_I2C                                  hi2c1

#endif
