#ifndef __MM_UTIL_H__
#define __MM_UTIL_H__


#include <stdint.h>

// marcos
// device i2c address.
#define SA2_I2C_ADDRESS                                 (0x50)
#define MAG3100_ADDR                                    (0x40)
#define LSM330_ADDR                                     (0x1E << 1)
#define SA2_EEPROM_IIC_ADDR                             (0xA0)
#define HOST_EEPROM_IIC_ADDR                            (0xA4)

#define SA2_FW_BASE                                     (0x20015000) //(0x0801B900)
#define SA2_FW_SZ                                       (0x8000)

#define META_EVENT_OFFSET(x)                            (((x) - 1) << 1)
#define SENTRAL_A2_I2C_TIMEOUT                          (1000U)


#define PARAM_IO_RETRY_MAX                              (100)


#define EEPROM_WARMSTART_MAGIC                          (0xDEADF00D)

#define EEPROM_WARMSTART_MAGIC_ADDR                     (0x0)
#define EEPROM_WARMSTART_START                          (0x10)       // (0x08080000)
#define EEPROM_WARMSTART_END                            (0x190 - 1)

// app code re-red-efine
#define EEPROM_COFF_ACC_OFFSET_X                        (0x1A0)
#define EEPROM_COFF_ACC_OFFSET_Y                        (0x1A4)
#define EEPROM_COFF_ACC_OFFSET_Z                        (0x1A8)

#define EEPROM_COFF_ACC_RATIO_X                         (0x1AC)
#define EEPROM_COFF_ACC_RATIO_Y                         (0x1B0)
#define EEPROM_COFF_ACC_RATIO_Z                         (0x1B4)

#define EEPROM_COFF_GYRO_OFFSET_X                       (0x1C0)
#define EEPROM_COFF_GYRO_OFFSET_Y                       (0x1C4)
#define EEPROM_COFF_GYRO_OFFSET_Z                       (0x1C8)

#define EEPROM_COFF_GYRO_RATIO_X                        (0x1CC)
#define EEPROM_COFF_GYRO_RATIO_Y                        (0x1D0)
#define EEPROM_COFF_GYRO_RATIO_Z                        (0x1D4)

#define EEPROM_COFF_MAG_OFFSET_X                        (0x1E0)
#define EEPROM_COFF_MAG_OFFSET_Y                        (0x1E4)
#define EEPROM_COFF_MAG_OFFSET_Z                        (0x1E8)

#define EEPROM_COFF_MAG_RATIO_X                         (0x1EC)
#define EEPROM_COFF_MAG_RATIO_Y                         (0x1F0)
#define EEPROM_COFF_MAG_RATIO_Z                         (0x1F4)


// enum
enum
{
    ACTION_NONE,
    ACTION_RECALIBRATION,
    ACTION_SAVE_WARMSTART,

};

enum
{
    OPS_SAVE_WARMSTART,
    OPS_LOAD_WARMSTART,
};


// typedef
typedef struct
{
    uint8_t paramNo;
    uint8_t size;
} ParamInfo;


// 7180 data struct

typedef struct
{
    float    qx;
    float    qy;
    float    qz;
    float    qw;
    uint16_t qtime;

    int16_t  mx;
    int16_t  my;
    int16_t  mz;
    uint16_t mtime;

    int16_t  ax;
    int16_t  ay;
    int16_t  az;
    uint16_t atime;

    int16_t  gx;
    int16_t  gy;
    int16_t  gz;
    uint16_t gtime;
}stc_7186_output_t;


// functions

int mm2_init(void);

int mm2_set_parameter(uint8_t page, uint8_t num, uint8_t *data, uint8_t size);
int mm2_get_parameter(uint8_t page, uint8_t num, uint8_t *data, uint8_t size);

int mm_reg_write(uint8_t reg, uint8_t *pdat, uint16_t size);
int mm_reg_read(uint8_t reg, uint8_t *pdat, uint16_t size);


int mm_save_warmstart(void);
int mm_load_warmstart(void);


int mm_eeprom_read(uint32_t addr, uint8_t * pbuf, uint16_t length);
int mm_eeprom_write(uint16_t addr, uint8_t * pbuf, uint16_t length);
int mm_fw_load_check(void);

int mm2_read_fifo(uint8_t *pbuf, uint32_t max_sz);

int mm_set_passthrough(uint8_t enable);

int mm_set_sensor_rate(uint8_t sid, uint16_t rate);


// other def
// page1-31 sensor status 15 byte
#pragma pack(push, 1)

typedef struct
{
    uint16_t acc_rate;
    uint16_t acc_range;
    uint8_t  acc_flags;             // 0:int  5-7 power mode.

    uint16_t gyro_rate;
    uint16_t gyro_range;
    uint8_t  gyro_flags;

    uint16_t mag_rate;
    uint16_t mag_range;
    uint8_t  mag_flags;

}stc_sa2_page1_sensor_stauts_t;
#pragma pack(pop)


// page1 33-96 ,sensor info struct, each param contains 16 bytes data.
#pragma pack(push, 1)

typedef struct
{
    uint8_t  sensor_type;           // param -32
    uint8_t  driver_id;
    uint8_t  driver_version;
    uint8_t  current;
    uint16_t current_range;
    uint8_t  flags;                 // bit 0:   irq
                                    //     5-7 power mode
    uint8_t  rev1;
    uint16_t rate;
    uint8_t  naxis;                 // number of axis.  xyz= 3
    uint8_t  matrix[5];             // physical sensor coff & offset
}stc_sa2_page1_sensorinfo_t;
#pragma pack(pop)


// page3 sensor info  15
#pragma pack(push, 1)
typedef struct
{
    uint8_t  sensor_type;
    uint8_t  driver_id;
    uint8_t  driver_version;
    uint8_t  current;
    uint16_t max_range;
    uint16_t resolution;
    uint16_t max_rate;
    uint16_t fifo_rev;
    uint16_t fifo_max;
    uint8_t  event_sz;              // report size
    uint8_t  min_rate;

}stc_sa2_page3_sensorinfo_t;
#pragma pack(pop)

// page5, sensor config
#pragma pack(push, 1)
typedef struct
{
    uint16_t rate;
    uint16_t max_latancy;
    uint16_t sensitivity;
    uint16_t drange;

}stc_sa2_page5_sensorcfg_t;
#pragma pack(pop)


//

typedef union
{
    /**
     * \brief Direct access to all flags.
     */
    uint16_t value;                 /**< the 16 bit register value for the EEPROM flags register */
    struct bits
    {
        /** \brief Do not execute the EEPROM image immediately after upload */
        uint16_t EEPROMNoExec : 1;
        /** \brief Reserved */
        uint16_t Reserved : 7;
        /** \brief The clock speed to upload the firmware at */
        uint16_t I2CClockSpeed : 3;
        /** \brief Rom version expected */
        uint16_t ROMVerExp : 4;
        /** \brief Reserved */
        uint16_t reserved1 : 1;
    } bits;                         /**< the bit fields within the EEPROM flags register */
}FwFlags;


typedef struct
{
    uint8_t  imageSignatureLsb;
    uint8_t  imageSignatureMsb;
    FwFlags  flags;
    uint32_t crc;
    uint32_t reserved;
    uint16_t imageLength;
    uint16_t reserved2;
    uint8_t  data[0];
} FirmwareHeader;

//CHIP_STATUS_REG bits
#define EEDETECTED                                      (0x01)
#define EEUPLOADDONE                                    (0x02)
#define EEUPLOADERR                                     (0x04)
#define FWIDLE                                          (0x08)
#define NOEEPROM                                        (0x10)

#define MAX_EEPROM_I2C_WRITE                            64


enum
{
    SA2_RPOG_OK,
    SA2_RPOG_BIN_CHK_FAIL,
    SA2_RPOG_ERASE_EEP_FAIL,
    SA2_RPOG_WRITE_EEP_FAIL,

    SA2_RPOG_COMP_EEP_FAIL,
    SA2_RPOG_UNKNOWN_FAIL,


};


#endif

