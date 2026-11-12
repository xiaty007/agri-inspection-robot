#ifndef __MM_TYPES_H__
#define __MM_TYPES_H__


#define SENTRAL_TYPE_7180                               0
#define SENTRAL_TYPE_7184                               4
#define SENTRAL_TYPE_7186                               6


//#define SENTRAL_TYPE_USING                              SENTRAL_TYPE_7186


#if SENTRAL_TYPE_USING == SENTRAL_TYPE_7180
#define  SENTRAL_ROM_VERSION                            (0x09E6) // 7180
#elif SENTRAL_TYPE_USING == SENTRAL_TYPE_7184
#define  SENTRAL_ROM_VERSION                            (0x1F9D) // 7184
#elif SENTRAL_TYPE_USING == SENTRAL_TYPE_7186
#define  SENTRAL_ROM_VERSION                            (0x27F2) // 7186 A2
#else
#error "invalid sentral type defined"
#endif

// sentral reg values.
#define  SENTRAL_A2_PROD_ID                             0x86     // A2


// FIRMWARE CONSTANTS
#define FIRMWARE_HEADER_SIZE                            16
#define MAX_FIRMWARE_SIZE                               32768
#define IMAGE_SIGNATURE_LSB                             0x2A
#define IMAGE_SIGNATURE_MSG                             0x65


// SA2 REGISTER MAP
#define FIFO_START                                      0x0
#define FIFO_FLUSH_REG                                  0x32     // flush sensor data from fifo
#define CHIP_CONTROL_REG                                0x34     // enable cpu
#define HOST_STATUS_REG                                 0x35     // host status
#define INT_STATUS_REG                                  0x36     // interrupt status
#define CHIP_STATUS_REG                                 0x37     // chip status
#define BYTES_REMANING_REG                              0x38     // LSB of remaing bytes in FIFO
#define PARAM_ACK_REG                                   0x3A
#define PARAM_SAVE_REG                                  0x3B
#define ERR_REG                                         0x50     // error register
#define INT_STATE_REG                                   0x51     // interrupt state
#define DEBUG_VAL_REG                                   0x52     // debug value
#define DEBUG_STATE_REG                                 0x53     // debug state
#define PARAM_PAGE_SELECT_REG                           0x54     // parameter page select (and transfer size)
#define HOST_INTERFACE_CTRL_REG                         0x55
#define PARAM_LOAD_REG                                  0x5C
#define PARAM_REQUEST_REG                               0x64
#define HOST_IRQ_TIME_REG                               0x6C
#define ROM_VERSION_REG                                 0x70
#define RAM_VERSION_REG                                 0x72
#define PRODUCT_ID_REG                                  0x90     // Product ID  7180 = 0x80
                                                                 //
#define REVISION_ID_REG                                 0x91     // Product ID
#define UPLOAD_ADDRESS_REG                              0x94     // u16, Initialize RAM address
#define SR_UPLOAD_DATA_REG                              0x96     // Firmware upload address
#define HOST_CRC_REG                                    0x97     // Firmware upload CRC
#define RESET_REQ_REG                                   0x9B     // Request system reset
#define PASS_THROUGH_RDY_REG                            0x9E
#define SCL_LOW_CYCLES_REG                              0x9F
#define PASS_THROUGH_CFG_REG                            0xA0


// 7180 register update.
#if SENTRAL_TYPE_USING == SENTRAL_TYPE_7180

#define Q_DATA_REG                                      0x0      // float *4
#define Q_TIME_REG                                      0x10     // u16

#define MAG_DATA_REG                                    0x12     // i16*3
#define MAG_TIME_REG                                    0x18

#define ACCEL_DATA_REG                                  0x1A     // i16*3
#define ACCEL_TIME_REG                                  0x20

#define GYRO_DATA_REG                                   0x22     // i16*3
#define GYRO_TIME_REG                                   0x28


#define QDIV_RATE_REG                                   0x32     // along with gyro

#define ENABLE_EVENT_REG                                0x33
#define HOST_CTRL_REG                                   0x34
#define EVENT_STAT_REG                                  0x35

#define ALGO_STAT_REG                                   0x38     // bit1~7 not working.


#define ALGO_CTRL_REG                                   0x54
#define MAG_RATE_REG                                    0x55
#define ACCEL_RATE_REG                                  0x56
#define GYRO_RATE_REG                                   0x57


#define ACCX_SCALE_REG                                  0x5B     // 2 reg
#define ACCY_SCALE_REG                                  0x5D
#define ACCZ1_SCALE_REG                                 0x5F
#define ACCZ2_SCALE_REG                                 0x69

#define ACCX_OFFSET_REG                                 0x6A     // 2 reg
#define ACCY_OFFSET_REG                                 0x6C
#define ACCZ_OFFSET_REG                                 0x6E


#undef PARAM_LOAD_REG
#define PARAM_LOAD_REG                                  0x60


#endif


// register values.
// FIFO FLUSH
#define FIFO_FLUSH_DISCARD_NON_WAKE                     0xFA
#define FIFO_FLUSH_DISCARD_WAKE                         0xFB
#define FIFO_FLUSH_TRANSFER_NON_WAKE                    0xFC
#define FIFO_FLUSH_TRANSFER_WAKE                        0xFD
#define FIFO_FLUSH_DISCARD_ALL                          0xFE
#define FIFO_FLUSH_TRANSFER_ALL                         0xFF

// CHIP CONTROL
#define CHIP_CONTROL_CPU_RUN                            0x01
#define CHIP_CONTROL_HOST_UPLOAD                        0x02

// HOST STATUS
#define HOST_STATUS_RESET                               0X01
#define HOST_STATUS_ALGORITHM_STANDBY                   0x02
#define HOST_STATUS_INTERFACE_BITS                      0x1C
#define HOST_STATUS_INTERFACE_K                         0x00
#define HOST_STATUS_INTERFACE_L                         0x04
#define HOST_STATUS_INTERFACE_L_EXTENDED                0x08
#define HOST_STATUS_ALGORITHM_BITS                      0xE0
#define HOST_STATUS_ALGORITHM_SPACEPOINT                0x20

// HOST INTERFACE CONTROL
#define HOST_IFACE_CTRL_FLAG_ALGORITHM_STANDBY          0x01
#define HOST_IFACE_CTRL_FLAG_ABORT_TRANSFER             0x02
#define HOST_IFACE_CTRL_FLAG_UPDATE_TRANSFER_CNT        0x04
#define HOST_IFACE_CTRL_FLAG_WAKE_FIFO_INT_DISABLE      0x08
#define HOST_IFACE_CTRL_FLAG_NED_COORDINATES            0x10
#define HOST_IFACE_CTRL_FLAG_AP_SUSPEND                 0x20
#define HOST_IFACE_CTRL_FLAG_REQ_SENSOR_SELF_TEST       0x40
#define HOST_IFACE_CTRL_FLAG_NON_WAKE_FIFO_INT_DISABLE  0x80

// PARAM IO PAGES
#define PARAM_PAGE_SYSTEM                               1
#define PARAM_PAGE_WARM_START                           2
#define PARAM_PAGE_SENSOR_INFO                          3
#define PARAM_PAGE_SENSOR_CONF                          5
#define PARAM_PAGE_9                                    9
#define PARAM_PAGE_KNOBS                                13
#define PARAM_PAGE_GPS_DATA                             14
#define PARAM_PAGE_FW_VER                               15

// SYSTEM PARAMETERS
#define PARAM_META_EVENT_CONTROL                        1
#define PARAM_FIFO_CONTROL                              2
#define PARAM_SENSOR_STATUS_BANK_0                      3
#define SENSORS_PER_STATUS_BANK                         16
#define PARAM_WAKE_META_EVENT_CONTROL                   29
#define PARAM_HOST_IRQ_TIMESTAMP                        30
#define PARAM_PHYSICAL_SENSOR_STATUS                    31
#define PARAM_PHYSICAL_SENSOR_PRESENT                   32


// WARM_START PARAMETERS
#define PARAM_RAW_SENSOR_CONTROL                        127

#define RAW_ACCEL_CONTROL_BIT                           0x01
#define RAW_MAG_CONTROL_BIT                             0x02
#define RAW_GYRO_CONTROL_BIT                            0x04

#define SENSOR_TYPE_PADDING                             0
#define SENSOR_TYPE_ACCELEROMETER                       1
#define SENSOR_TYPE_MAGNETIC_FIELD                      2
#define SENSOR_TYPE_ORIENTATION                         3
#define SENSOR_TYPE_GYROSCOPE                           4
#define SENSOR_TYPE_LIGHT                               5
#define SENSOR_TYPE_PRESSURE                            6
#define SENSOR_TYPE_TEMPERATURE                         7
#define SENSOR_TYPE_PROXIMITY                           8
#define SENSOR_TYPE_GRAVITY                             9
#define SENSOR_TYPE_LINEAR_ACCELERATION                 10
#define SENSOR_TYPE_ROTATION_VECTOR                     11
#define SENSOR_TYPE_RELATIVE_HUMIDITY                   12
#define SENSOR_TYPE_AMBIENT_TEMPERATURE                 13
#define SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED         14
#define SENSOR_TYPE_GAME_ROTATION_VECTOR                15
#define SENSOR_TYPE_GYROSCOPE_UNCALIBRATED              16
#define SENSOR_TYPE_SIGNIFICANT_MOTION                  17
#define SENSOR_TYPE_STEP_DETECTOR                       18
#define SENSOR_TYPE_STEP_COUNTER                        19
#define SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR         20
#define SENSOR_TYPE_HEART_RATE                          21

#define SENSOR_TYPE_CAR_DETECTOR                        21
#define SENSOR_TYPE_TILT_DETECTOR                       22
#define SENSOR_TYPE_WAKE_GESTURE                        23
#define SENSOR_TYPE_GLANCE_GESTURE                      24
#define SENSOR_TYPE_PICK_UP_GESTURE                     25
#define SENSOR_TYPE_TURNOVER                            26

#define SENSOR_TYPE_SHAKE                               27

#define SENSOR_TYPE_RAW_MAGNETIC_FIELD                  29
#define SENSOR_TYPE_ACTIVITY                            31
#define SENSOR_TYPE_CAR_MAG_DATA                        32
#define SENSOR_TYPE_CAR_PARAMETER_REPORT                33
#define SENSOR_TYPE_VISIBLE_END                         63
// SENtrace virtual sensors
#define SENSOR_TYPE_ALTITUDE_FUSION                     21
#define SENSOR_TYPE_ALTITUDE                            22
#define SENSOR_TYPE_PDR                                 23
#define SENSOR_TYPE_PDR_DISTANCE                        24
#define SENSOR_TYPE_PDR_PACE                            25
#define SENSOR_TYPE_PDR_PATH                            26
#define SENSOR_TYPE_PDR_EST_POSITION                    27
#define SENSOR_TYPE_PDR_PRE_POSITION                    28
#define SENSOR_TYPE_GPS_REPORT                          29
#define SENSOR_TYPE_GPS_REPORT_2                        30
// FITNESS
#define SENSOR_TYPE_FITNESS_ACTIVITY                    29

#define SENSOR_TYPE_WAKE_OFFTSET                        64

#define SENSOR_TYPE_ACCELEROMETER_WAKE                  65
#define SENSOR_TYPE_MAGNETIC_FIELD_WAKE                 66
#define SENSOR_TYPE_ORIENTATION_WAKE                    67
#define SENSOR_TYPE_GYROSCOPE_WAKE                      68
#define SENSOR_TYPE_LIGHT_WAKE                          69
#define SENSOR_TYPE_PRESSURE_WAKE                       70
#define SENSOR_TYPE_TEMPERATURE_WAKE                    71
#define SENSOR_TYPE_PROXIMITY_WAKE                      72
#define SENSOR_TYPE_GRAVITY_WAKE                        73
#define SENSOR_TYPE_LINEAR_ACCELERATION_WAKE            74
#define SENSOR_TYPE_ROTATION_VECTOR_WAKE                75
#define SENSOR_TYPE_RELATIVE_HUMIDITY_WAKE              76
#define SENSOR_TYPE_AMBIENT_TEMPERATURE_WAKE            77
#define SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED_WAKE    78
#define SENSOR_TYPE_GAME_ROTATION_VECTOR_WAKE           79
#define SENSOR_TYPE_GYROSCOPE_UNCALIBRATED_WAKE         80
#define SENSOR_TYPE_SIGNIFICANT_MOTION_WAKE             81
#define SENSOR_TYPE_STEP_DETECTOR_WAKE                  82
#define SENSOR_TYPE_STEP_COUNTER_WAKE                   83
#define SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR_WAKE    84
#define SENSOR_TYPE_HEART_RATE_WAKE                     85
#define SENSOR_TYPE_TILT_DETECTOR_WAKE                  86
#define SENSOR_TYPE_WAKE_GESTURE_WAKE                   87
#define SENSOR_TYPE_GLANCE_GESTURE_WAKE                 88
#define SENSOR_TYPE_PICK_UP_GESTURE_WAKE                89
#define SENSOR_TYPE_ACTIVITY_WAKE                       95
// SENtrace virtual sensors
#define SENSOR_TYPE_ALTITUDE_FUSION_WAKE                85
#define SENSOR_TYPE_ALTITUDE_WAKE                       86
#define SENSOR_TYPE_PDR_WAKE                            87
#define SENSOR_TYPE_PDR_DISTANCE_WAKE                   88
#define SENSOR_TYPE_PDR_PACE_WAKE                       89
#define SENSOR_TYPE_PDR_PATH_WAKE                       90
#define SENSOR_TYPE_PDR_EST_POSITION_WAKE               91
#define SENSOR_TYPE_PDR_PRE_POSITION_WAKE               92
#define SENSOR_TYPE_GPS_REPORT_WAKE                     93
#define SENSOR_TYPE_GPS_REPORT_2_WAKE                   94

#define SENSOR_TYPE_RESETEVT                            100


#define SENSOR_TYPE_DEBUG                               245
#define SENSOR_TYPE_TIMESTAMP_WAKE                      246
#define SENSOR_TYPE_TIMESTAMP_OVERFLOW_WAKE             247
#define SENSOR_TYPE_META_WAKE                           248
#define SENSOR_TYPE_RAW_GYRO                            249
#define SENSOR_TYPE_RAW_MAG                             250
#define SENSOR_TYPE_RAW_ACCEL                           251
#define SENSOR_TYPE_TIMESTAMP                           252
#define SENSOR_TYPE_TIMESTAMP_OVERFLOW                  253
#define SENSOR_TYPE_META                                254
#define SENSOR_TYPE_MAX                                 255

#define META_EVENT_FLUSH_COMPLETE                       1
#define META_EVENT_SAMPLE_RATE_CHANGED                  2
#define META_EVENT_POWER_MODE_CHANGED                   3
#define META_EVENT_ERROR                                4
#define META_EVENT_MAG_TRANSIENT_CHANGED                5
#define META_EVENT_CAL_STATUS_CHANGED                   6
#define META_EVENT_STILLNESS_CHANGED                    7
#define META_EVENT_CALIBRATION_STABLE                   9
#define META_EVENT_ACCEL_CAL                            10
#define META_EVENT_SENSOR_EVENT                         11
#define META_EVENT_FIFO_OVERFLOW                        12
#define META_EVENT_DYNAMIC_RANGE_CHANGED                13
#define META_EVENT_FIFO_WATERMARK                       14
#define META_EVENT_SELF_TEST_RESULT                     15
#define META_EVENT_INITIALIZED                          16
#define META_EVENT_TRANSFER_CAUSE                       17


#endif // __MM_TYPES_H__
