#include <stdio.h>
#include <string.h>
#include <time.h>
#include <math.h>

#include "stm32f4xx_it.h"
#include "app_def.h"
#include "mm_types.h"
#include "mm_loop.h"
#include "mm_util.h"
#include "util.h"


#define MAX_FIFO_SIZE                                   (4 * 1024)


extern uint32_t irq_sets;

float baro2height(float fBaro);

extern __weak int user_algo(int sid, stc_mm_data_t *pMMdata);


uint8_t gfifo_buf[MAX_FIFO_SIZE]; // temp buffer to store A2 FIFO, bump up to 3000, this should be more than enough to handle Parking FIFO




static stc_app_t stcAppObj =
{
    .pRAM = (stc_app_ram_t *)APP_RAM_DATA_BASE,
};


stc_app_t * get_app(void)
{
    return &stcAppObj;
}


#if SENTRAL_TYPE_USING == SENTRAL_TYPE_7186


stc_mm_data_t stc_mm_data;

static int gmm_inited = 0;

static char * SID_NAME[256] = { 0 };

#define SAMPLE_RATE                 (50)


char * getsidname(int sid)
{
    return SID_NAME[sid] != NULL ? SID_NAME[sid] : "?";
}


int mm2_inited_callback(void)
{
    int ret = HAL_OK;
    int i = 0;
    stc_app_t *pApp = get_app();

    stc_mm_data_t *pMMdata = &pApp->mmdat;

    if (1 == gmm_inited)
    {
        APP_DBG("mm is already inited...\n");
        return HAL_BUSY;
    }

    gmm_inited = 1;

    for (i = 0; i < SID_MAXSZ; i++)
    {
        pMMdata->skipcnt[i] = 0;
    }

    memset(&stc_mm_data, 0x0, sizeof(stc_mm_data));

    SID_NAME[SENSOR_TYPE_ACCELEROMETER] = "acc";
    SID_NAME[SENSOR_TYPE_MAGNETIC_FIELD] = "mag";
    SID_NAME[SENSOR_TYPE_ORIENTATION] = "hpr";
    SID_NAME[SENSOR_TYPE_GYROSCOPE] = "gyro";
    SID_NAME[SENSOR_TYPE_ROTATION_VECTOR] = "vect";
    SID_NAME[SENSOR_TYPE_PRESSURE] = "baro";
    SID_NAME[SENSOR_TYPE_LINEAR_ACCELERATION] = "lacc";

//    SID_NAME[SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED] = "u-mag";

    int sid = SENSOR_TYPE_ORIENTATION;

//    sid = SENSOR_TYPE_GYROSCOPE;
//    ret = mm_set_sensor_rate(sid, 104);
//    APP_DBG("enable sid=%d, ret=%d\n", sid, ret);

//    sid = SENSOR_TYPE_ACCELEROMETER;
//    ret = mm_set_sensor_rate(sid, 50);
//    printf("enable sid=%d, ret=%d\n", sid, ret );

//      sid = SENSOR_TYPE_ORIENTATION;
//      ret = mm_set_sensor_rate(sid, 50);
//      APP_DBG("enable sid=%d, ret=%d\n", sid, ret);

    sid = SENSOR_TYPE_PRESSURE;
    ret = mm_set_sensor_rate(sid, 1);
    APP_DBG("enable sid=%d, ret=%d\n", sid, ret);
//
//    sid = SENSOR_TYPE_ACCELEROMETER;
//    ret = mm_set_sensor_rate(sid, 15);
//    printf("enable sid=%d, ret=%d\n", sid, ret );
//
    sid = SENSOR_TYPE_ROTATION_VECTOR;
    ret = mm_set_sensor_rate(sid, 200);
    APP_DBG("enable sensor %d(%s), ret=%d\n", sid, getsidname(sid), ret);

//    sid = SENSOR_TYPE_MAGNETIC_FIELD;
//    ret = mm_set_sensor_rate(sid, 50);
//    APP_DBG("enable sensor %d(%s), ret=%d\n", sid, getsidname(sid), ret);


//    sid = 0x15;
//    ret = mm_set_sensor_rate(sid, 1);
//    APP_DBG("enable sid=%d, ret=%d\n", sid, ret );


    return HAL_OK;

}


#define SKIPCNT             (2)
#define SKIP_WS_CNT         (300)
int skip_ws_cnt = 0;

int mm2_parse_fifo(uint8_t * pbuf, uint32_t fifo_sz)
{
    int offset = 0;
    int i = 0;
    uint8_t sid = 0;

    uint8_t pkt_len = 0;
    uint8_t * pPkt = NULL;
    stc_app_t *pApp = get_app();
    stc_mm_data_t *pMMdata = &pApp->mmdat;

    //hexdump("parse-> ", pbuf, fifo_sz);

    while (offset < fifo_sz)
    {
        sid = pbuf[offset];
        pPkt = &pbuf[offset];
        pkt_len = 1;


        if (++pMMdata->skipcnt[sid] == 1 && sid < 32)
        {
            APP_DBG("sid-%4s[%d]: ", getsidname(sid), sid);
        }

        switch (sid)
        {
            case SENSOR_TYPE_PADDING:
                pkt_len = 1;
                break;

            case SENSOR_TYPE_TIMESTAMP: // 3
            {
                uint8_t * p = (uint8_t *)&pMMdata->ts;

                p[0] = pPkt[1];
                p[1] = pPkt[2];
                pkt_len = 3;
                break;
            }

            case SENSOR_TYPE_TIMESTAMP_OVERFLOW:
            {
                uint8_t * p = (uint8_t *)&pMMdata->ts;

                p[2] = pPkt[1];
                p[3] = pPkt[2];
                pkt_len = 3;
                break;
            }

            case SENSOR_TYPE_TIMESTAMP_WAKE:
            case SENSOR_TYPE_TIMESTAMP_OVERFLOW_WAKE:
                pkt_len = 3;
                break;

            case SENSOR_TYPE_DEBUG:

                /* don't push timestamp packets */
                // pkt_len = pPkt[1] & 0x6F;        // strlen
                //APP_DBG("dbg> %s\n", &pPkt[2]);

                pkt_len = 14;           // pktlen fix=14
                break;

            case SENSOR_TYPE_META:
            case SENSOR_TYPE_META_WAKE:
            {

                uint8_t mid = pPkt[1];

                APP_DBG("meta event: %#x, %#x %#x\n", mid, pPkt[2], pPkt[3]);
                switch (mid)
                {
                    case META_EVENT_INITIALIZED:
                    {
                        uint16_t v = 0;

                        v = pPkt[2] + pPkt[3] * 256;
                        APP_DBG("RAM version chk: %d\n\n", v);

                        mm2_inited_callback();
                    }
                }

                pkt_len = 4;
                break;
            }

            case SENSOR_TYPE_ACCELEROMETER:
            case SENSOR_TYPE_GRAVITY:
            case SENSOR_TYPE_LINEAR_ACCELERATION:
            {
                /* output:  sensor raw data
                   format:  x:s16, y:s16 z:s16  status(u8)
                   unit:    g(m/s2) = val/scale * 9.8, scale default=8192
                 */

                short raw[4] = { 0 };
                uint8_t status = 0;
                const float fSCALE = 8192;

                raw[0] = (short)pPkt[1] + (short)pPkt[2] * 256;
                raw[1] = (short)pPkt[3] + (short)pPkt[4] * 256;
                raw[2] = (short)pPkt[5] + (short)pPkt[6] * 256;
                status = pPkt[7];

                // right hand
                raw[3] = raw[0];

                raw[0] = -raw[1];
                raw[1] = raw[3];
                raw[2] = -raw[2];

                if (pApp->runtime.UserAction)
                {
                    for (int i = 0; i < AXIS_NUM; i++)
                    {
                        if (raw[i] > 8400 || raw[i] < -8400)
                        {
                            // out of range. skip it.
                            continue;
                        }

                        pMMdata->acc.dat[i] = pMMdata->acc.dat[i] * MMA_RATIO + (float)raw[i] / fSCALE * (1 - MMA_RATIO);
                    }

                }else
                {
                    pMMdata->acc.dat[0] = (float)raw[0] / fSCALE;
                    pMMdata->acc.dat[1] = (float)raw[1] / fSCALE;
                    pMMdata->acc.dat[2] = (float)raw[2] / fSCALE;
                }

                //user_calc_distance(pMMdata);

                if (pMMdata->skipcnt[sid] > SKIPCNT)
                {
//                    APP_DBG("%6d, %6d, %6d,  s= %d, t=%.4f\n", raw[0], raw[1], raw[2], status,
//                                                (float)pMMdata->ts / 32000);

                    APP_DBG("%8.3f, %8.3f, %8.3f, s= %d, t=%.4f\n", pMMdata->acc.dat[0],
                            pMMdata->acc.dat[1],
                            pMMdata->acc.dat[2],
                            status, (float)pMMdata->ts / 32000);

                    pMMdata->skipcnt[sid] = 0;
                }


                pkt_len = 8;
                break;
            }

            case SENSOR_TYPE_MAGNETIC_FIELD:
            {
                /* output:  sensor raw data
                   format:  x:s16, y:s16 z:s16  status(u8)
                   unit:    mag(uT) = val/scale   scale default=75
                 */

                short raw[3] = { 0 };
                uint8_t status = 0;
                const float fSCALE = 75;

                raw[0] = (short)pPkt[1] + (short)pPkt[2] * 256;
                raw[1] = (short)pPkt[3] + (short)pPkt[4] * 256;
                raw[2] = (short)pPkt[5] + (short)pPkt[6] * 256;
                status = pPkt[7];

                pMMdata->mag.dat[0] = (float)raw[0] / fSCALE;
                pMMdata->mag.dat[1] = (float)raw[1] / fSCALE;
                pMMdata->mag.dat[2] = (float)raw[2] / fSCALE;

                if (pMMdata->skipcnt[sid] > SKIPCNT)
                {
//                    APP_DBG("%6d, %6d, %6d,  s= %d, t=%.4f\n", raw[0], raw[1], raw[2], status,
//                                                (float)pMMdata->ts / 32000);

                    APP_DBG("%.3f, %.3f, %.3f, s= %d, t=%.4f\n", pMMdata->mag.dat[0],
                            pMMdata->mag.dat[1],
                            pMMdata->mag.dat[2],
                            status, (float)pMMdata->ts / 32000);

                    pMMdata->skipcnt[sid] = 0;
                }

                pkt_len = 8;
                break;
            }


            case SENSOR_TYPE_ORIENTATION:
            {
                /* output:  sensor raw data
                   format:  x:s16, y:s16 z:s16  status(u8)
                   unit:    hpr(degree) = val/scale, scale default=360/32768.

                   calibration: make figure8 pattern gesture, output is valid until status = 3.
                 */

                short r[AXIS_NUM] = { 0 };
                uint8_t status = 0;
                const float fSCALE = 360.0 / 32768;

                r[0] = (short)pPkt[1] + (short)pPkt[2] * 256;
                r[1] = (short)pPkt[3] + (short)pPkt[4] * 256;
                r[2] = (short)pPkt[5] + (short)pPkt[6] * 256;
                status = pPkt[7];

                for (i = 0; i < AXIS_NUM; i++)
                {
                    pMMdata->hpr.dat[i] = (float)r[i] * fSCALE;

                }
                pMMdata->hpr.dat[0] += 180;         // make -180 +180  -> 0 ~360

                if (pMMdata->skipcnt[sid] > SKIPCNT)
                {
                    //sa2_read_page2_warmstart(pModule);
                    APP_DBG("%8.3f,%8.3f,%8.3f,  s=%d\n",
                            pMMdata->hpr.dat[0],
                            pMMdata->hpr.dat[1],
                            pMMdata->hpr.dat[2], status);

                    pMMdata->skipcnt[sid] = 0;
                }

                pkt_len = 8;
                break;
            }


            case SENSOR_TYPE_GYROSCOPE:
            {
                /* output:  sensor gyro raw data
                   format:  x:s16, y:s16 z:s16  status(u8)
                   unit:    d(dps) = val/scale, scale default=32768/2000.
                 */

                short r[3] = { 0 };
                uint8_t status = 0;
                const float fSCALE = 32768.0 / 2000;

                r[0] = (short)pPkt[1] + (short)pPkt[2] * 256;
                r[1] = (short)pPkt[3] + (short)pPkt[4] * 256;
                r[2] = (short)pPkt[5] + (short)pPkt[6] * 256;
                status = pPkt[7];

                for (i = 0; i < AXIS_NUM; i++)
                {
                    // conver dps to degree.  gyro default range is 2000.
                    pMMdata->gyro.dat[i] = ((float)r[i]) / fSCALE;
                }

                if (pMMdata->gyro_init_cal_cnt < 400)
                {
                    for (i = 0; i < AXIS_NUM; i++)
                    {
                        pMMdata->gyro_cal_offset.dat[i] += (float)r[i];
                    }
                    pMMdata->gyro_init_cal_cnt += 1;

                }else if (pMMdata->gyro_init_cal_cnt == 400 && pApp->runtime.bUseCalDat == 0)
                {
                    for (i = 0; i < AXIS_NUM; i++)
                    {
                        pMMdata->gyro_cal_offset.dat[i] = pMMdata->gyro_cal_offset.dat[i] / 400 / 2000;
                    }

                    APP_DBG("gyro offset > %.4f, %.4f, %.4f\n",
                            pMMdata->gyro_cal_offset.dat[0],

                            pMMdata->gyro_cal_offset.dat[1],
                            pMMdata->gyro_cal_offset.dat[2]);

                    pMMdata->gyro_init_cal_cnt += 1;
                    host_eeprom_write(EEPROM_COFF_GYRO_OFFSET_X, (uint8_t *)&pMMdata->gyro_cal_offset.dat[0], 4);
                    host_eeprom_write(EEPROM_COFF_GYRO_OFFSET_Y, (uint8_t *)&pMMdata->gyro_cal_offset.dat[1], 4);
                    host_eeprom_write(EEPROM_COFF_GYRO_OFFSET_Z, (uint8_t *)&pMMdata->gyro_cal_offset.dat[2], 4);

                }else
                {

                    // integral test.
                    for (i = 0; i < AXIS_NUM; i++)
                    {
                        // integral， 52 = sample rate
                        if (fabs(pMMdata->gyro.dat[i]) < 1)
                        {
                            continue;
                        }

                        pMMdata->rot.dat[i] += pMMdata->gyro.dat[i] * (1.0 / 104);
                    }
                }

                if (pMMdata->skipcnt[sid] > SKIPCNT)
                {
                    APP_DBG("%6d, %6d, %6d,  s= %d, t=%.3f\n", r[0], r[1], r[2], status, (float)pMMdata->ts / 32000);
                    APP_DBG("rot> %.2f, %.2f, %.2f\n", pMMdata->rot.dat[0], pMMdata->rot.dat[1], pMMdata->rot.dat[2]);
                    pMMdata->skipcnt[sid] = 0;
                }


                pkt_len = 8;
                break;
            }


            case SENSOR_TYPE_ROTATION_VECTOR:
            {
                /* output:  quanternion
                   format:  x:s16, y:s16 z:s16 w:s16  status(u16)
                   unit:    v(-) = v/scale         scale =2^14= 16384
                 */

                short r[5] = { 0 };
                const float fSCALE = 16384;

                r[0] = (short)pPkt[1] + (short)pPkt[2] * 256;                 // XYZW
                r[1] = (short)pPkt[3] + (short)pPkt[4] * 256;
                r[2] = (short)pPkt[5] + (short)pPkt[6] * 256;
                r[3] = (short)pPkt[7] + (short)pPkt[8] * 256;
                r[4] = (short)pPkt[9] + (short)pPkt[10] * 256;                // accuracy < 0.1

                for (i = 0; i < AXIS_NUM + 1; i++)
                {
                    pMMdata->quan.dat[i] = (float)r[i] / fSCALE;
                }

                float ac = (float)r[4] / fSCALE;

                if (pMMdata->skipcnt[sid] > SKIPCNT)
                {
                    APP_DBG("%.4f, %.4f, %.4f, %.4f , ac=%.3f ,t=%.3f, %8.2f\n",
                            pMMdata->quan.dat[0], pMMdata->quan.dat[1], pMMdata->quan.dat[2], pMMdata->quan.dat[3],
                            ac, (float)pMMdata->ts / 32000, pMMdata->baro);

                    pMMdata->skipcnt[sid] = 0;
                }

                pkt_len = 11;
                break;

            }


//            case SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED:
//            case SENSOR_TYPE_GYROSCOPE_UNCALIBRATED:
//            {
//                short val[6] = { 0 };
//                int i = 0;
//                uint8_t status = 0;
//
//                // xyz + bias xyz
//                for (i = 0; i < 6; i++)
//                {
//                    val[i] = (short)pPkt[i * 2 + 1] + (short)pPkt[i * 2 + 2] * 256;
//                }
//
//                status = pPkt[12];
//
//                if (pMMdata->skipcnt[sid] > 50)
//                {
//                    APP_DBG("%d, %d, %d,  %d, %d, %d,   s= %d,\n",
//                            val[0], val[1], val[2], val[3], val[4], val[5], status);
//
//                    pMMdata->skipcnt[sid] = 0;
//                }
//
//                pkt_len = 14;
//                break;
//            }

            case SENSOR_TYPE_PRESSURE:
            {
                /* output:  pressure sensor raw data.
                   format:  p:s24
                   unit:    v(Pa) = v/scale         scale =128
                 */

                int r = 0;
                const float fSCALE = 128.0;

                r = (int)pPkt[1] + (int)pPkt[2] * 256 + (int)pPkt[3] * 256 * 256;

                float fx = (float)r / fSCALE / 100; // hPa
                float h = baro2height(fx);          // relative height.

                //APP_DBG("%.3f, h=%.3f\n", fx, h);
                pMMdata->baro = fx;
                pkt_len = 4;
                break;
            }


            default:
                APP_E("unknown sid :%d, ofs=%d\n", sid, offset);
                pkt_len = 1;
                hexdump("fifo> ", &pbuf[offset], fifo_sz - offset);
                break;
        }


        offset += pkt_len;

        if (++skip_ws_cnt > SKIP_WS_CNT)
        {
            skip_ws_cnt = 0;
        }

        //user_algo(sid, pMMdata);


    }

    return HAL_OK;
}


int mm2_loop(void)
{

    int ret = HAL_OK;

    int read_len = 0;

   

    stc_app_t *pApp = get_app();
    // stc_mm_data_t *pMMdata = &pApp->mmdat;

    int gpio_ignore = TRUE;
    int fifo0_err_cnt = 0;

    APP_DBG("MM start loop.....\n");

    while (1)
    {

#ifdef MM_USING_GPIO6
        gpio_ignore = FALSE;
        // can change to irq mode.
        ret = CHECK_BITS(irq_sets, IRQ_EVT_SA2);
        ret |= HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
        CLEAR_BITS(irq_sets, IRQ_EVT_SA2);

        if (0 == ret && FALSE == gpio_ignore)
        {
            hal_delay(2);
            continue;
        }

        //APP_DBG("irqcnt %8d\n", ++irqcnt);
#else
        ret = 0;
        mm_reg_read(INT_STATUS_REG, (uint8_t *)&ret, 1);
        ret = ret & 0x1; // bit0: host int

        if (0 == ret)
        {
            hal_delay(2);
            continue;
        }
#endif


        // get data
        memset(gfifo_buf, 0x0, sizeof(gfifo_buf));
        read_len = 0;
        read_len = mm2_read_fifo(&gfifo_buf[read_len], sizeof(gfifo_buf));


        if (read_len < 1 && FALSE == gpio_ignore)
        {
            fifo0_err_cnt += 1;

            if (fifo0_err_cnt > 50)
            {
                APP_E("fifo_size continous err, len=%d\n", read_len);
                fifo0_err_cnt = 0;
            }
            continue;
        }

        fifo0_err_cnt = 0;


        Clock_Wait(0);   // refresh wdg.


        ret = mm2_parse_fifo(&gfifo_buf[0], read_len);


        if (BTN1_DOWN() && ACTION_NONE == pApp->runtime.UserAction)
        {
            // mm_save_warmstart();
        }

    }

    return ret;
}


#endif


float baro2height(float fBaro)
{
    static int i = 0;
    static float base_bar = 0;
    float h = 0;

    // =44300*(1-(63.64/101.325)^(1/5.256))
    if (i <= 3)
    {
    }else if (i < 14)
    {
        // set base.
        base_bar += fBaro;
        if (i == 13)
        {
            base_bar /= 10;
            APP_DBG("base baro = %.2f\n", base_bar);
        }
    }else
    {
        h = 44300 * (1 - powf(fBaro / base_bar /*101.325*/, 1 / 5.256));
    }

    i++;

    return h;


}


void app_reboot(uint16_t reason, uint8_t b_now)
{

    APP_E("host app reboot %X\n", reason);
    stc_app_t * pApp = get_app();

    pApp->pRAM->app_reset_reason = reason;

    Clock_Wait(10);
    HAL_NVIC_SystemReset();

}

