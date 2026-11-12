#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "board_config.h"

#include "app_def.h"
#include "mm_types.h"
#include "mm_util.h"
#include "util.h"
#include "hal_i2c.h"


extern stc_app_t * get_app(void);


void mm2_gpio_init(void);

static int mm2_param_write(uint8_t *values, uint8_t page, ParamInfo *paramList, uint8_t numParams);
static int mm2_param_read(uint8_t *values, uint8_t page, ParamInfo *paramList, uint8_t numParams);

int sa2_eeprom_fw_compare(uint32_t src_addr);
int sa2_upgrade_fw(uint32_t src_addr);


int mm_eeprom_read(uint32_t addr, uint8_t * pbuf, uint16_t length)
{
    return HAL_I2C_Mem_Read(&SENTRAL_A2_I2C, SA2_EEPROM_IIC_ADDR, addr, I2C_MEMADD_SIZE_16BIT, pbuf, length, 200);
}


int mm_eeprom_write(uint16_t addr, uint8_t * pbuf, uint16_t length)
{
    int ret = HAL_I2C_Mem_Write(&SENTRAL_A2_I2C, SA2_EEPROM_IIC_ADDR, addr, I2C_MEMADD_SIZE_16BIT, pbuf, length, 200);
    Clock_Wait(5);

    return ret;
}

int mm_reg_write(uint8_t reg, uint8_t *pdat, uint16_t size)
{
    // platform depedent
    return hal_i2c_write(0, SA2_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, pdat, size);

}

int mm_reg_read(uint8_t reg, uint8_t *pdat, uint16_t size)
{
    // platform depedent
    return hal_i2c_read(0, SA2_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, pdat, size);

}


int mm2_set_parameter(uint8_t page, uint8_t num, uint8_t *data, uint8_t size)
{
    ParamInfo info[] = { { num, size }, };

    return mm2_param_write(data, page, info, 1);
}


int mm2_get_parameter(uint8_t page, uint8_t num, uint8_t *data, uint8_t size)
{
    ParamInfo info[] = { { num, size }, };

    return mm2_param_read(data, page, info, 1);
}




int mm_set_passthrough(uint8_t enable)
{
    int ret = HAL_ERROR;
    enable = !!enable;
    mm_reg_write(PASS_THROUGH_CFG_REG, &enable, 1);

    int cnt = 0;
    uint8_t value = 0xFF;
    while (cnt++ < 10)
    {
        value = 0xFF;

        mm_reg_read(PASS_THROUGH_RDY_REG, &value, 1);

        if (enable == value)
        {
            ret = HAL_OK;
            break;
        }
        hal_delay(5);
    }
    return ret;
}


int mm_fw_load_check(void)
{
    int timeout_ms = 2000;        // ms
    int ret = FALSE;
    uint8_t u8val = 0;
    hal_delay(1000);

    while (timeout_ms > 0)
    {
        u8val = 0;
        ret = mm_reg_read(CHIP_STATUS_REG, &u8val, 1);

        APP_DBG("stat(0x37) =%x\n", u8val);

        if (u8val == 0x3)
        {
            ret = HAL_OK;
            break;
        }
        if (CHECK_BITS(u8val, 1 << 2))
        {
            ret = MM_FW_UPLOAD_ERR;
            break;
        }
        if (CHECK_BITS(u8val, 1 << 3))
        {
            ret = MM_FW_HALT;
            break;
        }

        if (CHECK_BITS(u8val, 1 << 4))
        {
            ret = MM_NO_EEPROM;
            break;
        }

        hal_delay(200);
        timeout_ms -= 200;
    }
    return ret;
}


int mm2_read_fifo(uint8_t *pbuf, uint32_t max_sz)
{
    // Check number of bytes available
    uint16_t bytesAvailable = 0;
    int ret = HAL_OK;

    const int FIFO_READ_MAX = 50; // 50 each

    ret = mm_reg_read(BYTES_REMANING_REG, (uint8_t *)&bytesAvailable, 2);

    if (bytesAvailable > 0)
    {

        int byteleft = bytesAvailable;
        int read_len = 0;
        int offset = 0;

        while (byteleft > 0)
        {
            read_len = MIN(byteleft, FIFO_READ_MAX);
            ret = mm_reg_read(FIFO_START, pbuf + offset, read_len);

            if (ret != HAL_OK)
            {
                break;
            }

            byteleft -= read_len;
            offset += read_len;

        }

    }else
    {
        ret = HAL_ERROR;
    }

    return bytesAvailable;

}


static int mm2_param_read(uint8_t *pdat, uint8_t page, ParamInfo *paramList, uint8_t numParams)
{
    uint8_t i = 0;
    uint8_t paramAck = 0;
    uint8_t paramNum = 0;
    uint8_t pageSelectValue = 0;
    uint16_t valIndex = 0;
    uint16_t retry = 0;


    for (i = 0; i < numParams; i++)
    {
        retry = 0;
        pageSelectValue = page | (paramList[i].size << 4);

        mm_reg_write(PARAM_PAGE_SELECT_REG, &pageSelectValue, 1);
        mm_reg_write(PARAM_REQUEST_REG, &paramList[i].paramNo, sizeof(paramList[i].paramNo));

        do
        {
            paramAck = 0;
            mm_reg_read(PARAM_ACK_REG, &paramAck, 1);

            // retry need to be at least 400 for Tethered Board
            if (paramAck == 0x80 || retry > 400)
            {
                APP_E("param read failed, no=0x%X,  ack=%X, rety=%d\r\n", paramList[i].paramNo, paramAck, retry);

                break;

            }

            retry++;
        } while (paramAck != paramList[i].paramNo);

        if (retry > 400)
        {
            break;
        }
        mm_reg_read(PARAM_SAVE_REG, &pdat[valIndex], paramList[i].size);

        valIndex += paramList[i].size;
    }


    paramNum = 0;
    mm_reg_write(PARAM_REQUEST_REG, &paramNum, 1);
    mm_reg_write(PARAM_PAGE_SELECT_REG, &paramNum, 1);


    return retry > 400 ? HAL_ERROR : HAL_OK;

}


int mm2_param_write(uint8_t *values, uint8_t page, ParamInfo *paramList, uint8_t numParams)
{
    uint8_t i, paramAck, paramNum, pageSelectValue;
    uint16_t valIndex = 0;
    uint16_t retry = 0;

    for (i = 0; i < numParams; i++)
    {
        retry = 0;
        pageSelectValue = page | (paramList[i].size << 4);

        mm_reg_write(PARAM_PAGE_SELECT_REG, &pageSelectValue, 1);
        mm_reg_write(PARAM_LOAD_REG, &values[valIndex], (uint16_t)paramList[i].size);

        paramNum = paramList[i].paramNo | 0x80;

        mm_reg_write(PARAM_REQUEST_REG, &paramNum, 1);

        do
        {
            paramAck = 0;
            mm_reg_read(PARAM_ACK_REG, &paramAck, 1);

            // retry need to be at least 400 for Tethered Board
            if (paramAck == 0x80 || retry > 400)
            {
                APP_E("param write failed. no=0x%X,  ack=%X, rety=%d\r\n", paramList[i].paramNo, paramAck, retry);
                break;
            }
            retry++;
        } while (paramAck != paramNum);

        if (retry > 400)
        {
            break;
        }
        valIndex += paramList[i].size;
    }

    paramNum = 0;
    mm_reg_write(PARAM_REQUEST_REG, &paramNum, 1);
    mm_reg_write(PARAM_PAGE_SELECT_REG, &paramNum, 1);

    return retry > 400 ? HAL_ERROR : HAL_OK;

}


int mm_set_sensor_rate(uint8_t sid, uint16_t rate)
{
    uint8_t paramPage = PARAM_PAGE_SENSOR_CONF;
    ParamInfo param[] = { { sid, sizeof(rate) } };
    return mm2_param_write((uint8_t *)&rate, paramPage, param, 1);
}


int mm2_init(void)
{
    int ret = HAL_OK;
    uint8_t buf[32] = { 0 };

    // mm id check.
    uint16_t u16value = 0;

    buf[0] = 1;
    mm_reg_write(RESET_REQ_REG, buf, 1);
    hal_delay(5);

    mm_reg_read(ROM_VERSION_REG, (uint8_t *)&u16value, 2);

    APP_DBG("ROM ver: %X\n", u16value);

    if (SENTRAL_ROM_VERSION != u16value)
    {
        APP_DBG("ROM ver check failed: val = %X!\n", u16value);
        return HAL_ERROR;
    }

    mm_reg_read(PRODUCT_ID_REG, (uint8_t *)&u16value, 2);
    APP_DBG("PROD ID: %X\n", u16value);


    mm_reg_read(0x0, buf, 16);
    hexdump("fifo 0 id?", buf, 16);


#ifdef MM_USING_GPIO6
    mm2_gpio_init();
#endif

    // enable mm cpu
    buf[0] = 0x1;
    mm_reg_write(RESET_REQ_REG, buf, 1);

    hal_delay(1000);

    buf[0] = 0x1;
    mm_reg_write(CHIP_CONTROL_REG, buf, 1);


    // check init
    ret = mm_fw_load_check();

    if (HAL_OK != ret)
    {
        APP_E("mm self init failed = %d !\n", ret);
        return HAL_ERROR;
    }

    u16value = 0;
    mm_reg_read(RAM_VERSION_REG, (uint8_t *)&u16value, 2);

    if (0 == u16value)
    {
        APP_E("RAM ver check failed: val = %X!\n", u16value);
        return HAL_ERROR;
    }

    APP_DBG("RAM version = %d\n", u16value);

    return HAL_OK;
}


// sa2 eeprom library.
/*
    GPIO6
    F411XE              PA10
*/
void mm2_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;


#if TARGET_BOARD == BOARD_STM32_NUCELO_F411
    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /*Configure GPIO pin : UserButton_Pin */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT | GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN; // GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
#elif TARGET_BOARD == BOARD_STM32_L073
#endif


}


/* return  1          ok
           else       false
 */
int sa2_fw_isvalid(uint32_t base)
{
    int ret = HAL_OK;
    uint32_t crc = 0;
    FirmwareHeader *pHead = (FirmwareHeader *)(base);

    //verify the FW image
    if (pHead->imageSignatureLsb != IMAGE_SIGNATURE_LSB || pHead->imageSignatureMsb != IMAGE_SIGNATURE_MSG)
    {
        ret = -1;
        goto end;
    }else if (pHead->imageLength > SA2_FW_SZ || pHead->imageLength % 4 != 0)
    {
        ret = -2;
        goto end;
    }

//    crc = crc32_mpeg((uint32_t *)&pHead->data[0], pHead->imageLength / 4, 0xFFFFFFFF);
//    if (crc != pHead->crc)
//    {
//        ret = -3;
//        goto end;
//    }
    crc = crc;                            // no warning

 end:
    APP_DBG("SA2 fw %08X check %d!\n", base, ret);

    return ret == HAL_OK;
}


