#include "app_def.h"

#include "board_config.h"

#include "util.h"


void led_blink(uint8_t cnt)
{
    int i = 0;

    DBG_LED_OFF();
    
    for(i = 0;i<cnt;i++){
        DBG_LED_ON();
        Clock_Wait(100);
        DBG_LED_OFF();
        Clock_Wait(100);
    }
}


void hexdump(char *ptitle, uint8_t *pdata, uint32_t len)
{
#if ENABLE_PRINTF
    int indent = 0;
    int v = 0;
    
    APP_DBG("%s [%-3d] ", ptitle, len);
    indent = strlen(ptitle)+7;
    
    for (uint32_t i = 0; i < len; i++)
    {   
        
        printf("%02X ", *(pdata + i));

        if( (i+1)%0x10 == 0){
            printf("\n");
            v = indent;

            while(v-->0){
                printf(" ");
            }
            Clock_Wait(0);
        }
    }
    
    printf("\n");
#endif
}




void Clock_Wait(uint32_t i)
{
#if ENABLE_WWDG
    while (i > 500)
    {
        HAL_WWDG_Refresh(&hwwdg);
        HAL_Delay(500);
        i -= 500;
    }

    if (i != 0)
    {
        // last one
        HAL_WWDG_Refresh(&hwwdg);
        HAL_Delay(i);
    }

    // make sure we exit with fresh restart watchdog
    HAL_WWDG_Refresh(&hwwdg);
#else
    HAL_Delay(i);
#endif
}

void hal_delay(uint32_t i){
    Clock_Wait(i);

}



int host_eeprom_read(uint32_t offset, void *pdata, uint16_t len)
{
    // todo
    return HAL_OK;

}



int host_eeprom_write(uint32_t offset, void *pdata, uint16_t len)
{
    // todo
    return HAL_OK;

}






