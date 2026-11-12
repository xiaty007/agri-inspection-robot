#ifndef __MM_LOOP_H__
#define __MM_LOOP_H__

#include <stdint.h>
#include "app_def.h"


stc_app_t *get_app(void);

int appsa2_check_stat(stc_app_t *pModule);
void app_reboot(uint16_t reason, uint8_t b_now);

int mm2_loop(void);


#endif /* __MM_LOOP_H__ */
