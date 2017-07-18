#ifndef __BOSCH_BMA2X2_H
#define __BOSCH_BMA2X2_H
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TWI_MASTER_CONFIG_CLOCK_PIN_NUMBER (1U)
#define TWI_MASTER_CONFIG_DATA_PIN_NUMBER (2U)

//typedef void (*FuncPtr)(void);

void bma2x2_power_on(uint8_t threshold);
void bma2x2_power_off(void);
int8_t bm2x2_check_chipid(void);


#ifdef __cplusplus
}
#endif

#endif

