#ifndef BATTERY_ADC_H__
#define BATTERY_ADC_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
	
extern uint16_t batter_volts;
	
void adc_config_init(void);
void start_read_adc(void);


#ifdef __cplusplus
}
#endif

#endif

