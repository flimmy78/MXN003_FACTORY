#ifndef VOICE_CHIP_H__
#define VOICE_CHIP_H__

#include "bsp.h"
#include "nrf_delay.h"

#ifdef __cplusplus
extern "C" {
#endif

__STATIC_INLINE void voice_chip_init(void)
{
	nrf_gpio_cfg_output(VOICE_CHIP_PIN);
	nrf_gpio_cfg_output(VOICE_MOSE_PIN);
	nrf_gpio_pin_write(VOICE_CHIP_PIN, 1);
	nrf_gpio_pin_write(VOICE_MOSE_PIN, 1);
}	


__STATIC_INLINE void voice_chip_open(void)
{
	nrf_gpio_pin_write(VOICE_MOSE_PIN, 0);
}	


__STATIC_INLINE void voice_chip_close(void)
{
	nrf_gpio_pin_write(VOICE_MOSE_PIN, 1);
}	

__STATIC_INLINE void voice_chip_output(uint8_t reg)
{
	nrf_gpio_pin_write(VOICE_CHIP_PIN, 0);
	nrf_delay_ms(4);

	for(int i = 0; i < reg; i++){
	nrf_gpio_pin_write(VOICE_CHIP_PIN, 1);
	nrf_delay_us(800);
	nrf_gpio_pin_write(VOICE_CHIP_PIN, 0);
	nrf_delay_us(2400);
	}
	nrf_gpio_pin_write(VOICE_CHIP_PIN, 1);
}	


#ifdef __cplusplus
}
#endif

#endif

