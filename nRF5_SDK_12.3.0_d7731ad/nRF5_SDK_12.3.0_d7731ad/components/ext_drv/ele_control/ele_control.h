#ifndef ELE_CONTROL_H__
#define ELE_CONTROL_H__

#include "bsp.h"

#ifdef __cplusplus
extern "C" {
#endif

__STATIC_INLINE void ele_mach_init(void)
{
	nrf_gpio_cfg_output(ELE_MACH_PIN);
	nrf_gpio_cfg_output(ELE_MACH_CONTRL_PIN1);
	nrf_gpio_cfg_output(ELE_MACH_CONTRL_PIN2);
	nrf_gpio_pin_write(ELE_MACH_PIN, 1);
	nrf_gpio_pin_write(ELE_MACH_CONTRL_PIN1, 0);
	nrf_gpio_pin_write(ELE_MACH_CONTRL_PIN2, 0);
}	

__STATIC_INLINE void ele_mach_corotation(void)
{

	nrf_gpio_pin_write(ELE_MACH_PIN, 0);
	nrf_gpio_pin_write(ELE_MACH_CONTRL_PIN1, 0);
	nrf_gpio_pin_write(ELE_MACH_CONTRL_PIN2, 1);
}	


__STATIC_INLINE void ele_mach_rollback(void)
{

	nrf_gpio_pin_write(ELE_MACH_PIN, 0);
	nrf_gpio_pin_write(ELE_MACH_CONTRL_PIN1, 1);
	nrf_gpio_pin_write(ELE_MACH_CONTRL_PIN2, 0);
}	

__STATIC_INLINE void ele_mach_close(void)
{
	nrf_gpio_pin_write(ELE_MACH_PIN, 1);
	nrf_gpio_pin_write(ELE_MACH_CONTRL_PIN1, 0);
	nrf_gpio_pin_write(ELE_MACH_CONTRL_PIN2, 0);
}	


#ifdef __cplusplus
}
#endif

#endif

