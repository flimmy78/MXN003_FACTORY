#ifndef ATA_TEST_MODEM_2G__
#define ATA_TEST_MODEM_2G__

#include "ata_custom_interface.h"

#ifdef __cplusplus
extern "C" {
#endif
	
custom_rsp_type_enum custom_sensor_func(custom_cmdLine *commandBuffer_p);
custom_rsp_type_enum custom_switdet_func(custom_cmdLine *commandBuffer_p);
custom_rsp_type_enum custom_elecontrl_func(custom_cmdLine *commandBuffer_p);
custom_rsp_type_enum custom_vochip_func(custom_cmdLine *commandBuffer_p);
custom_rsp_type_enum custom_adc_func(custom_cmdLine *commandBuffer_p);
custom_rsp_type_enum custom_rssi_func(custom_cmdLine *commandBuffer_p);

#ifdef __cplusplus
}
#endif

#endif

