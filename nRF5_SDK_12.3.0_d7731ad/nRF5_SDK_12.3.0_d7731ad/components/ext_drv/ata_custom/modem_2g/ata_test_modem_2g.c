#include "ata_custom_interface.h"
#include "uart_putbytes.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "sensor_drv.h"
#include "stdint.h"
#include "nrf_gpio.h"
#include "ele_control.h"
#include "bsp.h"
#include "voice_chip.h"
#include "battery_adc.h"

extern int8_t ble_rssi;
custom_rsp_type_enum custom_sensor_func(custom_cmdLine *commandBuffer_p)
{
    custom_cmd_mode_enum result;
    custom_rsp_type_enum ret_value  = CUSTOM_RSP_ERROR;
    result = custom_find_cmd_mode(commandBuffer_p);	
		sensor_type_enum sensor_type;	
		//cmd_data_struct cmd = at_get_at_para(commandBuffer_p);
    switch (result)
    {
        case CUSTOM_READ_MODE:
						sensor_type = sensor_get_type();
						//ret = sensor_type_auto_maching_init();
						PutUARTBytes("AT+MSENSOR=%d",sensor_type);
            ret_value = CUSTOM_RSP_OK;
            break;
        default:
            ret_value = CUSTOM_RSP_ERROR;
            break;
	}
    return ret_value;
}

custom_rsp_type_enum custom_switdet_func(custom_cmdLine *commandBuffer_p){
		custom_cmd_mode_enum result;
    custom_rsp_type_enum ret_value  = CUSTOM_RSP_ERROR;
    result = custom_find_cmd_mode(commandBuffer_p);
		int8_t ret;
		//cmd_data_struct cmd = at_get_at_para(commandBuffer_p);
    switch (result)
    {
        case CUSTOM_READ_MODE:
						ret = nrf_gpio_pin_read(BUTTON_1);
						ret = (ret << 1 ) |	nrf_gpio_pin_read(BUTTON_2);
						PutUARTBytes("AT+MSWITDET=%d",ret);			
            ret_value = CUSTOM_RSP_OK;
            break;
        default:
            ret_value = CUSTOM_RSP_ERROR;
            break;
	}
    return ret_value;
}

custom_rsp_type_enum custom_elecontrl_func(custom_cmdLine *commandBuffer_p){
		custom_cmd_mode_enum result;
    custom_rsp_type_enum ret_value  = CUSTOM_RSP_ERROR;
    result = custom_find_cmd_mode(commandBuffer_p);
	//	int8_t ret;
		cmd_data_struct cmd = at_get_at_para(commandBuffer_p);
    switch (result)
    {
				case CUSTOM_SET_OR_EXECUTE_MODE:
						if(strncmp(cmd.pars[0], "H",1) == 0)
								ele_mach_corotation();
						else if(strncmp(cmd.pars[0], "T",1) == 0)
								ele_mach_rollback();
						else if(strncmp(cmd.pars[0], "OFF",3) == 0)
								ele_mach_close();
						break;
        default:
            ret_value = CUSTOM_RSP_ERROR;
            break;
	}
    return ret_value;
}

custom_rsp_type_enum custom_vochip_func(custom_cmdLine *commandBuffer_p){
		custom_cmd_mode_enum result;
    custom_rsp_type_enum ret_value  = CUSTOM_RSP_ERROR;
    result = custom_find_cmd_mode(commandBuffer_p);
	//	int8_t ret;
		cmd_data_struct cmd = at_get_at_para(commandBuffer_p);
    switch (result)
    {
				case CUSTOM_SET_OR_EXECUTE_MODE:
						if(strncmp(cmd.pars[0], "OPEN",4) == 0){
							//NRF_LOG_INFO("open voice\r\n");
							voice_chip_open();
						}
						else if(strncmp(cmd.pars[0], "CLOSE",5) == 0){
							voice_chip_close();
						}
						else if(strncmp(cmd.pars[0], "OUTPUT",6) == 0)
						{
						//	NRF_LOG_INFO("output void\r\n");
							voice_chip_output(8);
						}
						break;
        default:
            ret_value = CUSTOM_RSP_ERROR;
            break;
	}
    return ret_value;
}

custom_rsp_type_enum custom_adc_func(custom_cmdLine *commandBuffer_p){
		custom_cmd_mode_enum result;
    custom_rsp_type_enum ret_value  = CUSTOM_RSP_ERROR;
    result = custom_find_cmd_mode(commandBuffer_p);
//		int8_t ret;
		//cmd_data_struct cmd = at_get_at_para(commandBuffer_p);
    switch (result)
    {
				case CUSTOM_READ_MODE:
						start_read_adc();
						PutUARTBytes("AT+MADC=%d",batter_volts);
						break;
        default:
            ret_value = CUSTOM_RSP_ERROR;
            break;
	}
    return ret_value;
}

custom_rsp_type_enum custom_rssi_func(custom_cmdLine *commandBuffer_p){
custom_cmd_mode_enum result;
    custom_rsp_type_enum ret_value  = CUSTOM_RSP_ERROR;
    result = custom_find_cmd_mode(commandBuffer_p);

    switch (result)
    {
				case CUSTOM_READ_MODE:
						
						PutUARTBytes("AT+MRSSI=%d",ble_rssi);
						break;
        default:
            ret_value = CUSTOM_RSP_ERROR;
            break;
	}
    return ret_value;
}




