#ifndef ATA_2G_MODEM_H__
#define ATA_2G_MODEM_H__
#include "ata_custom_interface.h"
#include "uart_putbytes.h"

custom_rsp_type_enum custom_sensor_func(custom_cmdLine *commandBuffer_p)
{
    custom_cmd_mode_enum result;
		//cmd_data_struct cmd;
    custom_rsp_type_enum ret_value  = CUSTOM_RSP_ERROR;
    result = custom_find_cmd_mode(commandBuffer_p);		
		//cmd = at_get_at_para(commandBuffer_p);
    switch (result)
    {
        case CUSTOM_READ_MODE:
						PutUARTBytes("+MSENSOR:1");
            ret_value = CUSTOM_RSP_OK;
            break;
        default:
            ret_value = CUSTOM_RSP_ERROR;
            break;
	}
    return ret_value;
}
#endif


