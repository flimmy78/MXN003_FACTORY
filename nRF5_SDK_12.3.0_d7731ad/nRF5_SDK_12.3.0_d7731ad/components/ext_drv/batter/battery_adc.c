#include "nrf_drv_adc.h"
#include "app_error.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

static nrf_adc_value_t adc_buf[1];

#define ADC_REF_VBG_VOLTAGE_IN_MILLIVOLTS 1200   
#define ADC_RES_10BIT                     1024    
#define ADC_INPUT_PRESCALER               3

#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE)\
			((((ADC_VALUE) * ADC_REF_VBG_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_10BIT) * ADC_INPUT_PRESCALER)
		
uint16_t batter_volts = 0;

static void adc_event_handler(nrf_drv_adc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_ADC_EVT_DONE)
    {
			
				uint16_t        batt_lvl_in_milli_volts;
				nrf_adc_value_t adc_result;
				uint32_t        err_code;
				
				adc_result = p_event->data.done.p_buffer[0];
			
				err_code = nrf_drv_adc_buffer_convert(p_event->data.done.p_buffer, 1);	
				APP_ERROR_CHECK(err_code);
			
				batt_lvl_in_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(adc_result);// +
			
				batter_volts = batt_lvl_in_milli_volts;
			
				NRF_LOG_INFO("adc %d\r\n",batter_volts);
                                 // DIODE_FWD_VOLT_DROP_MILLIVOLTS;
				//printf("AT+BATVAL:%d\r\n",batt_lvl_in_milli_volts);
				
    }
}



void adc_config_init(void)
{
    ret_code_t err_code = nrf_drv_adc_init(NULL, adc_event_handler);
    APP_ERROR_CHECK(err_code);

    static nrf_drv_adc_channel_t channel =
        NRF_DRV_ADC_DEFAULT_CHANNEL(NRF_ADC_CONFIG_INPUT_7);
    // channel.config.config.input = NRF_ADC_CONFIG_SCALING_SUPPLY_ONE_THIRD;
    channel.config.config.input = (uint32_t)NRF_ADC_CONFIG_SCALING_SUPPLY_ONE_THIRD;
    nrf_drv_adc_channel_enable(&channel);

    err_code = nrf_drv_adc_buffer_convert(&adc_buf[0], 1);
}


void start_read_adc(void)
{
    nrf_drv_adc_sample();
}

