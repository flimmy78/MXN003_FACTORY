#include "nrf_drv_adc.h"
#include "app_error.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#define ADC_BUFFER_SIZE                 3 
static nrf_adc_value_t                  adc_buffer[ADC_BUFFER_SIZE];                /**< ADC buffer. */
static uint8_t                          number_of_adc_channels;

#define ADC_REF_VBG_VOLTAGE_IN_MILLIVOLTS 1200   
#define ADC_RES_10BIT                     1024    
#define ADC_INPUT_PRESCALER               3

#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE)\
			((((ADC_VALUE) * ADC_REF_VBG_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_10BIT) * ADC_INPUT_PRESCALER)
		
uint16_t batter_volts = 0;

static void adc_event_handler(nrf_drv_adc_evt_t const * p_event)
{
    uint16_t        batt_lvl_in_milli_volts;
		nrf_adc_value_t adc_result;
	
    if (p_event->type == NRF_DRV_ADC_EVT_DONE)
    {
				uint32_t i;
				for (i = 0; i < p_event->data.done.size; i++)
				{
						adc_result = p_event->data.done.p_buffer[i];
						if(0 == (i % number_of_adc_channels)){
							batt_lvl_in_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(adc_result);
							batt_lvl_in_milli_volts = batt_lvl_in_milli_volts * 122 / 22;
							NRF_LOG_INFO("main batter %d\r\n",batt_lvl_in_milli_volts);			
						}else if(1 == (i % number_of_adc_channels)){
							batt_lvl_in_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(adc_result);
							batt_lvl_in_milli_volts = batt_lvl_in_milli_volts * 122 / 22;
							NRF_LOG_INFO("sub batter %d\r\n",batt_lvl_in_milli_volts);

						}else if(2 == (i % number_of_adc_channels)){
							batt_lvl_in_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(adc_result);
							batt_lvl_in_milli_volts = batt_lvl_in_milli_volts *3;
							NRF_LOG_INFO("system batter %d\r\n",batt_lvl_in_milli_volts);
						}
						
				}
        APP_ERROR_CHECK(nrf_drv_adc_buffer_convert(adc_buffer,ADC_BUFFER_SIZE));
    }
}


void adc_config_init(void)
{
 ret_code_t ret_code;
	
    //Initialize ADC
    nrf_drv_adc_config_t config = NRF_DRV_ADC_DEFAULT_CONFIG;
    ret_code = nrf_drv_adc_init(&config, adc_event_handler);
    APP_ERROR_CHECK(ret_code);
	
    //Configure and enable ADC channel 0
    static nrf_drv_adc_channel_t m_channel_0_config = NRF_DRV_ADC_DEFAULT_CHANNEL(NRF_ADC_CONFIG_INPUT_7); 
    m_channel_0_config.config.config.input = NRF_ADC_CONFIG_SCALING_INPUT_FULL_SCALE;
    nrf_drv_adc_channel_enable(&m_channel_0_config);
	
    //Configure and enable ADC channel 1
    static nrf_drv_adc_channel_t m_channel_1_config = NRF_DRV_ADC_DEFAULT_CHANNEL(NRF_ADC_CONFIG_INPUT_6); 
    m_channel_1_config.config.config.input = NRF_ADC_CONFIG_SCALING_INPUT_FULL_SCALE;
    nrf_drv_adc_channel_enable(&m_channel_1_config);
	
    //Configure and enable ADC channel 2
    static nrf_drv_adc_channel_t m_channel_2_config = NRF_DRV_ADC_DEFAULT_CHANNEL(NRF_ADC_CONFIG_INPUT_DISABLED);	
    m_channel_2_config.config.config.input = NRF_ADC_CONFIG_SCALING_SUPPLY_ONE_THIRD;
    nrf_drv_adc_channel_enable(&m_channel_2_config);
	
    number_of_adc_channels = 3;    //Set equal to the number of configured ADC channels, for the sake of UART output.
		APP_ERROR_CHECK(nrf_drv_adc_buffer_convert(adc_buffer,ADC_BUFFER_SIZE));
}


void start_read_adc(void)
{
    nrf_drv_adc_sample();
}

