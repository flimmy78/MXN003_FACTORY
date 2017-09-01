/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

/** @file
 *
 * @defgroup ble_sdk_app_beacon_main main.c
 * @{
 * @ingroup ble_sdk_app_beacon
 * @brief Beacon Transmitter Sample Application main file.
 *
 * This file contains the source code for an Beacon transmitter sample application.
 */

#include <stdbool.h>
#include <stdint.h>
#include "ble_advdata.h"
#include "nordic_common.h"
#include "softdevice_handler.h"
#include "bsp.h"
#include "app_timer.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_drv_adc.h"
#include "app_pwm.h"
#include "nrf_delay.h"
#include "app_uart.h"

#define CENTRAL_LINK_COUNT              0                                 /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           0                                 /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                 /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define APP_CFG_NON_CONN_ADV_TIMEOUT    0                                 /**< Time for which the device must be advertising in non-connectable mode (in seconds). 0 disables timeout. */
#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(640, UNIT_0_625_MS) /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */

#define APP_BEACON_INFO_LENGTH          0x17                              /**< Total length of information advertised by the Beacon. */
#define APP_ADV_DATA_LENGTH             0x15                              /**< Length of manufacturer specific data in the advertisement. */
#define APP_DEVICE_TYPE                 0x02                              /**< 0x02 refers to Beacon. */
#define APP_MEASURED_RSSI               0xC3                              /**< The Beacon's measured RSSI at 1 meter distance in dBm. */
#define APP_COMPANY_IDENTIFIER          0x0059                            /**< Company identifier for Nordic Semiconductor ASA. as per www.bluetooth.org. */
#define APP_MAJOR_VALUE                 0x01, 0x02                        /**< Major value used to identify Beacons. */
#define APP_MINOR_VALUE                 0x03, 0x04                        /**< Minor value used to identify Beacons. */
#define APP_BEACON_UUID                 0x01, 0x12, 0x23, 0x34, \
                                        0x45, 0x56, 0x67, 0x78, \
                                        0x89, 0x9a, 0xab, 0xbc, \
                                        0xcd, 0xde, 0xef, 0xf0            /**< Proprietary UUID for Beacon. */

#define DEAD_BEEF                       0xDEADBEEF                        /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define APP_TIMER_PRESCALER             0                                 /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                 /**< Size of timer operation queues. */

#if defined(USE_UICR_FOR_MAJ_MIN_VALUES)
#define MAJ_VAL_OFFSET_IN_BEACON_INFO   18                                /**< Position of the MSB of the Major Value in m_beacon_info array. */
#define UICR_ADDRESS                    0x10001080                        /**< Address of the UICR register used by this example. The major and minor versions to be encoded into the advertising data will be picked up from this location. */
#endif

APP_TIMER_DEF(battery_timer_id); 
APP_TIMER_DEF(led_on_timer_id); 
APP_TIMER_DEF(led_off_timer_id); 

#define ADC_BUFFER_SIZE                 2               //Size of buffer for ADC samples. Buffer size should be multiple of number of adc channels located.

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */
#define BATTER_TIMER_INTERVAL           APP_TIMER_TICKS(2000, APP_TIMER_PRESCALER)    /**< Defines the interval between consecutive app timer interrupts in milliseconds. */
#define LED_ON_TIMER_INTERVAL           APP_TIMER_TICKS(2000, APP_TIMER_PRESCALER)
#define LED_OFF_TIMER_INTERVAL          APP_TIMER_TICKS(50000, APP_TIMER_PRESCALER)

static uint8_t index = 0;
#define UART_BUFFER_SIZE (128)
static uint8_t data_array[UART_BUFFER_SIZE];

static uint8_t timer_start = 0;
static void uart_timeout_handler(void * p_context);
#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                1                                         /**< UART RX buffer size. */

APP_TIMER_DEF(uart_timer_id);
#define UART_IMTES_INTERVAL       APP_TIMER_TICKS(100, APP_TIMER_PRESCALER)

static nrf_adc_value_t                  adc_buffer[ADC_BUFFER_SIZE];                /**< ADC buffer. */
static uint8_t                          number_of_adc_channels;

#define ADC_REF_VBG_VOLTAGE_IN_MILLIVOLTS 1200   
#define ADC_RES_10BIT                     1023    
#define ADC_INPUT_PRESCALER               1

APP_PWM_INSTANCE(PWM1,1);                   // Create the instance "PWM1" using TIMER1.
#define LED_PIN   8

typedef struct
{
	uint16_t adc1;
	uint16_t adc2;
	uint16_t adc3;
} ADC_DATA;

ADC_DATA adc_data;

#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE)\
			((((ADC_VALUE) * ADC_REF_VBG_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_10BIT) * ADC_INPUT_PRESCALER)
			
static ble_gap_adv_params_t m_adv_params;                                 /**< Parameters to be passed to the stack when starting advertising. */
static uint8_t m_beacon_info[APP_BEACON_INFO_LENGTH] =                    /**< Information advertised by the Beacon. */
{
    APP_DEVICE_TYPE,     // Manufacturer specific information. Specifies the device type in this
                         // implementation.
    APP_ADV_DATA_LENGTH, // Manufacturer specific information. Specifies the length of the
                         // manufacturer specific data in this implementation.
    APP_BEACON_UUID,     // 128 bit UUID value.
    APP_MAJOR_VALUE,     // Major arbitrary value that can be used to distinguish between Beacons.
    APP_MINOR_VALUE,     // Minor arbitrary value that can be used to distinguish between Beacons.
    APP_MEASURED_RSSI    // Manufacturer specific information. The Beacon's measured TX power in
                         // this implementation.
};

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    uint8_t       flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;

    ble_advdata_manuf_data_t manuf_specific_data;

    manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;

#if defined(USE_UICR_FOR_MAJ_MIN_VALUES)
    // If USE_UICR_FOR_MAJ_MIN_VALUES is defined, the major and minor values will be read from the
    // UICR instead of using the default values. The major and minor values obtained from the UICR
    // are encoded into advertising data in big endian order (MSB First).
    // To set the UICR used by this example to a desired value, write to the address 0x10001080
    // using the nrfjprog tool. The command to be used is as follows.
    // nrfjprog --snr <Segger-chip-Serial-Number> --memwr 0x10001080 --val <your major/minor value>
    // For example, for a major value and minor value of 0xabcd and 0x0102 respectively, the
    // the following command should be used.
    // nrfjprog --snr <Segger-chip-Serial-Number> --memwr 0x10001080 --val 0xabcd0102
    uint16_t major_value = ((*(uint32_t *)UICR_ADDRESS) & 0xFFFF0000) >> 16;
    uint16_t minor_value = ((*(uint32_t *)UICR_ADDRESS) & 0x0000FFFF);

    uint8_t index = MAJ_VAL_OFFSET_IN_BEACON_INFO;

    m_beacon_info[index++] = MSB_16(major_value);
    m_beacon_info[index++] = LSB_16(major_value);

    m_beacon_info[index++] = MSB_16(minor_value);
    m_beacon_info[index++] = LSB_16(minor_value);
#endif

    manuf_specific_data.data.p_data = (uint8_t *) m_beacon_info;
    manuf_specific_data.data.size   = APP_BEACON_INFO_LENGTH;

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type             = BLE_ADVDATA_NO_NAME;
    advdata.flags                 = flags;
    advdata.p_manuf_specific_data = &manuf_specific_data;

    err_code = ble_advdata_set(&advdata, NULL);
    APP_ERROR_CHECK(err_code);

    // Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));

    m_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_NONCONN_IND;
    m_adv_params.p_peer_addr = NULL;                             // Undirected advertisement.
    m_adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval    = NON_CONNECTABLE_ADV_INTERVAL;
    m_adv_params.timeout     = APP_CFG_NON_CONN_ADV_TIMEOUT;
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code;

    err_code = sd_ble_gap_adv_start(&m_adv_params);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
    APP_ERROR_CHECK(err_code);
}

static const char m_target_periph_name[] = "";

typedef struct
{
	uint8_t *p_data;
	uint16_t data_len;
}data_t;

static uint32_t adv_report_parse(uint8_t type, data_t *p_advdata, data_t *p_typedata)
{
	uint32_t index = 0;
	uint8_t *p_data;
	
	p_data = p_advdata->p_data;
	
	while(index < p_advdata->data_len)
	{
		uint8_t field_length = p_data[index];
		uint8_t field_type = p_data[index+1];
		
		if(field_type == type)
		{
			p_typedata->p_data = &p_data[index + 2];
			p_typedata->data_len = field_length - 1;
			return NRF_SUCCESS;
		}
		index += field_length + 1;
	}
	
	return NRF_ERROR_NOT_FOUND;
}

#define BLE_UUID_NUS_SERVICE  0x0001 
#define NUS_SERVICE_UUID_TYPE 0x02
#define NUS_BASE_UUID                  {{0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x00, 0x00, 0x40, 0x6E}} /**< Used vendor specific UUID. */
//static ble_uuid_t find_uuids[] = {{BLE_UUID_NUS_SERVICE, BLE_UUID_TYPE_BLE}};
static const ble_uuid_t find_uuids = {
	.uuid = BLE_UUID_NUS_SERVICE,
	.type = NUS_SERVICE_UUID_TYPE
};

#define UUID16_SIZE             2                               /**< Size of 16 bit UUID */
#define UUID32_SIZE             4                               /**< Size of 32 bit UUID */
#define UUID128_SIZE            16                              /**< Size of 128 bit UUID */

static bool is_uuid_present(const ble_uuid_t *p_target_uuid,
                            const ble_gap_evt_adv_report_t *p_adv_report)
{
    uint32_t err_code;
    uint32_t index = 0;
    uint8_t *p_data = (uint8_t *)p_adv_report->data;
    ble_uuid_t extracted_uuid;

    while (index < p_adv_report->dlen)
    {
        uint8_t field_length = p_data[index];
        uint8_t field_type   = p_data[index + 1];
        if ( (field_type == BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_MORE_AVAILABLE)
           || (field_type == BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE)
           )
        {
            for (uint32_t u_index = 0; u_index < (field_length / UUID16_SIZE); u_index++)
            {
                err_code = sd_ble_uuid_decode(  UUID16_SIZE,
                                                &p_data[u_index * UUID16_SIZE + index + 2],
                                                &extracted_uuid);
                if (err_code == NRF_SUCCESS)
                {
                    if ((extracted_uuid.uuid == p_target_uuid->uuid)
                        && (extracted_uuid.type == p_target_uuid->type))
                    {
                        return true;
                    }
                }
            }
        }

        else if ( (field_type == BLE_GAP_AD_TYPE_32BIT_SERVICE_UUID_MORE_AVAILABLE)
                || (field_type == BLE_GAP_AD_TYPE_32BIT_SERVICE_UUID_COMPLETE)
                )
        {
            for (uint32_t u_index = 0; u_index < (field_length / UUID32_SIZE); u_index++)
            {
                err_code = sd_ble_uuid_decode(UUID16_SIZE,
                &p_data[u_index * UUID32_SIZE + index + 2],
                &extracted_uuid);
                if (err_code == NRF_SUCCESS)
                {
                    if ((extracted_uuid.uuid == p_target_uuid->uuid)
                        && (extracted_uuid.type == p_target_uuid->type))
                    {
                        return true;
                    }
                }
            }
        }

        else if ( (field_type == BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_MORE_AVAILABLE)
                || (field_type == BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_COMPLETE)
                )
        {
            err_code = sd_ble_uuid_decode(UUID128_SIZE,
                                          &p_data[index + 2],
                                          &extracted_uuid);
            if (err_code == NRF_SUCCESS)
            {
								printf("%x\r\n",extracted_uuid.uuid);
                if ((extracted_uuid.uuid == p_target_uuid->uuid)
                    && (extracted_uuid.type == p_target_uuid->type))
                {
                    return true;
                }
            }
        }
        index += field_length + 1;
    }
    return false;
}

static bool find_adv_name(const ble_gap_evt_adv_report_t *p_adv_report, const char *name_to_find)
{
	uint32_t err_code;
	data_t adv_data;
	data_t dev_name;
	
	adv_data.p_data = (uint8_t *)p_adv_report->data;
	adv_data.data_len = p_adv_report->dlen;
	
	err_code = adv_report_parse(BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME,
															&adv_data,
															&dev_name);
	
	if(err_code == NRF_SUCCESS)
	{
		if(memcmp(name_to_find, dev_name.p_data, dev_name.data_len) == 0)
		{
			return true;
		}
	}
	else
	{
		err_code = adv_report_parse(BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME,
															&adv_data,
															&dev_name);
		
		if(err_code != NRF_SUCCESS)
		{
			return false;
		}
		if(memcmp(m_target_periph_name, dev_name.p_data, dev_name.data_len) == 0)
		{
			return true;
		}
	}
	return false;
}

#define UUID16_EXTRACT(DST,SRC) \
		do													\
		{														\
			(*(DST)) = (SRC)[1];			\
			(*(DST)) << 8;						\
			(*(DST)) |= (SRC)[0];			\
		}while(0)
		
#define TARGET_UUID 0x1800
		
static bool find_adv_uuid(const ble_gap_evt_adv_report_t *p_adv_report, const uint16_t uuid_to_find)
{
	uint32_t err_code;
	data_t adv_data;
	data_t type_data;
	
	adv_data.p_data = (uint8_t *)p_adv_report->data;
	adv_data.data_len = p_adv_report->dlen;
	
	err_code = adv_report_parse(BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_MORE_AVAILABLE,
															&adv_data,
															&type_data);
	
	if(err_code != NRF_SUCCESS)
	{
		err_code =	adv_report_parse(BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE,
															&adv_data,
															&type_data); 
		
		if(err_code != NRF_SUCCESS)
		{
			printf("return error\r\n");
			return false;
		}
	}
	
	for(uint32_t u_index = 0; u_index < (type_data.data_len / sizeof(uint16_t)); u_index++)
	{
		uint16_t extracted_uuid;
		
		UUID16_EXTRACT(&extracted_uuid, &type_data.p_data[u_index * sizeof(uint16_t)]);
		printf("0x%x\r\n",extracted_uuid);
		if(extracted_uuid == uuid_to_find)
		{
			return true;
		}
	}
	
	return false;
	
}


static void on_ble_evt(ble_evt_t * p_ble_evt)
{
		//uint32_t err_code = NRF_SUCCESS;
		const ble_gap_evt_t * p_gap_evt = &p_ble_evt->evt.gap_evt;
	
		switch (p_ble_evt->header.evt_id)
		{
			case BLE_GAP_EVT_ADV_REPORT:
			{
				const ble_gap_evt_adv_report_t * p_adv_report = &p_gap_evt->params.adv_report;
				if(strlen(m_target_periph_name) != 0)
				{
					if(find_adv_name(&p_gap_evt->params.adv_report, m_target_periph_name))
					{
						printf("find the match send connect_request!!\r\n");
						(void)sd_ble_gap_scan_stop();
					}
				}
				else
				{
					if (is_uuid_present(&find_uuids, p_adv_report))
					{
							printf("uuid mach!!\r\n");
					}

				}
			}break;
		}
}

static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{

    on_ble_evt(p_ble_evt);
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);

    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
	
	    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for doing power management.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief ADC interrupt handler.
 */
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
							batt_lvl_in_milli_volts = batt_lvl_in_milli_volts * 122 / 22 + 30;
							//NRF_LOG_INFO("main batter %d\r\n",batt_lvl_in_milli_volts);
							adc_data.adc1 = batt_lvl_in_milli_volts;
							if(batt_lvl_in_milli_volts < 2300){
								nrf_gpio_pin_write(20,0);
							}
							else if(batt_lvl_in_milli_volts > 2400){
								nrf_gpio_pin_write(20,1);
							}
						}else{
							batt_lvl_in_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(adc_result);
							batt_lvl_in_milli_volts = batt_lvl_in_milli_volts * 122 / 22 + 40;
						//	NRF_LOG_INFO("sub batter %d\r\n",batt_lvl_in_milli_volts);
							adc_data.adc2 = batt_lvl_in_milli_volts;
						}
					//	NRF_LOG_INFO("ADC value channel %d: %d\r\n", (i % number_of_adc_channels), p_event->data.done.p_buffer[i]);
				}
				
        APP_ERROR_CHECK(nrf_drv_adc_buffer_convert(adc_buffer,ADC_BUFFER_SIZE));
    }

}

/**
 * @brief ADC initialization.
 */
static void adc_config(void)
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
//    static nrf_drv_adc_channel_t m_channel_2_config = NRF_DRV_ADC_DEFAULT_CHANNEL(NRF_ADC_CONFIG_INPUT_DISABLED);	
//    m_channel_2_config.config.config.input = NRF_ADC_CONFIG_SCALING_INPUT_ONE_THIRD;
//    nrf_drv_adc_channel_enable(&m_channel_2_config);
	
    number_of_adc_channels = 2;    //Set equal to the number of configured ADC channels, for the sake of UART output.
		APP_ERROR_CHECK(nrf_drv_adc_buffer_convert(adc_buffer,ADC_BUFFER_SIZE));

}


static void battery_timerout_handler(void * p_context)
{
    nrf_drv_adc_sample();
}

static void led_off_timerout_handler(void * p_context)
{
		//	app_timer_stop(led_on_timer_id);
			app_timer_stop(led_off_timer_id);
}

static void led_timerout_handler(void * p_context)
{
			nrf_gpio_pin_write(LED_PIN,1);
			nrf_delay_ms(200);
			nrf_gpio_pin_write(LED_PIN,0);
//		if(nrf_gpio_pin_read(21) == 1){
//			 app_timer_stop(led_on_timer_id);
//			 app_timer_start(led_off_timer_id, LED_OFF_TIMER_INTERVAL, NULL);
//		}else{
//			 app_timer_stop(led_off_timer_id);
//			 app_timer_start(led_on_timer_id, LED_ON_TIMER_INTERVAL, NULL);
//		}
//    nrf_gpio_pin_toggle(LED_PIN);
		
}



static void timers_init(void)
{
    uint32_t err_code;

    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

    // Create timers.
    err_code = app_timer_create(&battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_timerout_handler);
    APP_ERROR_CHECK(err_code);
	
	  err_code = app_timer_create(&led_on_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                led_timerout_handler);
    APP_ERROR_CHECK(err_code);
	
		err_code = app_timer_create(&led_off_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                led_off_timerout_handler);
    APP_ERROR_CHECK(err_code);
	
		err_code = app_timer_create(&uart_timer_id,
														APP_TIMER_MODE_REPEATED,
														uart_timeout_handler);
		APP_ERROR_CHECK(err_code);
}

static void application_timers_start(void)
{
    uint32_t err_code;

    // Start application timers.
    err_code = app_timer_start(battery_timer_id, BATTER_TIMER_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
	
	  err_code = app_timer_start(led_on_timer_id, LED_ON_TIMER_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
	
		err_code = app_timer_start(led_off_timer_id, LED_OFF_TIMER_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}


static volatile bool ready_flag;            // A flag indicating PWM status.

void pwm_ready_callback(uint32_t pwm_id)    // PWM callback function
{
    ready_flag = true;
}
typedef struct 
{
	short  position;
	short  length;
	char   character[128];
} custom_cmdLine;

typedef enum
{
	CUSTOM_RSP_ERROR = -1,
	CUSTOM_RSP_OK = 0,
	CUSTOM_RSP_LATER
} custom_rsp_type_enum;

typedef struct
{
	char *commandString;
	custom_rsp_type_enum (*commandFunc)(custom_cmdLine *commandBuffer_p);
} custom_atcmd;

static custom_rsp_type_enum custom_test_func(custom_cmdLine *commandBuffer_p)
{
	printf("enter custom_test_func adc1 = %d; adc2= %d\r\n",adc_data.adc1, adc_data.adc2);
	return  CUSTOM_RSP_OK;
}

const custom_atcmd custom_cmd_table[ ] =
{    
	{"AT%CUSTOM",custom_test_func},
	//{"AT+MSENSOR",custom_sensor_func},
  {NULL, NULL}  // this lind should not be removed, it will be treat as 
};

static bool uart_custom_common_hdlr(char *full_cmd_string)
{
	printf("data:%s\r\n",	full_cmd_string);
	char buffer[128];
	char *cmd_name, *cmdString;
	uint8_t index = 0; 
	uint16_t length;
	uint16_t i;
	custom_cmdLine command_line;
	cmd_name = buffer;
	length = strlen(full_cmd_string);
	length = length > 128 ? 128 : length;  

	while ((full_cmd_string[index] != '=' ) &&  //might be TEST command or EXE command
				(full_cmd_string[index] != '?' ) && // might be READ command
			(full_cmd_string[index] != 13 ) && //carriage return
	index < length)  
	{
		cmd_name[index] = full_cmd_string[index] ;
		index ++;
	}
	cmd_name[index] = '\0' ;   
	
    for (i = 0 ; custom_cmd_table[i].commandString != NULL; i++ )
    {
        cmdString = custom_cmd_table[i].commandString;
        if (strcmp(cmd_name, cmdString) == 0 )
        {
            strncpy(command_line.character, full_cmd_string, 128);
            command_line.character[127] = '\0';
            command_line.length = strlen(command_line.character);
            command_line.position = index;
            if (custom_cmd_table[i].commandFunc(&command_line) == CUSTOM_RSP_OK) {
								//PutUARTBytes("OK");
						}else{
								//PutUARTBytes("\r\nERROR\r\n");
            }
            return true;
        }
    }   
			
		return true;
}

static void uart_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
		app_timer_stop(uart_timer_id);
		uart_custom_common_hdlr((char *)data_array);
		memset(data_array, 0, UART_BUFFER_SIZE);
		timer_start = 0;
		index = 0;
}

static void uart_event_handle(app_uart_evt_t * p_event)
{
	switch (p_event->evt_type)
	{
				case APP_UART_DATA_READY:
						UNUSED_VARIABLE(app_uart_get(&data_array[index]));
						index++;
				
						if(timer_start == 0){
							app_timer_start(uart_timer_id, UART_IMTES_INTERVAL, NULL);
							timer_start = 1;
						}		
						
						if((index >= (UART_BUFFER_SIZE)))
						{
							memset(data_array, 0, UART_BUFFER_SIZE);
							index = 0;
						} 
						
						if((data_array[index - 1] == '\n') && (timer_start == 1)){
							app_timer_stop(uart_timer_id);
							uart_custom_common_hdlr((char *)data_array);
							
							memset(data_array, 0, UART_BUFFER_SIZE);
							index = 0;
							timer_start = 0;
						}
				
						break;
	      case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
	
	}
}

void uart_init(void)
{
    uint32_t                     err_code;
    const app_uart_comm_params_t comm_params =
    {
        RX_PIN_NUMBER,
        TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT( &comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}

#define SCAN_INTERVAL           0x00A0                          /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW             0x0050                          /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_ACTIVE             1                               /**< If 1, performe active scanning (scan requests). */
#define SCAN_SELECTIVE          0                               /**< If 1, ignore unknown devices (non whitelisted). */
#define SCAN_TIMEOUT            0x0000                          /**< Timout when scanning. 0x0000 disables timeout. */


static const ble_gap_scan_params_t m_scan_params =
{
    .active   = 1,
    .interval = SCAN_INTERVAL,
    .window   = SCAN_WINDOW,
    .timeout  = SCAN_TIMEOUT,
    #if (NRF_SD_BLE_API_VERSION == 2)
        .selective   = 0,
        .p_whitelist = NULL,
    #endif
    #if (NRF_SD_BLE_API_VERSION == 3)
        .use_whitelist = 0,
    #endif
};

static void scan_start(void)
{
    ret_code_t ret;

    ret = sd_ble_gap_scan_start(&m_scan_params);
    APP_ERROR_CHECK(ret);

    ret = bsp_indication_set(BSP_INDICATE_SCANNING);
    APP_ERROR_CHECK(ret);
}


/**
 * @brief Function for application main entry.
 */
int main(void)
{
    uint32_t err_code;
    // Initialize.
    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    //APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
    //err_code = bsp_init(BSP_INIT_LED, APP_TIMER_TICKS(100, APP_TIMER_PRESCALER), NULL);
    //APP_ERROR_CHECK(err_code);
		uart_init();
		nrf_gpio_cfg_output(20);
		nrf_gpio_pin_write(20,1);	
	
		nrf_gpio_cfg_output(LED_PIN);
		nrf_gpio_pin_write(LED_PIN,0);	
//	 /* 1-channel PWM, 200Hz*/
//    app_pwm_config_t pwm1_cfg = APP_PWM_DEFAULT_CONFIG_1CH(90000000L, LED_PIN);
//	/* Switch the polarity of the second channel. */
//    pwm1_cfg.pin_polarity[1] = APP_PWM_POLARITY_ACTIVE_HIGH;//APP_PWM_POLARITY_ACTIVE_HIGH;
//	
//		err_code = app_pwm_init(&PWM1,&pwm1_cfg,pwm_ready_callback);
//    APP_ERROR_CHECK(err_code);
//    app_pwm_enable(&PWM1);
//	
//		ready_flag = false;
//		while (app_pwm_channel_duty_set(&PWM1, 0, 90) == NRF_ERROR_BUSY);
//		while (!ready_flag);
//					APP_ERROR_CHECK(app_pwm_channel_duty_set(&PWM1, 1, 90));
					
		//app_pwm_uninit(&PWM1);
		 
		adc_config();
		timers_init();
		ble_stack_init();
		advertising_init();

    // Start execution.
    NRF_LOG_INFO("BLE Beacon started\r\n");
		application_timers_start();
    advertising_start();
		
		scan_start();

    // Enter main loop.
    for (;; )
    {	
        if (NRF_LOG_PROCESS() == false)
        {
            power_manage();
        }
    }
}


/**
 * @}
 */
