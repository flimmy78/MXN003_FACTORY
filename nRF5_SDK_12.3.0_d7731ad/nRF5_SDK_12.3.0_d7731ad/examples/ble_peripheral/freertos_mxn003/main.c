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
// Board/nrf6310/ble/ble_app_hrs_rtx/main.c
/**
 *
 * @brief Heart Rate Service Sample Application with RTX main file.
 *
 * This file contains the source code for a sample application using RTX and the
 * Heart Rate service (and also Battery and Device Information services).
 * This application uses the @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_nus.h"
#include "ble_bas.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "boards.h"
#include "sensorsim.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "peer_manager.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "fds.h"
#include "fstorage.h"
#include "ble_conn_state.h"
#include "nrf_drv_clock.h"
#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "app_uart.h"
#include "ble_dfu.h"
#include <string.h>
#include <stdarg.h>
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"

#define IS_SRVC_CHANGED_CHARACT_PRESENT  1                                /**< Include the Service Changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */

#if (NRF_SD_BLE_API_VERSION == 3)
#define NRF_BLE_MAX_MTU_SIZE            GATT_MTU_SIZE_DEFAULT             /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */
#endif

#define CENTRAL_LINK_COUNT               0                                /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT            1                                /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define DEVICE_NAME                      "MXN003"                     /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                "1.0.0.20170815_alpha "           				 /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                 300                              /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS       0                              /**< The advertising time-out in units of seconds. */

#define APP_TIMER_PRESCALER              0                                /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE          4                                /**< Size of timer operation queues. */

#define BATTERY_LEVEL_MEAS_INTERVAL      2000                             /**< Battery level measurement interval (ms). */
#define MIN_BATTERY_LEVEL                81                               /**< Minimum simulated battery level. */
#define MAX_BATTERY_LEVEL                100                              /**< Maximum simulated battery level. */
#define BATTERY_LEVEL_INCREMENT          1                                /**< Increment between each simulated battery level measurement. */

#define RR_INTERVAL_INTERVAL             300                              /**< RR interval interval (ms). */
#define MIN_RR_INTERVAL                  100                              /**< Minimum RR interval as returned by the simulated measurement function. */
#define MAX_RR_INTERVAL                  500                              /**< Maximum RR interval as returned by the simulated measurement function. */
#define RR_INTERVAL_INCREMENT            1                                /**< Value by which the RR interval is incremented/decremented for each call to the simulated measurement function. */

#define VOI_INTERVAL_INTERVAL             300                              /**< RR interval interval (ms). */
#define MIN_VOI_INTERVAL                  100                              /**< Minimum RR interval as returned by the simulated measurement function. */
#define MAX_VOI_INTERVAL                  500                              /**< Maximum RR interval as returned by the simulated measurement function. */
#define VOI_INTERVAL_INCREMENT            1 

#define UART_INTERVAL_INTERVAL             3000                              /**< RR interval interval (ms). */
#define MIN_UART_INTERVAL                  100                              /**< Minimum RR interval as returned by the simulated measurement function. */
#define MAX_UART_INTERVAL                  500                              /**< Maximum RR interval as returned by the simulated measurement function. */
#define UART_INTERVAL_INCREMENT            1 


#define SENSOR_CONTACT_DETECTED_INTERVAL 5000                             /**< Sensor Contact Detected toggle interval (ms). */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(400, UNIT_1_25_MS)            /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(600, UNIT_1_25_MS)            /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY   5000                             /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY    30000                            /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT     3                                /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                   1                                /**< Perform bonding. */
#define SEC_PARAM_MITM                   0                                /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                   0                                /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS               0                                /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES        BLE_GAP_IO_CAPS_NONE             /**< No I/O capabilities. */
#define SEC_PARAM_OOB                    0                                /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE           7                                /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE           16                               /**< Maximum encryption key size. */

#define DEAD_BEEF                        0xDEADBEEF                       /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define OSTIMER_WAIT_FOR_QUEUE           2                                /**< Number of ticks to wait for the timer queue to be ready */

#define APP_FEATURE_NOT_SUPPORTED        BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

static uint16_t  m_conn_handle = BLE_CONN_HANDLE_INVALID; /**< Handle of the current connection. */
static ble_bas_t m_bas;                                   /**< Structure used to identify the battery service. */
static ble_nus_t m_nus;                                   /**< Structure to identify the Nordic UART Service. */
static ble_dfu_t m_dfus;     

static uint8_t voice_count = 0;
static uint8_t voice_class = 0;
static uint8_t uart_timeout = 0;
#define SDAH nrf_gpio_pin_write(VOICE_CHIP_PIN, 1)
#define SDAL nrf_gpio_pin_write(VOICE_CHIP_PIN, 0)

static ble_uuid_t m_adv_uuids[] =                         /**< Universally unique service identifiers. */
{
		//{BLE_UUID_NUS_SERVICE, BLE_UUID_TYPE_VENDOR_BEGIN}
    //{BLE_UUID_BATTERY_SERVICE, BLE_UUID_TYPE_BLE},
    {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}
};

static TimerHandle_t m_battery_timer;        /**< Definition of battery timer. */
static TimerHandle_t m_voice_timer;
static TimerHandle_t m_uart_timer;
static SemaphoreHandle_t m_ble_event_ready;  /**< Semaphore raised if there is a new event to be processed in the BLE thread. */

static TaskHandle_t m_ble_stack_thread;      /**< Definition of BLE stack thread. */
static TaskHandle_t m_logger_thread;         /**< Definition of Logger thread. */

static void advertising_start(void);

static void uart_onoff(int8_t on)
{
	if(1 == on){
	 NRF_UART0->ENABLE        = (UART_ENABLE_ENABLE_Enabled << UART_ENABLE_ENABLE_Pos);
   NRF_UART0->TASKS_STARTRX = 1;
   NRF_UART0->TASKS_STARTTX = 1;
	}else{
		NRF_UART0->TASKS_STOPTX = 1;
		NRF_UART0->TASKS_STOPRX = 1;
		NRF_UART0->ENABLE       = (UART_ENABLE_ENABLE_Disabled << UART_ENABLE_ENABLE_Pos);
	}
}

static void uart_getline(uint8_t * line,uint32_t timeout)
{
    uint8_t i = 0, ch = 0;
    uint32_t err_code;

		do {
			err_code = app_uart_get(&ch);
			if (NRF_ERROR_NOT_FOUND == err_code)
			{
					if(timeout-- > 0){
						nrf_delay_ms(1000);
						sd_app_evt_wait();
					}else{
						break;
					}
			}
			else if (NRF_SUCCESS == err_code)   
			{
				line[i++] = ch;
			}
			else
			{
				APP_ERROR_CHECK(err_code);
			}
		}
		while (ch != '\n');
}

static uint8_t* get_uart_responese(uint8_t * command_buffer)
{
		uart_getline(command_buffer,5);
		
		return command_buffer;
}

static int8_t uart_data_analyze(char *str, int buffer, int timeout)
{
		uint8_t cmd_buffer[10] = {0};
//		time_t start;
//		start = time(NULL);
		timeout = 10;
		while(1){
			get_uart_responese(cmd_buffer);
			if(strstr((const char *)cmd_buffer,(const char *)str) != NULL)
				return 1;
		}
		return -1;
}

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


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;

    switch (p_evt->evt_id)
    {
        case PM_EVT_BONDED_PEER_CONNECTED:
        {
            NRF_LOG_INFO("Connected to a previously bonded device.\r\n");
        } break;

        case PM_EVT_CONN_SEC_SUCCEEDED:
        {
            NRF_LOG_INFO("Connection secured. Role: %d. conn_handle: %d, Procedure: %d\r\n",
                         ble_conn_state_role(p_evt->conn_handle),
                         p_evt->conn_handle,
                         p_evt->params.conn_sec_succeeded.procedure);
        } break;

        case PM_EVT_CONN_SEC_FAILED:
        {
            /* Often, when securing fails, it shouldn't be restarted, for security reasons.
             * Other times, it can be restarted directly.
             * Sometimes it can be restarted, but only after changing some Security Parameters.
             * Sometimes, it cannot be restarted until the link is disconnected and reconnected.
             * Sometimes it is impossible, to secure the link, or the peer device does not support it.
             * How to handle this error is highly application dependent. */
        } break;

        case PM_EVT_CONN_SEC_CONFIG_REQ:
        {
            // Reject pairing request from an already bonded peer.
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
            pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
        } break;

        case PM_EVT_STORAGE_FULL:
        {
            // Run garbage collection on the flash.
            err_code = fds_gc();
            if (err_code == FDS_ERR_BUSY || err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
            {
                // Retry.
            }
            else
            {
                APP_ERROR_CHECK(err_code);
            }
        } break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
        {
            advertising_start();
        } break;

        case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
        {
            // The local database has likely changed, send service changed indications.
            pm_local_database_has_changed();
        } break;

        case PM_EVT_PEER_DATA_UPDATE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_data_update_failed.error);
        } break;

        case PM_EVT_PEER_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
        } break;

        case PM_EVT_PEERS_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
        } break;

        case PM_EVT_ERROR_UNEXPECTED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
        } break;

        case PM_EVT_CONN_SEC_START:
        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
        case PM_EVT_PEER_DELETE_SUCCEEDED:
        case PM_EVT_LOCAL_DB_CACHE_APPLIED:
        case PM_EVT_SERVICE_CHANGED_IND_SENT:
        case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
        default:
            break;
    }
}


/**@brief Function for performing battery measurement and updating the Battery Level characteristic
 *        in Battery Service.
 */
static void battery_level_update(void)
{
    uint32_t err_code;
    uint8_t  battery_level;

    battery_level = 98; //(uint8_t)sensorsim_measure(&m_battery_sim_state, &m_battery_sim_cfg);

    err_code = ble_bas_battery_level_update(&m_bas, battery_level);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_PACKETS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }
}

static void voice_chip_output(uint8_t reg)
{
	SDAL;
	nrf_delay_ms(2);
	for(uint8_t i = 0; i < 8; i++)
	{
		SDAH;
		if(reg & 1){
			nrf_delay_us(1200);
			SDAL;
			nrf_delay_us(400);
		}else{
			nrf_delay_us(400);
			SDAL;
			nrf_delay_us(1200);
		}
		reg >>= 1;
	}
	SDAH;
//	nrf_gpio_pin_write(VOICE_CHIP_PIN, 0);
//	nrf_delay_ms(4);

//	for(int i = 0; i < reg; i++){
//		nrf_gpio_pin_write(VOICE_CHIP_PIN, 1);
//		nrf_delay_us(800);
//		nrf_gpio_pin_write(VOICE_CHIP_PIN, 0);
//		nrf_delay_us(2400);
//	}
//	nrf_gpio_pin_write(VOICE_CHIP_PIN, 1);
}	

void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	static uint8_t i = 1;
	if(i < voice_count){
		i++;
		if (pdPASS != xTimerStart(m_voice_timer, OSTIMER_WAIT_FOR_QUEUE))
		{
			APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
		}
	}else{
		i = 1;
		nrf_gpio_pin_write(VOICE_MOSE_PIN, 1);
		ble_nus_string_send(&m_nus,(uint8_t *) "voice:accomplish", sizeof("voice:accomplish"));
	}
}

void in_pin_handler_test(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	switch(pin){
		case 3:
			if( nrf_gpio_pin_read(MAC_CHECK_PIN1) == 1){
				NRF_LOG_INFO("MAC_CHECK_PIN1 HIGH\r\n");
			}
			else{
				NRF_LOG_INFO("MAC_CHECK_PIN1 LOW\r\n");
			}
			break;
		case 4:
		if( nrf_gpio_pin_read(MAC_CHECK_PIN2) == 1){
				NRF_LOG_INFO("MAC_CHECK_PIN2 HIGH\r\n");
			}
			else{
				NRF_LOG_INFO("MAC_CHECK_PIN2 LOW\r\n");
			}
			break;
		default:
			NRF_LOG_INFO("default ...!!!\r\n");
			break;		
	}

}


static void uart_timeout_handler(TimerHandle_t xTimer)
{
	UNUSED_PARAMETER(xTimer);
	NRF_LOG_INFO("uart_timeout_handler\r\n");
	NRF_LOG_FLUSH();
	uart_timeout = 1;
}


static void voice_timeout_handler(TimerHandle_t xTimer)
{
	NRF_LOG_INFO("voice_timeout_handler REG =  %d\r\n",voice_class);
	UNUSED_PARAMETER(xTimer);
	nrf_gpio_pin_write(VOICE_MOSE_PIN, 0);
	nrf_delay_ms(500);
	voice_chip_output(voice_class);
//	static uint8_t i = 1;
//	if(i < voice_count){
//		if (pdPASS != xTimerStart(m_voice_timer, OSTIMER_WAIT_FOR_QUEUE))
//		{
//			APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
//		}
//		i++;
//	}else{
//		i = 0;
//	}
}


/**@brief Function for handling the Battery measurement timer time-out.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in] xTimer Handler to the timer that called this function.
 *                   You may get identifier given to the function xTimerCreate using pvTimerGetTimerID.
 */
static void battery_level_meas_timeout_handler(TimerHandle_t xTimer)
{
    UNUSED_PARAMETER(xTimer);
    battery_level_update();
}





/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{

//    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
//    // Create timers.
    m_battery_timer = xTimerCreate("BATT",
                                   BATTERY_LEVEL_MEAS_INTERVAL,
                                   pdTRUE,
                                   NULL,
                                   battery_level_meas_timeout_handler);
	
	  m_voice_timer = xTimerCreate("VOICE",
                                   VOI_INTERVAL_INTERVAL,
                                   pdFALSE,
                                   NULL,
                                   voice_timeout_handler);
	
		m_uart_timer = xTimerCreate("UART",
                                   UART_INTERVAL_INTERVAL,
                                   pdFALSE,
                                   NULL,
                                   uart_timeout_handler);
	
	
    /* Error checking */
    if ((NULL == m_battery_timer)
			 || (NULL == m_voice_timer)
			 || (NULL == m_uart_timer))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);
		/*
    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_HEART_RATE_SENSOR_HEART_RATE_BELT);
    APP_ERROR_CHECK(err_code);
		*/

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

static void U_PutUARTBytes(uint8_t *data,uint16_t len)
{
   uint16_t index;

   for(index = 0; index < len; index++)
			while(app_uart_put(*(data+index)) != NRF_SUCCESS);
}

static void U_PutUARTByte(char * fmt, int size){
	U_PutUARTBytes((uint8_t*)fmt, size);
	U_PutUARTBytes((uint8_t*)"\r\n", 2);
}


static void PutUARTBytes(const char *fmt, ...)
{
    static char logCbuf[1024];
    va_list args;
    char *p;
    int n, m;

    memset(logCbuf, 0, 1024);
    p = logCbuf;
    m = 1020;
    va_start(args, fmt);
    n = vsnprintf(p, m, fmt, args);
    va_end(args);

    U_PutUARTByte(logCbuf, n);
}

typedef struct 
{
	short  position;
	short  length;
	char   character[20];
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

typedef enum
{
	CUSTOM_WRONG_MODE,
	CUSTOM_SET_OR_EXECUTE_MODE,
	CUSTOM_READ_MODE,
	CUSTOM_TEST_MODE,
	CUSTOM_ACTIVE_MODE
} custom_cmd_mode_enum;

typedef struct
{
	short  position;
	uint8_t   	 part;
	char     	 	 rcv_msg[20];
	char       	 *pars[20];
	uint16_t      rcv_length;
} cmd_data_struct;

typedef enum
{
    CM_Main,
    CM_Par1,
    CM_Par2,
    CM_Par3,
    CM_Par4,
    CM_Par5,
    CM_Par6,
    CM_Par7,
    CM_Par8,
    CMD_MAX_Pars
} Cmd_Pars;

static custom_cmd_mode_enum custom_find_cmd_mode(custom_cmdLine *cmd_line)
{
    custom_cmd_mode_enum result;
   // if (cmd_line->position < cmd_line->length - 1) //modfiy by xuzhoubin for no '\r\n'
		if (cmd_line->position < cmd_line->length)
    {
        switch (cmd_line->character[cmd_line->position])
        {
            case '?':  /* AT+...? */
            {
                cmd_line->position++;
                result = CUSTOM_READ_MODE;
                break;
            }
            case '=':  /* AT+...= */
            {
                cmd_line->position++;								
                if ((cmd_line->position < cmd_line->length ) &&
                    (cmd_line->character[cmd_line->position] == '?'))
                {
                    cmd_line->position++;
                    result = CUSTOM_TEST_MODE;
                }
                else
                {
                    result = CUSTOM_SET_OR_EXECUTE_MODE;
                }
                break;
            }
            default:  /* AT+... */
            {
                result = CUSTOM_ACTIVE_MODE;
                break;
            }
        }
    }
    else
    {
        result = CUSTOM_ACTIVE_MODE;
    }
    return (result);
}

static int fun_str_analyse(char *str_data, char **tar_data, int limit, char startChar, char *endChars, char splitChar)
{
    static char *blank = "";
    int len, i = 0, j = 0, status = 0;
    char *p;
    if(str_data == NULL || tar_data == NULL)
    {
        return -1;
    }
    len = strlen(str_data);
	
		if(str_data[len - 1]  == '\n')  //add by xuzhoubin for have  \r\n,数据解析error
				len -=2;
	
    for(i = 0, j = 0, p = str_data; i < len; i++, p++)
    {
        if(status == 0 && (*p == startChar || startChar == NULL))
        {
            status = 1;
            if(j >= limit)
            {
                return -2;
            }
            if((startChar == NULL))
            {
                tar_data[j++] = p;
            }
            else if(*(p + 1) == splitChar)
            {
                tar_data[j++] = blank;
            }
            else
            {
                tar_data[j++] = p + 1;
            }
        }
        if(status == 0)
        {
            continue;
        }
        if(strchr(endChars, *p) != NULL)
        {
            *p = 0;
            break;
        }
        if(*p == splitChar)
        {
            *p = 0;
            if(j >= limit)
            {
                return -3;
            }
            if(strchr(endChars, *(p + 1)) != NULL || *(p + 1) == splitChar)
            {
                tar_data[j++] = blank;
            }
            else
            {
                tar_data[j++] = p + 1;
            }
        }
    }
    for(i = j; i < limit; i++)
    {
        tar_data[i] = blank;
    }
    return j;
}

static int cmd_analyse(cmd_data_struct * command)
{
	char        *data_ptr, split_ch = ',';
	int         cmd_Len, par_len;
	
	
	if(command == NULL || command->rcv_length < 1)
  {
        return -1;
  }
	//fun_toUpper(command->rcv_msg);
	data_ptr = &command->rcv_msg[command->position];//parse_sms_head(command->rcv_msg); 
	
	cmd_Len = strlen(data_ptr);
	

    if(data_ptr[cmd_Len - 3] == '#' && data_ptr[cmd_Len - 2] == 0x0D && data_ptr[cmd_Len - 1] == 0x0A)
    {
        data_ptr[cmd_Len - 3] = 0;
    }
    else if(data_ptr[cmd_Len - 2] == '#' && data_ptr[cmd_Len - 1] == 0x0D)
    {
        data_ptr[cmd_Len - 2] = 0;
    }
    else if(data_ptr[cmd_Len - 1] == '#')
    {
        data_ptr[cmd_Len - 1] = 0;
    }
			
		par_len = fun_str_analyse(data_ptr, command->pars, 20, NULL, "\r\n", split_ch);

    if(par_len > 20 || par_len <= 0)
    {
        return -1;
    }
		
		if(par_len > 20 || par_len <= 0)
    {
        return -1;
    }
		
		if((par_len - CM_Main) > 0)
    {
        command->part = par_len - CM_Main;
    }
    else
    {
        command->part = 0;
    }	

	return 1;
}

int8_t wait_reponse(char *desire_rep, uint16_t timeout)
{
	uint8_t tmp_tout = 0;
	uint8_t tmp_i = 0;
	uint8_t p;
	uint8_t str[30];
	while(1){
		while(NRF_SUCCESS != app_uart_get(&p)){
			//nrf_delay_ms(100);
//			tmp_tout ++;
//			if(tmp_tout > timeout)
//				return 2;
		}
		NRF_LOG_INFO("strxxx = %c\r\n",p);
		NRF_LOG_FLUSH();
		
		str[tmp_i] = p;
		tmp_i++;

		NRF_LOG_INFO("str[] = %s\r\n",(uint32_t)str);
		NRF_LOG_FLUSH();
		
		
		if((p == '\r') || (p == '\n')){
		NRF_LOG_INFO(" is r/r/n \r\n");
		NRF_LOG_FLUSH();
		}
		if(0 != strstr((char *)str,desire_rep)){
			return 1;
		}
	}
}


cmd_data_struct at_get_at_para(custom_cmdLine *commandBuffer_p)
{
		static cmd_data_struct at_cmd = {0};
		if(commandBuffer_p->length && (commandBuffer_p->length <(20)))
		{
			memset(&at_cmd, 0, sizeof(cmd_data_struct)); 
			memcpy(&at_cmd.rcv_msg, commandBuffer_p->character, commandBuffer_p->length);	

			at_cmd.rcv_length = commandBuffer_p->length;
			at_cmd.position   = commandBuffer_p->position;
			
			cmd_analyse(&at_cmd);
		}
		return at_cmd;
}

custom_rsp_type_enum custom_rssi_func(custom_cmdLine *commandBuffer_p){
		NRF_LOG_INFO("custom_rssi_func\r\n");
		PutUARTBytes("AT+RSSI\r\n");
		return CUSTOM_RSP_OK;
}

custom_rsp_type_enum custom_voice_func(custom_cmdLine *commandBuffer_p){
		custom_cmd_mode_enum result;
    custom_rsp_type_enum ret_value  = CUSTOM_RSP_ERROR;
    result = custom_find_cmd_mode(commandBuffer_p);
	
		cmd_data_struct cmd = at_get_at_para(commandBuffer_p);
		if(cmd.part != 2)
			return CUSTOM_RSP_ERROR;

		switch (result)
    {
				case CUSTOM_SET_OR_EXECUTE_MODE:
						voice_count = atoi(cmd.pars[1]);
						voice_class = atoi(cmd.pars[0]);
						ble_nus_string_send(&m_nus,(uint8_t *) "voice:start", sizeof("voice:start"));
						if (pdPASS != xTimerStart(m_voice_timer, OSTIMER_WAIT_FOR_QUEUE))
						{
							APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
						}
						ret_value = CUSTOM_RSP_OK;
						break;
        default:
            ret_value = CUSTOM_RSP_ERROR;
            break;
		}
		
		return ret_value;
}

void Delay(unsigned int nDelay)
{
	unsigned int i,j,k;
	for ( i=0;i<nDelay;i++ )
		for ( j=0;j<6144;j++ )
			k++;
}

int  uart_get_with_timeout(int32_t timeout_ms, char * rx_data, char *des_str)
{
    uint8_t i = 0, ch = 0;
    uint32_t err_code;

		int  ret = 1;
		do {
			err_code = app_uart_get(&ch);
			if (NRF_ERROR_NOT_FOUND == err_code)
			{
				if(timeout_ms-- > 0){
					Delay(1000);
				}else{
					return 2;
				}
			}
			else if (NRF_SUCCESS == err_code)   
			{
				rx_data[i++] = ch;
				if(strstr((const char *)rx_data,(const char *)des_str) != NULL){
				return 1;
				}
			}
			else
			{
				APP_ERROR_CHECK(err_code);
			}
		}
		while (1);
}


custom_rsp_type_enum custom_mod_func(custom_cmdLine *commandBuffer_p){
		custom_cmd_mode_enum result;
    custom_rsp_type_enum ret_value  = CUSTOM_RSP_ERROR;
    result = custom_find_cmd_mode(commandBuffer_p);

	
		cmd_data_struct cmd = at_get_at_para(commandBuffer_p);
		if(cmd.part != 1)
			return CUSTOM_RSP_ERROR;
		uint8_t status = atoi(cmd.pars[0]);
		switch (result)
    {
				case CUSTOM_SET_OR_EXECUTE_MODE:
						if(1 == status){
							nrf_gpio_pin_write(MODME_CONTRL_PIN, 0);
							uart_onoff(1);
							nrf_delay_ms(2000);
							uint8_t data_buffer[512] = {0};
							char str_buff[] = "+EUSIM: 1\r\n";
			
							uint8_t ret = uart_get_with_timeout(0,data_buffer,str_buff);
							if(ret == 1){
								ble_nus_string_send(&m_nus, (uint8_t *)"modem:open:sucess", sizeof("modem:open:sucess"));
							}else{
								ble_nus_string_send(&m_nus, (uint8_t *)"modem:open:fail", sizeof("modem:open:fail"));
							}
							NRF_LOG_INFO("ret = %d\r\n",ret);
							
							NRF_LOG_INFO("data::%s\r\n",(uint32_t)data_buffer);
		//--------------------------------------------					
							
//							uint8_t p;
//							while(1){
//							bool ret = uart_get_with_timeout(10,&p);
//								if(ret == false)
//									break;
//							
//							NRF_LOG_INFO("%c\r\n",p);
//							NRF_LOG_FLUSH();
//							}
						//	wait_reponse("+EUSIM",20);
//							uint8_t cmd_buffer[10] = {0};
//							while(1){
//								get_uart_responese(cmd_buffer);
//								NRF_LOG_INFO("data:%s  |||| size=%d\r\n", (uint32_t)cmd_buffer,sizeof(cmd_buffer));
//								if(strstr((const char *)cmd_buffer, "+EUSIM: 1") != NULL){
//									ble_nus_string_send(&m_nus, "modem_open", 10);
//									return CUSTOM_RSP_OK;
//								}
//							}
							//int ret = wait_reponse("+EUSIM",3);
							//NRF_LOG_INFO("xxxret = %d\r\n", ret);
//							uint8_t cmd_buffer[20] = {0};
//							uart_timeout = 0;
//							xTimerStart(m_uart_timer, OSTIMER_WAIT_FOR_QUEUE);
//							while(1){
//									get_uart_responese(cmd_buffer);
//									if(strstr((const char *)cmd_buffer, "+EUSIM") != NULL){
//										ble_nus_string_send(&m_nus, "modem_open", 10);
//										return CUSTOM_RSP_OK;
//										}else if (uart_timeout == 1){
//											ble_nus_string_send(&m_nus, "timeout", 7);
//											return CUSTOM_RSP_OK;
//										}
//								}
	
						}else{
							uart_onoff(0);
							nrf_gpio_pin_write(MODME_CONTRL_PIN, 1);
							ble_nus_string_send(&m_nus, (uint8_t *)"modem:close:sucess", sizeof("modem:close:sucess"));
							ret_value = CUSTOM_RSP_OK;
						}
						break;
        default:
            ret_value = CUSTOM_RSP_ERROR;
            break;
		}
		
		return ret_value;	
}

custom_rsp_type_enum custom_modem_version_func(custom_cmdLine *commandBuffer_p){
//		custom_cmd_mode_enum result;
//    custom_rsp_type_enum ret_value  = CUSTOM_RSP_ERROR;
//    result = custom_find_cmd_mode(commandBuffer_p);
//	
//		cmd_data_struct cmd = at_get_at_para(commandBuffer_p);
		//NRF_LOG_INFO("custom_modem_version_func\r\n");
			//PutUARTBytes("AT+EVERN");
			printf("AT+EVERN\r\n");
			nrf_delay_ms(10000);
//			uint8_t data_buffer[512] = {0};
//			char *data = NULL;
//			data = (char *)malloc(512);
//			//char str_buff[] = "V1_00_00_M170815\r\n";
//			char str_buff[] = "xxxxxxxxxxxxxxxxxx";
//			uint8_t ret = uart_get_with_timeout(2,data,str_buff);
//			if(ret == 1){				
//				ble_nus_string_send(&m_nus, (uint8_t *)"V1_00_00_M170815", sizeof("V1_00_00_M170815"));
//			}else{
//				ble_nus_string_send(&m_nus, (uint8_t *)"ver:error", sizeof("ver:error"));
//			}
//			NRF_LOG_INFO("ret = %d\r\n",ret);

//			NRF_LOG_INFO("data::%s\r\n",(uint32_t)data);
			
			//----------------------------------
	
		while(1){
			uint8_t cmd_buffer[20] = {0};
			uint8_t *p = get_uart_responese(cmd_buffer);
			if(cmd_buffer[0] == 0){
				NRF_LOG_INFO("get buff null \r\n");
				NRF_LOG_FLUSH();
				break;
			}
			NRF_LOG_INFO("data::%s\r\n",(uint32_t)cmd_buffer);
			NRF_LOG_FLUSH();
	
			if(strstr((const char *)cmd_buffer, "_M") != NULL){
				ble_nus_string_send(&m_nus, cmd_buffer, sizeof(cmd_buffer));
				return CUSTOM_RSP_OK;
			}
		}
//----------------------------------------------------------
			//PutUARTBytes("AT+ESTARTFOTA"); //升级
	
//		uart_timeout = 0;
//		xTimerStart(m_uart_timer, OSTIMER_WAIT_FOR_QUEUE);
//		while(uart_timeout != 1){
//			uint8_t cmd_buffer[20] = {0};
//			get_uart_responese(cmd_buffer);
//			NRF_LOG_INFO("xxx %s\r\n",(uint32_t)cmd_buffer);
//			NRF_LOG_FLUSH();
	//	xTimerStop(m_uart_timer, OSTIMER_WAIT_FOR_QUEUE);
//		
//		}
//		NRF_LOG_INFO("custom_modem_version_over\r\n");
//		NRF_LOG_FLUSH();
//		if(cmd.part != 0)
//			return CUSTOM_RSP_ERROR;
//		NRF_LOG_INFO("version result =%d\r\n", result);
//		//PutUARTBytes("AT+EVERN");
//		PutUARTBytes("AT+EGETTIME");
//		
//		uint8_t cmd_buffer[20] = {0};
//		uart_timeout = 0;
//		xTimerStart(m_uart_timer, OSTIMER_WAIT_FOR_QUEUE);
//		while(1){
//			get_uart_responese(cmd_buffer);
//			ble_nus_string_send(&m_nus, cmd_buffer, sizeof(cmd_buffer));
//				return CUSTOM_RSP_ERROR;


//		}		
////		switch (result)
//    {
//				case CUSTOM_READ_MODE:
//						PutUARTBytes("AT+EVERN");
////						uint8_t cmd_buffer[20] = {0};
////							while(1){
////									get_uart_responese(cmd_buffer);
////									if(strstr((const char *)cmd_buffer, "v") != NULL){
////											ble_nus_string_send(&m_nus, cmd_buffer, sizeof(cmd_buffer));
////									}
////							}
//						ret_value = CUSTOM_RSP_OK;
//						break;
//        default:
//            ret_value = CUSTOM_RSP_ERROR;
//            break;
//		}
//		
		//return ret_value;
}

custom_rsp_type_enum custom_open_gps_func(custom_cmdLine *commandBuffer_p){
	PutUARTBytes("AT+EGPSON");
//	uint8_t data_buffer[512] = {0};
//	char str_buff[] = "OK\r\n";
//	uint8_t ret = uart_get_with_timeout(20,data_buffer,str_buff);
//	if(ret == 1){
//		ble_nus_string_send(&m_nus, (uint8_t *)"gps:open:sucess", sizeof("gps:open:sucess"));
//	}else{
//		ble_nus_string_send(&m_nus, (uint8_t *)"gps:open:fail", sizeof("gps:open:fail"));
//	}
//	NRF_LOG_INFO("ret = %d\r\n",ret);

//	NRF_LOG_INFO("data::%s\r\n",(uint32_t)data_buffer);

//	while(1){
//			uint8_t cmd_buffer[30] = {0};
//			get_uart_responese(cmd_buffer);
//			NRF_LOG_INFO("%s\r\n", (uint32_t)cmd_buffer);
//			if(strstr((const char *)cmd_buffer, "OK") != NULL){
//				ble_nus_string_send(&m_nus, cmd_buffer, sizeof(cmd_buffer));
//				return CUSTOM_RSP_OK;
//			}else if(strstr((const char *)cmd_buffer, "ERROR") != NULL){
//				ble_nus_string_send(&m_nus, cmd_buffer, sizeof(cmd_buffer));
//				return CUSTOM_RSP_OK;
//			}
//		}
}

custom_rsp_type_enum custom_close_gps_func(custom_cmdLine *commandBuffer_p){
	PutUARTBytes("AT+EGPSOFF");
//	uint8_t data_buffer[512] = {0};
//	char str_buff[] = "OK\r\n";
//	uint8_t ret = uart_get_with_timeout(50,data_buffer,str_buff);
//	if(ret == 1){
//		ble_nus_string_send(&m_nus, (uint8_t *)"gps:close:sucess", sizeof("gps:close:sucess"));
//	}else{
//		ble_nus_string_send(&m_nus, (uint8_t *)"gps:close:fail", sizeof("gps:close:fail"));
//	}
//	NRF_LOG_INFO("ret = %d\r\n",ret);

//	NRF_LOG_INFO("data::%s\r\n",(uint32_t)data_buffer);
//	while(1){
//			uint8_t cmd_buffer[30] = {0};
//			get_uart_responese(cmd_buffer);
//			NRF_LOG_INFO("%s\r\n", (uint32_t)cmd_buffer);
//			if(strstr((const char *)cmd_buffer, "OK") != NULL){
//				ble_nus_string_send(&m_nus, cmd_buffer, sizeof(cmd_buffer));
//				return CUSTOM_RSP_OK;
//			}else if(strstr((const char *)cmd_buffer, "ERROR") != NULL){
//				ble_nus_string_send(&m_nus, cmd_buffer, sizeof(cmd_buffer));
//				return CUSTOM_RSP_OK;
//			}
//		}
}

custom_rsp_type_enum custom_get_gps_data_func(custom_cmdLine *commandBuffer_p){
	PutUARTBytes("AT+EGPSGET");
}

const custom_atcmd custom_cmd_table[ ] =
{
	{"AP+RSSI",custom_rssi_func},
	{"AP+VOICE",custom_voice_func},
	{"AP+MOD",custom_mod_func},
	{"AP+SWVER",custom_modem_version_func},
	{"AP+EGPSON",custom_open_gps_func},
	{"AP+EGPSOFF",custom_close_gps_func},
	{"AP+EGPSGET",custom_get_gps_data_func},
	{NULL, NULL}
};

static bool app_data_hdlr(char *full_cmd_string,uint16_t length)
{
		char buffer[20];
//		uint8_t re_Data[] = "OK";
//		uint8_t re_Data1[] = "ERROR";
		char *cmd_name, *cmdString;
		uint8_t index = 0; 
		uint16_t i;
		custom_cmdLine command_line;
		cmd_name = buffer;
		length = length > 20 ? 20 : length;  
	
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
			if (strcmp(cmd_name, cmdString) == 0 ){
					strncpy(command_line.character, full_cmd_string, 20);
					command_line.character[19] = '\0';
					command_line.length = strlen(command_line.character);
					command_line.position = index;
					if (custom_cmd_table[i].commandFunc(&command_line) == CUSTOM_RSP_OK) {
							//	ble_nus_string_send(&m_nus, re_Data, sizeof(re_Data));
						}else{
							//	ble_nus_string_send(&m_nus, re_Data1, sizeof(re_Data1));
            }
					return true;
			}
		}
		
		return true;
		
}
/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
	/*收到手机的消息*/
	//进行数据解析		
			uint8_t *data = p_data;
		//	NRF_LOG_INFO("data:%s length:%d\r\n",(uint32_t)p_data,length)
			app_data_hdlr((char *)data, length);
			memset(p_data, 0, sizeof(uint8_t)*length);
//		NRF_LOG_INFO("data:%s    length:%d\r\n",(uint32_t)p_data,length)
//		char *data = (char *)p_data;
//		uint8_t cmd = 0;
//		if(!strncmp(data,"AP+EVERN",length)){
//			cmd = 1;
//		}else if(!strncmp(data,"AP+VOICE",8)){
//			cmd = 2;
//		}else{
//			cmd = 0;
//		}
//		NRF_LOG_INFO("cmd = %d\r\n",cmd);
//		/*发送AT命令*/
//		switch(cmd){
//			case 1:
//				PutUARTBytes("AT+EVERN\r\n");
//				break;
//			case 2:
//					data += 8; length -=8;
//					NRF_LOG_INFO("data = %s length = %d  %d\r\n",(uint32_t)data,length, atoi(data));
//				break;
//			default:
//				break;
//		}
	//	memset(p_data,0,20);
//    for (uint32_t i = 0; i < length; i++)
//    {
//        while (app_uart_put(p_data[i]) != NRF_SUCCESS);
//    }
//    while (app_uart_put('\r') != NRF_SUCCESS);
//    while (app_uart_put('\n') != NRF_SUCCESS);
}
/**@snippet [Handling the data received over BLE] */

static void ble_dfu_evt_handler(ble_dfu_t * p_dfu, ble_dfu_evt_t * p_evt)
{
    switch (p_evt->type)
    {
        case BLE_DFU_EVT_INDICATION_DISABLED:
            NRF_LOG_INFO("Indication for BLE_DFU is disabled\r\n");
            break;

        case BLE_DFU_EVT_INDICATION_ENABLED:
            NRF_LOG_INFO("Indication for BLE_DFU is enabled\r\n");
            break;

        case BLE_DFU_EVT_ENTERING_BOOTLOADER:
            NRF_LOG_INFO("Device is entering bootloader mode!\r\n");
            break;
        default:
            NRF_LOG_INFO("Unknown event from ble_dfu\r\n");
            break;
    }
}

///
void ble_address_change(void)
{
	ble_gap_addr_t ble_addr;
	sd_ble_gap_address_get(&ble_addr);
	ble_addr.addr[0] = 0x99;
	ble_addr.addr[1] = 0x99;
	ble_addr.addr[2] = 0x99;
	ble_addr.addr[3] = 0x99;
	ble_addr.addr[4] = 0x99;
	ble_addr.addr[5] = 0xff;

	sd_ble_gap_address_set(BLE_GAP_ADDR_CYCLE_MODE_NONE,&ble_addr);
}

/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Heart Rate, Battery and Device Information services.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_bas_init_t bas_init;
    ble_dis_init_t dis_init;
		ble_nus_init_t nus_init;
		ble_dfu_init_t dfus_init;
	
	  // Initialize the Device Firmware Update Service.
    memset(&dfus_init, 0, sizeof(dfus_init));

    dfus_init.evt_handler                               = ble_dfu_evt_handler;
    dfus_init.ctrl_point_security_req_write_perm        = SEC_SIGNED;
    dfus_init.ctrl_point_security_req_cccd_write_perm   = SEC_SIGNED;

    err_code = ble_dfu_init(&m_dfus, &dfus_init);
    APP_ERROR_CHECK(err_code);
//    
//	
	 // Initialize nus Service.
		memset(&nus_init, 0, sizeof(nus_init));
		nus_init.data_handler = nus_data_handler;
	
		err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Battery Service.
    memset(&bas_init, 0, sizeof(bas_init));

    // Here the sec level for the Battery Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init.battery_level_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_report_read_perm);

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char *)MANUFACTURER_NAME);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting application timers.
 */
static void application_timers_start(void)
{
    // Start application timers.
    if (pdPASS != xTimerStart(m_battery_timer, OSTIMER_WAIT_FOR_QUEUE))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in]   p_evt   Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;//m_hrs.hrm_handles.cccd_handle;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);

    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast Adverstising\r\n");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;

        default:
            break;
    }
}


/**@brief Function for receiving the Application's BLE Stack events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected\r\n");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break; // BLE_GAP_EVT_CONNECTED

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected\r\n");
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break; // BLE_GAP_EVT_DISCONNECTED

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTC_EVT_TIMEOUT

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_TIMEOUT

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(m_conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_EVT_USER_MEM_REQUEST

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            ble_gatts_evt_rw_authorize_request_t  req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = p_ble_evt->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                               &auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

#if (NRF_SD_BLE_API_VERSION == 3)
        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
            err_code = sd_ble_gatts_exchange_mtu_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                       NRF_BLE_MAX_MTU_SIZE);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST
#endif

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    /** The Connection state module has to be fed BLE events in order to function correctly
     * Remember to call ble_conn_state_on_ble_evt before calling any ble_conns_state_* functions. */
	ble_conn_state_on_ble_evt(p_ble_evt);
	pm_on_ble_evt(p_ble_evt);


	ble_conn_params_on_ble_evt(p_ble_evt);
	bsp_btn_ble_on_ble_evt(p_ble_evt);
	on_ble_evt(p_ble_evt);
	ble_advertising_on_ble_evt(p_ble_evt);
	ble_dfu_on_ble_evt(&m_dfus, p_ble_evt);
	ble_nus_on_ble_evt(&m_nus, p_ble_evt);
  ble_bas_on_ble_evt(&m_bas, p_ble_evt);
	/*YOUR_JOB add calls to _on_ble_evt functions from each service your application is using
	ble_xxs_on_ble_evt(&m_xxs, p_ble_evt);
	ble_yys_on_ble_evt(&m_yys, p_ble_evt);
	*/
}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in]   sys_evt   System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    // Dispatch the system event to the fstorage module, where it will be
    // dispatched to the Flash Data Storage (FDS) module.
    fs_sys_event_handler(sys_evt);

    // Dispatch to the Advertising module last, since it will check if there are any
    // pending flash operations in fstorage. Let fstorage process system events first,
    // so that it can report correctly to the Advertising module.
    ble_advertising_on_sys_evt(sys_evt);
}


/**
 * @brief Event handler for new BLE events
 *
 * This function is called from the SoftDevice handler.
 * It is called from interrupt level.
 *
 * @return The returned value is checked in the softdevice_handler module,
 *         using the APP_ERROR_CHECK macro.
 */
static uint32_t ble_new_event_handler(void)
{
    BaseType_t yield_req = pdFALSE;

    // The returned value may be safely ignored, if error is returned it only means that
    // the semaphore is already given (raised).
    UNUSED_VARIABLE(xSemaphoreGiveFromISR(m_ble_event_ready, &yield_req));
    portYIELD_FROM_ISR(yield_req);
    return NRF_SUCCESS;
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
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, ble_new_event_handler);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);
	
		ble_enable_params.common_enable_params.vs_uuid_count = 4;

    // Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT, PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
#if (NRF_SD_BLE_API_VERSION == 3)
    ble_enable_params.gatt_enable_params.att_mtu = NRF_BLE_MAX_MTU_SIZE;
#endif
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
//static void bsp_event_handler(bsp_event_t event)
//{
//    uint32_t err_code;

//    switch (event)
//    {
//        case BSP_EVENT_SLEEP:
//            sleep_mode_enter();
//            break;

//        case BSP_EVENT_DISCONNECT:
//            err_code = sd_ble_gap_disconnect(m_conn_handle,
//                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
//            if (err_code != NRF_ERROR_INVALID_STATE)
//            {
//                APP_ERROR_CHECK(err_code);
//            }
//            break;

//        case BSP_EVENT_WHITELIST_OFF:
//            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
//            {
//                err_code = ble_advertising_restart_without_whitelist();
//                if (err_code != NRF_ERROR_INVALID_STATE)
//                {
//                    APP_ERROR_CHECK(err_code);
//                }
//            }
//            break;

//        default:
//            break;
//    }
//}


/**@brief Function for the Peer Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Peer Manager.
 */
static void peer_manager_init(bool erase_bonds)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    if (erase_bonds)
    {
        err_code = pm_peers_delete();
        APP_ERROR_CHECK(err_code);
    }

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advdata_t          advdata;
    ble_adv_modes_config_t options;

    // Build and set advertising data
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = false;
    advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = m_adv_uuids;

    memset(&options, 0, sizeof(options));
    options.ble_adv_fast_enabled  = true;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, NULL, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
//static void buttons_leds_init(bool * p_erase_bonds)
//{
//    bsp_event_t startup_event;

//    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
//                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
//                                 bsp_event_handler);

//    APP_ERROR_CHECK(err_code);

//    err_code = bsp_btn_ble_init(NULL, &startup_event);
//    APP_ERROR_CHECK(err_code);

//    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
//}

/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' i.e '\r\n' (hex 0x0D) or if the string has reached a length of
 *          @ref NUS_MAX_DATA_LENGTH.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
//    static uint8_t data_array[256];
//    static uint8_t index = 0;
//    uint32_t       err_code;

    switch (p_event->evt_type)
    {
//        case APP_UART_DATA_READY:
//           UNUSED_VARIABLE(app_uart_get(&data_array[index]));
//					NRF_LOG_INFO("%c\r\n",data_array[index]);
//           index++;				
//           if ((data_array[index - 1] == '\n') || (index >= (255)))
//            {
////							NRF_LOG_INFO("over.....!!!!!");
//								NRF_LOG_INFO("%s\r\n",(uint32_t)data_array);
//								NRF_LOG_FLUSH();
//								memset(data_array,0, sizeof(data_array));
//////									//PutUARTBytes("%s",data_array);
////////                err_code = ble_nus_string_send(&m_nus, data_array, index);
////////                if (err_code != NRF_ERROR_INVALID_STATE)
////////                {
////////                    APP_ERROR_CHECK(err_code);
//////                }

//                index = 0;
//            }
//            break;

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
/**@snippet [Handling the data received over UART] */

/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
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
/**@snippet [UART Initialization] */
static void gpio_init(void)
{
	/*语音芯片*/
	nrf_gpio_cfg_output(VOICE_CHIP_PIN);
	nrf_gpio_cfg_output(VOICE_MOSE_PIN);
	nrf_gpio_pin_write(VOICE_CHIP_PIN, 1);
	nrf_gpio_pin_write(VOICE_MOSE_PIN, 1);
	
	//2G 初始化
	nrf_gpio_cfg_output(MODME_CONTRL_PIN);
	nrf_gpio_pin_write(MODME_CONTRL_PIN, 1);
	
}


/**@brief Thread for handling the Application's BLE Stack events.
 *
 * @details This thread is responsible for handling BLE Stack events sent from on_ble_evt().
 *
 * @param[in]   arg   Pointer used for passing some arbitrary information (context) from the
 *                    osThreadCreate() call to the thread.
 */
static void ble_stack_thread(void * arg)
{
    uint32_t err_code;
    bool     erase_bonds;

    UNUSED_PARAMETER(arg);

    // Initialize.
    timers_init();
		gpio_init();
		uart_init();
		uart_onoff(0);
	//init gpio te
		err_code = nrf_drv_gpiote_init();
		APP_ERROR_CHECK(err_code);
//	//定义GPIOTE输出初始化结构体，并对其成员进行赋值
//		nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(true);
//	//初始化GPIO引脚
//		err_code = nrf_drv_gpiote_out_init(VOICE_CHECK_PIN, &out_config);
//		APP_ERROR_CHECK(err_code);
		//高电平到低电平产生变化
//		nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
		//低电平到高电平
			nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
		//任意电平变化
			nrf_drv_gpiote_in_config_t in_config1 = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
		//引脚上拉
		in_config.pull = NRF_GPIO_PIN_PULLUP;
		err_code = nrf_drv_gpiote_in_init(VOICE_CHECK_PIN, &in_config, in_pin_handler);
		APP_ERROR_CHECK(err_code);
		
		in_config1.pull = NRF_GPIO_PIN_PULLUP;
		err_code = nrf_drv_gpiote_in_init(MAC_CHECK_PIN1, &in_config1, in_pin_handler_test);
		APP_ERROR_CHECK(err_code);
		err_code = nrf_drv_gpiote_in_init(MAC_CHECK_PIN2, &in_config1, in_pin_handler_test);
		APP_ERROR_CHECK(err_code);
		
		//使能GPIOTE 通道事件
		nrf_drv_gpiote_in_event_enable(MAC_CHECK_PIN1,true);
		nrf_drv_gpiote_in_event_enable(MAC_CHECK_PIN2,true);
		nrf_drv_gpiote_in_event_enable(VOICE_CHECK_PIN,true);
//    buttons_leds_init(&erase_bonds);
    ble_stack_init();
    peer_manager_init(erase_bonds);
    gap_params_init();
    advertising_init();
    services_init();
    conn_params_init();

    application_timers_start();
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);

    while (1)
    {
        /* Wait for event from SoftDevice */
        while (pdFALSE == xSemaphoreTake(m_ble_event_ready, portMAX_DELAY))
        {
            // Just wait again in the case when INCLUDE_vTaskSuspend is not enabled
        }

        // This function gets events from the SoftDevice and processes them by calling the function
        // registered by softdevice_ble_evt_handler_set during stack initialization.
        // In this code ble_evt_dispatch would be called for every event found.
        intern_softdevice_events_execute();
    }
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code = ble_advertising_start(BLE_ADV_MODE_FAST);

    APP_ERROR_CHECK(err_code);
}

#if NRF_LOG_ENABLED
/**@brief Thread for handling the logger.
 *
 * @details This thread is responsible for processing log entries if logs are deferred.
 *          Thread flushes all log entries and suspends. It is resumed by idle task hook.
 *
 * @param[in]   arg   Pointer used for passing some arbitrary information (context) from the
 *                    osThreadCreate() call to the thread.
 */
static void logger_thread(void * arg)
{
    UNUSED_PARAMETER(arg);

    while(1)
    {
        NRF_LOG_FLUSH();

        vTaskSuspend(NULL); // Suspend myself
    }
}
#endif //NRF_LOG_ENABLED

/**@brief A function which is hooked to idle task.
 * @note Idle hook must be enabled in FreeRTOS configuration (configUSE_IDLE_HOOK).
 */
void vApplicationIdleHook( void )
{
     vTaskResume(m_logger_thread);
}

/**@brief Function for application main entry.
 */
int main(void)
{
    ret_code_t err_code;
    err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    // Do not start any interrupt that uses system functions before system initialisation.
    // The best solution is to start the OS before any other initalisation.

    // Init a semaphore for the BLE thread.
    m_ble_event_ready = xSemaphoreCreateBinary();
    if (NULL == m_ble_event_ready)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }

    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    // Start execution.
    if (pdPASS != xTaskCreate(ble_stack_thread, "BLE", 256, NULL, 2, &m_ble_stack_thread))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }

#if NRF_LOG_ENABLED
    // Start execution.
    if (pdPASS != xTaskCreate(logger_thread, "LOGGER", 256, NULL, 1, &m_logger_thread))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
#endif //NRF_LOG_ENABLED

    /* Activate deep sleep mode */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    // Start FreeRTOS scheduler.
    vTaskStartScheduler();

    while (true)
    {
        APP_ERROR_HANDLER(NRF_ERROR_FORBIDDEN);
    }
}


