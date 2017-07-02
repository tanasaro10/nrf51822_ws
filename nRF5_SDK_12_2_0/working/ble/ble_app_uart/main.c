/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "crc32.h"
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "nrf_nvic.h"
#include "ble_conn_state.h"
#include "myFunc.c"
#include "fstorage.h"
#include "fds.h"
//#include "nrf_dfu_types.h"
//#include "app_scheduler.h"


#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                           /**< Include the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */

#if (NRF_SD_BLE_API_VERSION == 3)
#define NRF_BLE_MAX_MTU_SIZE            GATT_MTU_SIZE_DEFAULT                       /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */
#endif

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */

#define CENTRAL_LINK_COUNT              1                                           /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                           /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/
																					/**< have to be changed in ble_app_uart_gcc_nrf51.ld */

#define DEVICE_NAME                     "INTERFON"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      0                                         /**< The advertising timeout (in units of seconds). */

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(7.5, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(30, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(10000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define SCAN_INTERVAL               0x0200                                        /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW                 0x0100                                        /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_TIMEOUT                0

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                512                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                512                                         /**< UART RX buffer size. */

#define TX_POWER_LEVEL                    (4)                                         /**< TX Power Level value.  -40, -30, -20, -16, -12, -8, -4, 0, and 4 dBm */
#define TIMER_INTERVAL     			APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER)	/**< Tick each 1 second ?*/

APP_TIMER_DEF(m_app_timer_id);                                                  /**< main application timer */
APP_TIMER_DEF(m_closing_door_timer_id);                                         /**< closing door timer */
APP_TIMER_DEF(m_unlock_door_open_timer_id);                                     /**< unlock the opening of the door timer */

//static uint8_t data_array[BLE_NUS_MAX_DATA_LEN]={0x41,0x42,0x43,0x43,0x44,0x45,0x46,0x47,0x48,0x49,0x4A,0x4B,0x4C,0x4D,0x4E,0x4F,0x50,0x51,0x0D,0x0A};
#define MAX_LIST_ARRAY					20											// maximum number of data array to be saved;

#define CTRL_Z 							26

static uint8_t in_cnt=0,out_cnt=0;
static uint32_t RELAY_PIN=15, RST_PIN=29;
static uint8_t uart_data_array[MAX_LIST_ARRAY][BLE_NUS_MAX_DATA_LEN];

static uint8_t gdataUart[100];
static ble_nus_t                        m_nus;                                      /**< Structure to identify the Nordic UART Service. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */

static ble_uuid_t m_adv_uuids[] = {{BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE},
									{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}}; /**< Universally unique service, Battery service, Nordic Uart service identifiers. */

static bool b_flag_send = false, gbDoorOpenStatus = false, bResetNrOfRings= false ; // bFirst_time signals when the system has started


/**
 * @brief Parameters used when scanning.*/
 
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


static evt_type_t gEVT_RCV = NO_EVT;
static cmd_type_t gCMD_TYPE = CMD_NO;
static uint8_t gCMD_TYPE_step=0;//, unWaitForResponse=0;
//static bool bProcessCMD= false;//, bNoResponseCheck = false;


int8_t gRSSI_desired=-100;
static uint8_t gDoorTimeoutSEC=4;	// multiple of seconds
static uint8_t gDoorUnlockOpeningSEC= 3; // multiple seconds -> delay between 2 consecutives opening
static uint32_t timestamp=0;		// time stamp in seconds from the start of application

static void write2uart(uint8_t* p_data, uint16_t length);

static void unlock_door_opening();

static void start_unlocking_opening();

static void start_closing();

static void close_door();

static void gsm_init();

void adv_scan_start();

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
	 
}


/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;
	// security mode is set to OPEN, not restricted ( more in ble_gap.h)
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);
	err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_GENERIC_REMOTE_CONTROL);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

/* there is a delay between 2 consecutive openings*/
static void unlock_door_opening(){
	gbDoorOpenStatus=false;
	bResetNrOfRings = true;
}

/* closing the door - this is called by the expiration of closing_door_timer*/
static void close_door(){
	nrf_gpio_pin_clear(RELAY_PIN);
	write2uart(ATH_CMD,ATH_CMD_SIZE);// send ATH cmd to GSM
	start_unlocking_opening();
}

static void timer_timeout_handler(){
	timestamp ++;
	
	if (timestamp==2){
		gsm_init();
	}
	/*each 5 seconds send a AT+CSQ command*/
	if (timestamp%5==0){	
		write2uart(AT_CSQ_CMD,AT_CSQ_CMD_SIZE);
	}
		
}

void open_door(){
	//printf("door open %d\r\n",gbDoorOpenStatus);
	if (gbDoorOpenStatus==false){
		nrf_gpio_pin_set(RELAY_PIN);
		start_closing();	// timer to close the door				
		gbDoorOpenStatus=true;
		sprintf(mybuf,"OpenDoor\r\n");
		ble_nus_string_send(&m_nus, ((uint8_t*)mybuf), 10); 
	}
}

static void start_closing(){
	uint32_t 	err_code;
	 // Start timer to close the door.
    err_code = app_timer_start(m_closing_door_timer_id, (gDoorTimeoutSEC*TIMER_INTERVAL), NULL);
    APP_ERROR_CHECK(err_code);
}

static void start_unlocking_opening(){
	uint32_t 	err_code;
	 // Start timer to close the door.
    err_code = app_timer_start(m_unlock_door_open_timer_id, (gDoorUnlockOpeningSEC*TIMER_INTERVAL), NULL);
    APP_ERROR_CHECK(err_code);
}

static void start_timer(){
	uint32_t 	err_code;
	 // Start application timer.
    err_code = app_timer_start(m_app_timer_id, TIMER_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}


static void decode(uint8_t* p_data, uint16_t length){
	//uint32_t err_code;
	switch (p_data[0]){
		case 'c':
			/*command as defined in cmd_type_t received*/
			gCMD_TYPE = (cmd_type_t)(p_data[1]-'0');
			gCMD_TYPE_step =0;
			
			//bProcessCMD = true;
			break;
		case 'i':
			switch (p_data[1]){
				case '0':
					// door timeout in seconds
					gDoorTimeoutSEC = (p_data[2]-'0')+1;
					break;
				case '1':
					// door unlock opening cfg
					gDoorUnlockOpeningSEC = (p_data[2]-'0')+1;
					break;
				case '3':
					gRSSI_desired = (-1)*(int8_t)((p_data[2]-'0')*10+(p_data[3]-'0'));
					break;
				case '4':
					NRF_POWER->GPREGRET = 0xB1;
					sd_nvic_SystemReset();
				default:					
					break;
			}
			break;
		case 'r':
			
			break;
		default:
					
			break;
	}
}

static void write2uart(uint8_t * p_data, uint16_t length)
{
    for (uint32_t i = 0; i < length; i++)
    {
        while (app_uart_put(p_data[i]) != NRF_SUCCESS);
    }
    while (app_uart_put('\r') != NRF_SUCCESS);
    while (app_uart_put('\n') != NRF_SUCCESS);
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
	write2uart(p_data, length);
	if (p_data[0]=='x' ) {
		decode(&(p_data[1]),(length-1));
	}
}

/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_nus_init_t nus_init;
	
    
	memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
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


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
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
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
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
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            //sleep_mode_enter();
            break;
        default:
            break;
    }
}



/**@brief Function for the application's SoftDevice event handler.
 *
 * @param[in] p_ble_evt SoftDevice event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt, uint16_t role)
{
    uint32_t err_code = NRF_SUCCESS;
	//static uint16_t ptimestamp=0;
	static uint8_t mHwId[6];
	const ble_gap_evt_t * p_gap_evt = &p_ble_evt->evt.gap_evt;   

    switch (p_ble_evt->header.evt_id)
    {
		case BLE_GAP_EVT_ADV_REPORT:
        {
            const ble_gap_evt_adv_report_t * p_adv_report = &p_gap_evt->params.adv_report;
			// BLE_GAP_ROLE_CENTRAL         
					
            // scan is automatically stopped by the connect
			memcpy(&(mHwId[0]),&(p_adv_report->peer_addr.addr[0]), 7);
				//(gbDoorOpenStatus==false)&&   
			if ((checkAdvertisingDevice(mHwId, gRSSI_desired, &m_nus))){
				//printf("Tag %d\r\n",(uint16_t)timestamp-(uint16_t)ptimestamp);
				//ptimestamp=timestamp;
			   open_door();					   
			}
			break; 
		} // BLE_GAP_EVT_ADV_REPORT
        case BLE_GAP_EVT_CONNECTED:
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
			memcpy(mHwId,&(p_ble_evt->evt.gap_evt.params.connected.peer_addr.addr[0]),6);
			//if ((gbDoorOpenStatus==false)&&(checkPhoneDeviceAndPrepareOpenDoor(mHwId, &m_nus))){
					open_door();
			//}			
			b_flag_send = true;
            break; // BLE_GAP_EVT_CONNECTED
		case BLE_EVT_TX_COMPLETE:
			//printf("Tick! \r");
			b_flag_send = true;
			break;
        case BLE_GAP_EVT_DISCONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
			//printf("\r Disconnected!\r\n");
            APP_ERROR_CHECK(err_code);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
			
			b_flag_send = false;			
            break; // BLE_GAP_EVT_DISCONNECTED

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
			//printf("\r Pairing Failed!\r\n");
			
            APP_ERROR_CHECK(err_code);
			b_flag_send = false;
            break; // BLE_GAP_EVT_SEC_PARAMS_REQUEST

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
			printf("\r No system attributes have been stored !\r\n");
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_SYS_ATTR_MISSING

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
			
				if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_SCAN)
				{
					//NRF_LOG_DEBUG("Scan timed out.\r\n");
					err_code = sd_ble_gap_scan_start(&m_scan_params);
					APP_ERROR_CHECK(err_code);
				}
				else if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
				{
					printf("Connection Request timed out.\r\n");
				}
				err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
					//printf("\r GATT Client Timeout %lu!\r\n", err_code);
					b_flag_send = false;
					APP_ERROR_CHECK(err_code);
            break; // BLE_GATTC_EVT_TIMEOUT

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
			//printf("\r GATT Server Timeout %lu!\r\n",err_code);								 
			b_flag_send = false;
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_TIMEOUT

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
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


/**@brief Function for dispatching a SoftDevice event to all modules with a SoftDevice
 *        event handler.
 *
 * @details This function is called from the SoftDevice event interrupt handler after a
 *          SoftDevice event has been received.
 *
 * @param[in] p_ble_evt  SoftDevice event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
	uint16_t conn_handle;
    uint16_t role;

    //ble_conn_state_on_ble_evt(p_ble_evt);
    //pm_on_ble_evt(p_ble_evt);

    // The connection handle should really be retrievable for any event type.
    conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
    role        = ble_conn_state_role(conn_handle);
	
    ble_conn_params_on_ble_evt(p_ble_evt);   
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
    
	on_ble_evt(p_ble_evt, role);
    ble_advertising_on_ble_evt(p_ble_evt);
    //bsp_btn_ble_on_ble_evt(p_ble_evt);

}

/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in] sys_evt  System stack event.
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

/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    // Initialize SoftDevice.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);
	//printf("RAM %lx",app_ram_base);
    //Check the ram settings against the used number of links
    //CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);
	//ble_enable_params.common_enable_params.vs_uuid_count   = 2;
    // Enable BLE stack.
#if (NRF_SD_BLE_API_VERSION == 3)
    ble_enable_params.gatt_enable_params.att_mtu = NRF_BLE_MAX_MTU_SIZE;
#endif
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
	
	// Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
}

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
    
    static uint8_t index = 0, lindex=0;
	uint8_t uChar;
	static uint8_t uartData[100];
   // uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:            
			UNUSED_VARIABLE(app_uart_get(&uChar));
			uart_data_array[in_cnt][index] = uChar; 
			uartData[lindex]= uChar;
			index++;
			lindex++;
			
			if ((uartData[lindex - 1] == '\n')||(lindex >= 99)){			
				//printf("LI:%s\r\n",(char*)(&uartData[0]));
				
				gEVT_RCV = getEvtFromUART((uartData),lindex);
				//printf("%d\r\n",gEVT_RCV);
				memcpy(&gdataUart[0],&(uartData[0]),lindex);	// it is the case if there are multiple events ? I don't think ...
				
				memset(&uartData[0], 0, sizeof(uint8_t)*lindex);
				lindex=0;				
			}
			
            if ((uart_data_array[in_cnt][index - 1] == '\n') || (index >= (BLE_NUS_MAX_DATA_LEN))) 
            {
                in_cnt++;				
				if (in_cnt==MAX_LIST_ARRAY){				
					in_cnt=0;					
				}
                index = 0;
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            //APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            //APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}


static void process_evt(evt_type_t evType){
	//static uint8_t nrOfRings=0;
	//uint8_t phoneNumber[8];
	//static uint8_t bPhoneIsValid = false;
	//printf("something\r\n");
	switch (evType){
	case EVT_RING: {
		open_door();
		/*
			if (bResetNrOfRings==true){
				nrOfRings=0;
				bResetNrOfRings=false;
			}
			if ((nrOfRings>=1)&&(bPhoneIsValid==true)) {
				open_door();
				bPhoneIsValid = false;
			} 			
			nrOfRings ++;*/
		break;
		}
	case EVT_CLIP: {
		//uint8_t mindex=8;
		/*
		if (gbDoorOpenStatus==false){
			while (gdataUart[mindex]!='\"'){
				phoneNumber[mindex-8]=gdataUart[mindex];
				mindex++;
			}
			bPhoneIsValid = checkPhoneValidity(phoneNumber);
		}*/
		break;
		}
	case EVT_CMGR:
		break;
	case EVT_CMTI: 
		break;
	case EVT_NO_CARRIER: 
		break;
	case EVT_OK:	
		break;
	default: 
		break;
	}
}




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


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advdata_t          advdata;
    ble_advdata_t          scanrsp;
    ble_adv_modes_config_t options;
	int8_t tx_power = TX_POWER_LEVEL;
	
	ble_advdata_manuf_data_t        manuf_data; // Variable to hold manufacturer specific data
    uint8_t data[]                      = "SomeData!"; // Our data to adverise
    manuf_data.company_identifier       = 0x0059; // Nordics company ID
    manuf_data.data.p_data              = data;     
    manuf_data.data.size                = sizeof(data);

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = false;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
	advdata.p_manuf_specific_data   = &manuf_data;
	advdata.p_tx_power_level = &tx_power;


    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = m_adv_uuids;

    memset(&options, 0, sizeof(options));
    options.ble_adv_fast_enabled  = true;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, &scanrsp, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
	
	err_code = sd_ble_gap_tx_power_set(tx_power);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for placing the application in low power state while waiting for events.
 */
 
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

/* GSM Module init*/
static void gsm_init(void){
	// RST_PIN set
	nrf_gpio_pin_set(RST_PIN);
	// GSM init commands
	write2uart(AT_CMD,AT_CMD_SIZE);// send AT cmd to GSM	
}

static void timers_init(void)
{

    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
	
	nrf_gpio_cfg_output(RELAY_PIN);
	nrf_gpio_cfg_output(RST_PIN);
	nrf_gpio_pin_clear(RELAY_PIN);
	nrf_gpio_pin_set(RST_PIN);
	
    // Create timers.
    uint32_t err_code;
    err_code = app_timer_create(&m_app_timer_id, APP_TIMER_MODE_REPEATED, timer_timeout_handler);
    APP_ERROR_CHECK(err_code); 
	err_code = app_timer_create(&m_closing_door_timer_id, APP_TIMER_MODE_SINGLE_SHOT, close_door);
    APP_ERROR_CHECK(err_code); 
	err_code = app_timer_create(&m_unlock_door_open_timer_id, APP_TIMER_MODE_SINGLE_SHOT, unlock_door_opening);
    APP_ERROR_CHECK(err_code); 
	//err_code = app_timer_create(&m_app_timer_id, APP_TIMER_MODE_REPEATED, timer_timeout_handler);
    //APP_ERROR_CHECK(err_code); 
}
	
void adv_scan_start(){
	
	ret_code_t err_code;

    (void) sd_ble_gap_scan_stop();

    err_code = sd_ble_gap_scan_start(&m_scan_params);
    // It is okay to ignore this error since we are stopping the scan anyway.
    if (err_code != NRF_ERROR_INVALID_STATE)
    {
        APP_ERROR_CHECK(err_code);
    }
	
	err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
}



/**@brief Application main function.
 */
int main(void)
{
    uint32_t err_code;
    //bool erase_bonds;

    // Initialize.
	timers_init();
    uart_init();
	init_myFunc();

    //buttons_leds_init(&erase_bonds);
	ble_stack_init();
    gap_params_init();

    services_init();
    advertising_init();
    conn_params_init();
	
	start_timer();
	
	adv_scan_start(); 
    printf("START APP\r\n");

    // Enter main loop.
    for (;;)
    {			
			if ((in_cnt!=out_cnt)&&(b_flag_send==true)){ // or even while ?
				err_code = ble_nus_string_send(&m_nus, &(uart_data_array[out_cnt][0]), BLE_NUS_MAX_DATA_LEN);  					
				
				if(err_code == BLE_ERROR_NO_TX_PACKETS)
				{
					b_flag_send = false;
				} else {								
					memset(&(uart_data_array[out_cnt][0]), 0, sizeof(uint8_t)*BLE_NUS_MAX_DATA_LEN);
					out_cnt+=1;		
					if (out_cnt==MAX_LIST_ARRAY) {
						out_cnt=0;
					}
				}
			}
			//printf("TICK\r\n");
			if (gEVT_RCV != NO_EVT){
			
				process_evt(gEVT_RCV);
				gEVT_RCV = NO_EVT;				
			}
     power_manage();
    }
}


/**
 * @}
 */
