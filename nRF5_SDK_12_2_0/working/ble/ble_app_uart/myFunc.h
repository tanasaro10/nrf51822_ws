

/* structure which save the characteristics of a BLE device*/

#define HW_ADDR_SIZE_BYTES				6
#define MAX_BLE_HWID_LIST				100		/* max number of registered BLE devices as beacons/tag*/
#define MAX_BLE_NonAdv_HWID_LIST		10		/* max number of registered BLE devices as non advertising */
#define MAX_BLE_HWID_ACTIVE_LIST		50		/* max number of registered BLE devices that are in proximity*/


typedef struct {
	uint8_t idx;
	uint8_t apNr;
	uint8_t hwID[HW_ADDR_SIZE_BYTES];
	uint8_t phone[9];
	uint8_t flags;
} ble_hwId_t; // 18 bytes length

#define FLAG_IS_TAG 0
#define FLAG_IS_PHONE 1
#define FLAG_IS_ACTIVE 2



typedef enum {
	CMD_NO =0,
	CMD_INIT=1,
	CMD_WEB_TIME_REQ = 2,
	CMD_WEB_TIME_REQ2=3 ,
	CMD_SEND_SMS=4,
	CMD_CFG_EMAIL=5,
	CMD_SEND_EMAIL=6,
	CMD_RST=7,
	CMD_READ_LAST_SMS,
} cmd_type_t;

typedef enum {
	NO_EVT =0,
	EVT_RING=1,
	EVT_CLIP = 2,
	EVT_CMTI=3 ,
	EVT_CMGR=4,
	EVT_NO_CARRIER=5,
	EVT_OK =6,
	EVT_CSQ=7
} evt_type_t;
#define NO_OF_DEFINED_EVENTS  7

#define AT_CMD_SIZE 4u
static  uint8_t AT_CMD[AT_CMD_SIZE] = {'A','T','\r','\n'};
#define ATH_CMD_SIZE 5u
static  uint8_t ATH_CMD[ATH_CMD_SIZE] = {'A','T','H','\r','\n'};
#define AT_CSQ_CMD_SIZE 8u
static  uint8_t AT_CSQ_CMD[AT_CSQ_CMD_SIZE] = {'A','T','+','C','S','Q','\r','\n'};



// RECEIVED MSG FROM SIM800L (GSM)
#define RING_RCV_SIZE 4u
static  uint8_t RING_RCV[RING_RCV_SIZE] = {'R','I','N','G'};
#define CLIP_RCV_SIZE 5u
static  uint8_t CLIP_RCV[CLIP_RCV_SIZE] = {'+','C','L','I','P'};
#define CMTI_RCV_SIZE 5u
static  uint8_t CMTI_RCV[CMTI_RCV_SIZE] = {'+','C','M','T','I'};
#define CMGR_RCV_SIZE 5u
static  uint8_t CMGR_RCV[CMGR_RCV_SIZE] = {'+','C','M','G','R'};
#define NO_CARRIER_RCV_SIZE 6u
static  uint8_t NO_CARRIER_RCV[NO_CARRIER_RCV_SIZE] = {'N','O',' ','C','A','R'};
#define OK_RCV_SIZE 2u
static  uint8_t OK_RCV[OK_RCV_SIZE] = {'O','K'};
#define CSQ_RCV_SIZE 5u
static  uint8_t CSQ_RCV[CSQ_RCV_SIZE] = {'+','C','S','Q',':'};
//uint32_t* MSG_RCVD[]={&RING_RCV[0],&CLIP_RCV[0],&CMTI_RCV[0],&CMGR_RCV[0],&NO_CARRIER_RCV[0],&OK_RCV[0]};
//uint8_t MSG_RCVD_SIZE ={RING_RCV_SIZE,CLIP_RCV_SIZE,CMTI_RCV_SIZE,CMGR_RCV_SIZE,NO_CARRIER_RCV_SIZE,OK_RCV_SIZE};


void init_myFunc();

void addNewDevice(uint8_t* hw_addr);

/* check if the advertising device is registered */
bool checkAdvertisingDevice(uint8_t* hw_addr, int8_t rssi,ble_nus_t * m_nus);

/* check if the device (scanner device) is registered*/
bool checkPhoneDevice(uint8_t* hw_addr, ble_nus_t * m_nus);
						  

						  
