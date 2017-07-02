#include <myFunc.h>

uint8_t j,k;
static ble_hwId_t gHWIdArray[MAX_BLE_HWID_LIST], gHWIDPhArray[MAX_BLE_NonAdv_HWID_LIST];
//int* MSG_RCVD[]={&RING_RCV[0],&CLIP_RCV[0],&CMTI_RCV[0],&CMGR_RCV[0],&NO_CARRIER_RCV[0],&OK_RCV[0]};
const uint8_t* MSG_RCVD[]	={0,	RING_RCV,			CLIP_RCV,		CMTI_RCV,		CMGR_RCV,		NO_CARRIER_RCV,			OK_RCV,			CSQ_RCV};
uint8_t MSG_RCVD_SIZE[] 	={0,	RING_RCV_SIZE,	CLIP_RCV_SIZE,	CMTI_RCV_SIZE,	CMGR_RCV_SIZE,	NO_CARRIER_RCV_SIZE,	OK_RCV_SIZE,	CSQ_RCV_SIZE};

char mybuf[BLE_NUS_MAX_DATA_LEN];
uint8_t gNrOfDevices; /* total number of registered BLEdevices*/
uint8_t gNrOfNonAdvDevices; /* total number of registered BLEdevices*/
//
//gHWIdArray[1]=0x00; //= {{0x00,0x00,0x00,0x00,0x00,0x00}};
/*public functions*/


void init_myFunc(){
	
	gNrOfDevices=0;
	gNrOfNonAdvDevices=0;
	
	gHWIdArray[gNrOfDevices]=(ble_hwId_t){0,76,{0x7F,0x9F,0x14,0x51,0x68,0xCB},{'7','2','4','0','7','2','3','2','0'},FLAG_IS_TAG}; // VT Dev1 Tag
	gNrOfDevices++;
	gHWIdArray[gNrOfDevices]=(ble_hwId_t){1,76,{0x64,0xD6,0x48,0xA7,0xF4,0xF9},{'7','2','4','0','7','2','3','2','0'},FLAG_IS_TAG}; // VT Dev2 Tag
	gNrOfDevices++;
	gHWIdArray[gNrOfDevices]=(ble_hwId_t){2,76,{0x7B,0x5B,0x01,0xD0,0xFF,0xFF},{'7','2','4','0','7','2','3','2','0'},FLAG_IS_TAG}; // VT iTag Alb
	gNrOfDevices++;
	gHWIdArray[gNrOfDevices]=(ble_hwId_t){3,88,{0x73,0xAC,0x01,0x90,0xFF,0xFF},{'7','6','7','4','4','7','2','2','0'},FLAG_IS_TAG}; // CS iTag Alb	
	gNrOfDevices++;
	
	
	gHWIDPhArray[gNrOfNonAdvDevices]=(ble_hwId_t){0,76,{0x2E,0xCF,0x26,0xD7,0x27,0x73},{'7','2','4','0','7','2','3','2','0'},FLAG_IS_PHONE}; // VT Phone
	gNrOfNonAdvDevices++;
}

void addNewDevice(uint8_t* hw_addr){
	memcpy(&(gHWIdArray[1].hwID[0]),hw_addr,6);
	gNrOfDevices++;	
}

bool checkAdvertisingDevice(uint8_t* hw_addr, int8_t rssi, ble_nus_t * m_nus){
	bool retVal=false;
	uint8_t i=0;
	while ((i<gNrOfDevices)&&(memcmp(hw_addr,gHWIdArray[i].hwID,HW_ADDR_SIZE_BYTES)!=0)){
		i++;
	}
	
	if (i<gNrOfDevices){		
		if ((int8_t)hw_addr[6]>(int8_t)rssi){
			sprintf(mybuf,"Tg %x%x%x%x%x%x %d\r\n",hw_addr[0],hw_addr[1],hw_addr[2],hw_addr[3],hw_addr[4],hw_addr[5],(int8_t)hw_addr[6]);
			ble_nus_string_send(m_nus, ((uint8_t*)mybuf), 20); 
			retVal=true;
		}
	}
	return retVal;
}

bool checkPhoneDevice(uint8_t* hw_addr, ble_nus_t * m_nus){
	bool retVal=false;
	uint8_t i=0;
	while ((i<gNrOfNonAdvDevices)&&(memcmp(hw_addr,gHWIDPhArray[i].hwID,HW_ADDR_SIZE_BYTES)!=0)){
		i++;
	}	
	if (i<gNrOfNonAdvDevices){		
		//if (gHWIDPhArray[i].isValid==true){
			sprintf(mybuf,"Ph %x%x%x%x%x%x\r\n",hw_addr[0],hw_addr[1],hw_addr[2],hw_addr[3],hw_addr[4],hw_addr[5]);
			ble_nus_string_send(m_nus, ((uint8_t*)mybuf), 17); 		// another way may be to add to uart_data_array	
			retVal=true;
		//}
	}
	return retVal;
}

evt_type_t getEvtFromUART(uint8_t * pData,uint8_t length){
uint8_t i=1;
evt_type_t retVal=NO_EVT;

	while((i<=NO_OF_DEFINED_EVENTS)&&(memcmp(pData,MSG_RCVD[i],MSG_RCVD_SIZE[i])!=0)){
		i++;
	}
	if (i<=NO_OF_DEFINED_EVENTS){
		retVal = (evt_type_t) i;
	}
	//printf("I:%d",i);
	return retVal;
}

bool checkPhoneValidity(uint8_t* phoneNr){
	bool retVal = true; // TODO change to false when ready !
	uint8_t i=0;
	/*while ((i<gNrOfNonAdvDevices)&&(memcmp(phoneNr,gHWIDPhArray[i].phone,9)!=0)){
		i++;
	}*/
	
	if (i<gNrOfNonAdvDevices){				
			retVal=true;
	}
	return retVal;
}