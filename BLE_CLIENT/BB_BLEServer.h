#ifndef __BB_BLESERVER_H
#define __BB_BLESERVER_H 

#include "BLEDevice.h"
#include "BLECharacteristic.h"
#include "BBUUID.h"

#if defined (__STDC__)
#if !defined(__PACKED_PRE) || !defined(__PACKED_POST)
#define __PACKED_PRE	
#define __PACKED_POST  __attribute__ ((packed))
#endif
#endif

// Macros
#define PROP_READ		BLECharacteristic::PROPERTY_READ
#define PROP_WRITE		BLECharacteristic::PROPERTY_WRITE
#define PROP_NOTIFY		BLECharacteristic::PROPERTY_NOTIFY
#define PROP_INDICATE	BLECharacteristic::PROPERTY_INDICATE

#define ADVERTISING_START BLEDevice::startAdvertising()
#define ADVERTISING_STOP BLEDevice::getAdvertising()->stop()


// GATT definitions
// -- Services and Characteristics
#define BMS_MOTOR_SRV_SERVICE			BLEUUID("42425a02-0000-1000-8000-005a45535953")
#define BMS_MOTOR_SRV_CHAR				BLEUUID("42427a02-0000-1000-8000-005a45535953")

#define ILOCKIT_SRV_SERVICE				BLEUUID("42425a03-0000-1000-8000-005a45535953")
#define ILOCKIT_SRV_CHAR				BLEUUID("42427a03-0000-1000-8000-005a45535953")
#define LOCK_CONTROL_SRV_CHAR			BLEUUID("42427a04-0000-1000-8000-005a45535953")

#define TIME_INFO_SRV_SERVICE			BLEUUID("42425a08-0000-1000-8000-005a45535953")
#define TIME_LORA_INFO_SRV_CHAR			BLEUUID("42427a08-0000-1000-8000-005a45535953")
#define TIME_SET_SRV_CHAR				BLEUUID("42427a07-0000-1000-8000-005a45535953")
#define TIME_ESPTIME_SRV_CHAR			BLEUUID("42427a06-0000-1000-8000-005a45535953")

#define HEART_RATE_SRV_SERVICE			BLEUUID("42425a01-0000-1000-8000-005a45535953")  
#define HEART_RATE_SRV_CHAR				BLEUUID("42427a01-0000-1000-8000-005a45535953")

#define LOCATION_SRV_SERVICE			BLEUUID("42425a10-0000-1000-8000-005a45535953")
#define LOCATION_SRV_CHAR				BLEUUID("42427a10-0000-1000-8000-005a45535953")

#define MPU_SRV_SERVICE					BLEUUID("42425a11-0000-1000-8000-005a45535953")
#define MPU_SRV_CHAR					BLEUUID("42427a11-0000-1000-8000-005a45535953")

BLEServer *pServer;
BLEService *pBmsService;
BLEService *pIlockitService;
BLEService *pTimeInfoService;
BLEService *pHeartRateService;
BLEService *pLocationService;
BLEService *pMpuService;

BLECharacteristic* pBmsMotorChar;
BLECharacteristic* pIlockitChar; 
BLECharacteristic* pLockControlChar;
BLECharacteristic* pLoraInfoChar;
BLECharacteristic* pTimeSetChar;
BLECharacteristic* pEspTimeChar;
BLECharacteristic* pHeartRateChar;
BLECharacteristic* pLocationChar;
BLECharacteristic* pMpuChar;

BLEDescriptor BmsMotorDescriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor IlockitDescriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor LockControlDescriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor LoraInfoDescriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor TimeSetDescriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor EspTimeDescriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor HeartRateDescriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor LocationDescriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor MpuDescriptor(BLEUUID((uint16_t)0x2901));

BLEAdvertising *pAdvertising;

typedef union PACKET_Ttag {
	uint32_t	ulValue;
	int32_t		lValue;
	uint16_t	usValue;
	int16_t		sValue;
	uint8_t		bValue;
	int8_t		sbValue;
	uint8_t		abValue[4];
} PACKET_T;

typedef __PACKED_PRE struct BLE_BMS_MOTOR_STRUCT_Ttag {
	uint8_t     bBatPercentage;
	uint16_t    usBmsTotalVoltage;
	uint32_t    ulBmsTime;
	uint8_t     bSpeed;
	uint16_t    usMotorTotalVoltage;
	uint16_t    usDistance;
} __PACKED_POST BLE_BMS_MOTOR_STRUCT_T;

typedef union BLE_BMS_MOTOR_PACKET_Ttag
{
	BLE_BMS_MOTOR_STRUCT_T tPacket;
	uint8_t abPacket[12];
}BLE_BMS_MOTOR_PACKET_T;

typedef __PACKED_PRE struct BLE_LOCK_STRUCT_Ttag {
	uint8_t     bLockState;
	uint32_t    ulLockChangeTime;
	uint16_t    usLockSessionTime;
} __PACKED_POST BLE_LOCK_STRUCT_T;

typedef union BLE_LOCK_PACKET_Ttag {
	BLE_LOCK_STRUCT_T tPacket;
	uint8_t abPacket[7];
}BLE_LOCK_PACKET_T;

typedef __PACKED_PRE struct BLE_TIME_PACKET_Ttag {
	PACKET_T		tLoraLastSendPackageTime;
	PACKET_T		tConfigCurrentTime;
	PACKET_T		tEspCurrentTime;
} __PACKED_POST BLE_TIME_PACKET_T;

typedef __PACKED_PRE struct BLE_HEARTRATE_STRUCT_Ttag {
    uint8_t    bHeartRate;
    uint32_t   ulHeartyTime;
} __PACKED_POST BLE_HEARTRATE_STRUCT_T;

typedef union BLE_HEARTRATE_PACKET_Ttag
{
    BLE_HEARTRATE_STRUCT_T tPacket;
    uint8_t abPacket[5];
}BLE_HEARTRATE_PACKET_T;

typedef __PACKED_PRE struct BLE_LOCATION_STRUCT_Ttag {
	union {
		float fLatitude;
		uint32_t ulLatitude;
	};
	union {
		float fLongitude;
		uint32_t ulLongitude;
	};
	union {
		float fElevation;
		uint32_t ulElevation;
	};
    uint32_t    ulLocationTime;
} __PACKED_POST BLE_LOCATION_STRUCT_T;

typedef union BLE_LOCATION_PACKET_Ttag
{
    BLE_LOCATION_STRUCT_T tPacket;
    uint8_t abPacket[12];
}BLE_LOCATION_PACKET_T;

typedef __PACKED_PRE struct BLE_MPU_STRUCT_Ttag {
        uint8_t     bCrashDetect;
		uint32_t	ulCrashTime;
		int8_t		sbDetectedForced;
} __PACKED_POST BLE_MPU_STRUCT_T;

typedef union BLE_MPU_PACKET_Ttag
{
    BLE_MPU_STRUCT_T tPacket;
    uint8_t abPacket[5];
}BLE_MPU_PACKET_T;

typedef __PACKED_PRE struct BLE_ONWRITE_STRUCT_Ttag {
	uint8_t	bLockControlValue;
	union {
		uint32_t ulTimeSet;
		uint8_t abTimeSet[4];
	};
} __PACKED_POST BLE_ONWRITE_PACKET_T;




bool isTimeSetOnWrite = false;
bool isLockControlOnWrite = false;

#endif
