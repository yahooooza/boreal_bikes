#ifndef __BB_BLE_CLIENT_H
#define __BB_BLE_CLIENT_H

#include "BLEAdvertisedDevice.h"
#include "BLERemoteService.h"
#include "BLERemoteCharacteristic.h"

#if defined (__STDC__)
#if !defined(__PACKED_PRE) || !defined(__PACKED_POST)
#define __PACKED_PRE	
#define __PACKED_POST  __attribute__ ((packed))
#endif
#endif

/* Client Makros */
// Start Stop Scanning for devices
#define SCAN_START  BLEDevice::getScan()->start(5, false)
#define SCAN_STOP   BLEDevice::getScan()->stop()

#define LOCK_OPEN   1
#define LOCK_CLOSE  0

BLEAdvertisedDevice *pIlockitDevice;
BLEAdvertisedDevice *pBmsDevice;
BLEAdvertisedDevice *pHeartyPatchDevice;
BLEAdvertisedDevice *pControllerDevice;
BLEAdvertisedDevice *pEspServerDevice;

BLEClient *pIlockitClient = nullptr;
BLEClient *pBmsClient = nullptr;
BLEClient *pHeartyPatchClient = nullptr;
BLEClient *pControllerClient = nullptr;
BLEClient *pEspServerClient = nullptr;

BLERemoteService *pIlockitRemoteService;
BLERemoteService *pBmsRemoteService;
BLERemoteService *pHeartyPatchRemoteService;
BLERemoteService *pControllerRemoteService;
BLERemoteService *pEspServerRemoteService;

BLERemoteCharacteristic *pIlockitRemoteCharacteristic;
BLERemoteCharacteristic *pBmsRemoteCharacteristic;
BLERemoteCharacteristic *pControllerRemoteCharacteristic;
BLERemoteCharacteristic *pHeartyPatchRemoteCharacteristic;
BLERemoteCharacteristic *pEspServerRemoteCharacteristic;

BLEScan* pScan;

#warning "Make sure all MAC address correct in BB_BLEClient.h"
#define BMS_MAC						BLEAddress("a4:c1:38:d4:41:94")
#define BMS_NAME					std::string("xiaoxiang BMS")
#define HEARTYPATCH_MAC				BLEAddress("30:ae:a4:69:eb:b6")
#define CONTROLLER_MAC				BLEAddress("94:e3:6d:73:c7:37")
#define ESP_SERVER_MAC				BLEAddress("80:7d:3a:c3:26:c6")
//#define ESP_SERVER_MAC			BLEAddress("24:0a:c4:0d:16:f6")
//#define ESP_SERVER_MAC			BLEAddress("d8:a0:1d:40:a5:5e")


/* Flags */
/* flag for scanner */
bool isIlockitFound, isBmsFound, isHeartyFound, isControllerFound, isEspServerFound = false;
bool isScanStop = false;

/* flag for client/device connected */
bool isIlockitConnected, isBmsConnected, isHeartyConnected, isControllerConnected, isEspServerConnected = false;

/* flag for client notification availability */
bool isIlockitNotifyAvailable, isHpNotifyAvailable, isBmsNotifyAvailable, isControllerNotifyAvailable, isLockControlNotifyAvailable, isConfigTimeNotifyAvailable = false;
bool isBmsWriteInfoSuccess, isHeartRateAvailable = false;

const uint8_t notificationOff[] = { 0x00, 0x00 };
const uint8_t notificationOn[] = { 0x01, 0x00 };

typedef __PACKED_PRE struct BLE_HEARTRATE_CLIENT_STRUCT_Ttag {
		uint8_t    bFlag;
		uint8_t    bHeartRate;
		uint16_t   usEnergy;
		uint16_t   usRRInterval;
} __PACKED_POST BLE_HEARTRATE_CLIENT_STRUCT_T;

typedef union BLE_HEARTRATE_CLIENT_PACKET_Ttag
{
    BLE_HEARTRATE_CLIENT_STRUCT_T tPacket;
    uint8_t abPacket[6];
}BLE_HEARTRATE_CLIENT_PACKET_T;

#endif
