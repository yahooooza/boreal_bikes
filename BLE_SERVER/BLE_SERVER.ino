/************************************************************************************************************************/
/*!
* @author		zzulkifli, edoelling, lbierstedt (ZeSys e.V)
* @file			BLE_CLIENT.ino
* @date			2019-03.01
* @version		1.0
* @brief		Program for ESP BLE Server. The server provide the service and characteristics and acts as a gateway
* between end users and BLE client.
*
* Used library:
*	Library									| Release				| Notes
*	----------------------------------------|-----------------------|------------------
*	ESP32 BLE								| -						| author: nkolban, chegewara (https://github.com/nkolban/esp32-snippets/tree/master/cpp_utils)
*	BMS packet handler						| 1.0.0					|
*	Controller packet handler				| 1.0.0					|
*
* Changes:
*	Date       | Description
*	-----------|---------------------------------------------------------------------------------------------------------
*	2019-03-01 | initial version
*
* @note
*	-	the current ESP32 BLE library has been modified at certain point to get more data about the connected device
*	-	please use the BLE library provided in git server.
*	-	current libbt.a stack in Arduino IDE is not stable, so please use the replace the libbt.a stack provided in git.
*
* @warning
*
*/
/************************************************************************************************************************/

#include <Arduino.h>
#include <BLEClient.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLERemoteService.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <time.h>
#include <sys/time.h>
#include <BMSPacketHandler.h>
#include <ControllerPacketHandler.h>
#include "BBUUID.h"
#include "BB_BLEServer.h"
#include "BB_BLEClient.h"
#include "LoraPacketHandler.h"
#include "endian.h"
#include "esp_system.h"

#if defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#define LOG_TAG ""
#else
#include "esp_log.h"
static const char* LOG_TAG = "BLEDevice";
#endif

// flags
boolean deviceConnected = false;
boolean loopWdt = false;

// Global Values

/************************************************************************************************************************/
/*!
						   888888888888 88 88b           d88 88 888b      88   ,ad8888ba,
								88      88 888b         d888 88 8888b     88  d8"'    `"8b
								88      88 88`8b       d8'88 88 88 `8b    88 d8'
								88      88 88 `8b     d8' 88 88 88  `8b   88 88
								88      88 88  `8b   d8'  88 88 88   `8b  88 88      88888
								88      88 88   `8b d8'   88 88 88    `8b 88 Y8,        88
								88      88 88    `888'    88 88 88     `8888  Y8a.    .a88
								88      88 88     `8'     88 88 88      `888   `"Y88888P"
*/
/************************************************************************************************************************/
static unsigned long start_time = 0;

/************************************************************************************************************************/
/*!
									 88888888ba,        db   888888888888   db
									 88      `"8b      d88b       88       d88b
									 88        `8b    d8'`8b      88      d8'`8b
									 88         88   d8'  `8b     88     d8'  `8b
									 88         88  d8YaaaaY8b    88    d8YaaaaY8b
									 88         8P d8""""""""8b   88   d8""""""""8b
									 88      .a8P d8'        `8b  88  d8'        `8b
									 88888888Y"' d8'          `8b 88 d8'          `8b



						88888888ba     db        ,ad8888ba,  88      a8P  88888888888 888888888888
						88      "8b   d88b      d8"'    `"8b 88    ,88'   88               88
						88      ,8P  d8'`8b    d8'           88  ,88"     88               88
						88aaaaaa8P' d8'  `8b   88            88,d88'      88aaaaa          88
						88""""""'  d8YaaaaY8b  88            8888"88,     88"""""          88
						88        d8""""""""8b Y8,           88P   Y8b    88               88
						88       d8'        `8b Y8a.    .a8P 88     "88,  88               88
						88      d8'          `8b `"Y8888Y"'  88       Y8b 88888888888      88
*/
/************************************************************************************************************************/
/* Packet for BLE server and client */
BLE_BMS_MOTOR_PACKET_T  		bmsMotorServerPacket = { 0 };
BLE_LOCK_PACKET_T				ilockitServerPacket = { 0 };
BLE_TIME_PACKET_T 				timeInfoServerPacket = { 0 };
BLE_HEARTRATE_PACKET_T  		heartRateServerPacket = { 0 };
BLE_LOCATION_PACKET_T			locationServerPacket = { 0 };
BLE_MPU_PACKET_T				mpuServerPacket = { 0 };
BLE_ONWRITE_PACKET_T			onWritePacket = { 0 };
BMS_PACKET_STRUCT_T				bmsPacket = { 0 };

/* Packet for LORAWAN */
LORA_DATA_PACKET_T				loraPacket = { 0 };

/************************************************************************************************************************/
/*!
										+-+-+-+ +-+-+-+-+-+-+ +-+-+-+-+-+-+-+-+
										|B|L|E| |S|E|R|V|E|R| |C|A|L|L|B|A|C|K|
										+-+-+-+ +-+-+-+-+-+-+ +-+-+-+-+-+-+-+-+
*/
/************************************************************************************************************************/
class BorealisServerCallback : public BLEServerCallbacks {

	void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t *param) {

		char remoteAddress[18];

		sprintf(
			remoteAddress,
			"%.2X:%.2X:%.2X:%.2X:%.2X:%.2X",
			param->connect.remote_bda[0],
			param->connect.remote_bda[1],
			param->connect.remote_bda[2],
			param->connect.remote_bda[3],
			param->connect.remote_bda[4],
			param->connect.remote_bda[5]
		);

		ESP_LOGI(LOG_TAG, "BorealisServerCallback onConnect, MAC: %s", remoteAddress);
		ESP_LOGI(LOG_TAG, "pServer->getConnectedCount(): %d", pServer->getConnectedCount());

		// limit the connected client to two clients
		if (pServer->getConnectedCount() < 2) pAdvertising->start();
		else pAdvertising->stop();
	}

	void onDisconnect(BLEServer* pServer, esp_ble_gatts_cb_param_t *param) {

		char remoteAddress[20];

		sprintf(
			remoteAddress,
			"%.2X:%.2X:%.2X:%.2X:%.2X:%.2X",
			param->disconnect.remote_bda[0],
			param->disconnect.remote_bda[1],
			param->disconnect.remote_bda[2],
			param->disconnect.remote_bda[3],
			param->disconnect.remote_bda[4],
			param->disconnect.remote_bda[5]
		);

		ESP_LOGI(LOG_TAG, "BorealisServerCallback onDisconnect, MAC: %s", remoteAddress);
		ESP_LOGI(LOG_TAG, "pServer->getConnectedCount(): %d", pServer->getConnectedCount());

		// limit the connected client to two clients
		if (pServer->getConnectedCount() < 2) pAdvertising->start();
		else pAdvertising->stop();

	}
};

/************************************************************************************************************************/
/*!
						 +-+-+-+ +-+-+-+-+-+-+ +-+-+-+-+-+-+-+-+-+-+-+-+-+-+ +-+-+-+-+-+-+-+-+
						 |B|L|E| |S|E|R|V|E|R| |C|H|A|R|A|C|T|E|R|I|S|T|I|C| |C|A|L|L|B|A|C|K|
						 +-+-+-+ +-+-+-+-+-+-+ +-+-+-+-+-+-+-+-+-+-+-+-+-+-+ +-+-+-+-+-+-+-+-+
*/
/************************************************************************************************************************/
class ILOCKITLockControlCharacteristicCallbacks : public BLECharacteristicCallbacks {
	void onWrite(BLECharacteristic *pCharacteristic) {
		onWritePacket.bLockControlValue = pCharacteristic->getData()[0];
		ESP_LOGI(LOG_TAG, "onWritePacket.blockControlValue: %.2X", onWritePacket.bLockControlValue);
		isLockControlOnWrite = true;
	}
};

class TimeSetCharacteristicCallbacks : public BLECharacteristicCallbacks {
	void onWrite(BLECharacteristic *pCharacteristic) {
		memcpy(onWritePacket.abTimeSet, pCharacteristic->getData(), 4);
		timeInfoServerPacket.tConfigCurrentTime.ulValue = onWritePacket.ulTimeSet;
		ESP_LOGI(LOG_TAG, "onWritePacket.ulTimeSet: %ld", onWritePacket.ulTimeSet);
		isTimeSetOnWrite = true;
	}
};

void setupBLEServer() {
	// Initialise the BLE server
	ESP_LOGI(LOG_TAG, "BLE Server is starting...");
	pServer = BLEDevice::createServer();

	// set the BLE server callback
	pServer->setCallbacks(new BorealisServerCallback());


	// configure the bms motor server service and characteristics
	pBmsService = pServer->createService(BMS_MOTOR_SRV_SERVICE);
	pBmsMotorChar = pBmsService->createCharacteristic(BMS_MOTOR_SRV_CHAR, PROP_WRITE | PROP_READ | PROP_NOTIFY);
	BmsMotorDescriptor.setValue("BMS and Motor Controller Data");
	pBmsMotorChar->addDescriptor(&BmsMotorDescriptor);
	pBmsMotorChar->addDescriptor(new BLE2902());
	pBmsMotorChar->setValue(bmsMotorServerPacket.abPacket, sizeof(bmsMotorServerPacket.abPacket));

	// configure the ilockit service and characteristics
	pIlockitService = pServer->createService(ILOCKIT_SRV_SERVICE);
	pIlockitChar = pIlockitService->createCharacteristic(ILOCKIT_SRV_CHAR, PROP_WRITE | PROP_READ | PROP_NOTIFY);
	IlockitDescriptor.setValue("Lock State, Session Time, Last Lock Change");
	pIlockitChar->addDescriptor(&IlockitDescriptor);
	pIlockitChar->addDescriptor(new BLE2902());
	pIlockitChar->setValue(ilockitServerPacket.abPacket, sizeof(ilockitServerPacket.abPacket));
	pLockControlChar = pIlockitService->createCharacteristic(LOCK_CONTROL_SRV_CHAR, PROP_WRITE | PROP_READ | PROP_NOTIFY);
	LockControlDescriptor.setValue("Lock Control");
	pLockControlChar->addDescriptor(&LockControlDescriptor);
	pLockControlChar->addDescriptor(new BLE2902());
	pLockControlChar->setCallbacks(new ILOCKITLockControlCharacteristicCallbacks());

	// configure the heart rate service and characteristics
	pHeartRateService = pServer->createService(HEART_RATE_SRV_SERVICE);
	pHeartRateChar = pHeartRateService->createCharacteristic(HEART_RATE_SRV_CHAR, PROP_WRITE | PROP_READ | PROP_NOTIFY);
	HeartRateDescriptor.setValue("Heart rate in bpm");
	pHeartRateChar->addDescriptor(&HeartRateDescriptor);
	pHeartRateChar->addDescriptor(new BLE2902());
	pHeartRateChar->setValue(heartRateServerPacket.abPacket, sizeof(heartRateServerPacket.abPacket));

	// configure the time info service and characteristics
	pTimeInfoService = pServer->createService(TIME_INFO_SRV_SERVICE);
	pLoraInfoChar = pTimeInfoService->createCharacteristic(TIME_LORA_INFO_SRV_CHAR, PROP_WRITE | PROP_READ | PROP_NOTIFY);
	LoraInfoDescriptor.setValue("LORAWAN last send packet time");
	pLoraInfoChar->addDescriptor(&LoraInfoDescriptor);
	pLoraInfoChar->addDescriptor(new BLE2902());
	pLoraInfoChar->setValue(timeInfoServerPacket.tLoraLastSendPackageTime.abValue, 4);
	pTimeSetChar = pTimeInfoService->createCharacteristic(TIME_SET_SRV_CHAR, PROP_WRITE | PROP_READ | PROP_NOTIFY);
	TimeSetDescriptor.setValue("Set the ESP32 current time");
	pTimeSetChar->addDescriptor(&TimeSetDescriptor);
	pTimeSetChar->addDescriptor(new BLE2902());
	pTimeSetChar->setCallbacks(new TimeSetCharacteristicCallbacks());
	pTimeSetChar->setValue(timeInfoServerPacket.tConfigCurrentTime.abValue, 4);
	pEspTimeChar = pTimeInfoService->createCharacteristic(TIME_ESPTIME_SRV_CHAR, PROP_WRITE | PROP_READ | PROP_NOTIFY);
	EspTimeDescriptor.setValue("Current ESP32 time");
	pEspTimeChar->addDescriptor(&EspTimeDescriptor);
	pEspTimeChar->addDescriptor(new BLE2902());
	pEspTimeChar->setValue(timeInfoServerPacket.tEspCurrentTime.abValue, 4);

	// configure the location info service and characteristics
	pLocationService = pServer->createService(LOCATION_SRV_SERVICE);
	pLocationChar = pLocationService->createCharacteristic(LOCATION_SRV_CHAR, PROP_WRITE | PROP_READ | PROP_NOTIFY);
	LocationDescriptor.setValue("Lat, Long, Alt, Time recorded");
	pLocationChar->addDescriptor(&LocationDescriptor);
	pLocationChar->addDescriptor(new BLE2902());
	pLocationChar->setValue(locationServerPacket.abPacket, sizeof(locationServerPacket.tPacket));

	// configure the mpu service and characteristics
	pMpuService = pServer->createService(MPU_SRV_SERVICE);
	pMpuChar = pMpuService->createCharacteristic(MPU_SRV_CHAR, PROP_WRITE | PROP_READ | PROP_NOTIFY);
	MpuDescriptor.setValue("MPU threshold detection");
	pMpuChar->addDescriptor(&MpuDescriptor);
	pMpuChar->addDescriptor(new BLE2902());
	pMpuChar->setValue(mpuServerPacket.abPacket, sizeof(mpuServerPacket.abPacket));

	// start all services
	pBmsService->start();
	pIlockitService->start();
	pHeartRateService->start();
	pTimeInfoService->start();
	pLocationService->start();
	pMpuService->start();

	// initialise the server advertising 
	pAdvertising = pServer->getAdvertising();

	// add ilockit service uuid to be advertised
	pAdvertising->addServiceUUID(pIlockitService->getUUID());
	pAdvertising->setScanResponse(false);
	pAdvertising->setMinPreferred(0x00);

	// start advertising
	pAdvertising->start();
}


void setup() {

	Serial.begin(115200);

	// initialiser the ble controller
	BLEDevice::init("ZSY");

	// setup the ble server
	setupBLEServer();

	start_time = millis();
}

void loop() {

	// check if there is any write operation every 1 s
	if (millis() - start_time >= 1000) {

		ESP_LOGI(LOG_TAG, "Check if there is any write operation");

		if (isLockControlOnWrite) {
			pLockControlChar->notify();
			isLockControlOnWrite = false;
		}

		if (isTimeSetOnWrite) {
			pTimeSetChar->notify();
			isTimeSetOnWrite = false;
		}

		start_time = millis();
	}

	delay(250);

} // End of loop
