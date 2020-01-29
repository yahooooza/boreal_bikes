/************************************************************************************************************************/
/*!
* @author		zzulkifli, edoelling, lbierstedt (ZeSys e.V)
* @file			BLE_CLIENT.ino
* @date			2019-03.01
* @version		1.0
* @brief		Program for ESP BLE Client. The clients collect the data from multiple BLE server and other wired devices
* and then send the data to the LORAWAN server and BLE server for end users. 
*
* Used library:
*	Library									| Release				| Notes
*	----------------------------------------|-----------------------|------------------
*	ESP32 BLE								| -						| author: nkolban, chegewara (https://github.com/nkolban/esp32-snippets/tree/master/cpp_utils)
*	MPU9250									| 1.0.1					| author: brian.taylor@bolderflight.com (https://github.com/bolderflight/MPU9250)
*	MPU9250 Impact detection				| 1.0.0					|
*	BMS packet handler						| 1.0.0					|
*	Controller packet handler				| 1.0.0					|
*	L76 (based on Sparkfun I2C GPS library)	| 1.0.1					| author: Nathan Seidle (https://github.com/sparkfun/SparkFun_I2C_GPS_Arduino_Library)	
*	TinyGPSPlus								| 1.0.2					| author: Mikal Hart (https://github.com/mikalhart/TinyGPSPlus)
*	Universal 8bit Graphics Library 		| 2.24.3				| author: Oli Kraus (https://github.com/olikraus/u8g2)
*	MCCI LoRaWAN LMIC						| 2.3.1					| https://github.com/mcci-catena/arduino-lmic
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
*	-	all the used library may have been modified to suit the ESP32 use case.
*
* @warning
*
*/
/************************************************************************************************************************/

#include <Arduino.h>
#include <SPI.h>
#include <BLEClient.h>
#include <BLEDevice.h>
#include <BLERemoteService.h>
#include <BLE2902.h>
#include <time.h>
#include <sys/time.h>
#include <BMSPacketHandler.h>
#include <ControllerPacketHandler.h>
#include <MPU9250_Impact.h>
#include <L76.h>
#include <TinyGPS++.h>
#include <lmic.h>
#include <hal/hal.h>
#include <U8g2lib.h>
#include "BBUUID.h"
#include "BB_BLEServer.h"
#include "BB_BLEClient.h"
#include "LoraPacketHandler.h"
#include "endian.h"
#include "xbm.h"
#include <array>
#include "freertos/task.h"

#if defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#define LOG_TAG ""
#endif

/************************************************************************************************************************/
/*!
					88888888ba,   88  ad88888ba  88888888ba  88                 db   8b        d8
					88      `"8b  88 d8"     "8b 88      "8b 88                d88b   Y8,    ,8P
					88        `8b 88 Y8,         88      ,8P 88               d8'`8b   Y8,  ,8P
					88         88 88 `Y8aaaaa,   88aaaaaa8P' 88              d8'  `8b   "8aa8"
					88         88 88   `"""""8b, 88""""""'   88             d8YaaaaY8b   `88'
					88         8P 88         `8b 88          88            d8""""""""8b   88
					88      .a8P  88 Y8a     a8P 88          88           d8'        `8b  88
					88888888Y"'   88  "Y88888P"  88          88888888888 d8'          `8b 88

*/
/************************************************************************************************************************/
#define DISPLAY_AVAIL	1

#if DISPLAY_AVAIL > 0
/* U8G2 class definition, U8G2_16BIT in u8g2.h should be enable for the used display size */
U8G2_SSD1322_NHD_256X64_F_4W_HW_SPI u8g2(U8G2_R1, /* cs=*/ 15, /* dc=*/ 2, /* reset=*/ U8X8_PIN_NONE);
#endif

struct tm *pTmDisplay;

/************************************************************************************************************************/
/*!
									  88888888ba  88b           d88  ad88888ba
									  88      "8b 888b         d888 d8"     "8b
									  88      ,8P 88`8b       d8'88 Y8,
									  88aaaaaa8P' 88 `8b     d8' 88 `Y8aaaaa,
									  88""""""8b, 88  `8b   d8'  88   `"""""8b,
									  88      `8b 88   `8b d8'   88         `8b
									  88      a8P 88    `888'    88 Y8a     a8P
									  88888888P"  88     `8'     88  "Y88888P"
*/
/************************************************************************************************************************/
BMSPacketHandler bms;

/************************************************************************************************************************/
/*!
		   88b           d88                                               ,ad8888ba,                   88
		   888b         d888              ,d                              d8"'    `"8b ,d               88
		   88`8b       d8'88              88                             d8'           88               88
		   88 `8b     d8' 88  ,adPPYba, MM88MMM ,adPPYba,  8b,dPPYba,    88          MM88MMM 8b,dPPYba, 88
		   88  `8b   d8'  88 a8"     "8a  88   a8"     "8a 88P'   "Y8    88            88    88P'   "Y8 88
		   88   `8b d8'   88 8b       d8  88   8b       d8 88            Y8,           88    88         88
		   88    `888'    88 "8a,   ,a8"  88,  "8a,   ,a8" 88             Y8a.    .a8P 88,   88         88
		   88     `8'     88  `"YbbdP"'   "Y888 `"YbbdP"'  88              `"Y8888Y"'  "Y888 88         88
*/
/************************************************************************************************************************/
ControllerPacketHandler controller;

/************************************************************************************************************************/
/*!
									  88b           d88 88888888ba  88        88
									  888b         d888 88      "8b 88        88
									  88`8b       d8'88 88      ,8P 88        88
									  88 `8b     d8' 88 88aaaaaa8P' 88        88
									  88  `8b   d8'  88 88""""""'   88        88
									  88   `8b d8'   88 88          88        88
									  88    `888'    88 88          Y8a.    .a8P
									  88     `8'     88 88           `"Y8888Y"'
*/
/************************************************************************************************************************/
ImpactDetector IMU = ImpactDetector(100, 2000, 0.1, 2.0, 4.0, 0x68, SDA, SCL);
const uint8_t mpuIntPin = 39;
bool isImuConnected = false;
bool isCrashDetected = false;
std::array<float, 3> afImpact;
std::array<float, 3> afValues;

/************************************************************************************************************************/
/*!
										  ,ad8888ba,  88888888ba   ad88888ba
										 d8"'    `"8b 88      "8b d8"     "8b
										d8'           88      ,8P Y8,
										88            88aaaaaa8P' `Y8aaaaa,
										88      88888 88""""""'     `"""""8b,
										Y8,        88 88                  `8b
										 Y8a.    .a88 88          Y8a     a8P
										  `"Y88888P"  88           "Y88888P"
*/
/************************************************************************************************************************/
I2CGPS L76;
TinyGPSPlus gps;
boolean isGpsConnected = false;

/************************************************************************************************************************/
/*!
		  88          ,ad8888ba,   88888888ba         db   I8,        8        ,8I   db        888b      88
		  88         d8"'    `"8b  88      "8b       d88b  `8b       d8b       d8'  d88b       8888b     88
		  88        d8'        `8b 88      ,8P      d8'`8b  "8,     ,8"8,     ,8"  d8'`8b      88 `8b    88
		  88        88          88 88aaaaaa8P'     d8'  `8b  Y8     8P Y8     8P  d8'  `8b     88  `8b   88
		  88        88          88 88""""88'      d8YaaaaY8b `8b   d8' `8b   d8' d8YaaaaY8b    88   `8b  88
		  88        Y8,        ,8P 88    `8b     d8""""""""8b `8a a8'   `8a a8' d8""""""""8b   88    `8b 88
		  88         Y8a.    .a8P  88     `8b   d8'        `8b `8a8'     `8a8' d8'        `8b  88     `8888
		  88888888888 `"Y8888Y"'   88      `8b d8'          `8b `8'       `8' d8'          `8b 88      `888
*/
/************************************************************************************************************************/
// Schedule TX every this many seconds (might become longer due to duty cycle limitations).
const unsigned TX_INTERVAL = 20;
bool isLoraSessionKeyAvailable = false;
bool isLoraTaskSet = false;
bool isLoraPacketSent = false;

#warning "Don't forget to update the eui and key from the LoraPacketHandler.h for OTAA function"
// get the eui and key from the LoraPacketHandler.h
void os_getArtEui(u1_t* buf) { memcpy_P(buf, APPEUI, 8); }
void os_getDevEui(u1_t* buf) { memcpy_P(buf, DEVEUI, 8); }
void os_getDevKey(u1_t* buf) { memcpy_P(buf, APPKEY, 16); }

osjob_t sendjob;

char nwkKeyBuffer[100];		// buffer for saving network session key, use for printing
char artKeyBuffer[100];		// buffer for saving application session key, use for printing

#warning "Choose bbfipy as device for current config, else configured the pin as stated in FiPy_pinout table"
// LMIC pin mapping
const lmic_pinmap lmic_pins = {
	.nss = 18,							// Lora Chip Select pin
	.rxtx = LMIC_UNUSED_PIN,				
	.rst = LMIC_UNUSED_PIN,
	.dio = {23, 23, 23},	// workaround to use 1 pin for all 3 radio dio pins
	.mosi = 27,						// Lora MOSI pin
	.miso = 19,						// Lora MISO pin
	.sck = 5,						// Lora SCK pin
};

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
time_t lock_lastTime, lock_nowTime;
double lock_difftime;
uint16_t lock_diffTimeInMinutes;
time_t lora_lastSendPackageTime;
time_t heartRate_time;
time_t location_time;
time_t bmsMotor_time;
struct tm gpstm;

uint32_t ttnGetKeyTime;

uint32_t loopStartTime, loopFinishTime, loopTime, lastLoopTime = 0;
uint32_t bmsTaskTime, heartyTaskTime, controllerTaskTime, serverTaskTime, gpsTaskTime, displayTaskTime = 0;

time_t rawTime;
uint32_t lastConfigTime = 0;
bool isSessionTimeCountEnable = false;


/************************************************************************************************************************/
/*!
		   88888888888 88888888ba  88888888888 88888888888 88888888ba 888888888888 ,ad8888ba,    ad88888ba
		   88          88      "8b 88          88          88      "8b     88     d8"'    `"8b  d8"     "8b
		   88          88      ,8P 88          88          88      ,8P     88    d8'        `8b Y8,
		   88aaaaa     88aaaaaa8P' 88aaaaa     88aaaaa     88aaaaaa8P'     88    88          88 `Y8aaaaa,
		   88"""""     88""""88'   88"""""     88"""""     88""""88'       88    88          88   `"""""8b,
		   88          88    `8b   88          88          88    `8b       88    Y8,        ,8P         `8b
		   88          88     `8b  88          88          88     `8b      88     Y8a.    .a8P  Y8a     a8P
		   88          88      `8b 88888888888 88888888888 88      `8b     88      `"Y8888Y"'    "Y88888P"
*/
/************************************************************************************************************************/
/* Task Handle */
TaskHandle_t xTaskTtn;													// task handler for TTN task (display included)
TaskHandle_t xTaskI2c;													// task handler for I2c task (GPS, IMU)
TaskHandle_t xTaskMain;													// task handler for main task
TaskHandle_t xTaskWatchdog;												// task handler for watchdog task

/* Semaphore for the task */
SemaphoreHandle_t xSemaphoreI2c;										// semaphore handle for i2c 
SemaphoreHandle_t xSemaphoreSpi;										// semaphore handle for spi 
SemaphoreHandle_t xSemaphoreBLE;										// semaphore handle for BLE task (if needed)
EventGroupHandle_t xWatchdogEvent;										// FreeRTOS event handle for watchdog task

/* Task ID for the watchdog event function */
const uint32_t mainTaskId	= (1 << 0);									// main task id for watchdog
const uint32_t ttnTaskId	= (1 << 1);									// ttn task id for watchdog
const uint32_t i2cTaskId	= (1 << 2);									// i2c task id for watchdog
const uint32_t allTaskId	= (mainTaskId | ttnTaskId | i2cTaskId);		// all task id for watchdog
uint32_t wd_result = 0;													// watchdog event result

/* flag for the watchdog function*/
bool isMainTaskStopResponding, isTtnTaskStopResponding, isI2cTaskStopResponding = false;

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
/* Packet for LORAWAN */
LORA_DATA_PACKET_T				loraPacket = { 0 };					// packet for lorawan data

/* Packet for BLE Client */
BLE_BMS_MOTOR_PACKET_T  		bmsMotorServerPacket = { 0 };		// packet for bms and motor controller data
BLE_LOCK_PACKET_T				ilockitServerPacket = { 0 };		// packet for lock data
BLE_TIME_PACKET_T 				timeInfoServerPacket = { 0 };		// packet for time info data
BLE_HEARTRATE_PACKET_T  		heartRateServerPacket = { 0 };		// packet for heart rate data 
BLE_LOCATION_PACKET_T			locationServerPacket = { 0 };		// packet for location data
BLE_MPU_PACKET_T				mpuServerPacket = { 0 };			// packet for imu data
BLE_ONWRITE_PACKET_T			onWritePacket = { 0 };				// packet for onWrite data from BLE server
BMS_PACKET_STRUCT_T				bmsPacket = { 0 };					// packet for incoming bms data

/************************************************************************************************************************/
/*!
									  88b           d88 88  ad88888ba    ,ad8888ba,
									  888b         d888 88 d8"     "8b  d8"'    `"8b
									  88`8b       d8'88 88 Y8,         d8'
									  88 `8b     d8' 88 88 `Y8aaaaa,   88
									  88  `8b   d8'  88 88   `"""""8b, 88
									  88   `8b d8'   88 88         `8b Y8,
									  88    `888'    88 88 Y8a     a8P  Y8a.    .a8P
									  88     `8'     88 88  "Y88888P"    `"Y8888Y"'
*/
/************************************************************************************************************************/
int connectCounter = 0;		// BLE connection tries counter



/************************************************************************************************************************/
/*!
										  +-+-+-+ +-+-+-+-+ +-+-+-+-+-+-+-+-+
										  |B|L|E| |S|C|A|N| |C|A|L|L|B|A|C|K|
										  +-+-+-+ +-+-+-+-+ +-+-+-+-+-+-+-+-+
*/
/************************************************************************************************************************/
class AdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
	void onResult(BLEAdvertisedDevice advertisedDevice) {
		Serial.printf("Advertised Device: %s\n", advertisedDevice.toString().c_str());

		if (advertisedDevice.haveServiceUUID() && advertisedDevice.getAddress().equals(BMS_MAC)) {
			pBmsDevice = new BLEAdvertisedDevice(advertisedDevice);
			Serial.printf("BMS Device found\n");
			isBmsFound = true;
		}

		else if (advertisedDevice.haveServiceUUID() && (advertisedDevice.getAddress().equals(HEARTYPATCH_MAC))) {
			pHeartyPatchDevice = new BLEAdvertisedDevice(advertisedDevice);
			Serial.printf("HEARTYPATCH Device found\n");
			isHeartyFound = true;
		}
		else if (advertisedDevice.haveServiceUUID() && (advertisedDevice.getAddress().equals(CONTROLLER_MAC))) {
			pControllerDevice = new BLEAdvertisedDevice(advertisedDevice);
			Serial.printf("Motor Controller Device found\n");
			isControllerFound = true;
		}
		else if (advertisedDevice.haveServiceUUID() && (advertisedDevice.getAddress().equals(ESP_SERVER_MAC))) {
			pEspServerDevice = new BLEAdvertisedDevice(advertisedDevice);
			Serial.printf("ESP Server Device found\n");
			isEspServerFound = true;
		}

		if (isEspServerFound && isBmsFound && isHeartyFound && isControllerFound) {
			Serial.printf("All device found, stop scanning\n");
			pScan->stop();
			isScanStop = true;
		}
	}
};

/************************************************************************************************************************/
/*!
							 +-+-+-+ +-+-+-+-+-+-+ +-+-+-+-+-+-+-+-+-+-+ +-+-+-+-+-+-+-+-+
							 |B|L|E| |C|L|I|E|N|T| |C|O|N|N|E|C|T|I|O|N| |C|A|L|L|B|A|C|K|
							 +-+-+-+ +-+-+-+-+-+-+ +-+-+-+-+-+-+-+-+-+-+ +-+-+-+-+-+-+-+-+
*/
/************************************************************************************************************************/
class BMSClientCallbacks : public BLEClientCallbacks {
	void onConnect(BLEClient* pClient) {
		isBmsConnected = true;
		ESP_LOGI(LOG_TAG, "BMSClientCallbacks onConnect");
	}

	void onDisconnect(BLEClient* pClient) {

		ESP_LOGI(LOG_TAG, "BMSClientCallbacks onDisconnect");
		
		isBmsConnected = false;

		// free the allocated memory and calls destructor for the pClient
		// COMMENT this if it bring to stack error
		delete pClient;

	}
};
class HeartyPatchClientCallbacks : public BLEClientCallbacks {
	void onConnect(BLEClient* pClient) {
		isHeartyConnected = true;
		ESP_LOGI(LOG_TAG, "HeartyPatchClientCallbacks onConnect");
	}

	void onDisconnect(BLEClient* pClient) {
		ESP_LOGI(LOG_TAG, "HeartyPatchClientCallbacks onDisconnect");
		
		isHeartyConnected = false;

		// free the allocated memory and calls destructor for the pClient
		// COMMENT this if it bring to stack error
		delete pClient;
	}
};
class ControllerClientCallbacks : public BLEClientCallbacks {
	void onConnect(BLEClient* pClient) {
		isControllerConnected = true;
		ESP_LOGI(LOG_TAG, "ControllerClientCallbacks onConnect");
	}

	void onDisconnect(BLEClient* pClient) {
		ESP_LOGI(LOG_TAG, "ControllerClientCallbacks onDisconnect");
		
		isControllerConnected = false;

		// free the allocated memory and calls destructor for the pClient
		// COMMENT this if it bring to stack error
		delete pClient;
	}
};
class EspServerClientCallbacks : public BLEClientCallbacks {
	void onConnect(BLEClient* pClient) {
		isEspServerConnected = true;
		ESP_LOGI(LOG_TAG, "EspServerClientCallbacks onConnect");
	}

	void onDisconnect(BLEClient* pClient) {
		ESP_LOGI(LOG_TAG, "EspServerClientCallbacks onDisconnect");
		
		isEspServerConnected = false;

		// free the allocated memory and calls destructor for the pClient
		delete pClient;
	}
};

/************************************************************************************************************************/
/*!
								 +-+-+-+ +-+-+-+-+-+-+ +-+-+-+-+-+-+ +-+-+-+-+-+-+-+-+
								 |B|L|E| |C|L|I|E|N|T| |N|O|T|I|F|Y| |C|A|L|L|B|A|C|K|
								 +-+-+-+ +-+-+-+-+-+-+ +-+-+-+-+-+-+ +-+-+-+-+-+-+-+-+
*/
/************************************************************************************************************************/
void bmsNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {

	BMS_PACKET_STRUCT_T rPacket = { 0 };

	// read, verify and decode the incoming packet from BMS BLE server
	if (bms.bmsReadInfoStatus(&rPacket, pData, length)) {
		ESP_LOGI(LOG_TAG, "Get BMS packet");

		// copy the incoming data into the bms packet
		memcpy(&bmsPacket.tInfoStatus, &rPacket.tInfoStatus, sizeof(rPacket.tInfoStatus));
		
		// since all the BMS packet in big endian, swapping is not needed
		bmsMotorServerPacket.tPacket.usBmsTotalVoltage = bmsPacket.tInfoStatus.usTotalVoltage;
		bmsMotorServerPacket.tPacket.bBatPercentage = bmsPacket.tInfoStatus.bRelStateOfCharge;

		// get the current time
		time(&rawTime);
		bmsMotorServerPacket.tPacket.ulBmsTime = bswap32((uint32_t)rawTime);	// swap the time into big-endian as required for the server

		ESP_LOGI(LOG_TAG, "BMS Voltage: %d, RSOC: %d%%, ulBmsTime: %d\n", bswap16(bmsMotorServerPacket.tPacket.usBmsTotalVoltage), bmsMotorServerPacket.tPacket.bBatPercentage, bswap32(bmsMotorServerPacket.tPacket.ulBmsTime));

		isBmsNotifyAvailable = true;
	}
}

void controllerNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {

	CONTROLLER_PACKET_STRUCT_T controllerPacket = { 0 };

	// read, verify and decode the incoming packet from Motor Controller BLE server
	if (controller.readParsePacket(&controllerPacket, pData, length)) {

		// assign all the needed data into bms motor server packet, byteswap due to endian requirement from the server
		bmsMotorServerPacket.tPacket.usMotorTotalVoltage = bswap16((uint16_t)(controllerPacket.tPacket.bTotalVoltage*1000.0 / 3.7)); // convert to mV
		bmsMotorServerPacket.tPacket.usDistance = bswap16(controllerPacket.tPacket.usTotalDistance);
		bmsMotorServerPacket.tPacket.bSpeed = (uint8_t)controllerPacket.tPacket.ulSpeedKmH;
		ESP_LOGI(LOG_TAG,
			"Vtotal: %.3f V, Dtotal: %d km, v: %d km/h\n",
			controllerPacket.tPacket.bTotalVoltage / 3.7,
			controllerPacket.tPacket.usTotalDistance,
			controllerPacket.tPacket.ulSpeedKmH);

		// get the current time
		time(&rawTime);
		bmsMotorServerPacket.tPacket.ulBmsTime = bswap32((uint32_t)rawTime); // swap the time into big-endian as required for the server

		isControllerNotifyAvailable = true;
	}
}



/************************************************************************************************************************/
/*!
* @brief		update lora packet
* @retval		none
*/
/************************************************************************************************************************/
void updateLoraPacket() {

	// assign all the needed data into lora packet
	loraPacket.tPacket.ulGpsLatitude = locationServerPacket.tPacket.ulLatitude;
	loraPacket.tPacket.ulGpsLongitude = locationServerPacket.tPacket.ulLongitude;
	loraPacket.tPacket.ulGpsAltitude = locationServerPacket.tPacket.ulElevation;
	loraPacket.tPacket.usBmsTotalVoltage = bmsMotorServerPacket.tPacket.usBmsTotalVoltage;
	loraPacket.tPacket.usControllerTotalVoltage = bmsMotorServerPacket.tPacket.usMotorTotalVoltage;
	loraPacket.tPacket.bControllerSpeedKmh = bmsMotorServerPacket.tPacket.bSpeed;
	loraPacket.tPacket.usControllerTotalDistance = bmsMotorServerPacket.tPacket.usDistance;
	loraPacket.tPacket.ulRtcTimeOfStartSession = ilockitServerPacket.tPacket.usLockSessionTime;
	loraPacket.tPacket.bHeartyBpm = heartRateServerPacket.tPacket.bHeartRate;
	loraPacket.tPacket.bMpuFlags = mpuServerPacket.tPacket.bCrashDetect;
	loraPacket.tPacket.ulMpuCrashTime = mpuServerPacket.tPacket.ulCrashTime;

	Serial.printf("LoraPacket : ");

	for (uint8_t i = 0; i < sizeof(loraPacket.tPacket); i++) {
		Serial.printf("0x%.2X ", loraPacket.abPacket[i]);
	}

	Serial.println();
}

/************************************************************************************************************************/
/*!
* @brief		set the serial packet
* @param[in]	*tSerialWritePacket	pointer to the serial packet to be set
* @param[in]	len					payload length
* @param[in]	cmdID				command ID
* @param[in]	*val				payload value in byte array
* @retval		packet size
*/
/************************************************************************************************************************/
void do_send(osjob_t* j) {
	// Check if there is not a current TX/RX job running
	if (LMIC.opmode & OP_TXRXPEND) {
		ESP_LOGI(LOG_TAG, "OP_TXRXPEND, not sending");

	}
	else {
		if (isLoraSessionKeyAvailable) {
			time(&rawTime);
			timeInfoServerPacket.tLoraLastSendPackageTime.ulValue = bswap32((uint32_t)rawTime);
			updateLoraPacket();

			// Prepare upstream data transmission at the next possible time.
			LMIC_setTxData2(1, loraPacket.abPacket, sizeof(loraPacket.abPacket), 0);
			ESP_LOGI(LOG_TAG, "Lora Packet queued\n");

			isLoraPacketSent = true;
		}

		else {
			LMIC_setTxData2(1, loraPacket.abPacket, sizeof(loraPacket.abPacket), 0);
		}
	}
	// Next TX is scheduled after TX_COMPLETE event.
}

/************************************************************************************************************************/
/*!
* @brief		set the serial packet
* @param[in]	*tSerialWritePacket	pointer to the serial packet to be set
* @param[in]	len					payload length
* @param[in]	cmdID				command ID
* @param[in]	*val				payload value in byte array
* @retval		packet size
*/
/************************************************************************************************************************/
void onEvent(ev_t ev) {
	switch (ev) {
	case EV_JOINING:
		ESP_LOGI(LOG_TAG, "%d: EV_JOINING", os_getTime());
		break;
	case EV_JOINED:
		ESP_LOGI(LOG_TAG, "%d: EV_JOINED", os_getTime());
		{
			u4_t netid = 0;
			devaddr_t devaddr = 0;
			u1_t nwkKey[16];
			u1_t artKey[16];
			LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);

			isLoraSessionKeyAvailable = true;

			sprintf(
				nwkKeyBuffer,
				"%.2X,%.2X,%.2X,%.2X,%.2X,%.2X,%.2X,%.2X,%.2X,%.2X,%.2X,%.2X,%.2X,%.2X,%.2X,%.2X",
				nwkKey[0], nwkKey[1], nwkKey[2], nwkKey[3],
				nwkKey[4], nwkKey[5], nwkKey[6], nwkKey[7],
				nwkKey[8], nwkKey[9], nwkKey[10], nwkKey[11],
				nwkKey[12], nwkKey[13], nwkKey[14], nwkKey[15]
			);

			sprintf(
				artKeyBuffer,
				"%.2X,%.2X,%.2X,%.2X,%.2X,%.2X,%.2X,%.2X,%.2X,%.2X,%.2X,%.2X,%.2X,%.2X,%.2X,%.2X",
				artKey[0], artKey[1], artKey[2], artKey[3],
				artKey[4], artKey[5], artKey[6], artKey[7],
				artKey[8], artKey[9], artKey[10], artKey[11],
				artKey[12], artKey[13], artKey[14], artKey[15]
			);
			ESP_LOGI(LOG_TAG, "netid: %d,devaddr: 0x%.2X, artKey: %s, nwkKey: %s", netid, devaddr, artKeyBuffer, nwkKeyBuffer);


		}
		// Disable link check validation (automatically enabled
		// during join, but because slow data rates change max TX
		// size, we don't use it in this example.
		LMIC_setLinkCheckMode(0);
		break;
	case EV_JOIN_FAILED:
		ESP_LOGI(LOG_TAG, "%d: EV_JOIN_FAILED", os_getTime());
		break;
	case EV_REJOIN_FAILED:
		ESP_LOGI(LOG_TAG, "%d: EV_REJOIN_FAILED", os_getTime());
		break;
	case EV_TXCOMPLETE:
		ESP_LOGI(LOG_TAG, "%d: EV_TXCOMPLETE (includes waiting for RX windows)", os_getTime());
		if (LMIC.txrxFlags & TXRX_ACK)
			ESP_LOGI(LOG_TAG, "Received ack");
		if (LMIC.dataLen)
			ESP_LOGI(LOG_TAG, "Received %d bytes of payload", LMIC.dataLen);

		// Schedule next transmission
		os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
		break;
	case EV_LOST_TSYNC:
		ESP_LOGI(LOG_TAG, "%d: EV_LOST_TSYNC", os_getTime());
		break;
	case EV_RESET:
		ESP_LOGI(LOG_TAG, "%d: EV_RESET", os_getTime());
		break;
	case EV_RXCOMPLETE:
		// data received in ping slot
		ESP_LOGI(LOG_TAG, "%d: EV_RXCOMPLETE", os_getTime());
		break;
	case EV_LINK_DEAD:
		ESP_LOGI(LOG_TAG, "%d: EV_LINK_DEAD", os_getTime());
		break;
	case EV_LINK_ALIVE:
		ESP_LOGI(LOG_TAG, "%d: EV_LINK_ALIVE", os_getTime());
		break;
	case EV_TXSTART:
		ESP_LOGI(LOG_TAG, "EV_TXSTART", os_getTime());
		break;
	default:
		ESP_LOGI(LOG_TAG, "%d: LORA Unknown event: %d", os_getTime(), (unsigned)ev);
		break;
	}
}

/************************************************************************************************************************/
/*!
* @brief		setup the BLE scanner
* @retval		none
*/
/************************************************************************************************************************/
void setupBLEScanner() {
	pScan = BLEDevice::getScan();
	pScan->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCallbacks());
	pScan->setInterval(100);
	pScan->setWindow(99);
	pScan->setActiveScan(true);
	pScan->start(5, true);														// start the BLE scan
}


/************************************************************************************************************************/
/*!
* @brief		setup the system time
* @param[in]	setTime				epoch time in time_t format
* @retval		none
*/
/************************************************************************************************************************/
void setupTime(time_t setTime) {

	// log the set time into the lastConfigtime
	lastConfigTime = setTime;

	struct timeval now = { .tv_sec = setTime };

	// set the system time
	settimeofday(&now, NULL);

	// set the enviroment time zone into Berlin,Germany
	setenv("TZ", "CET-1CEST-2,M3.5.0/02:00:00,M10.5.0/03:00:00", 1);	// Berlin, Germany
	
	// set time zone
	tzset();

	struct tm *timeinfo;
	timeinfo = localtime(&setTime);
	ESP_LOGI(LOG_TAG, "setTime: %ld", setTime);
	ESP_LOGI(LOG_TAG, "The current date/time is: %s", asctime(timeinfo));
}

/************************************************************************************************************************/
/*!
* @brief		setup the IMU
* @retval		none
*/
/************************************************************************************************************************/
void setupIMU() {

	// start communication with IMU
	uint8_t status = IMU.begin();

	if (status < 1) {
		ESP_LOGE(LOG_TAG, "Status: 0x%.2X", status);
		ESP_LOGE(LOG_TAG, "Check IMU wiring or try cycling power");
	}

	else {
		ESP_LOGI(LOG_TAG, "IMU connected");
		isImuConnected = true;

		// calibrate the IMU
		while (!IMU.calibrateAccel()) {};
		
		// set the range for accelerometer and gyrometer
		IMU.setAccelRange(IMU.ACCEL_RANGE_16G);
		IMU.setGyroRange(IMU.GYRO_RANGE_500DPS);
	}
}

/************************************************************************************************************************/
/*!
* @brief		setup the GPS
* @retval		none
*/
/************************************************************************************************************************/
void setupGPS() {

	// setup the GPS i2c communication
	isGpsConnected = L76.begin(Wire, I2C_SPEED_STANDARD, SDA, SCL);

	if (!isGpsConnected) ESP_LOGI(LOG_TAG, "GPS failed to connect");

	else {
		ESP_LOGI(LOG_TAG, "GPS connected");

		// check if there is valid data from GPS
		if (L76.available()) {
			ESP_LOGI(LOG_TAG, "Check and encode GPS data");
			while (L76.available()) {
				// encode the incoming data from gps
				gps.encode(L76.read());
			}

			// setup gps time as current system time if the time data is valid
			if (gps.time.isValid()) {
				ESP_LOGI(LOG_TAG, "GPS Time is valid, set the time based on GPS time");
				setupTime(gps.time.value());
			}
			else ESP_LOGI(LOG_TAG, "GPS Time is not yet valid");
		}
	}
}

/************************************************************************************************************************/
/*!
* @brief		update the display frame
* @param[in]	totalVoltage		pointer to the total voltage data in uint16_t format
* @param[in]	sessionCount		pointer to the session count data in uint16_t format
* @param[in]	timeInfo			pointer to the time info structure in tm structure format
* @param[in]	bpmCount			pointer to the heart rate count in bpm in uint8_t format
* @param[in]	lcState				pointer to the lock state in uint8_t format
* @retval		none
*/
/************************************************************************************************************************/
void disp_frame(uint16_t *totalVoltage, uint16_t *sessionCount, struct tm *timeInfo, uint8_t *bpmCount, uint8_t lcState) {
#if DISPLAY_AVAIL > 0
	//ESP_LOGI(LOG_TAG, "update display");
	u8g2.clearBuffer();								/** clear the internal memory */
	u8g2.setFont(u8g2_font_helvR12_tr);				/** choose font */
	u8g2.setFontRefHeightAll();  					/* this will add some extra space for the text inside the buttons */
	u8g2.setCursor(0, 16);
	u8g2.printf("Vbms: ");
	u8g2.setCursor(0, 32);
	u8g2.printf("%.2f", bswap16(*totalVoltage) / 100.0);	/** BMS voltage */
	u8g2.drawStr(52, 32, "V");
	u8g2.drawStr(0, 56, "Session:");
	u8g2.setCursor(0, 72);
	u8g2.printf("%03d", *sessionCount);						/** session time */
	u8g2.drawStr(37, 72, "min");
	u8g2.setCursor(8, 104);
	if (timeInfo->tm_hour > 12) {
		u8g2.printf("%02d:%02d", timeInfo->tm_hour - 12, timeInfo->tm_min);		/** time */
		u8g2.drawStr(18, 120, "pm");
	}
	else {
		u8g2.printf("%02d:%02d", timeInfo->tm_hour, timeInfo->tm_min);			/** time */
		u8g2.drawStr(18, 120, "am");
	}

	u8g2.setCursor(2, 152);
	u8g2.printf("%3d", *bpmCount);												/** heart rate */
	u8g2.drawStr(0, 172, "bpm");

	if (heartIconCycle) {
		u8g2.drawXBMP(34, 145, heart_ekg_width, heart_ekg_height, heart_ekg_bits);
		heartIconCycle = false;
	}
	else {
		u8g2.drawXBMP(34, 145, heart_full_width, heart_full_height, heart_full_bits);
		heartIconCycle = true;
	}

	u8g2.drawXBMP(0, 180, lock_width, lock_height, lock_bits);

	u8g2.sendBuffer();								// transfer internal memory to the display
#endif
}

/************************************************************************************************************************/
/*!
* @brief		update the display
* @retval		none
*/
/************************************************************************************************************************/
void updateDisplay() {
#if DISPLAY_AVAIL > 0
	// get the current time
	time(&rawTime);

	// save the local time into tm structure pointer
	pTmDisplay = localtime(&rawTime);

	// update the display frame
	disp_frame(
		&bmsPacket.tInfoStatus.usTotalVoltage,
		&lock_diffTimeInMinutes,
		pTmDisplay,
		&heartRateServerPacket.tPacket.bHeartRate,
		ilockitServerPacket.tPacket.bLockState
	);
#endif
}

/************************************************************************************************************************/
/*!
* @brief		setup the display
* @retval		none
*/
/************************************************************************************************************************/
void setupDisplay() {
#if DISPLAY_AVAIL > 0

	// begin the communication with the display
	u8g2.begin();

	// update the display
	updateDisplay();
#endif
}

/************************************************************************************************************************/
/*!
* @brief		setup the BLE client
* @retval		none
*/
/************************************************************************************************************************/
void setupBLEClient() {

	ESP_LOGI(LOG_TAG, "Initialise the BLE Clients");

	// Bms client
	pBmsClient = BLEDevice::createClient();
	pBmsClient->setClientCallbacks(new BMSClientCallbacks());

	// HeartyPatch client
	pHeartyPatchClient = BLEDevice::createClient();
	pHeartyPatchClient->setClientCallbacks(new HeartyPatchClientCallbacks());

	// Motor controller clinet
	pControllerClient = BLEDevice::createClient();
	pControllerClient->setClientCallbacks(new ControllerClientCallbacks());

	// ESP Server client
	pEspServerClient = BLEDevice::createClient();
	pEspServerClient->setClientCallbacks(new EspServerClientCallbacks());

}

/************************************************************************************************************************/
/*!
* @brief		check the availability of the ble service, characteristic needed
* @param[in]	pClient					pointer to the BLE client object
* @param[in]	pRemoteService			pointer to the BLE remote service object
* @param[in]	pRemoteCharacteristic	pointer to the BLE remote characteristic object
* @param[in]	remoteServiceUUID		remote service UUID
* @param[in]	remoteCharUUID			remote characteristic UUID
* @retval		true, if all available, false if one of them not available
*/
/************************************************************************************************************************/
bool checkServiceCharacteristic(BLEClient* &pClient,
								BLERemoteService* &pRemoteService,
								BLERemoteCharacteristic* &pRemoteCharacteristic,
								BLEUUID remoteServiceUUID,
								BLEUUID remoteCharUUID) 
{
	ESP_LOGI(LOG_TAG, "Check Service Characteristic");

	if (pClient == nullptr) {
		ESP_LOGE(LOG_TAG, "pClient is null pointer");
		return false;
	}

	// check if connected to the ble server
	if (!pClient->isConnected()) {
		return false;
	}

	// get the remote service of the BLE client object based on the given UUID
	pRemoteService = pClient->getService(remoteServiceUUID);

	if (pRemoteService == nullptr) {
		ESP_LOGE(LOG_TAG, "pRemoteService is null pointer");
		pClient->disconnect();
		return false;
	}

	// get the remote characteristic of the BLE remote service object based on the given UUID
	pRemoteCharacteristic = pRemoteService->getCharacteristic(remoteCharUUID);

	if (pRemoteCharacteristic == nullptr) {
		ESP_LOGE(LOG_TAG, "pRemoteCharacteristic is null pointer");
		pClient->disconnect();
		return false;
	}

	ESP_LOGI(LOG_TAG, "Finish check service characteristics");

	return true;
}

/************************************************************************************************************************/
/*!
* @brief		connect to the server and handle all the required action with the server
* @param[in]	pDevice					pointer to the BLE device object
* @param[in]	pClient					pointer to the BLE client object
* @retval		true if all the action required has been done, false if something wrong happened in the middle of the action
*/
/************************************************************************************************************************/
bool connectToServer(BLEAdvertisedDevice* &pDevice, BLEClient* &pClient) {

	if (pDevice == nullptr) {
		return false;
	}

	// get the device mac address
	BLEAddress devAddress = pDevice->getAddress();

	ESP_LOGI(LOG_TAG, "Forming a connection to %s", devAddress.toString().c_str());

	// create client if pClient is null-pointer else memory leak could be occured
	if (pClient == nullptr) {
		ESP_LOGI(LOG_TAG, "pClient is nullptr so create client...");

		// create BLE client object
		pClient = BLEDevice::createClient();


		// set the client callbacks based on the mac address of the device
		if (devAddress.equals(HEARTYPATCH_MAC)) {
			pClient->setClientCallbacks(new HeartyPatchClientCallbacks());

		}

		else if (devAddress.equals(BMS_MAC)) {
			pClient->setClientCallbacks(new BMSClientCallbacks());

		}

		else if (devAddress.equals(CONTROLLER_MAC)) {
			pClient->setClientCallbacks(new ControllerClientCallbacks());
		}

		else if (devAddress.equals(ESP_SERVER_MAC)) {
			pClient->setClientCallbacks(new EspServerClientCallbacks());
		}

		else {
			return false;
		}

		ESP_LOGI(LOG_TAG, "Finish create client..");

	}

	if (devAddress.equals(HEARTYPATCH_MAC)) {

		// if not connected, connect to the device
		if (!isHeartyConnected) {
			// connect to the device
			isHeartyConnected = pClient->connect(pDevice);

			// give time for connection 
			delay(10);

			// if failed to connect
			if (!isHeartyConnected) {
				ESP_LOGE(LOG_TAG, "HeartyPatch remote connection failed!");
				return false;
			}
			else {
				ESP_LOGI(LOG_TAG, "HeartyPatch remote connection success!");
			}
		}

		// check the availability of the client, remote service, and the remote characteristic
		if (checkServiceCharacteristic(pClient, pHeartyPatchRemoteService, pHeartyPatchRemoteCharacteristic, HEART_RATE_SERVICE_UUID, HEART_RATE_MEASUREMENT_CHAR_UUID)) {
			if (pHeartyPatchRemoteCharacteristic->canRead()) {
				ESP_LOGI(LOG_TAG, "Read the heart rate characteristic");
				if (pClient->isConnected()) {
					// read the heart rate value
					std::string value = pHeartyPatchRemoteCharacteristic->readValue();
					
					delay(10);
					
					ESP_LOGI(LOG_TAG, "Finish read value");
					
					// assign the heart rate value to the heart rate packet
					heartRateServerPacket.tPacket.bHeartRate = value[1];

					// get the current time
					time(&rawTime);

					// save the current time into the heart rate packet
					heartRateServerPacket.tPacket.ulHeartyTime = bswap32(rawTime);

					ESP_LOGI(LOG_TAG, "Heart Rate: %d bpm (0x%.2X)\n", value[1], value[1]);

					return true;
				}
				else return false;
			}
			else return false;
		}
		else return false;
	}

	else if (devAddress.equals(BMS_MAC)) {

		// if not connected, connect to the device
		if (!isBmsConnected) {
			// connect to the device
			isBmsConnected = pClient->connect(pDevice);
			
			delay(10);
			
			// if failed to connect
			if (!isBmsConnected) {
				ESP_LOGE(LOG_TAG, "BMS remote connection failed!");
				return false;
			}

			else ESP_LOGI(LOG_TAG, "BMS remote connection success!");
		}

		// check the availability of the client, remote service, and the remote characteristic
		if (checkServiceCharacteristic(pClient, pBmsRemoteService, pBmsRemoteCharacteristic, BMS_RW_SERVICE_UUID, BMS_RX_CHAR_UUID)) {
			// notify function is needed because BMS send two packet at the same time, can't request bigger MTU size with the used BLE dongle
			if (pBmsRemoteCharacteristic->canNotify()) {
				ESP_LOGI(LOG_TAG, "Register for BMS RX notification");

				// this task delay needed for reliable register for notify callback
				delay(100);

				// register the notify callback for the given remote characteristic
				pBmsRemoteCharacteristic->registerForNotify(bmsNotifyCallback);

				ESP_LOGI(LOG_TAG, "BMS RX client notification turned on");
				// turn on the BMS RX client notification
				pBmsRemoteCharacteristic->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOn, 2, true);
				
				delay(10);

				while (isBmsConnected && !isBmsNotifyAvailable) {
					// write info status read command to the bms
					if (bmsWriteInfoStatus()) {
						// wait for the incoming packet from the notification
						delay(500);
					}
				};

				return true;
			}

			else return false;
		}

		else return false;
	}

	else if (devAddress.equals(CONTROLLER_MAC)) {
		// if not connected, connect to the device
		if (!isControllerConnected) {

			// connect to the device
			isControllerConnected = pClient->connect(pDevice);

			delay(10);

			// if failed to connect
			if (!isControllerConnected) {
				ESP_LOGE(LOG_TAG, "Controller remote connection failed!");
				return false;
			}
			else ESP_LOGI(LOG_TAG, "Controller remote connection success!");
		}

		// check the availability of the client, remote service, and the remote characteristic
		if (checkServiceCharacteristic(pClient, pControllerRemoteService, pControllerRemoteCharacteristic, CONTROLLER_RW_SERVICE_UUID, CONTROLLER_RX_CHAR_UUID)) {
			// with the used ble dongle for the motor controller, value can only be get via notify function.
			// the read function always gives null when used (tried with nRF connect app)
			if (pControllerRemoteCharacteristic->canNotify()) {
				ESP_LOGI(LOG_TAG, "Register for motor controller RX notification");

				// this task delay needed for reliable register for notify callback
				delay(100);

				// register the notification callback for motor controller data
				pControllerRemoteCharacteristic->registerForNotify(controllerNotifyCallback);

				ESP_LOGI(LOG_TAG, "Motor Controller RX client notification turned on");
				pControllerRemoteCharacteristic->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOn, 2, true);
				
				delay(10);

				// wait for incoming data from callback
				while (isControllerConnected && !isControllerNotifyAvailable) {
					delay(150);
				};

				return true;
			}
			else return false;
		}
		else return false;
	}

	else if (devAddress.equals(ESP_SERVER_MAC)) {

		// if not connected, connect to the device
		if (!isEspServerConnected) {

			// connect to the device
			isEspServerConnected = pClient->connect(pDevice);

			if (!isEspServerConnected) {
				ESP_LOGE(LOG_TAG, "ESP Server remote connection failed!");
				return false;
			}
			else {
				ESP_LOGI(LOG_TAG, "ESP Server remote connection success!");
			}
		}

		// check the availability of the client, remote service, and the remote characteristic
		if (checkServiceCharacteristic(pClient, pEspServerRemoteService, pEspServerRemoteCharacteristic, TIME_INFO_SRV_SERVICE, TIME_SET_SRV_CHAR)) {
			if (pClient->isConnected() && pEspServerRemoteCharacteristic->canRead()) {
				// check if system time need to be configured from the server
				onWritePacket.ulTimeSet = pEspServerRemoteCharacteristic->readUInt32();
				if (lastConfigTime != onWritePacket.ulTimeSet && onWritePacket.ulTimeSet != 0) {
					lastConfigTime = onWritePacket.ulTimeSet;

					ESP_LOGI(LOG_TAG, "Set ESP32 time");
					timeInfoServerPacket.tConfigCurrentTime.ulValue = onWritePacket.ulTimeSet;
					time(&rawTime);
					setupTime(onWritePacket.ulTimeSet);

					if (onWritePacket.ulTimeSet >= rawTime) {
						lock_lastTime += (onWritePacket.ulTimeSet - rawTime);
						ilockitServerPacket.tPacket.ulLockChangeTime = bswap32(bswap32(ilockitServerPacket.tPacket.ulLockChangeTime) + (onWritePacket.ulTimeSet - rawTime));
					}
					else {
						lock_lastTime -= (rawTime - timeInfoServerPacket.tConfigCurrentTime.ulValue);
						ilockitServerPacket.tPacket.ulLockChangeTime = bswap32(bswap32(ilockitServerPacket.tPacket.ulLockChangeTime) - (rawTime - onWritePacket.ulTimeSet));
					}

				}
				ESP_LOGI(LOG_TAG, "onWritePacket.ulTimeSet : %d", onWritePacket.ulTimeSet);
			}
		}
		else return false;

		// update the the related value to the server
		updateValue();

		return true;
	}

	else return false;
}

/************************************************************************************************************************/
/*!
* @brief		update the GPS info into the location packet
* @retval		none
*/
/************************************************************************************************************************/
void updateGPSInfo() {

	// check if time is valid
	if (gps.time.isValid()) {
		locationServerPacket.tPacket.ulLocationTime = bswap32(gps.time.value());
	}
	else {
		ESP_LOGE(LOG_TAG, "GPS Time is not yet valid");
	}

	// check if gps location is valid
	if (gps.location.isValid())
	{
		locationServerPacket.tPacket.fLatitude = (float)gps.location.lat();
		ESP_LOGI(LOG_TAG, "Latitude: %.6f", locationServerPacket.tPacket.fLatitude);
		locationServerPacket.tPacket.ulLatitude = bswap32(locationServerPacket.tPacket.ulLatitude);

		locationServerPacket.tPacket.fLongitude = (float)gps.location.lng();
		ESP_LOGI(LOG_TAG, "Longitude: %.6f", locationServerPacket.tPacket.fLongitude);
		locationServerPacket.tPacket.ulLongitude = bswap32(locationServerPacket.tPacket.ulLongitude);

	}
	else {
		ESP_LOGE(LOG_TAG, "GPS Location is not yet valid");
	}

	// check if gps altitude is valid
	if (gps.altitude.isValid()) {
		locationServerPacket.tPacket.fElevation = (float)gps.altitude.meters();
		ESP_LOGI(LOG_TAG, "Altitude: %.6f m", locationServerPacket.tPacket.fElevation);
		locationServerPacket.tPacket.ulElevation = bswap32(locationServerPacket.tPacket.ulElevation);
	}
	else {
		ESP_LOGE(LOG_TAG, "GPS Altitude is not yet valid");
	}
}

/************************************************************************************************************************/
/*!
* @brief		send heart rate info packet to the BLE server
* @retval		true if success, false if failed
*/
/************************************************************************************************************************/
bool sendHeartRatePacket() {
	ESP_LOGI(LOG_TAG, "Send heart rate packet");
	if (isEspServerConnected) {
		if (checkServiceCharacteristic(pEspServerClient, pEspServerRemoteService, pEspServerRemoteCharacteristic, HEART_RATE_SRV_SERVICE, HEART_RATE_SRV_CHAR)) {
			ESP_LOGI(LOG_TAG, "Check true");
			if (pEspServerRemoteCharacteristic->canWrite()) {
				ESP_LOGI(LOG_TAG, "Heart rate can write");
				pEspServerRemoteCharacteristic->writeValue(heartRateServerPacket.abPacket, sizeof(heartRateServerPacket.tPacket), false);
				delay(10);
				ESP_LOGI(LOG_TAG, "Send heart rate packet success");
				return true;
			}
		}
		else return false;
	}
	return false;
}

/************************************************************************************************************************/
/*!
* @brief		send bms motor info packet to the BLE server
* @retval		true if success, false if failed
*/
/************************************************************************************************************************/
bool sendBmsMotorPacket() {
	if (isEspServerConnected) {
		if (checkServiceCharacteristic(pEspServerClient, pEspServerRemoteService, pEspServerRemoteCharacteristic, BMS_MOTOR_SRV_SERVICE, BMS_MOTOR_SRV_CHAR)) {
			if (pEspServerRemoteCharacteristic->canWrite()) {
				pEspServerRemoteCharacteristic->writeValue(bmsMotorServerPacket.abPacket, sizeof(bmsMotorServerPacket.tPacket), false);
				delay(10);
				return true;
			}
		}
		else return false;
	}
	return false;
}

/************************************************************************************************************************/
/*!
* @brief		send location info packet to the BLE server
* @retval		true if success, false if failed
*/
/************************************************************************************************************************/
bool sendLocationInfoPacket() {
	if (isEspServerConnected) {
		ESP_LOGI(LOG_TAG, "Send location info packet");
		if (checkServiceCharacteristic(pEspServerClient, pEspServerRemoteService, pEspServerRemoteCharacteristic, LOCATION_SRV_SERVICE, LOCATION_SRV_CHAR)) {
			if (pEspServerRemoteCharacteristic->canWrite()) {
				ESP_LOGI(LOG_TAG, "Write location info packet into char");
				pEspServerRemoteCharacteristic->writeValue(locationServerPacket.abPacket, sizeof(locationServerPacket.tPacket), false);
				delay(10);
				ESP_LOGI(LOG_TAG, "Write success");
				return true;
			}
		}
		else return false;
	}
	return false;
}

/************************************************************************************************************************/
/*!
* @brief		send lora info packet to the BLE server
* @retval		true if success, false if failed
*/
/************************************************************************************************************************/
bool sendLoraInfoPacket() {
	if (isEspServerConnected) {
		if (checkServiceCharacteristic(pEspServerClient, pEspServerRemoteService, pEspServerRemoteCharacteristic, TIME_INFO_SRV_SERVICE, TIME_LORA_INFO_SRV_CHAR)) {
			if (pEspServerRemoteCharacteristic->canWrite()) {
				ESP_LOGI(LOG_TAG, "Write value to lora info char");
				pEspServerRemoteCharacteristic->writeValue(timeInfoServerPacket.tLoraLastSendPackageTime.abValue, sizeof(timeInfoServerPacket.tLoraLastSendPackageTime.abValue), false);
				delay(10);
				ESP_LOGI(LOG_TAG, "Write success");
				return true;
			}
			else ESP_LOGI(LOG_TAG, "Can't write value to char");
		}
		else return false;
	}
	return false;
}

/************************************************************************************************************************/
/*!
* @brief		send current time packet to the BLE server
* @retval		true if success, false if failed
*/
/************************************************************************************************************************/
bool sendEspCurrentTimePacket() {
	// check if server still connected
	if (isEspServerConnected) {
		ESP_LOGI(LOG_TAG, "Update ESP32 current time on the server");
		time(&rawTime);
		timeInfoServerPacket.tEspCurrentTime.ulValue = bswap32((uint32_t)rawTime);
		if (checkServiceCharacteristic(pEspServerClient, pEspServerRemoteService, pEspServerRemoteCharacteristic, TIME_INFO_SRV_SERVICE, TIME_ESPTIME_SRV_CHAR)) {
			if (pEspServerRemoteCharacteristic->canWrite()) {
				pEspServerRemoteCharacteristic->writeValue(timeInfoServerPacket.tEspCurrentTime.abValue, sizeof(timeInfoServerPacket.tEspCurrentTime.abValue), false);
				delay(10);
				return true;
			}
			else ESP_LOGE(LOG_TAG, "Current time update failed");
		}
		else return false;
	}
	return false;
}

/************************************************************************************************************************/
/*!
* @brief		send mpu crash packet to the BLE server
* @retval		true if success, false if failed
*/
/************************************************************************************************************************/
bool sendMpuCrashPacket() {
	if (isEspServerConnected) {
		if (checkServiceCharacteristic(pEspServerClient, pEspServerRemoteService, pEspServerRemoteCharacteristic, MPU_SRV_SERVICE, MPU_SRV_CHAR)) {
			if (pEspServerRemoteCharacteristic->canWrite()) {
				ESP_LOGI(LOG_TAG, "Write MPU crash packet to characteristic");
				pEspServerRemoteCharacteristic->writeValue(mpuServerPacket.abPacket, sizeof(mpuServerPacket.abPacket), false);
				delay(10);
				ESP_LOGI(LOG_TAG, "Write success");
				return true;
			}
		}
		else return false;
	}
	return false;
}

/************************************************************************************************************************/
/*!
* @brief		update all the related value to the BLE server
* @retval		true if success, false if failed
*/
/************************************************************************************************************************/
bool updateValue() {
	ESP_LOGI(LOG_TAG, "Update all value");

	if (isSessionTimeCountEnable) {
		ESP_LOGI(LOG_TAG, "Session time count is on enable");
		time(&lock_nowTime);
		lock_difftime = difftime(lock_nowTime, lock_lastTime);
		lock_diffTimeInMinutes = (uint16_t)lock_difftime / 60;
		ESP_LOGI(LOG_TAG, "Session time : %.0f seconds", lock_difftime);
		ilockitServerPacket.tPacket.usLockSessionTime = bswap16(lock_diffTimeInMinutes);
	}

	// send heart rate packet to the server
	sendHeartRatePacket();

	// if there is any new data from bms or controller
	if (isBmsNotifyAvailable || isControllerNotifyAvailable) {
		ESP_LOGI(LOG_TAG, "Update bms and controller packet on the server");

		// send the bms motor packet to the server
		if (sendBmsMotorPacket()) {
			ESP_LOGI(LOG_TAG, "Bms/Controller packet sent!");
			if (isBmsNotifyAvailable) isBmsNotifyAvailable = false;
			if (isControllerNotifyAvailable) isControllerNotifyAvailable = false;
		}
		else ESP_LOGE(LOG_TAG, "Bms/Controller packet failed to sent!");
	}

	//Check to see if new GPS info is available
	if (isGpsConnected) {
		if (gps.time.isUpdated())
		{
			ESP_LOGI(LOG_TAG, "Update GPS packet on the server");
			updateGPSInfo();
			if (sendLocationInfoPacket()) ESP_LOGI(LOG_TAG, "GPS packet sent!");
			else ESP_LOGE(LOG_TAG, "GPS packet failed to sent!");
		}
	}

	// if there is any lora packet sent
	if (isLoraPacketSent) {
		ESP_LOGI(LOG_TAG, "Update Lora info packet on the server");
		if (sendLoraInfoPacket()) {
			ESP_LOGI(LOG_TAG, "Lora info packet sent!");
			isLoraPacketSent = false;
		}
		else ESP_LOGE(LOG_TAG, "Lora info packet failed to sent!");
	}

	// send the esp current time packet
	sendEspCurrentTimePacket();

	if (isCrashDetected) {
		ESP_LOGI(LOG_TAG, "Update MPU crash packet on the server");
		if (sendMpuCrashPacket()) {
			ESP_LOGI(LOG_TAG, "MPU crash packet sent!");
			isCrashDetected = false;
		}
		else ESP_LOGE(LOG_TAG, "MPU crash packet failed to sent!");
		delay(10);
	}

	return true;
}

/************************************************************************************************************************/
/*!
* @brief		check BLE connection to determined if every device needed has been found
* @retval		none
*/
/************************************************************************************************************************/
void checkBLEConnection() {
	ESP_LOGI(LOG_TAG, "heartyPatch connected : %d, heartyPatchFound : %d", isHeartyConnected, isHeartyFound);
	ESP_LOGI(LOG_TAG, "bms connected : %d, isBmsFound: %d", isBmsConnected, isBmsFound);
	ESP_LOGI(LOG_TAG, "controller connected : %d, isControllerFound: %d", isControllerConnected, isControllerFound);
	ESP_LOGI(LOG_TAG, "esp server connected : %d, isEspServerFound: %d", isEspServerConnected, isEspServerFound);
	ESP_LOGI(LOG_TAG, "isLoraSessionKeyAvailable: %d", isLoraSessionKeyAvailable);
	if (!isHeartyFound)		ESP_LOGE(LOG_TAG, "Rescan needed for HEARTYPATCH");
	if (!isBmsFound)		ESP_LOGE(LOG_TAG, "Rescan needed for BMS");
	if (!isControllerFound)	ESP_LOGE(LOG_TAG, "Rescan needed for Mot. Controller");
	if (!isEspServerFound)	ESP_LOGE(LOG_TAG, "Rescan needed for ESP Server");
	if (!isEspServerFound || !isHeartyFound || !isBmsFound || !isControllerFound) {
		ESP_LOGI(LOG_TAG, "Scan start");
		pScan->start(2, true);
	}
}

/************************************************************************************************************************/
/*!
* @brief		write read info status command to the BMS
* @retval		true if success, false if failed
*/
/************************************************************************************************************************/
bool bmsWriteInfoStatus() {

	ESP_LOGI(LOG_TAG, "Request BMS info status data");

	BMS_PACKET_STRUCT_T wSerialPacket = { 0 };

	/** set the serial packet to be sent */
	uint8_t packetSize = bms.setSerialPacket(&wSerialPacket, BMSRegister::BMS_READ_REG, BMSRegister::BMS_REG_INFO_STATUS, 0, NULL);

	uint8_t abSendPacket[50] = { 0 };

	memcpy(abSendPacket, &wSerialPacket, packetSize);

	if (checkServiceCharacteristic(pBmsClient, pBmsRemoteService, pBmsRemoteCharacteristic, BMS_RW_SERVICE_UUID, BMS_TX_CHAR_UUID)) {
		if (pBmsRemoteCharacteristic->canWrite()) {
			ESP_LOGI(LOG_TAG, "Sending request to BMS!");
			pBmsRemoteCharacteristic->writeValue(abSendPacket, packetSize);
			delay(10);
			ESP_LOGI(LOG_TAG, "Request BMS info status data sent!");
			return true;
		}
		else return false;
	}
	return false;
}

void setup() {

	// configure the serial port
	Serial.begin(115200);

	// wait for the serial port to be open
	//while (!Serial);


	// setup the system time with epoch time of 0
	setupTime(0);

	// setup the GPS
	setupGPS();

	// setup the IMU
	setupIMU();

	// setup the display
	setupDisplay();


	// assign the semaphore for the mutex
	xSemaphoreI2c = xSemaphoreCreateMutex();
	xSemaphoreSpi = xSemaphoreCreateMutex();
	xSemaphoreBLE = xSemaphoreCreateMutex();

	// initialise the BLE controller
	BLEDevice::init("ZESYS_BB");

	// setup the BLE scanner
	setupBLEScanner();

	// create the FreeRTOS event group
	xWatchdogEvent = xEventGroupCreate();

	// create and start the ttn task (spi task) on core 1 with priority 2
	xTaskCreatePinnedToCore(ttnTask, "ttnTask", 4096, (void*)1, 2, &xTaskTtn, 1);

	// create and start i2c task on core 1 with priority 1
	xTaskCreatePinnedToCore(i2cTask, "i2cTask", 4096, (void*)1, 1, &xTaskI2c, 1);

	lastLoopTime = millis();

	// create and start the main task on core 1 with priority 1
	xTaskCreatePinnedToCore(mainTask, "mainTask", 81920, (void*)1, 1, &xTaskMain, 1);

	// create and start the watchdog task on core 1 with priority 3
	xTaskCreatePinnedToCore(wdTask, "wdTask", 4096, (void*)1, 3, &xTaskWatchdog, 1);

}

/************************************************************************************************************************/
/*!
* @brief		watchdog task to handle all task
*/
/************************************************************************************************************************/
void wdTask(void * parameter) {
	Serial.printf( "Start watchdog task...\n");

	for (;;) {

		// read bits within an RTOS event group every 20000 to check if all task still responsive
		wd_result = xEventGroupWaitBits(	xWatchdogEvent,
											allTaskId,
											pdTRUE,
											pdTRUE,
											20000 / portTICK_RATE_MS);

		Serial.printf( "System free heap : %d\n", ESP.getFreeHeap());

		if ((wd_result & allTaskId) == allTaskId) {
			Serial.printf( "System is healthy..\n");
		}
		else {
			if (!(wd_result & mainTaskId)) {
				Serial.printf( "mainTask stopped responding..restart task\n");
				isMainTaskStopResponding = true;
			}
			if (!(wd_result & ttnTaskId)) {
				Serial.printf( "ttnTask stopped responding..restart task\n");
				isTtnTaskStopResponding = true;
			}
			if (!(wd_result & i2cTaskId)) {
				Serial.printf( "i2cTask stopped responding..restart task\n");
				isI2cTaskStopResponding = true;
			}
		}

		delay(1);

	}
}

/************************************************************************************************************************/
/*!
* @brief		ttn task (or spi task), handle all task required for lora and display (both use SPI HW resource)
*/
/************************************************************************************************************************/
void ttnTask(void * parameter) {
	
	ESP_LOGI(LOG_TAG, "Start TTN task...");

	// LMIC init
	os_init();
	// Reset the MAC state. Session and pending data transfers will be discarded.
	LMIC_reset();

	// Start job (sending automatically starts OTAA too)
	do_send(&sendjob);

	xSemaphoreGive(xSemaphoreSpi);

	ttnGetKeyTime = millis();
	displayTaskTime = millis();
	for (;;) {

		if (xSemaphoreTake(xSemaphoreSpi, 10 / portTICK_RATE_MS) == pdTRUE) {
			os_runloop_once();
			
			if (millis() - displayTaskTime > 500) {
				updateDisplay();
				displayTaskTime = millis();
			}

			if (isLoraSessionKeyAvailable || (millis() - ttnGetKeyTime > 10000)) {
				if (!isLoraTaskSet) {
					ESP_LOGI(LOG_TAG, "Set ttn task priority to 1");
					vTaskPrioritySet(xTaskTtn, 1);
					isLoraTaskSet = true;
				}

				// set bits to alert watchdog that the task still responsive
				xEventGroupSetBits(xWatchdogEvent, ttnTaskId);

				ttnGetKeyTime = millis();

				xSemaphoreGive(xSemaphoreSpi);

				delay(1);

			}

			else {
				// set bits to alert watchdog that the task still responsive
				xEventGroupSetBits(xWatchdogEvent, ttnTaskId);
				xSemaphoreGive(xSemaphoreSpi);
			}
		}
	}
}

/************************************************************************************************************************/
/*!
* @brief		i2c task, handle all task required for imu and gps (both use I2C HW resource)
*/
/************************************************************************************************************************/
void i2cTask(void * parameter) {
	ESP_LOGI(LOG_TAG, "Start I2C task...");

	for (;;) {

		if (xSemaphoreTake(xSemaphoreI2c, 10 / portTICK_RATE_MS) == pdTRUE) {
			if (isImuConnected) {
				// check if there is any crash happened
				if (IMU.detector() == true) {
					afImpact = IMU.getLastImpact();
					ESP_LOGW(LOG_TAG, "Impact detected!");
					ESP_LOGW(LOG_TAG, "[Impact]: G-Force=%f, AbsAccel=%f , AbsGyro=%f", afImpact[0], afImpact[1], afImpact[2]);
					time(&rawTime);
					mpuServerPacket.tPacket.ulCrashTime = bswap32((uint32_t)rawTime);
					mpuServerPacket.tPacket.bCrashDetect = 0x01;
					mpuServerPacket.tPacket.sbDetectedForced = (int8_t)(afImpact[0] * 10.0);
					isCrashDetected = true;
				}
				else {
					//ESP_LOGI(LOG_TAG, "[Values]: G-Force=%f, AbsAccel=%f, AbsGyro=%f", IMU.getCurrentValues()[0], IMU.getCurrentValues()[1], IMU.getCurrentValues()[2]);
				}
			}

			if (isGpsConnected) {
				// check if any gps data available
				if (L76.available()) {
					ESP_LOGI(LOG_TAG, "Check and encode gps data if available..");

					while (L76.available()) {
						gps.encode(L76.read()); //Feed the GPS parser
					}
				}
			}

			xSemaphoreGive(xSemaphoreI2c);

			// set bits to alert watchdog that the task still responsive
			xEventGroupSetBits(xWatchdogEvent, i2cTaskId);

		}


		delay(1);
	}
}

void mainTask(void * parameter) {
	ESP_LOGI(LOG_TAG, "Start main task...");

	for (;;) {

		ESP_LOGI(LOG_TAG, "Free Heap: %d", ESP.getFreeHeap());
		ESP_LOGI(LOG_TAG, "System loop time: %f s", (millis() - lastLoopTime) / 1000.0);

		loopStartTime = millis();


		// connect to the BMS and get the data
		if (!isBmsConnected && isBmsFound) {
			while (!isBmsConnected && connectCounter < 5) {
				ESP_LOGI(LOG_TAG, "isBmsConnected: %d, cntr: %d", isBmsConnected, connectCounter);
				isBmsConnected = connectToServer(pBmsDevice, pBmsClient);
				connectCounter++;
				delay(100);
			}
			connectCounter = 0;

			if (isBmsConnected) {
				// disconnect from bms
				pBmsClient->disconnect();

				// wait until the disconnection is finished
				while (isBmsConnected) {
					delay(1);
				};
				
				pBmsClient = nullptr;
				delay(250);
			}
		}

		bmsTaskTime = millis();

		// connect to the heartypatch and get the heart rate data
		if (!isHeartyConnected && isHeartyFound) {
			while (!isHeartyConnected && connectCounter < 5) {
				ESP_LOGI(LOG_TAG, "isHeartyConnected: %d, cntr: %d", isHeartyConnected, connectCounter);
				isHeartyConnected = connectToServer(pHeartyPatchDevice, pHeartyPatchClient);
				connectCounter++;
				delay(100);
			}

			connectCounter = 0;

			if (isHeartyConnected) {
				// disconnect from hearty patch
				pHeartyPatchClient->disconnect();

				// wait until the disconnection is finished
				while (isHeartyConnected) {
					delay(1);
				};

				pHeartyPatchClient = nullptr;
				delay(250);

			}
		}

		heartyTaskTime = millis();

		// connect to motor controller and get the motor controller data
		if (!isControllerConnected && isControllerFound) {
			while (!isControllerConnected && connectCounter < 5) {
				ESP_LOGI(LOG_TAG, "isControllerConnected: %d, cntr: %d", isControllerConnected, connectCounter);
				isControllerConnected = connectToServer(pControllerDevice, pControllerClient);
				connectCounter++;
				delay(100);
			}

			connectCounter = 0;

			if (isControllerConnected) {

				// disconnect from motor controller
				pControllerClient->disconnect();

				// wait until the disconnection is finished
				while (isControllerConnected) {
					delay(1);
				};
				pControllerClient = nullptr;
				delay(250);
			}
		}

		controllerTaskTime = millis();

		// connect to esp server and then get and write the related data from/to the server characteristics 
		if (!isEspServerConnected && isEspServerFound) {
			while (!isEspServerConnected && connectCounter < 5) {
				ESP_LOGI(LOG_TAG, "isEspServerConnected: %d, cntr: %d", isEspServerConnected, connectCounter);
				isEspServerConnected = connectToServer(pEspServerDevice, pEspServerClient);
				connectCounter++;
				delay(100);
			}
			connectCounter = 0;

			if (isEspServerConnected) {

				// disconnect from esp server
				pEspServerClient->disconnect();

				// wait until the disconnection is finished
				while (isEspServerConnected) {
					delay(1);
				};

				pEspServerClient = nullptr;
				delay(250);
			}
		}

		serverTaskTime = millis();

		loopFinishTime = millis();
		ESP_LOGI(LOG_TAG, "loopStartTime: %d\n", loopStartTime);
		ESP_LOGI(LOG_TAG, "loopFinishTime : %d\n", loopFinishTime);
		ESP_LOGI(LOG_TAG, "diff loop time : %d\n", loopFinishTime - loopStartTime);
		ESP_LOGI(LOG_TAG, "bmsTask time : %d\n", bmsTaskTime - loopStartTime);
		ESP_LOGI(LOG_TAG, "heartyTask time : %d\n", heartyTaskTime - bmsTaskTime);
		ESP_LOGI(LOG_TAG, "controllerTask time : %d\n", controllerTaskTime - heartyTaskTime);
		ESP_LOGI(LOG_TAG, "serverTask time : %d\n", serverTaskTime - controllerTaskTime);

		// check BLE connection to scan any missing devices
		checkBLEConnection();

		// set bits to alert watchdog that the task still responsive
		xEventGroupSetBits(xWatchdogEvent, mainTaskId);
	}
}

void loop() {

	// if the main task stop responding
	if (isMainTaskStopResponding) {
		isMainTaskStopResponding = false;

		Serial.printf("Delete all BLE resources..\n");

		esp_ble_gattc_app_unregister(3);

		// Set all client to nullptr so that new client can be created
		pIlockitClient = nullptr;
		pHeartyPatchClient = nullptr;
		pBmsClient = nullptr;
		pControllerClient = nullptr;
		pEspServerClient = nullptr;

		// wait until every device safely disconnected
		delay(2000);

		// delete the main task
		vTaskDelete(xTaskMain);

		// delay to make sure the task is safely deleted
		delay(1000);

		// create the new main task
		xTaskCreatePinnedToCore(mainTask, "mainTask", 81920, (void*)1, 1, &xTaskMain, 1);
	}

	// if TTN task stop responding
	if (isTtnTaskStopResponding) {
		isTtnTaskStopResponding = false;

		// release the semaphore for spi related functions (on TTN ask)
		xSemaphoreGive(xSemaphoreSpi);

		// delete the TTN task
		vTaskDelete(xTaskTtn);

		// delay to make sure the task is safely deleted
		delay(1000);

		// create the new TTN task
		xTaskCreatePinnedToCore(ttnTask, "ttnTask", 4096, (void*)1, 2, &xTaskTtn, 1);
	}

	// if the i2c task stop responding
	if (isI2cTaskStopResponding) {
		isI2cTaskStopResponding = false;

		// release the semaphore for i2c related functions
		xSemaphoreGive(xSemaphoreI2c);

		// delete the i2c task
		vTaskDelete(xTaskI2c);

		// delay to make sure the task is safely deleted
		delay(1000);

		// create the new i2c task
		xTaskCreatePinnedToCore(i2cTask, "i2cTask", 4096, (void*)1, 1, &xTaskI2c, 1);
	}
}


