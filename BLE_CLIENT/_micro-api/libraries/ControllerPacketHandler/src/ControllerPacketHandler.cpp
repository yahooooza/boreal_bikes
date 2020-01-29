/************************************************************************************************************************/
/*!
* @copyright	Zentrum zur Foerderung eingebetteter Systeme e.V.
* @author		zzulfifli
* @file			Controller_PacketHandler.cpp
* @date			11.02.2019
* @version		1.0
* @brief		Controller packet handler library program file
* @details
*
* Changes:
*	Date       | Description
*	-----------|------------------------------------------------------------------------
*	2019-02-11 | initial version
*
* @note
*
* @warning
*
*/
/************************************************************************************************************************/

#if defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#define LOG_TAG ""
#else
#include "esp_log.h"
static const char* LOG_TAG = "BLEDevice";
#endif

#include "ControllerPacketHandler.h"
#include "byteswap.h"


ControllerPacketHandler::ControllerPacketHandler()
{
	blPacket2Parse = false;

}

ControllerPacketHandler::ControllerPacketHandler(HardwareSerial *port)
{
	_serPort = port;
	blPacket2Parse = false;

}

ControllerPacketHandler::~ControllerPacketHandler()
{

}

CONTROLLER_ERROR_E ControllerPacketHandler::readSerialPacket(CONTROLLER_PACKET_STRUCT_T * tSerialReadPacket)
{
	uint8_t temp = 0, index = 0;
	bool readSuffix = false;

	SERIAL_CONTROLLER_PROCESS_STATE_t processState = SP_STATE_READ_HEADER;
	CONTROLLER_ERROR_E bRet = ERR_CONTROLLER_OK;

	if (!_serPort->available()) return ERR_CONTROLLER_NO_DATA_AVAIL;
	else {
		while (_serPort->available()) {
			temp = _serPort->read();
			//ESP_LOGV(LOG_TAG, "%d ", temp);

			switch (processState) {
			case SP_STATE_READ_HEADER: {
				if (temp == controllerHeader) {									/** header is found */
					tSerialReadPacket->bHeader = temp;
					processState = SP_STATE_READ_PAYLOAD;
				}
				break;
			}
			case SP_STATE_READ_PAYLOAD: {
				tSerialReadPacket->abData[index++] = temp;
				if (index == 10){
					processState = SP_STATE_READ_SUFFIX;
				}				/** read the command ID register */
				
				break;
			}
			case SP_STATE_READ_SUFFIX: {
				tSerialReadPacket->abData[index] = temp;					/** save the received data (suffix) into read packet */
				if (temp == controllerSuffix) {
					readSuffix = true;
					//ESP_LOGV(LOG_TAG, "Suffix found!");
				}
				else bRet = ERR_CONTROLLER_SUFFIX;										/** else return the suffix error */
				break;
			}
			}

			if (ERR_CONTROLLER_OK != bRet) break;
		}
	}

	if (!readSuffix) return ERR_CONTROLLER_SHORT_DATA;
	else return bRet;
}

CONTROLLER_ERROR_E ControllerPacketHandler::readSerialPacket(CONTROLLER_PACKET_STRUCT_T * tSerialReadPacket, uint8_t *data, uint8_t len)
{
	uint8_t temp = 0, index = 0, idx = 0;
	bool readSuffix = false;
	uint8_t dataSize = len;
	SERIAL_CONTROLLER_PROCESS_STATE_t processState = SP_STATE_READ_HEADER;
	CONTROLLER_ERROR_E bRet = ERR_CONTROLLER_OK;

	//ESP_LOGV(LOG_TAG, "dataSize: %d", dataSize);

	if (!dataSize) return ERR_CONTROLLER_NO_DATA_AVAIL;
	else {
		while (dataSize--) {

			temp = (uint8_t)data[idx++];
			//ESP_LOGV(LOG_TAG, "data[%d]: 0x%02X", idx - 1, temp);

			switch (processState) {
			case SP_STATE_READ_HEADER: {
				if (temp == controllerHeader) {								/** header is found */
					tSerialReadPacket->bHeader = temp;
					processState = SP_STATE_READ_PAYLOAD;
				}
				break;
			}
			case SP_STATE_READ_PAYLOAD: {
				tSerialReadPacket->abData[index++] = temp;
				if (index == 10){
					processState = SP_STATE_READ_SUFFIX;
				}				/** read the command ID register */
				
				break;
			}
			case SP_STATE_READ_SUFFIX: {
				tSerialReadPacket->abData[index] = temp;					/** save the received data (suffix) into read packet */
				if (temp == controllerSuffix) {
					readSuffix = true;
					//ESP_LOGV(LOG_TAG, "Suffix found!");
				}
				else bRet = ERR_CONTROLLER_SUFFIX;										/** else return the suffix error */
				break;
			}
			}

			if (ERR_CONTROLLER_OK != bRet) break;
		}
	}

	if (!readSuffix) return ERR_CONTROLLER_SHORT_DATA;
	else return bRet;
}

CONTROLLER_ERROR_E ControllerPacketHandler::readSerialPacket(CONTROLLER_PACKET_STRUCT_T * tSerialReadPacket, std::string data)
{
	uint8_t temp = 0, index = 0, idx = 0;
	bool readSuffix = false;
	uint8_t dataSize = data.length();
	SERIAL_CONTROLLER_PROCESS_STATE_t processState = SP_STATE_READ_HEADER;
	CONTROLLER_ERROR_E bRet = ERR_CONTROLLER_OK;

	//ESP_LOGV(LOG_TAG, "dataSize: 0x%02X", dataSize);

	if (!dataSize) return ERR_CONTROLLER_NO_DATA_AVAIL;
	else {
		while (dataSize--) {

			temp = (uint8_t)data[idx++];
			//ESP_LOGV(LOG_TAG, "data[%d]: 0x%02X", idx - 1, temp);

			switch (processState) {
			case SP_STATE_READ_HEADER: {
				if (temp == controllerHeader) {									/** header is found */
					tSerialReadPacket->bHeader = temp;
					processState = SP_STATE_READ_PAYLOAD;
				}
				break;
			}
			case SP_STATE_READ_PAYLOAD: {
				tSerialReadPacket->abData[index++] = temp;
				if (index == 10){
					processState = SP_STATE_READ_SUFFIX;
				}				/** read the command ID register */
				
				break;
			}
			case SP_STATE_READ_SUFFIX: {
				tSerialReadPacket->abData[index] = temp;					/** save the received data (suffix) into read packet */
				if (temp == controllerSuffix) {
					readSuffix = true;
					//ESP_LOGV(LOG_TAG, "Suffix found!");
				}
				else bRet = ERR_CONTROLLER_SUFFIX;										/** else return the suffix error */
				break;
			}
			}

			if (ERR_CONTROLLER_OK != bRet) break;
		}
	}

	if (!readSuffix) return ERR_CONTROLLER_SHORT_DATA;
	else return bRet;
}

void ControllerPacketHandler::readParsePacket(uint8_t *pData, uint8_t len)
{
	CONTROLLER_PACKET_STRUCT_T rSerialPacket = { 0 };
	CONTROLLER_ERROR_E bRet = ERR_CONTROLLER_OK;

	if (blPacket2Parse) {

		uint8_t totalLength = lastPacketLength + len;

		for (uint8_t i = 0; i < len; i++) {
			abControllerReadPacket[lastPacketLength + i] = pData[i];
		}

		if (ERR_CONTROLLER_OK != (bRet = readSerialPacket(&rSerialPacket, abControllerReadPacket, totalLength))) {
			//ESP_LOGV(LOG_TAG, "bRet: %d", bRet);
		}

		//ESP_LOGV(LOG_TAG, "Total voltage: %f", (rSerialPacket.tPacket.bTotalVoltage / 3.7));
		
		blPacket2Parse = false;
	}

	else {
		if (ERR_CONTROLLER_OK != (bRet = readSerialPacket(&rSerialPacket, pData, len))) {
			//ESP_LOGV(LOG_TAG, "bRet: %d", bRet);

			if (bRet == ERR_CONTROLLER_SHORT_DATA) {
				memcpy(abControllerReadPacket, &rSerialPacket, len);
				lastPacketLength = len;
				blPacket2Parse = true;
			}
		}
		else {
			//ESP_LOGV(LOG_TAG, "Total voltage: %f V", (rSerialPacket.tPacket.bTotalVoltage / 3.7));
		}
	}
}

bool ControllerPacketHandler::readParsePacket(CONTROLLER_PACKET_STRUCT_T *readPacket, uint8_t * pData, uint8_t len)
{
	CONTROLLER_ERROR_E bRet = ERR_CONTROLLER_OK;

	if (blPacket2Parse) {

		blPacket2Parse = false;

		uint8_t totalLength = lastPacketLength + len;

		for (uint8_t i = 0; i < len; i++) {
			abControllerReadPacket[lastPacketLength + i] = pData[i];
		}

		if (ERR_CONTROLLER_OK != (bRet = readSerialPacket(readPacket, abControllerReadPacket, totalLength))) {
			//ESP_LOGV(LOG_TAG, "bRet: %d", bRet);
			return false;
		}
		else return true;
	}

	else {
		if (ERR_CONTROLLER_OK != (bRet = readSerialPacket(readPacket, pData, len))) {
			//ESP_LOGV(LOG_TAG, "bRet: %d", bRet);

			if (bRet == ERR_CONTROLLER_SHORT_DATA) {
				memcpy(abControllerReadPacket, readPacket, len);
				lastPacketLength = len;
				blPacket2Parse = true;
			}

			return false;
		}
		else return true;
	}
}
