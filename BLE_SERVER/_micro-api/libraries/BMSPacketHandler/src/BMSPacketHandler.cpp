/************************************************************************************************************************/
/*!
* @copyright	Zentrum zur Foerderung eingebetteter Systeme e.V.
* @author		zzulkifli
* @file			BMS_PacketHandler.cpp
* @date			11.02.2019
* @version		1.0
* @brief		BMS packet handler library program file
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

#include "BMSPacketHandler.h"
#include "byteswap.h"


BMSPacketHandler::BMSPacketHandler()
{
	blPacket2Parse = false;

}

BMSPacketHandler::BMSPacketHandler(HardwareSerial *port)
{
	_serPort = port;
	blPacket2Parse = false;

}

BMSPacketHandler::~BMSPacketHandler()
{

}

uint8_t BMSPacketHandler::setSerialPacket(BMS_PACKET_STRUCT_T * tSerialWritePacket, uint8_t mode, uint8_t cmdID, uint8_t len, uint8_t * val)
{
	BMS_CHECKSUM_PACKET_T tCheckSum = { 0 };

	tSerialWritePacket->bHeader = 0xDD;
	tSerialWritePacket->tRequestMode.bMode = mode;
	tSerialWritePacket->tRequestMode.bCmdID = cmdID;
	tSerialWritePacket->bLength = len;

	tCheckSum.checkSum = len + cmdID;
	
	for (uint8_t i = 0; i < len; i++) {
		tSerialWritePacket->abData[i] = val[i];
		tCheckSum.checkSum += val[i];
	}
	
	tCheckSum.checkSum = ~tCheckSum.checkSum + 1;
	tSerialWritePacket->abData[len] = tCheckSum.abCheckSum[1];
	tSerialWritePacket->abData[len+1] = tCheckSum.abCheckSum[0];
	tSerialWritePacket->abData[len+2] = 0x77;

	return (uint8_t)(sizeof(tSerialWritePacket->bHeader) + sizeof(tSerialWritePacket->tRequestMode) + sizeof(tSerialWritePacket->bLength) + len + sizeof(tCheckSum) + 1);
}

void BMSPacketHandler::sendSerialPacket(BMS_PACKET_STRUCT_T * tSerialWritePacket, uint8_t len)
{
	uint8_t abSendPacket[30] = { 0 };

	memcpy(abSendPacket, tSerialWritePacket, len);
	
	for (int i = 0; i < len; i++) {
		_serPort->write(abSendPacket[i]);
	}
}

BMS_ERROR_E BMSPacketHandler::readSerialPacket(BMS_PACKET_STRUCT_T * tSerialReadPacket)
{
	uint8_t temp = 0, index = 0;
	bool readSuffix = false;

	SERIAL_BMS_PROCESS_STATE_t processState = SP_STATE_READ_HEADER;
	BMS_CHECKSUM_PACKET_T tCheckSum = { 0 };
	BMS_ERROR_E bRet = ERR_BMS_OK;

	if (!_serPort->available()) return ERR_BMS_NO_DATA_AVAIL;
	else {
		while (_serPort->available()) {
			temp = _serPort->read();
			//ESP_LOGD(LOG_TAG, "%d ", temp);

			switch (processState) {
			case SP_STATE_READ_HEADER: {
				if (temp == bmsHeader) {									/** header is found */
					tSerialReadPacket->bHeader = temp;
					processState = SP_STATE_READ_REGISTER;
				}
				break;
			}
			case SP_STATE_READ_REGISTER: {
				tSerialReadPacket->tResponseMode.bCmdID = temp;				/** read the command ID register */
				processState = SP_STATE_CHECK_ERROR;
				break;
			}
			case SP_STATE_CHECK_ERROR: {
				tSerialReadPacket->tResponseMode.bStatus = temp;			/** read the error status */
				if (!temp) processState = SP_STATE_READ_PAYLOAD_LENGTH;		/** if no error */
				else {
					bRet = BMS_ERROR_E(temp);								/** if error occured, exit the while condition */
				}
				break;
			}
			case SP_STATE_READ_PAYLOAD_LENGTH: {
				tSerialReadPacket->bLength = temp;							/** read the payload length */
				tCheckSum.checkSum = temp;									/** transfer the data to checksum buffer */
				if (!temp) {												/** if payload length is 0 */
					tCheckSum.checkSum = ~tCheckSum.checkSum + 1;			/** calculate the checksum */
					processState = SP_STATE_MSB_CHECKSUM;					/** go directly to check checksum state */
				}
				else processState = SP_STATE_READ_PAYLOAD;					/** else go to read payload state */
				break;
			}
			case SP_STATE_READ_PAYLOAD: {
				tSerialReadPacket->abData[index++] = temp;					/** save the data into payload buffer */
				tCheckSum.checkSum += temp;									/** accumulate the data into checksum buffer */
				if (index == tSerialReadPacket->bLength) {					/** when all payload has been read */
					tCheckSum.checkSum = ~tCheckSum.checkSum + 1;			/** calculate the checksum */
					processState = SP_STATE_MSB_CHECKSUM;					/** and go to check checksum state */
				}
				break;
			}
			case SP_STATE_MSB_CHECKSUM: {
				tSerialReadPacket->abData[index++] = temp;					/** save the received data (checksum MSB) into read packet */
				if (tCheckSum.abCheckSum[1] == temp) processState = SP_STATE_LSB_CHECKSUM;	/** if checksum MSB is correct, go to LSB checksum state */
				else bRet = ERR_BMS_CHECKSUM;									/** else return the checksum error */
				break;
			}
			case SP_STATE_LSB_CHECKSUM: {
				tSerialReadPacket->abData[index++] = temp;					/** save the received data (checksum LSB) into read packet */
				if (tCheckSum.abCheckSum[0] == temp) processState = SP_STATE_READ_SUFFIX;	/** if checksum LSB is correct, go to the read suffix state */
				else bRet = ERR_BMS_CHECKSUM;									/** else return the checksum error */
				break;
			}
			case SP_STATE_READ_SUFFIX: {
				tSerialReadPacket->abData[index] = temp;					/** save the received data (suffix) into read packet */
				if (temp == bmsSuffix) {
					readSuffix = true;
					//ESP_LOGD(LOG_TAG, "Suffix found!");
				}
				else bRet = ERR_BMS_SUFFIX;										/** else return the suffix error */
				break;
			}
			}

			if (ERR_BMS_OK != bRet) break;
		}
	}

	if (!readSuffix) return ERR_BMS_SHORT_DATA;
	else return bRet;
}

BMS_ERROR_E BMSPacketHandler::readSerialPacket(BMS_PACKET_STRUCT_T * tSerialReadPacket, uint8_t *data, uint8_t len)
{
	uint8_t temp = 0, index = 0, idx = 0;
	bool readSuffix = false;
	uint8_t dataSize = len;
	SERIAL_BMS_PROCESS_STATE_t processState = SP_STATE_READ_HEADER;
	BMS_CHECKSUM_PACKET_T tCheckSum = { 0 };
	BMS_ERROR_E bRet = ERR_BMS_OK;

	//ESP_LOGD(LOG_TAG, "dataSize: %d", dataSize);

	if (!dataSize) return ERR_BMS_NO_DATA_AVAIL;
	else {
		while (dataSize--) {

			temp = (uint8_t)data[idx++];
			//ESP_LOGD(LOG_TAG, "data[%d]: 0x%02X", idx - 1, temp);

			switch (processState) {
			case SP_STATE_READ_HEADER: {
				if (temp == bmsHeader) {									/** header is found */
					tSerialReadPacket->bHeader = temp;
					processState = SP_STATE_READ_REGISTER;
				}
				break;
			}
			case SP_STATE_READ_REGISTER: {
				tSerialReadPacket->tResponseMode.bCmdID = temp;				/** read the command ID register */
				processState = SP_STATE_CHECK_ERROR;
				break;
			}
			case SP_STATE_CHECK_ERROR: {
				tSerialReadPacket->tResponseMode.bStatus = temp;			/** read the error status */
				if (!temp) processState = SP_STATE_READ_PAYLOAD_LENGTH;		/** if no error */
				else {
					bRet = BMS_ERROR_E(temp);								/** if error occured, exit the while condition */
				}
				break;
			}
			case SP_STATE_READ_PAYLOAD_LENGTH: {
				tSerialReadPacket->bLength = temp;							/** read the payload length */
				tCheckSum.checkSum = temp;									/** transfer the data to checksum buffer */
				if (!temp) {												/** if payload length is 0 */
					tCheckSum.checkSum = ~tCheckSum.checkSum + 1;			/** calculate the checksum */
					processState = SP_STATE_MSB_CHECKSUM;					/** go directly to check checksum state */
				}
				else processState = SP_STATE_READ_PAYLOAD;					/** else go to read payload state */
				break;
			}
			case SP_STATE_READ_PAYLOAD: {
				tSerialReadPacket->abData[index++] = temp;					/** save the data into payload buffer */
				tCheckSum.checkSum += temp;									/** accumulate the data into checksum buffer */
				if (index == tSerialReadPacket->bLength) {					/** when all payload has been read */
					tCheckSum.checkSum = ~tCheckSum.checkSum + 1;			/** calculate the checksum */
					processState = SP_STATE_MSB_CHECKSUM;					/** and go to check checksum state */
				}
				break;
			}
			case SP_STATE_MSB_CHECKSUM: {
				tSerialReadPacket->abData[index++] = temp;					/** save the received data (checksum MSB) into read packet */
				if (tCheckSum.abCheckSum[1] == temp) processState = SP_STATE_LSB_CHECKSUM;	/** if checksum MSB is correct, go to LSB checksum state */
				else bRet = ERR_BMS_CHECKSUM;									/** else return the checksum error */
				break;
			}
			case SP_STATE_LSB_CHECKSUM: {
				tSerialReadPacket->abData[index++] = temp;					/** save the received data (checksum LSB) into read packet */
				if (tCheckSum.abCheckSum[0] == temp) processState = SP_STATE_READ_SUFFIX;	/** if checksum LSB is correct, go to the read suffix state */
				else bRet = ERR_BMS_CHECKSUM;									/** else return the checksum error */
				break;
			}
			case SP_STATE_READ_SUFFIX: {
				tSerialReadPacket->abData[index] = temp;					/** save the received data (suffix) into read packet */
				if (temp == bmsSuffix) {
					readSuffix = true;
					//ESP_LOGD(LOG_TAG, "Suffix found!");
				}
				else bRet = ERR_BMS_SUFFIX;										/** else return the suffix error */
				break;
			}
			}

			if (ERR_BMS_OK != bRet) break;
		}
	}

	if (!readSuffix) return ERR_BMS_SHORT_DATA;
	else return bRet;
}

BMS_ERROR_E BMSPacketHandler::readSerialPacket(BMS_PACKET_STRUCT_T * tSerialReadPacket, std::string data)
{
	uint8_t temp = 0, index = 0, idx = 0;
	bool readSuffix = false;
	uint8_t dataSize = data.length();
	SERIAL_BMS_PROCESS_STATE_t processState = SP_STATE_READ_HEADER;
	BMS_CHECKSUM_PACKET_T tCheckSum = { 0 };
	BMS_ERROR_E bRet = ERR_BMS_OK;

	//ESP_LOGD(LOG_TAG, "dataSize: 0x%02X", dataSize);

	if (!dataSize) return ERR_BMS_NO_DATA_AVAIL;
	else {
		while (dataSize--) {

			temp = (uint8_t)data[idx++];
			//ESP_LOGD(LOG_TAG, "data[%d]: 0x%02X", idx - 1, temp);

			switch (processState) {
			case SP_STATE_READ_HEADER: {
				if (temp == bmsHeader) {									/** header is found */
					tSerialReadPacket->bHeader = temp;
					processState = SP_STATE_READ_REGISTER;
				}
				break;
			}
			case SP_STATE_READ_REGISTER: {
				tSerialReadPacket->tResponseMode.bCmdID = temp;				/** read the command ID register */
				processState = SP_STATE_CHECK_ERROR;
				break;
			}
			case SP_STATE_CHECK_ERROR: {
				tSerialReadPacket->tResponseMode.bStatus = temp;			/** read the error status */
				if (!temp) processState = SP_STATE_READ_PAYLOAD_LENGTH;		/** if no error */
				else {
					bRet = BMS_ERROR_E(temp);								/** if error occured, exit the while condition */
				}
				break;
			}
			case SP_STATE_READ_PAYLOAD_LENGTH: {
				tSerialReadPacket->bLength = temp;							/** read the payload length */
				tCheckSum.checkSum = temp;									/** transfer the data to checksum buffer */
				if (!temp) {												/** if payload length is 0 */
					tCheckSum.checkSum = ~tCheckSum.checkSum + 1;			/** calculate the checksum */
					processState = SP_STATE_MSB_CHECKSUM;					/** go directly to check checksum state */
				}
				else processState = SP_STATE_READ_PAYLOAD;					/** else go to read payload state */
				break;
			}
			case SP_STATE_READ_PAYLOAD: {
				tSerialReadPacket->abData[index++] = temp;					/** save the data into payload buffer */
				tCheckSum.checkSum += temp;									/** accumulate the data into checksum buffer */
				if (index == tSerialReadPacket->bLength) {					/** when all payload has been read */
					tCheckSum.checkSum = ~tCheckSum.checkSum + 1;			/** calculate the checksum */
					processState = SP_STATE_MSB_CHECKSUM;					/** and go to check checksum state */
				}
				break;
			}
			case SP_STATE_MSB_CHECKSUM: {
				tSerialReadPacket->abData[index++] = temp;					/** save the received data (checksum MSB) into read packet */
				if (tCheckSum.abCheckSum[1] == temp) processState = SP_STATE_LSB_CHECKSUM;	/** if checksum MSB is correct, go to LSB checksum state */
				else bRet = ERR_BMS_CHECKSUM;									/** else return the checksum error */
				break;
			}
			case SP_STATE_LSB_CHECKSUM: {
				tSerialReadPacket->abData[index++] = temp;					/** save the received data (checksum LSB) into read packet */
				if (tCheckSum.abCheckSum[0] == temp) processState = SP_STATE_READ_SUFFIX;	/** if checksum LSB is correct, go to the read suffix state */
				else bRet = ERR_BMS_CHECKSUM;									/** else return the checksum error */
				break;
			}
			case SP_STATE_READ_SUFFIX: {
				tSerialReadPacket->abData[index] = temp;					/** save the received data (suffix) into read packet */
				if (temp == bmsSuffix) {
					readSuffix = true;
					//ESP_LOGD(LOG_TAG, "Suffix found!");
				}
				else bRet = ERR_BMS_SUFFIX;										/** else return the suffix error */
				break;
			}
			}

			if (ERR_BMS_OK != bRet) break;
		}
	}

	if (!readSuffix) return ERR_BMS_SHORT_DATA;
	else return bRet;
}

void BMSPacketHandler::bmsReadInfoStatus(uint8_t *pData, uint8_t len) {

	BMS_PACKET_STRUCT_T rSerialPacket = { 0 };
	BMS_ERROR_E bRet = ERR_BMS_OK;

	if (blPacket2Parse) {

		uint8_t totalLength = lastPacketLength + len;

		for (uint8_t i = 0; i < len; i++) {
			abBMSReadPacket[lastPacketLength + i] = pData[i];
		}

		if (ERR_BMS_OK != (bRet = readSerialPacket(&rSerialPacket, abBMSReadPacket, totalLength))) {
			//ESP_LOGD(LOG_TAG, "bRet: %d", bRet);
		}

		//ESP_LOGD(LOG_TAG, "Total voltage: %d", __bswap_16(rSerialPacket.tInfoStatus.usTotalVoltage));
		
		Serial.printf("Total voltage: %d", __bswap_16(rSerialPacket.tInfoStatus.usTotalVoltage));

		blPacket2Parse = false;
	}

	else {
		if (ERR_BMS_OK != (bRet = readSerialPacket(&rSerialPacket, pData, len))) {
			//ESP_LOGD(LOG_TAG, "bRet: %d", bRet);

			if (bRet == ERR_BMS_SHORT_DATA) {
				memcpy(abBMSReadPacket, &rSerialPacket, len);
				lastPacketLength = len;
				blPacket2Parse = true;
			}
		}
	}
}

bool BMSPacketHandler::bmsReadInfoStatus(BMS_PACKET_STRUCT_T *tPacket, uint8_t *pData, uint8_t len) {

	BMS_ERROR_E bRet = ERR_BMS_OK;

	if (blPacket2Parse) {

		uint8_t totalLength = lastPacketLength + len;

		for (uint8_t i = 0; i < len; i++) {
			abBMSReadPacket[lastPacketLength + i] = pData[i];
		}

		if (ERR_BMS_OK != (bRet = readSerialPacket(tPacket, abBMSReadPacket, totalLength))) {
			//ESP_LOGD(LOG_TAG, "bRet: %d", bRet);
		}

		blPacket2Parse = false;

		return true;
	}

	else {
		if (ERR_BMS_OK != (bRet = readSerialPacket(tPacket, pData, len))) {
			//ESP_LOGD(LOG_TAG, "bRet: %d", bRet);

			if (bRet == ERR_BMS_SHORT_DATA) {
				memcpy(abBMSReadPacket, tPacket, len);
				lastPacketLength = len;
				blPacket2Parse = true;
			}

			return false;
		}

		else return true;
	}
}
