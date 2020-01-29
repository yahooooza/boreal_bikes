/************************************************************************************************************************/
/*!
* @copyright	Zentrum zur Foerderung eingebetteter Systeme e.V.
* @author		zzulkifli
* @file			BMS_PacketHandler.h
* @date			11.02.2019
* @version		1.0
* @brief		BMS packet handler header file
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

#ifndef __BMS_PACKETHANDLER_PUBLIC_H
#define __BMS_PACKETHANDLER_PUBLIC_H


#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#if defined (__STDC__)
#if !defined(__PACKED_PRE) || !defined(__PACKED_POST)
#define __PACKED_PRE	
#define __PACKED_POST  __attribute__ ((packed))
#endif
#endif

#include <string>
#include "BMSSerialPacket.h"

#define BMS_MAX_BUFFER_LEN (uint8_t)40

typedef __PACKED_PRE enum BMS_ERROR_Etag {
	ERR_BMS_OK,								//!< 0x00
	ERR_BMS_CHECKSUM,						//!< 0x01
	ERR_BMS_SUFFIX,							//!< 0x02
	ERR_BMS_SHORT_DATA,						//!< 0x03
	ERR_BMS_NO_DATA_AVAIL,					//!< 0x04
	ERR_BMS_DETECTED = 0x80,				//!< 0x80
} __PACKED_POST BMS_ERROR_E;

class BMSPacketHandler
{
 public:

	 typedef __PACKED_PRE enum SERIAL_BMS_PROCESS_STATEtag {
		 SP_STATE_READ_HEADER,
		 SP_STATE_READ_REGISTER,
		 SP_STATE_CHECK_ERROR,
		 SP_STATE_READ_PAYLOAD_LENGTH,
		 SP_STATE_READ_PAYLOAD,
		 SP_STATE_MSB_CHECKSUM,
		 SP_STATE_LSB_CHECKSUM,
		 SP_STATE_READ_SUFFIX,
	 } __PACKED_POST SERIAL_BMS_PROCESS_STATE_t;

	 BMSPacketHandler();
	 BMSPacketHandler(HardwareSerial *port);
	 virtual ~BMSPacketHandler();
	 
	 uint8_t setSerialPacket(BMS_PACKET_STRUCT_T *tSerialWritePacket, uint8_t mode, uint8_t cmdID, uint8_t len, uint8_t * val);
	 void sendSerialPacket(BMS_PACKET_STRUCT_T *tSerialWritePacket, uint8_t len);
	 BMS_ERROR_E readSerialPacket(BMS_PACKET_STRUCT_T *tSerialReadPacket);
	 BMS_ERROR_E readSerialPacket(BMS_PACKET_STRUCT_T *tSerialReadPacket, uint8_t *data, uint8_t len);
	 BMS_ERROR_E readSerialPacket(BMS_PACKET_STRUCT_T *tSerialReadPacket, std::string data);
	 void bmsReadInfoStatus(uint8_t *pData, uint8_t len);
	 bool bmsReadInfoStatus(BMS_PACKET_STRUCT_T *tPacket, uint8_t *pData, uint8_t len);

private:
	friend class BLEClient;
	friend class BLERemoteService;
	friend class BLERemoteCharacteristic;

	HardwareSerial *_serPort;

	const uint8_t bmsHeader = 0xDD;
	const uint8_t bmsSuffix = 0x77;
	bool blPacket2Parse;
	uint8_t lastPacketLength = 0;
	uint8_t abBMSReadPacket[BMS_MAX_BUFFER_LEN];
};

#endif

