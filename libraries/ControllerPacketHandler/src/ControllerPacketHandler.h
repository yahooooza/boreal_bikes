/************************************************************************************************************************/
/*!
* @copyright	Zentrum zur Foerderung eingebetteter Systeme e.V.
* @author		zzulkifli
* @file			CONTROLLER_PacketHandler.h
* @date			11.02.2019
* @version		1.0
* @brief		CONTROLLER packet handler header file
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

#ifndef __CONTROLLER_PACKETHANDLER_PUBLIC_H
#define __CONTROLLER_PACKETHANDLER_PUBLIC_H


#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#if defined (__STDC__)
#if !defined(__PACKED_PRE) || !defined(__PACKED_POST)
#define __PACKED_PRE	
#define __PACKED_POST	__attribute__ ((packed))
#endif
#endif

#include <string>
#include "ControllerSerialPacket.h"

#define CONTROLLER_MAX_BUFFER_LEN (uint8_t)40

typedef __PACKED_PRE enum CONTROLLER_ERROR_Etag {
	ERR_CONTROLLER_OK,								//!< 0x00
	ERR_CONTROLLER_CHECKSUM,						//!< 0x01
	ERR_CONTROLLER_SUFFIX,							//!< 0x02
	ERR_CONTROLLER_SHORT_DATA,						//!< 0x03
	ERR_CONTROLLER_NO_DATA_AVAIL,					//!< 0x04
	ERR_CONTROLLER_DETECTED = 0x80,				//!< 0x80
} __PACKED_POST CONTROLLER_ERROR_E;

class ControllerPacketHandler
{
 public:

	 typedef __PACKED_PRE enum SERIAL_PROCESS_STATEtag {
		 SP_STATE_READ_HEADER,
		 SP_STATE_READ_PAYLOAD,
		 SP_STATE_READ_SUFFIX,
	 } __PACKED_POST SERIAL_CONTROLLER_PROCESS_STATE_t;

	 ControllerPacketHandler();
	 ControllerPacketHandler(HardwareSerial *port);
	 virtual ~ControllerPacketHandler();
	 
	 CONTROLLER_ERROR_E readSerialPacket(CONTROLLER_PACKET_STRUCT_T *tSerialReadPacket);
	 CONTROLLER_ERROR_E readSerialPacket(CONTROLLER_PACKET_STRUCT_T *tSerialReadPacket, uint8_t *data, uint8_t len);
	 CONTROLLER_ERROR_E readSerialPacket(CONTROLLER_PACKET_STRUCT_T *tSerialReadPacket, std::string data);
	 void readParsePacket(uint8_t *pData, uint8_t len);
	 bool readParsePacket(CONTROLLER_PACKET_STRUCT_T *readPacket, uint8_t *pData, uint8_t len);

private:
	friend class BLEClient;
	friend class BLERemoteService;
	friend class BLERemoteCharacteristic;

	HardwareSerial *_serPort;

	const uint8_t controllerHeader = 0xAA;
	const uint8_t controllerSuffix = 0x85;
	bool blPacket2Parse;
	uint8_t abControllerReadPacket[CONTROLLER_MAX_BUFFER_LEN];
	uint8_t lastPacketLength = 0;
};

#endif

