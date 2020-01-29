#ifndef __LORAWAN_PACKETHANDLER_PUBLIC_H
#define __LORAWAN_PACKETHANDLER_PUBLIC_H

#if defined (__STDC__)
#if !defined(__PACKED_PRE) || !defined(__PACKED_POST)
#define __PACKED_PRE	
#define __PACKED_POST  __attribute__ ((packed))
#endif
#endif

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

typedef __PACKED_PRE struct LORA_DATA_STRUCT_Ttag {
	union {
		float		fGpsLatitude;
		uint32_t	ulGpsLatitude;
	};
	union {
		float		fGpsLongitude;
		uint32_t	ulGpsLongitude;
	};
	union {
		float		fGpsAltitude;
		uint32_t	ulGpsAltitude;
	};
	uint16_t		usBmsTotalVoltage;
	uint16_t		usControllerTotalVoltage;
	uint8_t			bControllerSpeedKmh;
	uint16_t		usControllerTotalDistance;
	uint32_t		ulRtcTimeOfStartSession;
	uint8_t			bHeartyBpm;
	uint8_t			bMpuFlags;
	uint32_t		ulMpuCrashTime;
} __PACKED_POST LORA_DATA_STRUCT_T;

typedef union LORA_DATA_PACKET_Ttag {
	uint8_t		abPacket[29];
	LORA_DATA_STRUCT_T tPacket;
}LORA_DATA_PACKET_T;

// This EUI must be in little-endian format, so least-significant-byte first. When copying an EUI from ttnctl output,
// this means to reverse the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3, 0x70. 
//static const uint8_t PROGMEM APPEUI[8] = { 0xE9, 0x63, 0x01, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
static const uint8_t PROGMEM APPEUI[8] = { 0x5A, 0x66, 0x01, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };

// This should also be in little endian format, see above.
//static const uint8_t PROGMEM DEVEUI[8] = { 0xEE, 0x50, 0xFF, 0x78, 0xBE, 0x84, 0xAF, 0x00 };
//static const uint8_t PROGMEM DEVEUI[8] = { 0xCB, 0xF7, 0x69, 0x92, 0xD2, 0xBF, 0x70, 0x00 };
//static const uint8_t PROGMEM DEVEUI[8] = { 0x05, 0x0F, 0x5A, 0xD3, 0xC7, 0x42, 0xBF, 0x00 };
static const uint8_t PROGMEM DEVEUI[8] = { 0x59, 0x82, 0x81, 0x33, 0x7E, 0xCF, 0xE9, 0x00 };

// This key should be in big endian format (or, since it is not really a number but a block of memory, endianness does 
// not really apply). In practice, a key taken from ttnctl can be copied as-is. The key shown here is the semtech default key.
//static const uint8_t PROGMEM APPKEY[16] = { 0x8B, 0x96, 0xA5, 0xF8, 0x7C, 0x55, 0xB9, 0xBE, 0xB8, 0xE9, 0xD5, 0x76, 0x92, 0x2F, 0x7A, 0xA7 };
//static const uint8_t PROGMEM APPKEY[16] = { 0xCE, 0xC0, 0x93, 0x1E, 0x98, 0xE2, 0x83, 0x22, 0xC5, 0x4A, 0x88, 0xAC, 0x3D, 0xF5, 0x80, 0x06 };
//static const uint8_t PROGMEM APPKEY[16] = { 0x18, 0x67, 0x62, 0xBD, 0xC7, 0x9E, 0xE2, 0x25, 0x00, 0x76, 0xD9, 0xA9, 0xA5, 0x34, 0x7B, 0x66 };
static const uint8_t PROGMEM APPKEY[16] = { 0xE4, 0xAD, 0xE2, 0xEB, 0xD8, 0x78, 0xD6, 0x63, 0x98, 0x8C, 0xF9, 0xA4, 0xF4, 0xE5, 0x01, 0x3A };

#endif
