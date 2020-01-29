/************************************************************************************************************************/
/*!
* @copyright	Zentrum zur Foerderung eingebetteter Systeme e.V.
* @author		zzulkifli
* @file			ControllerSerialPacket.h
* @date			11.02.2019
* @version		1.0
* @brief		Motor controller serial packet header file
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

#ifndef __CONTROLLER_SERIAL_PACKET_PUBLIC_H
#define __CONTROLLER_SERIAL_PACKET_PUBLIC_H

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

/** payload struct for reading controller values */
typedef __PACKED_PRE struct CONTROLLER_VALUES_READ_STRUCT_Ttag
{
	uint8_t		bTotalVoltage;
	uint8_t		usTotalDistance;
	uint32_t	ulSpeedKmH;						
	uint32_t	ulErpsFiltered;						
} __PACKED_POST CONTROLLER_VALUES_READ_STRUCT_T;

/** packet struct for controller serial packet */
typedef __PACKED_PRE struct CONTROLLER_PACKET_STRUCT_Ttag
{
	uint8_t bHeader;									/** header for the serial transmission (0xDD) */

	union
	{
		uint8_t		abData[11];							/** payload data  + suffix (0x77) */
		int32_t		lData;								/** payload data in int32_t format */
		uint32_t	ulData;								/** payload data in uint32_t format */
		int16_t		iData;								/** payload data in int16_t format */
		uint16_t	uiData;								/** payload data in uint16_t format */
		uint8_t		bData;								/** payload data in uint8_t format */
		float		fData;								/** payload data in float format */
		CONTROLLER_VALUES_READ_STRUCT_T tPacket;		/** payload data in info status format */
	};
} __PACKED_POST CONTROLLER_PACKET_STRUCT_T;

extern const char *controllerValuesReadSlot[];

#endif