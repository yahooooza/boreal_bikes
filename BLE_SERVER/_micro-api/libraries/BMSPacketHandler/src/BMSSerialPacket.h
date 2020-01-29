/************************************************************************************************************************/
/*!
* @copyright	Zentrum zur Foerderung eingebetteter Systeme e.V.
* @author		zzulkifli
* @file			BMS_Serial_Packet.h
* @date			11.02.2019
* @version		1.0
* @brief		BMS serial packet header file
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

#ifndef __BMS_SERIAL_PACKET_PUBLIC_H
#define __BMS_SERIAL_PACKET_PUBLIC_H

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

/** advanced protection state struct for register 0x03 info and status */
typedef __PACKED_PRE struct BMS_IS_PROTECTIONSTATE_STRUCT_Ttag
{
	uint16_t	bfCellOverVoltage : 1;
	uint16_t	bfCellUnderVoltage : 1;
	uint16_t	bfPackOverVoltage : 1;
	uint16_t	bfPackUnderVoltage : 1;
	uint16_t	bfChargeOverTemp : 1;
	uint16_t	bfChargeUnderTemp : 1;
	uint16_t	bfDischargeOverTemp : 1;
	uint16_t	bfDischargeUnderTemp : 1;
	uint16_t	bfChargeOverCurrent : 1;
	uint16_t	bfDischargeOvercurrent : 1;
	uint16_t	bfShortProtection : 1;
	uint16_t	bfICError : 1;
	uint16_t	bfSoftwareLockMOS : 1;
	uint16_t	bfChargeOverTime : 1;
	uint16_t	bfReserved : 2;
} __PACKED_POST BMS_IS_PROTECTIONSTATE_STRUCT_T;

/** payload struct for register 0x03 info and status */
typedef __PACKED_PRE struct BMS_INFO_STATUS_READ_STRUCT_Ttag
{
	uint16_t	usTotalVoltage;							/** total voltage [10 mV] */
	uint16_t	usTotalCurrent;							/** total current [10 mA] */
	uint16_t	usResidualCapacity;						/** residual capacity [10 mAh] */
	uint16_t	usNominalCapacity;						/** nominal capacity [10 mAh] */
	uint16_t	usCycleTimes;
	uint16_t	usDateOfManufacture;
	uint16_t	usBalancedStateLow;
	uint16_t	usBalancedStateHigh;
	BMS_IS_PROTECTIONSTATE_STRUCT_T	sProtectionState;
	uint8_t		bSoftwareVersion;
	uint8_t		bRelStateOfCharge;
	uint8_t		bFETState;
	uint8_t		bBatterySerialNumber;
	uint8_t		bNTCCount;
	uint16_t	bNTCTemp1;								/** NTC temperature 1 [0.1 K] */
	uint16_t	bNTCTemp2;								/** NTC temperature 2 [0.1 K] */
} __PACKED_POST BMS_INFO_STATUS_READ_STRUCT_T;

/** packet struct for BMS request mode */
typedef __PACKED_PRE struct BMS_IF_REQ_MODE_STRUCT_Ttag
{
	uint8_t bMode;										/** 0xA5 for read reg., 0x5A for write reg */
	uint8_t bCmdID;										/** register number */
} __PACKED_POST BMS_IF_REQ_MODE_STRUCT_T;

/** packet struct for BMS response mode */
typedef __PACKED_PRE struct BMS_IF_RESP_MODE_STRUCT_Ttag
{
	uint8_t bCmdID;										/** register number */
	uint8_t bStatus;									/** packet status 0x00 no error, 0x80 error occured */
} __PACKED_POST BMS_IF_RESP_MODE_STRUCT_T;

/** packet struct for BMS serial packet */
typedef __PACKED_PRE struct BMS_PACKET_STRUCT_Ttag
{
	uint8_t bHeader;									/** header for the serial transmission (0xDD) */
	union {
		BMS_IF_REQ_MODE_STRUCT_T tRequestMode;			/** request mode struct */
		BMS_IF_RESP_MODE_STRUCT_T tResponseMode;		/** response mode struct */
	};
	uint8_t bLength;
	union
	{
		uint8_t		abData[50];							/** payload data + 2byte checksum + suffix (0x77) */
		uint16_t	auiData[10];						/** array of payload data in uint16_t format */
		int32_t		lData;								/** payload data in int32_t format */
		uint32_t	ulData;								/** payload data in uint32_t format */
		int16_t		iData;								/** payload data in int16_t format */
		uint16_t	uiData;								/** payload data in uint16_t format */
		uint8_t		bData;								/** payload data in uint8_t format */
		float		fData;								/** payload data in float format */
		BMS_INFO_STATUS_READ_STRUCT_T tInfoStatus;		/** payload data in info status format */
	};
} __PACKED_POST BMS_PACKET_STRUCT_T;

/** union for BMS checksum */
typedef union BMS_CHECKSUM_PACKET_Ttag
{
	uint16_t	checkSum;
	uint8_t	abCheckSum[2];
} BMS_CHECKSUM_PACKET_T;

extern const char *bmsInfoStatusReadSlot[];

class BMSRegister {
public:
	typedef __PACKED_PRE enum BMS_RW_REG_Etag
	{
		BMS_WRITE_REG = 0x5A,				/** Write register mode */
		BMS_READ_REG = 0xA5,				/** Read register mode */
	} __PACKED_POST BMS_RW_REG_E;

	typedef __PACKED_PRE enum BMS_REG_Etag
	{
		BMS_REG_INFO_STATUS = 0x03,				/* 0x03 */
		BMS_REG_BATT_VOLTAGE,					/* 0x04 */
		BMS_REG_HW_VERSION,						/* 0x05 */
		BMS_REG_FULL_CHARGE_CAPACITY,			/* 0x10 [CC] mAH	*/
		BMS_REG_CYCLE_CAPACITY,					/* 0x11 [CC] mAH	*/
		BMS_REG_FULL_CHARGE_VOLTAGE,			/* 0x12 [CC] mV		*/
		BMS_REG_DISCHARG_CUTOFF_VOLTAGE,		/* 0x13 [CC] mV		*/
		BMS_REG_DISCHARGE_RATE,					/* 0x14 [CC] %		*/
		BMS_REG_MANUFACTURE_DATE,				/* 0x15 [OC]		*/
		BMS_REG_SERIAL_NUMBER,					/* 0x16 [OC]		*/
		BMS_REG_CHARGE_CYCLE_COUNT,				/* 0x17 [OC]		*/
		BMS_REG_CHARGE_OTP_TRIGGER,				/* 0x18 [BP] °C		*/
		BMS_REG_CHARGE_OTP_RELEASE,				/* 0x19 [BP] °C		*/
		BMS_REG_CHARGE_UTP_TRIGGER,				/* 0x1A [BP] °C		*/
		BMS_REG_CHARGE_UTP_RELEASE,				/* 0x1B [BP] °C		*/
		BMS_REG_DISCHARGE_OTP_TRIGGER,			/* 0x1C [BP] °C		*/
		BMS_REG_DISCHARGE_OTP_RELEASE,			/* 0x1D [BP] °C		*/
		BMS_REG_DISCHARGE_UTP_TRIGGER,			/* 0x1E [BP] °C		*/
		BMS_REG_DISCHARGE_UTP_RELEASE,			/* 0x1F [BP] °C		*/
		BMS_REG_PACK_OVP_TRIGGER,				/* 0x20 [BP] mV		*/
		BMS_REG_PACK_OVP_PRELEASE,				/* 0x21 [BP] mV		*/
		BMS_REG_PACK_UVP_TRIGGER,				/* 0x22 [BP] mV		*/
		BMS_REG_PACK_UVP_RELEASE,				/* 0x23 [BP] mV		*/
		BMS_REG_CELL_OVP_TRIGGER,				/* 0x24 [BP] mV		*/
		BMS_REG_CELL_OVP_RELEASE,				/* 0x25 [BP] mV		*/
		BMS_REG_CELL_UVP_TRIGGER,				/* 0x26 [BP] mV		*/
		BMS_REG_CELL_UVP_RELEASE,				/* 0x27 [BP] mV		*/
		BMS_REG_CHARGE_OCP,						/* 0x28 [BP] mA		*/
		BMS_REG_DISCHARGE_OCP,					/* 0x29 [BP] mA		*/
		BMS_REG_BALANCE_START_VOLTAGE,			/* 0x2A */
		BMS_REG_BALANCE_WINDOW,					/* 0x2B */
		BMS_REG_SENSE_RESISTOR,					/* 0x2C [OC] mOhm	*/
		BMS_REG_FUNCTION_CONFIG,				/* 0x2D */
		BMS_REG_NTC_CONFIG,						/* 0x2E */
		BMS_REG_PACK_NUM,						/* 0x2F [OC]		*/
		BMS_REG_FET_CTRL_TIME_SET,				/* 0x30 [CC] s		*/
		BMS_REG_LED_DISP_TIME_SET,				/* 0x31 [CC] s		*/
		BMS_REG_VOLT_CAP_80,					/* 0x32 [CC] mV		*/
		BMS_REG_VOLT_CAP_60,					/* 0x33 [CC] mV		*/
		BMS_REG_VOLT_CAP_40,					/* 0x34 [CC] mV		*/
		BMS_REG_VOLT_CAP_20,					/* 0x35 [CC] mV		*/
		BMS_REG_HARD_CELL_OVP,					/* 0x36 */
		BMS_REG_HARD_CELL_UVP,					/* 0x37 */
		BMS_REG_CHG_DSG_OCP2,					/* 0x38 */
		BMS_REG_SC_RELEASE_TIME,				/* 0x39 */
		BMS_REG_CHARGE_OUTP_DELAY,				/* 0x3A [BP] s		*/
		BMS_REG_DISCHARGE_OUTP_DELAY,			/* 0x3B [BP] s		*/
		BMS_REG_PACK_OUVP_DELAY,				/* 0x3C [BP] s		*/
		BMS_REG_CELL_OUVP_DELAY,				/* 0x3D [BP] s		*/
		BMS_REG_CHARGE_OCP_DELAY,				/* 0x3E [BP] s		*/
		BMS_REG_DISCHARGE_OCP_DELAY,			/* 0x3F [BP] s		*/
		BMS_REG_CHARGE_TIME,					/* 0x40 ??*/
		BMS_REG_MANUFACTURER_NAME,				/* 0xA0 [OC]		*/
		BMS_REG_DEVICE_NAME,					/* 0xA1 [OC]		*/
		BMS_REG_BARCODE,						/* 0xA2 [OC]		*/
	} __PACKED_POST BMS_REG_E;
};

#endif

