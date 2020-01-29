#ifndef __BB__UUID_H
#define __BB__UUID_H

#include "BLEUUID.h"

/************************************************************************************************************************/
/*!
		ad88888ba 888888888888   db        888b      88 88888888ba,        db        88888888ba  88888888ba,
	   d8"     "8b     88       d88b       8888b     88 88      `"8b      d88b       88      "8b 88      `"8b
	   Y8,             88      d8'`8b      88 `8b    88 88        `8b    d8'`8b      88      ,8P 88        `8b
	   `Y8aaaaa,       88     d8'  `8b     88  `8b   88 88         88   d8'  `8b     88aaaaaa8P' 88         88
		 `"""""8b,     88    d8YaaaaY8b    88   `8b  88 88         88  d8YaaaaY8b    88""""88'   88         88
			   `8b     88   d8""""""""8b   88    `8b 88 88         8P d8""""""""8b   88    `8b   88         8P
	   Y8a     a8P     88  d8'        `8b  88     `8888 88      .a8P d8'        `8b  88     `8b  88      .a8P
		"Y88888P"      88 d8'          `8b 88      `888 88888888Y"' d8'          `8b 88      `8b 88888888Y"'
*/
/************************************************************************************************************************/

/** Battery Service & Characteristics */
#define BATTERY_SERVICE_UUID 					BLEUUID("0000180f-0000-1000-8000-00805f9b34fb")
#define BATTERY_STATE_CHAR_UUID 				BLEUUID("00002a19-0000-1000-8000-00805f9b34fb")
#define HEART_RATE_SERVICE_UUID 				BLEUUID("0000180d-0000-1000-8000-00805f9b34fb")
#define HEART_RATE_MEASUREMENT_CHAR_UUID 		BLEUUID("00002a37-0000-1000-8000-00805f9b34fb")

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

/** BMS Read Write Service & Characteristics */
#define BMS_RW_SERVICE_UUID 		BLEUUID("0000ff00-0000-1000-8000-00805f9b34fb")
#define BMS_RX_CHAR_UUID			BLEUUID("0000ff01-0000-1000-8000-00805f9b34fb")
#define BMS_TX_CHAR_UUID 			BLEUUID("0000ff02-0000-1000-8000-00805f9b34fb")

/************************************************************************************************************************/
/*!
							____ ____ ____ ____ ____ ____ _________ ____ ____ ____ ____ ____
						   ||H |||E |||A |||R |||T |||Y |||       |||P |||A |||T |||C |||H ||
						   ||__|||__|||__|||__|||__|||__|||_______|||__|||__|||__|||__|||__||
						   |/__\|/__\|/__\|/__\|/__\|/__\|/_______\|/__\|/__\|/__\|/__\|/__\|
*/
/************************************************************************************************************************/

/** Hearty Patch Service & Characteristics */
#define HEARTYPATCH_RR_SERVICE_UUID				BLEUUID("cd5c7491-4448-7db8-ae4c-d1da8cba36d0")
#define HEARTYPATCH_RR_MEASUREMENT_CHAR_UUID	BLEUUID("01bfa86f-970f-8d96-d44d-9023c47faddc")

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

/** Motor Controller Service & Characteristics */
#define CONTROLLER_RW_SERVICE_UUID	BLEUUID("0000fff0-0000-1000-8000-00805f9b34fb")
#define CONTROLLER_RX_CHAR_UUID		BLEUUID("0000fff1-0000-1000-8000-00805f9b34fb")

/************************************************************************************************************************/
/*!


 88888888ba                                   88  ad88888ba
 88      "8b                                  "" d8"     "8b
 88      ,8P                                     Y8,
 88aaaaaa8P' ,adPPYYba, ,adPPYba, 8b,dPPYba,  88 `Y8aaaaa,    ,adPPYba, 8b,dPPYba, 8b       d8  ,adPPYba, 8b,dPPYba,
 88""""88'   ""     `Y8 I8[    "" 88P'    "8a 88   `"""""8b, a8P_____88 88P'   "Y8 `8b     d8' a8P_____88 88P'   "Y8
 88    `8b   ,adPPPPP88  `"Y8ba,  88       d8 88         `8b 8PP""""""" 88          `8b   d8'  8PP""""""" 88
 88     `8b  88,    ,88 aa    ]8I 88b,   ,a8" 88 Y8a     a8P "8b,   ,aa 88           `8b,d8'   "8b,   ,aa 88
 88      `8b `"8bbdP"Y8 `"YbbdP"' 88`YbbdP"'  88  "Y88888P"   `"Ybbd8"' 88             "8"      `"Ybbd8"' 88
								  88
								  88
*/
/************************************************************************************************************************/

/** Raspi Service & Characteristics */
//#define RASPI_RW_SERVICE_UUID		BLEUUID("0000fff0-0000-1000-8000-00805f9b34fb")
//#define BMS_MOTOR_INFO_CHAR_UUID	BLEUUID("0000fff0-0000-1000-8000-00805f9b34fb")	
//#define LOCK_INFO_CHAR_UUID			BLEUUID("0000fff0-0000-1000-8000-00805f9b34fb")	
//#define LOCK_CONTROL_CHAR_UUID		BLEUUID("0000fff0-0000-1000-8000-00805f9b34fb")	
//#define LORA_INFO_CHAR_UUID			BLEUUID("0000fff0-0000-1000-8000-00805f9b34fb")	
//#define TIME_RESET_CHAR_UUID		BLEUUID("0000fff0-0000-1000-8000-00805f9b34fb")	
//#define ESP_CURRENT_TIME_CHAR_UUID	BLEUUID("0000fff0-0000-1000-8000-00805f9b34fb")	
//#define HEART_RATE_CHAR_UUID		BLEUUID("0000fff0-0000-1000-8000-00805f9b34fb")	
//#define LOCATION_INFO_CHAR_UUID		BLEUUID("0000fff0-0000-1000-8000-00805f9b34fb")	
//#define MPU_DETECTION_CHAR_UUID		BLEUUID("0000fff0-0000-1000-8000-00805f9b34fb")	

#endif