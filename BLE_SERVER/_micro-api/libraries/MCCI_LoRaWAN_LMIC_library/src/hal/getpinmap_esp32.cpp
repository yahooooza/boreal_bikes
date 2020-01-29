/*

Module:  getconfig_featherm0lora.cpp

Function:
		Arduino-LMIC C++ HAL pinmap for Adafruit Feather M0 LoRa

Copyright & License:
		See accompanying LICENSE file.

Author:
		Terry Moore, MCCI       November 2018

*/

#include <arduino_lmic_hal_boards.h>
#include <Arduino.h>

#include "../lmic/oslmic.h"

namespace Arduino_LMIC {

	class HalConfiguration_ESP32_t : public HalConfiguration_t
	{
	public:
		enum DIGITAL_PINS : uint8_t
		{
			PIN_SX1276_NSS = 18,
			PIN_SX1276_NRESET = HalPinmap_t::UNUSED_PIN,
			PIN_SX1276_DIO0 = 23,
			PIN_SX1276_DIO1 = 23,
			PIN_SX1276_DIO2 = 23,
			PIN_SX1276_MOSI = 27,
			PIN_SX1276_MISO = 19,
			PIN_SX1276_SCK = 5,
			PIN_SX1276_ANT_SWITCH_RX = HalPinmap_t::UNUSED_PIN,
			PIN_SX1276_ANT_SWITCH_TX_BOOST = HalPinmap_t::UNUSED_PIN,
			PIN_SX1276_ANT_SWITCH_TX_RFO = HalPinmap_t::UNUSED_PIN,
			PIN_VDD_BOOST_ENABLE = HalPinmap_t::UNUSED_PIN,
		};

		virtual void begin(void) override
		{
			digitalWrite(PIN_SX1276_NSS, 1);
			pinMode(PIN_SX1276_NSS, OUTPUT);
		}

		// virtual void end(void) override

		// virtual ostime_t setModuleActive(bool state) override

	};

	static HalConfiguration_ESP32_t myConfig;

	static const HalPinmap_t myPinmap =
	{
	.nss = HalConfiguration_ESP32_t::PIN_SX1276_NSS,      // chip select is D7
	.rxtx = HalConfiguration_ESP32_t::PIN_SX1276_ANT_SWITCH_RX, // RXTX is D29
	.rst = HalConfiguration_ESP32_t::PIN_SX1276_NRESET,   // NRESET is D8

	.dio = {HalConfiguration_ESP32_t::PIN_SX1276_DIO0,    // DIO0 (IRQ) is D25
			HalConfiguration_ESP32_t::PIN_SX1276_DIO1,    // DIO1 is D26
			HalConfiguration_ESP32_t::PIN_SX1276_DIO2,    // DIO2 is D27
		   },
	.mosi = HalConfiguration_ESP32_t::PIN_SX1276_MOSI,
	.miso = HalConfiguration_ESP32_t::PIN_SX1276_MISO,
	.sck = HalConfiguration_ESP32_t::PIN_SX1276_SCK,
	.rxtx_rx_active = 0,
	.rssi_cal = 10,
	.spi_freq = 8000000,     /* 8MHz */
	.pConfig = &myConfig
	};

	const HalPinmap_t *GetPinmap_ESP32(void)
	{
		return &myPinmap;
	}

}; // namespace Arduino_LMIC
