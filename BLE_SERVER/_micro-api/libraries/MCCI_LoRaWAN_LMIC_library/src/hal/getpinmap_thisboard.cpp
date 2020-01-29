/*

Module:  getconfig_thisboard.cpp

Function:
        Return a suitable LMIC config for this board.

Copyright & License:
        See accompanying LICENSE file.

Author:
        Terry Moore, MCCI       November 2018

*/

#include <arduino_lmic_hal_boards.h>

namespace Arduino_LMIC {

const HalPinmap_t *GetPinmap_ThisBoard(void)
{
#if defined(ESP32)
	return GetPinmap_ESP32();
#else
	#pragma message("Board not supported -- use an explicit pinmap")
	return nullptr;
#endif
}

}; // namespace Arduino_LMIC

