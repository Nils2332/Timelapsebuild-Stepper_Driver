#include "System.h"
#include <cstdarg>
#include <stdio.h>
#include <stdarg.h>

#include "gpio.h"
#include "usart.h"


namespace System {

	void print(const char *fmt, ...) {
	//#if DEBUG
	#ifndef NDEBUG
		char buffer[100];

		va_list arp;
		va_start(arp, fmt);
		uint16_t length = vsnprintf(buffer, sizeof(buffer), fmt, arp);
		va_end(arp);

		HAL_UART_Transmit(&huart2, (uint8_t*) buffer, length, 1000);
	#endif
	//#endif
	}

}
