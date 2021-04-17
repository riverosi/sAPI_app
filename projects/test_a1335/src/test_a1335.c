/*=============================================================================
 * Copyright (c) 2021, Ignacio Riveros <riverosignacio138@gmail.com>
 * All rights reserved.
 * License: gpl-3.0 (see LICENSE.txt)
 * Date: 2021/04/12
 * Version: 1.0
 *===========================================================================*/

/*=====[Inclusions of function dependencies]=================================*/

#include "test_a1335.h"
#include "sapi.h"

/*=====[Definition macros of private constants]==============================*/

/*=====[Definitions of extern global variables]==============================*/
const uint32_t _ANGLE_I2C_CFG =  100000;
/*=====[Definitions of public global variables]==============================*/

/*=====[Definitions of private global variables]=============================*/
/**
 * C++ version 0.4 char* style "itoa":
 * Written by Lukas Chmela
 * Released under GPLv3.

 */
char* itoa(int value, char* result, int base) {
   // check that the base if valid
   if (base < 2 || base > 36) { *result = '\0'; return result; }

   char* ptr = result, *ptr1 = result, tmp_char;
   int tmp_value;

   do {
      tmp_value = value;
      value /= base;
      *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * base)];
   } while ( value );

   // Apply negative sign
   if (tmp_value < 0) *ptr++ = '-';
   *ptr-- = '\0';
   while(ptr1 < ptr) {
      tmp_char = *ptr;
      *ptr--= *ptr1;
      *ptr1++ = tmp_char;
   }
   return result;
}
/*=====[Main function, program entry point after power on or reset]==========*/

int main(void) {
	// ----- Setup -----------------------------------
	boardInit();
	uartInit(UART_USB, 115200);
	angle_i2cDriverInit(_ANGLE_I2C_CFG, 0x0C);
	angle_setConfig(
			_ANGLE_CDS_NO_CHANGLE | _ANGLE_HDR_RESET_1 | _ANGLE_SFR_RESET_1
					| _ANGLE_CSR_STA_1 | _ANGLE_CXE_1 | _ANGLE_CER_1);
	uint16_t Angle;
	char buffer_string[5];

	// ----- Repeat for ever -------------------------
	while (true) {
		Angle = angle_getAngle();
		itoa((int)Angle, buffer_string, 10);
		uartWriteString(UART_USB, buffer_string);
		uartWriteByte(UART_USB, ',');
		gpioToggle(LED3);
		delay(100);
		__WFI();
	}

	// YOU NEVER REACH HERE, because this program runs directly or on a
	// microcontroller and is not called by any Operating System, as in the
	// case of a PC program.
	return 0;
}
