/*=============================================================================
 * Copyright (c) 2021, Ignacio Riveros <riverosignacio138@gmail.com>
 * All rights reserved.
 * License: gpl-3.0 (see LICENSE.txt)
 * Date: 2021/04/14
 * Version: 1.0
 *===========================================================================*/

/*=====[Inclusions of function dependencies]=================================*/

#include "test_rs485.h"
#include "sapi.h"

/*=====[Definition macros of private constants]==============================*/
#define ARRAY_SIZE 8
/*=====[Definitions of extern global variables]==============================*/

/*=====[Definitions of public global variables]==============================*/
uint8_t data_array[ARRAY_SIZE] = {0};
/*=====[Definitions of private global variables]=============================*/

/*=====[Main function, program entry point after power on or reset]==========*/

int main( void )
{
   // ----- Setup -----------------------------------
   boardInit();

   uartInit(UART_485, 115200);

   // ----- Repeat for ever -------------------------
   while( true ) {
      gpioToggle(LED2);
      uartWriteByteArray(UART_485, data_array, ARRAY_SIZE);
      delay(100);
   }

   // YOU NEVER REACH HERE, because this program runs directly or on a
   // microcontroller and is not called by any Operating System, as in the 
   // case of a PC program.
   return 0;
}
