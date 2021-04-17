/*=============================================================================
 * Copyright (c) 2021, Ignacio Riveros <riverosignacio138@gmail.com>
 * All rights reserved.
 * License: gpl-3.0 (see LICENSE.txt)
 * Date: 2021/04/12
 * Version: 1.0
 *===========================================================================*/

/*=====[Inclusions of function dependencies]=================================*/

#include "test_nrf24.h"
#include "sapi.h"

/*=====[Definition macros of private constants]==============================*/

/*=====[Definitions of extern global variables]==============================*/

/*=====[Definitions of public global variables]==============================*/
/** Variable used for SysTick Counter */
static volatile uint32_t tick_cnt = 0;
/*=====[Definitions of private global variables]=============================*/
/* FUNCION que se ejecuta cada vez que ocurre un Tick. */
void myTickHook(void *ptr) {
	if (tick_cnt%100 == 0) {
		gpioToggle(LED3);
	}
	tick_cnt++;
}
/*=====[Main function, program entry point after power on or reset]==========*/
int main(void) {
	// ----- Setup -----------------------------------
	boardInit();

	tickConfig(10);
	tickCallbackSet(myTickHook, NULL);

	nrf24l01_t RX;
	RX.spi.cfg = nrf24l01_spi_default_cfg;
	RX.cs.n = GPIO_1;
	RX.ce.n = GPIO_3;
	RX.irq.n = GPIO_5;
	RX.mode = PRX;
	RX.en_ack_pay = false;
	RX.pin_int_num = 5;

	Nrf24Init(&RX);
	Nrf24SetRXPacketSize(&RX, 0x00, 32); // Set length of pipe 0 in 32 (used for the Pedal Left)
	Nrf24SetRXPacketSize(&RX, 0x01, 32); // Set length of pipe 1 in 32 (used for the Pedal Right)
	Nrf24EnableRxMode(&RX); /* Enable RX mode */
	Nrf24SecondaryDevISRConfig(&RX); /* Config ISR (only use one module in PRX mode on board)*/

	float float_data = 0.0f;
	/* RX_data[0] Store Pedal Left data
	 * RX_data[1] Store Pedal Rigth data
	 */
	nrf24l01p_pedal_data RX_data[2] = { 0 };

	// ----- Repeat for ever -------------------------
	while (true) {

		memcpy(&float_data, &rcv_fr_PTX[1], sizeof(float_data)); /*Convert array data to float data*/

		if (rcv_fr_PTX[0] == 0x01) {/*Pedal L*/
			RX_data[0].data_ready = true;
			RX_data[0].force_node = float_data;
			if (float_data > 0.2) {
				gpioWrite(LEDB, ON);
			} else {
				gpioWrite(LEDB, OFF);
			}
		}
		if (rcv_fr_PTX[0] == 0x02) {/*Pedal R*/
			RX_data[1].data_ready = true;
			RX_data[1].force_node = float_data;
			if (float_data > 0.2) {
				gpioWrite(LED2, ON);
			} else {
				gpioWrite(LED2, OFF);
			}
		}
	}

	// YOU NEVER REACH HERE, because this program runs directly or on a
	// microcontroller and is not called by any Operating System, as in the
	// case of a PC program.
	return 0;
}

