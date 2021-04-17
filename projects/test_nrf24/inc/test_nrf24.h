/*=============================================================================
 * Copyright (c) 2021, Ignacio Riveros <riverosignacio138@gmail.com>
 * All rights reserved.
 * License: gpl-3.0 (see LICENSE.txt)
 * Date: 2021/04/12
 * Version: 1.0
 *===========================================================================*/

/*=====[Avoid multiple inclusion - begin]====================================*/

#ifndef __TEST_NRF24_H__
#define __TEST_NRF24_H__

/*=====[Inclusions of public function dependencies]==========================*/

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "nrf24l01.h"
#include "chip.h"
/*=====[C++ - begin]=========================================================*/

#ifdef __cplusplus
extern "C" {
#endif

/*=====[Definition macros of public constants]===============================*/

/*=====[Public function-like macros]=========================================*/

/*=====[Definitions of public data types]====================================*/

typedef struct {
	float force_node; /** <= force in [Kg]*/
	float time_node; /** <= time in [ms]*/
	float battery_voltage; /** <= voltage in battery [Volts]*/
	bool data_ready; /** <= data ready flag*/
} nrf24l01p_pedal_data;

/*=====[Prototypes (declarations) of public functions]=======================*/

/*=====[Prototypes (declarations) of public interrupt functions]=============*/
void GPIO5_IRQHandler(void) {
	NVIC_DisableIRQ(PIN_INT5_IRQn);
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(5));
	GpioInterrupt(5);
	NVIC_EnableIRQ(PIN_INT5_IRQn);
}
/*=====[C++ - end]===========================================================*/

#ifdef __cplusplus
}
#endif

/*=====[Avoid multiple inclusion - end]======================================*/

#endif /* __TEST_NRF24_H__ */
