/* nRF24L01.c
 *
 *  ORIGINAL LIBRARY REPOSITORY: https://github.com/pradeepa-s/pdlib_nrf24l01
 *  See LICENSE-NRF24L01 attached on folder or visit ORIGINAL LIBRARY REPOSITORY
 *
 *  Copyright (C) 2015 Pradeepa Senanayake <pradeepa.kck@gmail.com>
 *  Copyright (C) 2018 Julián Botello <jlnbotello@gmail.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */

/** @addtogroup NRF24L01
 *  @{
 */

/*==================[inclusions]=============================================*/

#include "nrf24l01.h"
#include "chip.h"
#include "string.h"

/*==================[macros and definitions]=================================*/

#define INTERNAL_STATE_INIT				(1 << 0)	/**< Initial state Flag */
#define INTERNAL_STATE_DYNPL			(1 << 1)	/**< Dynamic Payload Flag  */
#define INTERNAL_STATE_ACKPL			(1 << 2)	/**< Acknowledge Payload Flag  */
#define INTERNAL_STATE_FEATURE_ENABLED	(1 << 3)	/**< Feature Enabled Flag  */
#define INTERNAL_STATE_POWER_UP			(1 << 4)	/**< Power Up Flag  */
#define INTERNAL_STATE_STAND_BY			(1 << 5)	/**< Stand By Flag  */

#define NUMBER_OF_DEVICES	2 	/**< Two are used when testing driver. One dev board with 2 RF modules (PRX AND PTX) */

#define BUF_SIZE 	32 			/**< Max payload */

/*==================[external data definition]===============================*/

/* Buffers of PTX */
uint8_t snd_to_PRX[BUF_SIZE]; /**< Data to send to PRX */
uint8_t rcv_fr_PRX[BUF_SIZE]; /**< Data received from PRX */
/* Buffers of PRX */
uint8_t snd_to_PTX[BUF_SIZE]; /**< Data to send to PTX */
uint8_t rcv_fr_PTX[BUF_SIZE]; /**< Data received from PTX  */

/*==================[internal data definition]===============================*/

static nrf24l01_t* primary_dev; /**< RX or TX. When testing assign TX (two RF modules in the same dev board) */
static nrf24l01_t* secondary_dev; /**< RX if there are two RF modules in the same dev board */
static uint16_t internal_states[NUMBER_OF_DEVICES]; /**< Internal states */
static uint8_t g_status[NUMBER_OF_DEVICES]; /**< Global status*/

const spiConfig_t nrf24l01_spi_default_cfg = { SSP_1, SPI_BITS8, SPI_CLK_MODE0, 1000000}; //max 8MHz

/*==================[internal functions definition]==========================*/

/*-------- SPI and GPIO control --------*/

static uint8_t Nrf24SpiReadByte(nrf24l01_t *nrf24) {
	uint8_t rx_byte;
	SpiDevReadBlocking(&nrf24->spi, &rx_byte, 1);
	return rx_byte;
}

static uint8_t Nrf24SpiReadWriteByte(nrf24l01_t *nrf24, uint8_t tx_byte) {
	uint8_t rx_byte;
	SpiDevRWBlocking(&nrf24->spi, &tx_byte, &rx_byte, 1);
	return rx_byte;
}

static void Nrf24SpiWriteMultiByte(nrf24l01_t *nrf24, uint8_t *buffer, uint32_t n) {
	SpiDevWriteBlocking(&nrf24->spi, buffer, n);
}

static void Nrf24CeLow(nrf24l01_t *nrf24) {
	GpioWrite(&nrf24->ce, GPIO_LOW);

	if (internal_states[nrf24->spi.id] & INTERNAL_STATE_POWER_UP) {
		internal_states[nrf24->spi.id] |= INTERNAL_STATE_STAND_BY;
	} else {
		internal_states[nrf24->spi.id] &= (~INTERNAL_STATE_STAND_BY);
	}
}

static void Nrf24CeHigh(nrf24l01_t *nrf24) {
	GpioWrite(&nrf24->ce, GPIO_HIGH);
	if (internal_states[nrf24->spi.id] & INTERNAL_STATE_POWER_UP) {
		internal_states[nrf24->spi.id] &= (~INTERNAL_STATE_STAND_BY);
	}
}

static void Nrf24CsLow(nrf24l01_t *nrf24) {
	GpioWrite(&nrf24->cs, GPIO_LOW);
}

static void Nrf24CsHigh(nrf24l01_t *nrf24) {
	GpioWrite(&nrf24->cs, GPIO_HIGH);
}

/*==================[external functions definition]=========================*/

/*-------- Basic APIs --------*/

void Nrf24Init(nrf24l01_t *nrf24) {
	SpiDevInit(&nrf24->spi);

	nrf24->cs.dir = GPIO_OUT;
	nrf24->cs.init_st = GPIO_HIGH; //CS is active LOW
	GpioConfig(&nrf24->cs);

	nrf24->ce.dir = GPIO_OUT;
	nrf24->ce.init_st = GPIO_LOW; //CE is active HIGH
	GpioConfig(&nrf24->ce);

	Nrf24RegisterInit(nrf24);

	internal_states[nrf24->spi.id] = 0x00;
	internal_states[nrf24->spi.id] |= INTERNAL_STATE_INIT;
}

int8_t Nrf24SendData(nrf24l01_t *nrf24, uint8_t *data, uint32_t length) {
	int8_t ret = Nrf24SubmitData(nrf24, data, length);

	if (ret == NRF24_SUCCESS) {
		ret = Nrf24AttemptTx(nrf24);
	}
	return ret;
}

int8_t Nrf24SendDataTo(nrf24l01_t *nrf24, uint8_t *address, uint8_t *data, uint32_t length) {
	int8_t ret;

	Nrf24SetTXAddress(nrf24, address);

	ret = Nrf24SendData(nrf24, data, length);

	return ret;
}

int8_t Nrf24WaitForDataRx(nrf24l01_t *nrf24, uint8_t *pipe) {
	int8_t ret = NRF24_ERROR;

	Nrf24EnableRxMode(nrf24);
	// Blocking
	while (ret == NRF24_ERROR) {
		ret = Nrf24IsDataReadyRx(nrf24, pipe);
	}
	Nrf24DisableRxMode(nrf24);

	return ret;
}

uint8_t Nrf24GetRxDataAmount(nrf24l01_t *nrf24, uint8_t pipe) {
	uint8_t reg;
	uint8_t ret = 0;

	if ((INTERNAL_STATE_DYNPL & internal_states[nrf24->spi.id]) == 0) {
		if (pipe < 6) {
			// static payload width register of Px;  x:0..5
			reg = Nrf24RegisterRead8(nrf24, NRF24_RX_PW_P0 + pipe);
			ret = (reg & 0x3F);
		} else {
			ret = 0;
		}
	} else {
		Nrf24SendRcvCommand(nrf24, NRF24_R_RX_PL_WID, &reg, 1);

		ret = reg;
	}

	return ret;
}

int8_t Nrf24GetData(nrf24l01_t *nrf24, uint8_t pipe, uint8_t *data,	uint8_t length) {
	int8_t ret = NRF24_SUCCESS;
	uint8_t fifo_status;
	uint8_t ready_length;
	uint8_t ready_pipe;

	if (null == data) {
		ret = NRF24_INVALID_ARGUMENT;
	} else {
		if (NRF24_SUCCESS == Nrf24IsDataReadyRx(nrf24, &ready_pipe)) {
			if (pipe == ready_pipe) {
				// Gets FIFO status
				fifo_status = Nrf24RegisterRead8(nrf24, NRF24_FIFO_STATUS);

				// Checks whether FIFO is empty
				if (0 == (fifo_status & NRF24_RX_EMPTY)) {

					//  Gets amount of data available
					ready_length = Nrf24GetRxDataAmount(nrf24, pipe);

					if ((length) >= ready_length) {
						(length) = ready_length;
					} else {
						// If the actual buffer size is smaller than the available data, we can miss data.
						ret = NRF24_BUFFER_TOO_SMALL;
					}

					if (NRF24_SUCCESS == ret) {

						// Read RX payload
						Nrf24ReadRxPayload(nrf24, data, ready_length);

						// TODO: Check whether we need to check the actual FIFO state before clearing the interrupt.
						// Clear RX_DR
						Nrf24ClearInterruptFlag(nrf24, INTERRUPT_DATA_READY);

						ret = ready_length;
					}

				} else {
					ret = NRF24_ERROR;
				}
			} else {
				ret = NRF24_INVALID_ARGUMENT;
			}
		} else {
			ret = NRF24_ERROR;
		}
	}

	return ret;
}

/*-------- Intermediate APIs --------*/

void Nrf24RegisterInit(nrf24l01_t *nrf24) {
	uint8_t ucRxAddr1[5] = { 0xE7, 0xE7, 0xE7, 0xE7, 0xE7 };
	uint8_t ucRxAddr2[5] = { 0xC2, 0xC2, 0xC2, 0xC2, 0xC2 };

	Nrf24FlushTX(nrf24);
	Nrf24FlushRX(nrf24);

	Nrf24CeLow(nrf24);
	//FIXME
	//Nrf24RegisterWrite8(nrf24,NRF24_CONFIG,0x09);
	Nrf24RegisterWrite8(nrf24, NRF24_CONFIG, NRF24_EN_CRC | NRF24_PRIM_RX);
	//Nrf24RegisterWrite8(nrf24,NRF24_EN_AA,0x3F);
	Nrf24RegisterWrite8(nrf24, NRF24_EN_AA,
			NRF24_ENAA_P5 | NRF24_ENAA_P4 | NRF24_ENAA_P3 | NRF24_ENAA_P2
					| NRF24_ENAA_P1 | NRF24_ENAA_P0);
	//Nrf24RegisterWrite8(nrf24,NRF24_EN_RXADDR,0x03);
	Nrf24RegisterWrite8(nrf24, NRF24_EN_RXADDR, NRF24_ERX_P1 | NRF24_ERX_P0);
	//Nrf24RegisterWrite8(nrf24,NRF24_SETUP_AW,0x03);
	Nrf24RegisterWrite8(nrf24, NRF24_SETUP_AW, NRF24_AW_5BYTES);
	//Nrf24RegisterWrite8(nrf24,NRF24_SETUP_RETR,0x03);
	Nrf24RegisterWrite8(nrf24, NRF24_SETUP_RETR, 0x00 | 0x03); //ARD(Auto Retransmit Delay)|ARC(Auto Retransmit Count) 250 us | 3 retransmitions
	Nrf24RegisterWrite8(nrf24, NRF24_RF_CH, 0x02);
	Nrf24RegisterWrite8(nrf24, NRF24_RF_SETUP, 0x0F); //air data rate 2Mbps | max power output| setup LNA gain
	Nrf24RegisterWrite8(nrf24, NRF24_STATUS, 0x70); // Clears interrupts RX_DR, TX_DS and MAX_RT
	//Nrf24RegisterWrite8(nrf24,NRF24_CD, 0x00); Not necessary. Read only register
	Nrf24RegisterWriteMulti(nrf24, NRF24_RX_ADDR_P0, ucRxAddr1, 5); // Writes default RX address in P0.
	Nrf24RegisterWriteMulti(nrf24, NRF24_RX_ADDR_P1, ucRxAddr2, 5); // Writes default RX address in P1.
	Nrf24RegisterWrite8(nrf24, NRF24_RX_ADDR_P2, 0xC3); // First MSBytes equal to P1. Writes last(right) byte address.
	Nrf24RegisterWrite8(nrf24, NRF24_RX_ADDR_P3, 0xC4); // First MSBytes equal to P1
	Nrf24RegisterWrite8(nrf24, NRF24_RX_ADDR_P4, 0xC5); // First MSBytes equal to P1
	Nrf24RegisterWrite8(nrf24, NRF24_RX_ADDR_P5, 0xC6); // First MSBytes equal to P1
	Nrf24RegisterWriteMulti(nrf24, NRF24_TX_ADDR, ucRxAddr1, 5); //Writes default TX address
	Nrf24RegisterWrite8(nrf24, NRF24_RX_PW_P0, 0x00); // Pipe not used because 0 bytes payload. MUST BE SET LATER
	Nrf24RegisterWrite8(nrf24, NRF24_RX_PW_P1, 0x00); // Pipe not used because 0 bytes payload
	Nrf24RegisterWrite8(nrf24, NRF24_RX_PW_P2, 0x00); // Pipe not used because 0 bytes payload
	Nrf24RegisterWrite8(nrf24, NRF24_RX_PW_P3, 0x00); // Pipe not used because 0 bytes payload
	Nrf24RegisterWrite8(nrf24, NRF24_RX_PW_P4, 0x00); // Pipe not used because 0 bytes payload
	Nrf24RegisterWrite8(nrf24, NRF24_RX_PW_P5, 0x00); // Pipe not used because 0 bytes payload
	Nrf24RegisterWrite8(nrf24, NRF24_DYNPD, 0x00); //dynamic payload disable in all pipes
	Nrf24RegisterWrite8(nrf24, NRF24_FEATURE, 0x00);
}

void Nrf24PowerDown(nrf24l01_t *nrf24) {
	uint8_t current_val = Nrf24RegisterRead8(nrf24, NRF24_CONFIG);
	current_val &= (~NRF24_PWR_UP);

	Nrf24RegisterWrite8(nrf24, NRF24_CONFIG, current_val);

	Nrf24CeLow(nrf24);

	internal_states[nrf24->spi.id] &= (~INTERNAL_STATE_POWER_UP);
}

void Nrf24PowerUp(nrf24l01_t *nrf24) {
	uint8_t current_val = Nrf24RegisterRead8(nrf24, NRF24_CONFIG);
	current_val |= (NRF24_PWR_UP);
	Nrf24RegisterWrite8(nrf24, NRF24_CONFIG, current_val);

	internal_states[nrf24->spi.id] |= INTERNAL_STATE_POWER_UP;
}

void Nrf24SetARD(nrf24l01_t *nrf24, uint16_t value) {
	uint8_t reg_val = 0;
	uint8_t cur_val = 0;

	if (value < 250) {
		value = 250;
	}

	// 250 - 0x0000
	// 500 - 0x0001
	// ...
	// 4000 - 0x1111
	value = ((value / 250) - 1);

	reg_val = ((value << 4) & 0xF0);

	cur_val = Nrf24RegisterRead8(nrf24, NRF24_SETUP_RETR);
	cur_val &= 0x0F;
	cur_val |= reg_val;
	Nrf24RegisterWrite8(nrf24, NRF24_SETUP_RETR, cur_val);
}

uint8_t Nrf24GetStatus(nrf24l01_t *nrf24) {
	Nrf24RegisterRead8(nrf24, NRF24_NOP);
	return g_status[nrf24->spi.id];
}

void Nrf24EnableFeatureDynPL(nrf24l01_t *nrf24, uint8_t pipe) {
	uint8_t data = 0x73;

	if ((internal_states[nrf24->spi.id] & INTERNAL_STATE_STAND_BY)
			|| (0 == (internal_states[nrf24->spi.id] & INTERNAL_STATE_POWER_UP))) {
		// Check whether features register is activated
		if (0
				== (internal_states[nrf24->spi.id]
						& INTERNAL_STATE_FEATURE_ENABLED)) {
			uint8_t data = 0x73;
			Nrf24SendCommand(nrf24, NRF24_ACTIVATE, &data, 1);

			internal_states[nrf24->spi.id] |= INTERNAL_STATE_FEATURE_ENABLED;
		}

		// Check whether DYN-PL feature is activated
		data = Nrf24RegisterRead8(nrf24, NRF24_FEATURE);

		if (0 == (data & NRF24_EN_DPL)) {
			Nrf24RegisterWrite8(nrf24, NRF24_FEATURE, (data | NRF24_EN_DPL));
		}

		if (pipe <= 6) {
			// Check whether DYN-PD for 'pipe' is activated
			data = Nrf24RegisterRead8(nrf24, NRF24_DYNPD);

			if (0 == (data & pipe)) {
				Nrf24RegisterWrite8(nrf24, NRF24_DYNPD,
						(data | (1 << pipe)));
			}
		}

		internal_states[nrf24->spi.id] |= INTERNAL_STATE_DYNPL;
	}
}

void Nrf24EnableFeatureAckPL(nrf24l01_t *nrf24) {
	uint8_t data = 0x73;

	if ((internal_states[nrf24->spi.id] & INTERNAL_STATE_STAND_BY)
			|| (0 == (internal_states[nrf24->spi.id] & INTERNAL_STATE_POWER_UP))) {
		//  Enable dynpl for pipe0
		Nrf24EnableFeatureDynPL(nrf24, 0x00);

		//  Check whether retransmission delay is sufficient
		data = Nrf24RegisterRead8(nrf24, NRF24_SETUP_RETR);

		if (0 == ((data & 0xF0) >> 4)) {
			Nrf24SetARD(nrf24, 500);
		}

		//  Enable auto ack payload
		data = Nrf24RegisterRead8(nrf24, NRF24_FEATURE);
		Nrf24RegisterWrite8(nrf24, NRF24_FEATURE, (data | NRF24_EN_ACK_PAY));

		internal_states[nrf24->spi.id] |= INTERNAL_STATE_ACKPL;
	}
}

uint8_t Nrf24GetInterruptStatus(nrf24l01_t *nrf24) {
	uint8_t status = Nrf24GetStatus(nrf24);

	status &= (NRF24_RX_DR | NRF24_TX_DS | NRF24_MAX_RT);
	status = (status >> 4);

	return status;
}

void Nrf24ClearInterruptFlag(nrf24l01_t *nrf24, uint8_t interrupt_bm) {
	uint8_t status = Nrf24GetStatus(nrf24);

	status &= ~(NRF24_RX_DR | NRF24_TX_DS | NRF24_MAX_RT);
	// Write 1 to clear bit
	if (interrupt_bm & INTERRUPT_MAX_RT) {
		status |= NRF24_MAX_RT;
	}

	if (interrupt_bm & INTERRUPT_DATA_READY) {
		status |= NRF24_RX_DR;
	}

	if (interrupt_bm & INTERRUPT_DATA_SENT) {
		status |= NRF24_TX_DS;
	}

	Nrf24RegisterWrite8(nrf24, NRF24_STATUS, status);
}

/*-------- TX mode related --------*/

void Nrf24FlushTX(nrf24l01_t *nrf24) {
	Nrf24SendCommand(nrf24, NRF24_FLUSH_TX, null, 0);
}

void Nrf24SetTXAddress(nrf24l01_t *nrf24, uint8_t* address) {
	Nrf24RegisterWriteMulti(nrf24, NRF24_TX_ADDR, (uint8_t*) address, 5);
}

int8_t Nrf24SetTxPayload(nrf24l01_t *nrf24, uint8_t* data, uint32_t length) {
	int8_t ret = NRF24_SUCCESS;

	if (data && length > 0) {
		//  Check whether TX fifo is full
		if (Nrf24IsTxFifoFull(nrf24)) {
			ret = NRF24_TX_FIFO_FULL;

		} else {
			Nrf24SendCommand(nrf24, NRF24_W_TX_PAYLOAD, data, length);
		}
	} else {
		ret = NRF24_ERROR;
	}

	return ret;
}

int8_t Nrf24SubmitData(nrf24l01_t *nrf24, uint8_t *data, uint32_t length) {
	int8_t ret;
	uint8_t address[5];
	uint8_t current_val;

	current_val = Nrf24RegisterRead8(nrf24, NRF24_EN_AA);
	 // Check the Auto Ack feature
	if (current_val & NRF24_ENAA_P0) {
		// Make sure the data pipe 0 has the correct PTX address
		Nrf24RegisterReadMulti(nrf24, NRF24_TX_ADDR, address, 5);
		Nrf24SetRxAddress(nrf24, NRF24_PIPE0, address);
	}
	ret = Nrf24SetTxPayload(nrf24, data, length);

	return ret;
}

void Nrf24EnableTxMode(nrf24l01_t *nrf24) {
	uint8_t current_val = 0;

	//  Power up the device
	Nrf24PowerUp(nrf24);

	//  Clear TX_DS and MAX_RT interrupts
	Nrf24ClearInterruptFlag(nrf24,INTERRUPT_MAX_RT | INTERRUPT_DATA_SENT);

	//  Set to TX mode
	current_val = Nrf24RegisterRead8(nrf24, NRF24_CONFIG);
	current_val &= (~NRF24_PRIM_RX);

	Nrf24RegisterWrite8(nrf24, NRF24_CONFIG, current_val);

	Nrf24CeHigh(nrf24);

}

void Nrf24DisableTxMode(nrf24l01_t *nrf24) {
	//uint8_t current_val = Nrf24GetStatus(nrf24);

	Nrf24CeLow(nrf24);
	//  Clear TX_DS and MAX_RT interrupts TODO: why?
	Nrf24ClearInterruptFlag(nrf24, INTERRUPT_MAX_RT | INTERRUPT_DATA_SENT);
}

uint8_t Nrf24IsTxFifoFull(nrf24l01_t *nrf24) {
	uint8_t tx_fifo_status;

	tx_fifo_status = Nrf24RegisterRead8(nrf24, NRF24_FIFO_STATUS);

	return ((tx_fifo_status & NRF24_FIFO_FULL) ? 1 : 0);
}

int8_t Nrf24IsTxFifoEmpty(nrf24l01_t *nrf24) {
	uint8_t tx_fifo_status;

	tx_fifo_status = Nrf24RegisterRead8(nrf24, NRF24_FIFO_STATUS);

	return ((tx_fifo_status & NRF24_TX_EMPTY) ? 1 : 0);
}

int8_t Nrf24AttemptTx(nrf24l01_t *nrf24) {
	int8_t ret = NRF24_SUCCESS;

	Nrf24EnableTxMode(nrf24);

	ret = Nrf24WaitForTxComplete(nrf24, 1);

	Nrf24DisableTxMode(nrf24);

	Nrf24PowerDown(nrf24);

	return ret;
}

int8_t Nrf24WaitForTxComplete(nrf24l01_t *nrf24, uint8_t busy_wait) {
	int8_t ret = NRF24_SUCCESS;

	Nrf24GetStatus(nrf24);

	if (busy_wait) {
		while ((g_status[nrf24->spi.id] & (NRF24_MAX_RT | NRF24_TX_DS)) == 0) {
			Nrf24GetStatus(nrf24);
		}
	} else {
		if ((g_status[nrf24->spi.id] & (NRF24_MAX_RT | NRF24_TX_DS)) == 0) {
			ret = NRF24_ERROR;
		}
	}

	if (g_status[nrf24->spi.id] & NRF24_MAX_RT) {

		ret = NRF24_TX_ARC_REACHED;
	}

	return ret;
}

uint8_t Nrf24GetAckDataAmount(nrf24l01_t *nrf24){
	uint8_t data_amount = 0;

	Nrf24SendRcvCommand(nrf24,NRF24_R_RX_PL_WID,&data_amount,1);

	return data_amount;
}

/*-------- RX mode related --------*/

void Nrf24FlushRX(nrf24l01_t *nrf24) {
	Nrf24SendCommand(nrf24, NRF24_FLUSH_RX, null, 0);
}

void Nrf24SetRxAddress(nrf24l01_t *nrf24, uint8_t pipe, uint8_t *address) {
	if (address) {
		switch (pipe) {
		case 0:
		case 1:
			Nrf24RegisterWriteMulti(nrf24, (NRF24_RX_ADDR_P0 + pipe),
					address, 5);
			break;
		case 2:
		case 3:
		case 4:
		case 5:
			Nrf24RegisterWrite8(nrf24, (NRF24_RX_ADDR_P0 + pipe),
					address[0]);
			break;
		default:
			break;
		}
	}
}

void Nrf24SetRXPacketSize(nrf24l01_t *nrf24, uint8_t pipe, uint8_t pkt_sz) {
	if (pkt_sz <= 32) {
		Nrf24RegisterWrite8(nrf24, (NRF24_RX_PW_P0 + pipe),pkt_sz);
	}
}

void Nrf24EnableRxMode(nrf24l01_t *nrf24) {
	uint8_t current_val = Nrf24GetStatus(nrf24);

	Nrf24PowerUp(nrf24);

	//  Clear RX_DR interrupt
	Nrf24ClearInterruptFlag(nrf24, INTERRUPT_DATA_READY);

	current_val = Nrf24RegisterRead8(nrf24, NRF24_CONFIG);
	current_val |= (NRF24_PRIM_RX | NRF24_PWR_UP);

	Nrf24RegisterWrite8(nrf24, NRF24_CONFIG, current_val);

	Nrf24CeHigh(nrf24);
}

void Nrf24DisableRxMode(nrf24l01_t *nrf24) {
	Nrf24CeLow(nrf24);
}

int8_t Nrf24IsDataReadyRx(nrf24l01_t *nrf24, uint8_t *pipe) {
	int8_t ret = NRF24_ERROR;
	uint8_t data_ready;

	if (null == pipe) {
		ret = NRF24_INVALID_ARGUMENT;
	} else {
		Nrf24GetStatus(nrf24);

		data_ready = ((g_status[nrf24->spi.id] & NRF24_RX_DR) ? 1 : 0);
		*pipe = ((g_status[nrf24->spi.id] & ( BIT3 | BIT2 | BIT1)) >> 1);

		// pipe will be 7 if the RX fifo is empty
		if (data_ready && (*pipe < 6)) {
			ret = NRF24_SUCCESS;
		} else {
			// Reset the value in the pipe number
			*pipe = 0xFF;
			ret = NRF24_ERROR;
		}
	}

	return ret;
}

void Nrf24ReadRxPayload(nrf24l01_t *nrf24, uint8_t* data, uint8_t length) {
	Nrf24SendRcvCommand(nrf24, NRF24_R_RX_PAYLOAD, data, length);
}

int8_t Nrf24SetAckPayload(nrf24l01_t *nrf24, uint8_t *data, uint8_t pipe, uint32_t length) {
	int8_t ret = NRF24_SUCCESS;
	uint8_t address = pipe;

	if (pipe < 6 && data && length > 0) {
		//  Check whether TX fifo is full
		if (Nrf24IsTxFifoFull(nrf24)) {
			ret = NRF24_TX_FIFO_FULL;
		} else {
			address = address & 0x07;
			address |= NRF24_W_ACK_PAYLOAD;

			Nrf24SendCommand(nrf24, address, data, length);
		}
	} else {
		ret = NRF24_ERROR;
	}

	return ret;
}

/*-------- Advanced APIs, Register level access --------*/

uint8_t Nrf24RegisterRead8(nrf24l01_t *nrf24, uint8_t reg) {
	uint8_t ucData;

	Nrf24CsLow(nrf24);

	g_status[nrf24->spi.id] = Nrf24SpiReadWriteByte(nrf24,
			NRF24_R_REGISTER | reg);

	ucData = Nrf24SpiReadWriteByte(nrf24, NRF24_NOP);

	Nrf24CsHigh(nrf24);

	return ucData;
}

uint8_t Nrf24RegisterReadMulti(nrf24l01_t *nrf24, uint8_t reg, uint8_t *buffer, uint32_t length) {
	uint32_t i;

	Nrf24CsLow(nrf24);

	g_status[nrf24->spi.id] = Nrf24SpiReadWriteByte(nrf24, NRF24_R_REGISTER | reg);

	for (i = 0; i < length; i++) {
		buffer[i] = Nrf24SpiReadWriteByte(nrf24, NRF24_NOP);
	}

	Nrf24CsHigh(nrf24);

	return g_status[nrf24->spi.id];
}

void Nrf24RegisterWrite8(nrf24l01_t *nrf24, uint8_t reg, uint8_t value) {
	uint8_t buffer[2];

	buffer[0] = (NRF24_W_REGISTER | reg);
	buffer[1] = value;

	Nrf24CsLow(nrf24);

	//  Send address and get status
	g_status[nrf24->spi.id] = Nrf24SpiReadWriteByte(nrf24, buffer[0]);

	//  Send data
	Nrf24SpiReadWriteByte(nrf24, buffer[1]);

	Nrf24CsHigh(nrf24);
}

void Nrf24RegisterWriteMulti(nrf24l01_t *nrf24, uint8_t reg, uint8_t *data, uint16_t length) {
	Nrf24CsLow(nrf24);

	g_status[nrf24->spi.id] = Nrf24SpiReadWriteByte(nrf24,NRF24_W_REGISTER | reg);
	Nrf24SpiWriteMultiByte(nrf24, data, length);

	Nrf24CsHigh(nrf24);
}

void Nrf24SendCommand(nrf24l01_t *nrf24, uint8_t command, uint8_t *data, uint32_t length) {
	uint8_t rx_buffer[length+1];
	rx_buffer[0] = command;
	for (uint8_t i = 0; i < length; i++) {
		rx_buffer[i + 1] = data[i];
	}
	Nrf24CsLow(nrf24);
	Nrf24SpiWriteMultiByte(nrf24, rx_buffer, length + 1);
	Nrf24CsHigh(nrf24);
}

void Nrf24SendRcvCommand(nrf24l01_t *nrf24, uint8_t command, uint8_t *data, uint32_t length) {
	if (data) {
		uint32_t i = 0;

		Nrf24CsLow(nrf24);

		g_status[nrf24->spi.id] = Nrf24SpiReadWriteByte(nrf24, command);

		for (i = 0; i < length; i++) {
			data[i] = Nrf24SpiReadWriteByte(nrf24, NRF24_NOP);
		}

		Nrf24CsHigh(nrf24);
	}
}

/*-------- Interrupts ISR configuration --------*/

void Nrf24PrimaryDevISRConfig(nrf24l01_t *nrf24){
	primary_dev=nrf24;
	nrf24->irq.dir=GPIO_IN_PULLUP;
	GpioConfig(&nrf24->irq);
	GpioInterruptConfig(&nrf24->irq, GPIO_IRQ_LEVEL_LOW, nrf24->pin_int_num, Nrf24PrimaryDevISR);
}

void Nrf24SecondaryDevISRConfig(nrf24l01_t *nrf24){
	secondary_dev=nrf24;
	nrf24->irq.dir=GPIO_IN_PULLUP;
	GpioConfig(&nrf24->irq);
	GpioInterruptConfig(&nrf24->irq, GPIO_IRQ_LEVEL_LOW, nrf24->pin_int_num, Nrf24SecondaryDevISR);
}

void Nrf24PrimaryDevISR(){
	Nrf24SelectISR(primary_dev);
}

void Nrf24SecondaryDevISR(){
	Nrf24SelectISR(secondary_dev);
}

void Nrf24SelectISR(nrf24l01_t *nrf24){
	if(nrf24->mode==PRX){
		if(nrf24->en_ack_pay==true){
			Nrf24ReceiveDataACKPayISR(nrf24, rcv_fr_PTX,snd_to_PTX,BUF_SIZE);
		} else {
			Nrf24ReceiveDataISR(nrf24, rcv_fr_PTX);
		}
	} else {
		if(nrf24->en_ack_pay==true){
			Nrf24TransmitDataACKPayISR(nrf24,rcv_fr_PRX);
		} else {
			Nrf24TransmitDataISR(nrf24);
		}
	}
}

void Nrf24TransmitDataISR(nrf24l01_t *nrf24) {
	int8_t status;

	/* Check the interrupt status from module to see whether
	 * the interrupt is due to Data-Sent or ARC reached.
	 */
	status = Nrf24WaitForTxComplete(nrf24, 0);

	if (NRF24_TX_ARC_REACHED == status) {

		// Retry transmission
		Nrf24EnableTxMode(nrf24);

	} else if (NRF24_SUCCESS == status) {

		if (0 == Nrf24IsTxFifoEmpty(nrf24)) {
			// Since TX FIFO is not empty we'll retry sending the rest of the data
			Nrf24EnableTxMode(nrf24);
		} else {
			// No data in the FIFO we'll disable the TX mode
			Nrf24DisableTxMode(nrf24);
			Nrf24PowerDown(nrf24);
		}
	}
}

void Nrf24TransmitDataACKPayISR(nrf24l01_t *nrf24,uint8_t *rcv_ack_pay) {
	int8_t status;
	uint8_t interrupt_flag = 0;
	uint8_t pipe = 0;
	uint8_t data_amount;

	/* Check the interrupt status from module to see whether
	   the interrupt is due to Data-Sent/Rcv or ARC reached.
	 */
	interrupt_flag = Nrf24GetInterruptStatus(nrf24);

	if (interrupt_flag & INTERRUPT_DATA_READY) {

		/* Data ready in PTX device means it is an ACK payload
		 * Get the pipe address, data amount and data. */
		status = Nrf24IsDataReadyRx(nrf24, &pipe); // Always will be pipe0 in PTX mode

		if (NRF24_SUCCESS == status) {
			data_amount = Nrf24GetAckDataAmount(nrf24); //From pipe0

			if (data_amount > 0) {
				if (rcv_ack_pay!=null) {
					Nrf24ReadRxPayload(nrf24,rcv_ack_pay, data_amount);

					// Clear interrupt
					Nrf24ClearInterruptFlag(nrf24,INTERRUPT_DATA_READY);
				}
			}
		}
	}

	if (interrupt_flag & INTERRUPT_MAX_RT) {
		// Retry transmission
		Nrf24EnableTxMode(nrf24);
	}

	if (interrupt_flag & INTERRUPT_DATA_SENT) {

		if (0 == Nrf24IsTxFifoEmpty(nrf24)) {
			// Since TX FIFO is not empty we'll retry sending the rest of the data
			Nrf24EnableTxMode(nrf24);
		} else {
			// No data in the FIFO we'll disable the TX mode
			Nrf24DisableTxMode(nrf24);
			Nrf24PowerDown(nrf24);
		}
	}
}

void Nrf24ReceiveDataISR(nrf24l01_t *nrf24, uint8_t *data) {

	int8_t status;
	uint8_t pipe_no = 0;
	uint8_t temp;

	// Check which pipe contains data
	status = Nrf24IsDataReadyRx(nrf24, &pipe_no);

	if (NRF24_SUCCESS == status) {
		// Get data amount in the pipe
		temp = Nrf24GetRxDataAmount(nrf24, pipe_no);

		memset(data, 0x00, 32); //FIXME: change this implementation to avoid memset

		// Get data from the pipe
		status = Nrf24GetData(nrf24, pipe_no, data, temp);

	}
}

void Nrf24ReceiveDataACKPayISR(nrf24l01_t *nrf24, uint8_t *data,uint8_t *next_ack_pay, uint8_t ack_pay_len) {

	int8_t status;
	uint8_t pipe_no = 0;
	uint8_t temp;
	uint8_t interrupt_flag = 0;

	// Check which pipe contains data
	status = Nrf24IsDataReadyRx(nrf24, &pipe_no);

	if (NRF24_SUCCESS == status) {
		// Get data amount in the pipe
		temp = Nrf24GetRxDataAmount(nrf24, pipe_no);

		// Get data from the pipe
		status = Nrf24GetData(nrf24, pipe_no, data, temp);

		// Put the next ack payload
		Nrf24SetAckPayload(nrf24,next_ack_pay, pipe_no, ack_pay_len);
	}

	// Check whether ACK payload is properly sent
	interrupt_flag = Nrf24GetInterruptStatus(nrf24);

	if (INTERRUPT_DATA_SENT & interrupt_flag) {
		Nrf24ClearInterruptFlag(nrf24, INTERRUPT_DATA_SENT | INTERRUPT_MAX_RT);
	}
}

void Nrf24TxTick(){
	int8_t status;
	if (1 == Nrf24IsTxFifoFull(primary_dev)) {
		Nrf24FlushTX(primary_dev);
	}
	// Send data
	status = Nrf24SubmitData(primary_dev, snd_to_PRX,BUF_SIZE);
	if (NRF24_SUCCESS == status) {
		Nrf24EnableTxMode(primary_dev);
	}

}

/** @} doxygen end group definition */

