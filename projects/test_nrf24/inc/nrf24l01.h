/* nRF24L01.h
 * Register definitions for manipulating the Nordic Semiconductor
 * nRF24L01+ RF transceiver chipsets.
 *
 *  Copyright (c) 2007 Stefan Engelke <mbox@stefanengelke.de>
 *  Some parts copyright (c) 2012 Eric Brundick <spirilis [at] linux dot com>
 *	Some parts copyright (c) 2018 Julián Botello <jlnbotello@gmail.com>
 *
 *  Permission is hereby granted, free of charge, to any person
 *  obtaining a copy of this software and associated documentation
 *  files (the "Software"), to deal in the Software without
 *  restriction, including without limitation the rights to use, copy,
 *  modify, merge, publish, distribute, sublicense, and/or sell copies
 *  of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *  The above copyright notice and this permission notice shall be
 *  included in all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 *  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 *  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 *  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 *  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 *  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 *  DEALINGS IN THE SOFTWARE.
 */

/** @addtogroup NRF24L01
   \brief Driver for RF transceiver NRF24L01
*  @{
*/

#ifndef MODULES_LPC4337_M4_DRIVERS_BM_INC_NRF24H_
#define MODULES_LPC4337_M4_DRIVERS_BM_INC_NRF24H_

/*==================[inclusions]=============================================*/

#include "spi_generic_device.h"
#include "gpio_hal.h"

/*==================[macros and definitions]=================================*/

#define	BIT0	1 << 0
#define	BIT1	1 << 1
#define	BIT2	1 << 2
#define	BIT3	1 << 3
#define	BIT4	1 << 4
#define	BIT5	1 << 5
#define	BIT6	1 << 6
#define	BIT7	1 << 7


/* Register map - Page 53 of nRF24L01 Product Specification */

#define NRF24_CONFIG      0x00 	/**< Configuration Register */
#define NRF24_EN_AA       0x01 	/**< AutoACK*/
#define NRF24_EN_RXADDR   0x02 	/**< Enabled RX Addresses */
#define NRF24_SETUP_AW    0x03 	/**< Setup of Address Widths */
#define NRF24_SETUP_RETR  0x04 	/**< Setup of Automatic Retransmission */
#define NRF24_RF_CH       0x05 	/**< RF Channel - Selected with 7 LSBits*/
#define NRF24_RF_SETUP    0x06 	/**< RF Setup Register */
#define NRF24_STATUS      0x07 	/**< Status Register */
#define NRF24_OBSERVE_TX  0x08 	/**< Transmit observe register */
#define NRF24_CD          0x09 	/**< Carrier Detect */
#define NRF24_RX_ADDR_P0  0x0A 	/**< Receive address data pipe 0 */
#define NRF24_RX_ADDR_P1  0x0B 	/**< Receive address data pipe 1 */
#define NRF24_RX_ADDR_P2  0x0C 	/**< Receive address data pipe 2 */
#define NRF24_RX_ADDR_P3  0x0D 	/**< Receive address data pipe 3 */
#define NRF24_RX_ADDR_P4  0x0E 	/**< Receive address data pipe 4 */
#define NRF24_RX_ADDR_P5  0x0F 	/**< Receive address data pipe 5 */
#define NRF24_TX_ADDR     0x10 	/**< Transmit address */
#define NRF24_RX_PW_P0    0x11 	/**< To set number of bytes in RX payload in data pipe 0 */
#define NRF24_RX_PW_P1    0x12 	/**< To set number of bytes in RX payload in data pipe 1 */
#define NRF24_RX_PW_P2    0x13 	/**< To set number of bytes in RX payload in data pipe 2 */
#define NRF24_RX_PW_P3    0x14 	/**< To set number of bytes in RX payload in data pipe 3 */
#define NRF24_RX_PW_P4    0x15 	/**< To set number of bytes in RX payload in data pipe 4 */
#define NRF24_RX_PW_P5    0x16 	/**< To set number of bytes in RX payload in data pipe 5 */
#define NRF24_FIFO_STATUS 0x17 	/**< FIFO Status Register */
#define NRF24_DYNPD       0x1C 	/**< Enable dynamic payload length */
#define NRF24_FEATURE     0x1D 	/**< Feature Register */


/* Register bit offsets */

/*Configuration Register bits*/
#define NRF24_MASK_RX_DR  BIT6
#define NRF24_MASK_TX_DS  BIT5
#define NRF24_MASK_MAX_RT BIT4
#define NRF24_EN_CRC      BIT3
#define NRF24_CRCO        BIT2
#define NRF24_PWR_UP      BIT1
#define NRF24_PRIM_RX     BIT0
/* Enable ‘Auto Acknowledgment - Pipe 5 to 0 */
#define NRF24_ENAA_P5     BIT5
#define NRF24_ENAA_P4     BIT4
#define NRF24_ENAA_P3     BIT3
#define NRF24_ENAA_P2     BIT2
#define NRF24_ENAA_P1     BIT1
#define NRF24_ENAA_P0     BIT0
/* Enabled RX Addresses - Pipe 5 to 0 */
#define NRF24_ERX_P5      BIT5
#define NRF24_ERX_P4      BIT4
#define NRF24_ERX_P3      BIT3
#define NRF24_ERX_P2      BIT2
#define NRF24_ERX_P1      BIT1
#define NRF24_ERX_P0      BIT0
/* Setup of Address Widths for all pipes - (01) 3 Bytes|(10) 4 Bytes|(11) 5 Bytes */
#define NRF24_AW          BIT0
#define NRF24_AW_3BYTES		1
#define NRF24_AW_4BYTES		2
#define NRF24_AW_5BYTES		3
/* Setup of Automatic Retransmission bits */
#define NRF24_ARD         BIT4 /**<Auto Retransmit Delay - Min: 250us, steps of 250us */
#define NRF24_ARC         BIT0 /**< Auto Retransmit Count - Up to 15 */
/* RF Setup Register bits */
#define NRF24_PLL_LOCK    BIT4
#define NRF24_RF_DR       BIT3
#define NRF24_RF_PWR      BIT1
#define NRF24_LNA_HCURR   BIT0
/* Status Register */
#define NRF24_RX_DR       BIT6 /**< Data Ready */
#define NRF24_TX_DS       BIT5 /**< Data Sent */
#define NRF24_MAX_RT      BIT4 /**< Maximum Retransmission reached */
#define NRF24_RX_P_NO     BIT1 /**< Data pipe number for the payload available for reading from RX_FIFO */
#define NRF24_TX_FULL     BIT0 /**<  TX FIFO full flag */
/* Transmit observe register */
#define NRF24_PLOS_CNT    BIT4
#define NRF24_ARC_CNT     BIT0
/* FIFO Status Register bits*/
#define NRF24_TX_REUSE    BIT6
#define NRF24_FIFO_FULL   BIT5
#define NRF24_TX_EMPTY    BIT4
#define NRF24_RX_FULL     BIT1
#define NRF24_RX_EMPTY    BIT0
/* Enable dynamic payload length bits */
#define NRF24_DYNPL_P5	 BIT5
#define NRF24_DYNPL_P4	 BIT4
#define NRF24_DYNPL_P3	 BIT3
#define NRF24_DYNPL_P2	 BIT2
#define NRF24_DYNPL_P1	 BIT1
#define NRF24_DYNPL_P0	 BIT0
/* Feature Register bits */
#define NRF24_EN_DPL      BIT2
#define NRF24_EN_ACK_PAY  BIT1
#define NRF24_EN_DYN_ACK  BIT0


/* INSTRUCTIONS - Page 46 of nRF24L01 Product Specification */

/* Read command and status registers - Make a OR with the 5 bit(LSBits) Register Map  Address
 * Data: 1 to 5 LSByte first*/
#define NRF24_R_REGISTER    0x00

/* Write command and status registers - in Power Down or Stand By modes
 * Data: 1 to 5 LSByte first*/
#define NRF24_W_REGISTER    0x20

/* Mask to make the OR with read register command
 * Data: 1 to 32 LSByte first*/
#define NRF24_REGISTER_MASK 0x1F

/* Read RX payload: 1 – 32 bytes
 * Data: 1 to 32 LSByte first*/
#define NRF24_R_RX_PAYLOAD  0x61

/* Write TX payload: 1 – 32 bytes */
#define NRF24_W_TX_PAYLOAD  0xA0

/* Flush TX FIFO, used in TX mode */
#define NRF24_FLUSH_TX      0xE1

/* Flush RX FIFO, used in RX mode */
#define NRF24_FLUSH_RX      0xE2

/* Used for a PTX device - Reuse last transmitted payload */
#define NRF24_REUSE_TX_PL   0xE3

/* R_RX_PL_WID, W_ACK_PAYLOAD, and W_TX_PAYLOAD_NOACK features registers are initially in a deactivated state
 * Data: send 1 byte 0x73 data after the command(NRF24_ACTIVATE) to activate them, send it again to deactivate */
#define NRF24_ACTIVATE		0x50

/* Read RX payload width for the top R_RX_PAYLOAD in the RX FIFO */
#define NRF24_R_RX_PL_WID   0x60

/* Write payload to be transmitted together with ACK packet on PIPE PPP. OR with PPP pipe */
#define NRF24_W_ACK_PAYLOAD 0xA8

/* Used in TX mode. Disables AUTOACK on this specific packet */
#define NRF24_W_TX_PAYLOAD_NOACK 0xB0

/* No Operation. Might be used to read the STATUS register */
#define NRF24_NOP 0xFF


/* Return codes */

#define NRF24_SUCCESS			 0
#define NRF24_ERROR				-1
#define NRF24_TX_FIFO_FULL		-2
#define NRF24_TX_ARC_REACHED	-3 	/**< Maximum Automatic Retransmission Counter reached */
#define NRF24_INVALID_ARGUMENT	-4
#define NRF24_BUFFER_TOO_SMALL	-5


/* Pipes numbers */

#define NRF24_PIPE0	0
#define NRF24_PIPE1	1
#define NRF24_PIPE2	2
#define NRF24_PIPE3	3
#define NRF24_PIPE4	4
#define NRF24_PIPE5	5


/* Interrupt masks*/

#define INTERRUPT_MAX_RT		1 << 0 /**< Maximum retransmissions reached */
#define INTERRUPT_DATA_SENT		1 << 1
#define INTERRUPT_DATA_READY	1 << 2

/*==================[typedef]================================================*/

/* RF modes */

typedef enum{
	PRX,PTX
}nrf24l01Mode_t;


/**
 * @brief NRF24L01 configuration structure
 */
typedef struct{
	spiDevice_t spi; 		/**< SPI device configuration structure  s*/
	nrf24l01Mode_t mode;	/**< Transmitter (PTX) or receiver (PRX) */
	gpioPin_t cs;			/**< Chip Select 	*/
	gpioPin_t ce;			/**< Chip Enable 	*/
	gpioPin_t irq;			/**< IRQ pin 		*/
	uint8_t pin_int_num;	/**< Pin interrupt number */
	bool en_ack_pay;		/**< Enable ACK payload */
}nrf24l01_t;

/*==================[external data declaration]==============================*/

extern const spiConfig_t nrf24l01_spi_default_cfg;

extern uint8_t snd_to_PRX[]; 	/**< 32 bytes buffer sent to primary receiver (PRX) on each transfer*/
extern uint8_t rcv_fr_PRX[]; 	/**< 32 bytes buffer received from primary receiver (PRX) on each transfer*/

extern uint8_t snd_to_PTX[]; 	/**< 32 bytes buffer sent to primary transmitter (PTX) on each connection*/
extern uint8_t rcv_fr_PTX[]; 	/**< 32 bytes buffer received from primary transmitter (PTX) on each connection*/


/*==================[external functions declaration]=========================*/

/*-------- Function prototypes --------*/

/*-------- Basic APIs --------*/

/**
 * @brief Initialize the RF module passed as parameter
 *
 * The device ID and the configuration of the SPI and GPIO
 * is passed in the first parameter
 *
 * @param[in] nrf24 NRF24 device configuration pointer
 *
 */
void Nrf24Init(nrf24l01_t *nrf24);

/**
 * @brief Sends application data
 *
 * The function will send the specified data using current configuration
 * (current air data rate, current power, current TX address, ... )
 * The function will return after the data is successfully transmitted or
 * maximum retransmissions were done. The module will be in Power Down state
 * when this function returns.
 *
 * @param[in]	nrf24	NRF24 device configuration pointer
 * @param[in]	data	Pointer to application data
 * @param[in] 	length	Length of data
 *
 * @return Result of transmission. See 'return codes'
 *
 */
int8_t Nrf24SendData(nrf24l01_t *nrf24, uint8_t *data, uint32_t length);

/**
 * @brief Sends data to the specified address
 *
 * The function will send the application data using
 * current configuration to the specified address.
 * The function will return after the data is successfully
 * transmitted or maximum retransmissions were done.
 * This function will change the TX address in the module.
 * To send again to the same address use \ref Nrf24SendData()
 *
 * @param[in]	nrf24	NRF24 device configuration pointer
 * @param[in] 	address Pointer to address
 * @param[in]	data	Data to send
 * @param[in]	length	Length of data
 *
 * @return Result of transmission. See 'return codes'
 *
 */
int8_t Nrf24SendDataTo(nrf24l01_t *nrf24, uint8_t *address, uint8_t *data, uint32_t length);
/**
 * @brief Wait(blocking) until any RX pipe has data
 *
 * @param[in]	nrf24	NRF24 device configuration pointer
 * @param[out] pipe	Pipe number which contains the RX payload
 *
 * @return NRF24_SUCCESS when data is in RX FIFO.
 *
 */
int8_t Nrf24WaitForDataRx(nrf24l01_t *nrf24, uint8_t *pipe);

/**
 * @brief Gets the RX payload. See extended explanation
 *
 * For static payload mode: Reads the number of data bytes available in the specified pipe
 * For dynamic payload mode: Reads the top most RX_FIFO payload length.
 *
 * @param[in]	nrf24	NRF24 device configuration pointer
 * @param[in] 	pipe	Pipe number to check payload (for dynamic payload DON'T CARE)
 *
 * @return Available number of bytes
 *
 */
uint8_t Nrf24GetRxDataAmount(nrf24l01_t *nrf24, uint8_t pipe);

/**
 * @brief Gets data available in RX FIFO
 *
 * Reads the length of data from the RX FIFO. If the available data amount is more
 * than the data buffer size returns an error. Payload is automatically deleted
 * from the FIFO once it is read. Also clears the data ready status bit in the module.
 *
 * @param[in]	nrf24	NRF24 device configuration pointer
 * @param[in]	pipe	Pipe number
 * @param[in]	data	Allocated buffer to store the RX data
 * @param[in]	length	Length in bytes of allocated data buffer
 *
 * @return (Positive) number with bytes read or (Negative) error code. See 'return codes'
 *
 */
int8_t Nrf24GetData(nrf24l01_t *nrf24, uint8_t pipe, uint8_t *data,	uint8_t length);

/*-------- Intermediate APIs --------*/

/**
 * @brief Sets the initial configuration of the device register
 *
 * @param[in]	nrf24	NRF24 device configuration pointer
 *
 */
void Nrf24RegisterInit(nrf24l01_t *nrf24);

/**
 * @brief Put the module in Power Down mode
 *
 * @param[in]	nrf24	NRF24 device configuration pointer
 *
 */
void Nrf24PowerDown(nrf24l01_t *nrf24);

/**
 * @brief Go to Standby mode
 *
 * Takes 1.5 ms to go to Standby
 *
 * @param[in]	nrf24	NRF24 device configuration pointer
 *
 */
void Nrf24PowerUp(nrf24l01_t *nrf24);

/**
 * @brief	Sets Automatic Retransmission Delay (ARD)
 *
 * @param[in]	nrf24	NRF24 device configuration pointer
 * @param[in]	value	Value of ARD
 *
 */
void Nrf24SetARD(nrf24l01_t *nrf24, uint16_t value);

/**
 * @brief Gets the status register of the module
 *
 * Also saves the status in the devices status array
 *
 * @param[in]	nrf24	NRF24 device configuration pointer
 *
 */
uint8_t Nrf24GetStatus(nrf24l01_t *nrf24);

/**
 * @brief Enable dynamic payload
 *
 * @param[in]	nrf24	NRF24 device configuration pointer
 * @param[in]	pipe	Pipe number to enable feature
 *
 */
void Nrf24EnableFeatureDynPL(nrf24l01_t *nrf24, uint8_t pipe);

/**
 * @brief Enable payload with ACK
 *
 * @param[in]	nrf24	NRF24 device configuration pointer
 *
 */
void Nrf24EnableFeatureAckPL(nrf24l01_t *nrf24);

/**
 * @brief Checks the interrupt status of the module
 *
 * @param[in]	nrf24	NRF24 device configuration pointer
 *
 * @return Interrupt status
 *
 */
uint8_t Nrf24GetInterruptStatus(nrf24l01_t *nrf24);

/**
 * @brief This function will clear the interrupt associated with the bit mask
 *
 * @param[in]	nrf24			NRF24 device configuration pointer
 * @param[in] 	interrupt_bm	Interrupt bit mask
 *
 * Interrupt Bit Mask 	| Description
 * :-------------------:|:----------------------------
 * INTERRUPT_MAX_RT   	| Maximum number of TX retransmits interrupt
 * INTERRUPT_DATA_READY | Data Ready RX FIFO interrupt
 * INTERRUPT_DATA_SENT  | Data Sent TX FIFO interrupt
 *
 */
void Nrf24ClearInterruptFlag(nrf24l01_t *nrf24, uint8_t interrupt_bm);

/*-------- TX mode related --------*/

/**
 * @brief Flush the TX FIFO
 *
 * @param[in]	nrf24	NRF24 device configuration pointer
 *
 */
void Nrf24FlushTX(nrf24l01_t *nrf24);

/**
 * @brief Sets the TX address
 *
 * The data	packet which is transmitted from this RF module will
 * contain this address as destination address
 *
 * @param[in]	nrf24	NRF24 device configuration pointer
 *
 */
void Nrf24SetTXAddress(nrf24l01_t *nrf24, uint8_t* address);

/**
 * @brief Sets the TX payload
 *
 * @param[in]	nrf24	NRF24 device configuration pointer
 * @param[in]	data	Pointer to buffer of data
 * @param[in]	length  Length of payload
 *
 * @return Result of payload set. See 'return codes'
 *
 * + NRF24_SUCCESS
 * + NRF24_ERROR
 * + NRF24_TX_FIFO_FULL
 *
 */
int8_t Nrf24SetTxPayload(nrf24l01_t *nrf24, uint8_t* data, uint32_t length);

/**
 * @brief Submit data to TX FIFO
 *
 * The function will send the application data to the TX FIFO
 * This function will write data to payload buffer and make
 * sure correct addresses are there for automatic ack if available.
 * The module needs to be Powered-Up and put into TX mode in order
 * to perform the transmission.
 *
 * @param[in]	nrf24	NRF24 device configuration pointer
 * @param[in]	data	Data to send
 * @param[in]	length	Length of data (Bytes)
 *
 * @return Returns result of submit. See 'return codes'
 *
 * + NRF24_SUCCESS
 * + NRF24_ERROR
 * + NRF24_TX_FIFO_FULL
 *
 */
int8_t Nrf24SubmitData(nrf24l01_t *nrf24, uint8_t *data, uint32_t length);

/**
 * @brief Put the module into TX mode
 *
 * If TX FIFO is empty the module will go to Standby II state
 *
 * @param[in]	nrf24	NRF24 device configuration pointer
 *
 */
void Nrf24EnableTxMode(nrf24l01_t *nrf24);

/**
 * @brief Put the module into Standby I state disabling TX mode.
 *
 * @param[in]	nrf24	NRF24 device configuration pointer
 */
void Nrf24DisableTxMode(nrf24l01_t *nrf24);

/**
 * @brief Gets the state of TX FIFO
 *
 * @param[in]	nrf24	NRF24 device configuration pointer
 *
 * @return TX FIFO state
 * + 0: TX FIFO is NOT full
 * + 1: TX FIFO is full
 *
 */
uint8_t Nrf24IsTxFifoFull(nrf24l01_t *nrf24);

/**
 * @brief Gets the state of TX FIFO
 *
 * @param[in]	nrf24	NRF24 device configuration pointer
 *
 * @return State of TX FIFO
 *
 * + 0: TX FIFO is NOT empty
 * + 1: TX FIFO is empty
 *
 */
int8_t Nrf24IsTxFifoEmpty(nrf24l01_t *nrf24);

/**
 * @brief Attempts TX
 *
 * This function will attempt to TX whatever data available in the
 * TX FIFO. It waits until the TX is done or maximum retransmissions
 * occur. Module will be in Power Down mode when it returns.
 *
 * @param[in]	nrf24	NRF24 device configuration pointer
 *
 * @return Result of transmission. See 'return codes'
 *
 */
int8_t Nrf24AttemptTx(nrf24l01_t *nrf24);

/**
 * @brief Waits transmission(without ACK) or delivery(with ACK) to complete
 *
 * The function can wait until the
 * -TX payload is successfully delivered (If ACK is received) or
 * -TX payload has successfully transmitted.
 *
 * @param[in]	nrf24	NRF24 device configuration pointer
 *
 * @return Result of transmission. See 'return codes'
 *
 */
int8_t Nrf24WaitForTxComplete(nrf24l01_t *nrf24, uint8_t busy_wait);

/**
 * @brief Gets ACK data amount
 *
 * @param[in]	nrf24	NRF24 device configuration pointer
 *
 * @return Data amount
 *
 */
uint8_t Nrf24GetAckDataAmount(nrf24l01_t *nrf24);

/*-------- RX mode related --------*/

/**
 * @brief Flush the RX FIFO
 *
 * @param[in]	nrf24	NRF24 device configuration pointer
 *
 */
void Nrf24FlushRX(nrf24l01_t *nrf24);

/**
 * @brief Sets the RX address
 *
 * P0 and P1 pipes have 5 byte address other pipes have 1 byte
 * address(LSB). Other bytes are taken from the P1 pipe address
 * (4 MSB shared with P1 and unique LSB)
 *
 * @param[in]	nrf24	NRF24 device configuration pointer
 * @param[in] 	pipe	Pipe number to set RX address
 * @param[in]	address	Buffer which contains the 1 byte or 5 bytes depending on pipe
 *
 */
void Nrf24SetRxAddress(nrf24l01_t *nrf24, uint8_t pipe,	uint8_t *address);

/**
 * @brief Sets the packet size for a RX pipe
 *
 * @param[in]	nrf24	NRF24 device configuration pointer
 * @param[in]	pipe	Pipe to set RX packet size
 * @param[in] 	pkt_sz	Packet size (Maximum is 32)
 *
 */
void Nrf24SetRXPacketSize(nrf24l01_t *nrf24, uint8_t pipe, uint8_t pkt_sz);

/**
 * @brief Powers up the module and put the module into RX mode
 *
 * @param[in]	nrf24	NRF24 device configuration pointer
 *
 */
void Nrf24EnableRxMode(nrf24l01_t *nrf24);

/**
 * @brief Put the module into Standby I state with disabling RX mode
 *
 * @param[in]	nrf24	NRF24 device configuration pointer
 *
 */
void Nrf24DisableRxMode(nrf24l01_t *nrf24);

/**
 * @brief Get the state of RX data
 *
 * @param[in]	nrf24	NRF24 device configuration pointer
 * @param[out]	pipe	Pipe number which contains the RX payload
 *
 * @return State of RX FIFO
 *
 * + NRF24_SUCCESS	 		Data is in RX FIFO
 * + NRF24_INVALID_ARGUMENT Invalid input argument
 * + NRF24_ERROR			Data is not in RX FIFO
 *
 */
int8_t Nrf24IsDataReadyRx(nrf24l01_t *nrf24, uint8_t *pipe);

/**
 * @brief Reads RX payload
 *
 * Reads the length amount of data from the RX FIFO.
 * Do NOT validates data sizes, buffer size, etc. TODO: validate length availability
 *
 * 					Payload is automatically deleted from the FIFO once it is read
 *
 * @param[in]	nrf24	NRF24 device configuration pointer
 * @param[out]	data 	Allocated buffer to store RX data
 * @param[in]	length 	Length to data.
 *
 */
void Nrf24ReadRxPayload(nrf24l01_t *nrf24, uint8_t* data, uint8_t length);

/**
 * @brief Sets ACK payload
 *
 * @param[in]	nrf24	NRF24 device configuration pointer
 * @param[in]	data	Buffer which contains the ACK data to write to TX FIFO
 * @param[in]	pipe	Which pipe to use (0-5)
 * @param[in]	length	Length of the data. Max 32 bytes
 *
 * @return Result of operation
 *
 * + NRF24_TX_FIFO_FULL		TX FIFO full
 * + NRF24_SUCCESS			Success
 * + NRF24_ERROR			Pipe >=6 ; data==NULL ; length==0
 *
 */
int8_t Nrf24SetAckPayload(nrf24l01_t *nrf24, uint8_t *data, uint8_t pipe, uint32_t length);

/*-------- Advanced APIs, Register level access --------*/

/**
 * @brief Reads a 8 bits register
 *
 * @param[in]	nrf24	NRF24 device configuration pointer
 *
 * @return Value of register
 *
 */
uint8_t Nrf24RegisterRead8(nrf24l01_t *nrf24, uint8_t reg);

/**
 * @brief Reads a multiple byte register*
 *
 * Also updates the devices status
 *
 * @param[in]	nrf24	NRF24 device configuration pointer
 * @param[in] 	reg		Register to read
 * @param[out]  buffer	Buffer to save read bytes
 * @param[in] 	length 	Number of bytes to read
 *
 * @return Status register
 *
 */
uint8_t Nrf24RegisterReadMulti(nrf24l01_t *nrf24, uint8_t reg, uint8_t *buffer, uint32_t length);

/**
 * @brief Writes a 8 bits register
 *
 * This function will write 1 byte of data to an 8 bits
 * register. The function will update the global status
 * variable too.
 *
 * @param[in]	nrf24	NRF24 device configuration pointer
 * @param[in]	reg		Register to write
 * @param[in]	value	Value to write in the register
 *
 */
void Nrf24RegisterWrite8(nrf24l01_t *nrf24, uint8_t reg, uint8_t value);

/**
 * @brief Writes a multiple bytes register
 *
 * This function will write more than one byte of data
 * to an the specified register. The function will update
 * the global status variable too.
 *
 * @param[in]	nrf24	NRF24 device configuration pointer
 * @param[in]	reg		Register to write
 * @param[in]	data	Pointer to data
 * @param[in]	length 	Length of data
 *
 */
void Nrf24RegisterWriteMulti(nrf24l01_t *nrf24, uint8_t reg, uint8_t *data, uint16_t length);

/**
 * @brief Sends a command with associated data
 *
 * This function will send a command and the the data associated
 * to the RF module. This will also update the status register.
 * If there is no payload for the command set 'data' to NULL and
 * make the 'length' 0
 *
 * @param[in]	nrf24	NRF24 device configuration pointer
 * @param[in]	command Command to send
 * @param[in]	data	Data associated with the command
 * @param[in]	length	Length of the data
 *
 */
void Nrf24SendCommand(nrf24l01_t *nrf24, uint8_t command, uint8_t *data, uint32_t length);

/**
 * @brief Sends command then receives the reply
 *
 * First sends command then reads the specified amount(length)
 * of data and stores it as the reply in the provided buffer.
 *
 * @param[in]	nrf24	NRF24 device configuration pointer
 * @param[in] 	command Command to send
 * @param[out]	data	Buffer to get data
 * @param[in]	length	Amount of data to get
 *
 */
void Nrf24SendRcvCommand(nrf24l01_t *nrf24, uint8_t command, uint8_t *data, uint32_t length);

/*-------- Interrupts ISR configuration --------*/

/**
 * @brief Configurates the ISR of the primary device
 *
 * @param[in]	nrf24	NRF24 device configuration pointer
 *
 */
void Nrf24PrimaryDevISRConfig(nrf24l01_t *nrf24);

/**
 * @brief Configurates the ISR of the secondary device
 * (will be RX when testing two RF module in the same dev board)
 *
 * @param[in]	nrf24	NRF24 device configuration pointer
 *
 */
void Nrf24SecondaryDevISRConfig(nrf24l01_t *nrf24);

/**
 * @brief Primary device callback ISR
 *
 */
void Nrf24PrimaryDevISR();

/**
 * @brief Secondary device callback ISR
 *
 */
void Nrf24SecondaryDevISR();

/**
 * @brief Selects the ISR based on the module mode (RX or TX)
 * and availability of ACK payload feature.
 *
 * @param[in]	nrf24	NRF24 device configuration pointer
 *
 */
void Nrf24SelectISR(nrf24l01_t *nrf24);

/**
 * @brief ISR for no ACK transmit interrupt
 *
 * @param[in]	nrf24	NRF24 device configuration pointer
 *
 */
void Nrf24TransmitDataISR(nrf24l01_t *nrf24);

/**
 * @brief ISR for ACK transmit interrupt
 *
 * @param[in]	nrf24		NRF24 device configuration pointer
 * @param[out] 	rcv_ack_pay	Buffer to receive ACK payload. Allocate 32 bytes
 *
 */
void Nrf24TransmitDataACKPayISR(nrf24l01_t *nrf24,uint8_t *rcv_ack_pay);

/**
 * @brief ISR for receive interrupt
 *
 * @param[in]	nrf24	NRF24 device configuration pointer
 * @param[out]	data	Buffer to store receive data. Allocate 32 bytes
 *
 */
void Nrf24ReceiveDataISR(nrf24l01_t *nrf24, uint8_t *data);

/**
 * @brief ISR for ACK receive interrupt
 *
 * The ACk payload is transmitir on the next ACK
 *
 * @param[in]	nrf24			NRF24 device configuration pointer
 * @param[out] 	data			Buffer to receive data. Allocate 32 bytes
 * @param[in]	next_ack_pay	Set ACK payload on TX FIFO. Will be transmit on next ACK
 * @param[in]	ack_len_pay		Length of ACK payload
 *
 */
void Nrf24ReceiveDataACKPayISR(nrf24l01_t *nrf24, uint8_t *data,uint8_t *next_ack_pay, uint8_t ack_pay_len);

/**
 * @brief To send data regularly. (Call within a timer handler)
 *
 * @param[in]	nrf24		NRF24 device configuration pointer
 * @param[in]	snd_to_prx	Data buffer to send.
 * @param[in]	length		Length of data to send. 1-32 bytes
 *
 */
void Nrf24TxTick();

/*-------- Template doxygen function documentation --------*/

/**
 * @brief Brief description of the function
 *
 * Extended description
 *
 * @param[in]
 * @param[out]
 *
 * @return
 *
 */

#endif /* MODULES_LPC4337_M4_DRIVERS_BM_INC_NRF24H_ */

/** @} doxygen end group definition */
