/*
 __angle_driver.c

 -----------------------------------------------------------------------------

 This file is part of mikroSDK.

 Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

 All rights reserved.

 ----------------------------------------------------------------------------- */

#include "__angle_driver.h"
#include <chip.h>
#include "stopwatch.h"

/* ------------------------------------------------------------------- MACROS */

/* ---------------------------------------------------------------- VARIABLES */
/**
 * Address of device
 * The default slave address for the A1335 is 00011xx, where
 * the two LSB bits are set by the package pins (SA1 and SA0)
 * being tied high or low.
 */
#ifdef   __ANGLE_DRV_I2C__
static uint8_t _slaveAddress;
#endif

/* Error register status */
const uint8_t _ANGLE_ERROR_REG_MASK = 0x34;
const uint8_t _ANGLE_EXTENDED_ERROR_REG_MASK = 0x36;
const uint8_t _ANGLE_EXTENDED_ERROR_REG = 0x26;
const uint8_t _ANGLE_ERROR_REG = 0x24;
const uint8_t _ANGLE_STATUS_REG = 0x22;

const uint8_t _ANGLE_SETTINGS_REG = 0x1E;

/* Change Processor State */
const uint16_t _ANGLE_CDS_NO_CHANGLE = 0x0000 << 13;
const uint16_t _ANGLE_CDS_IDLE_MODE = 0x0002 << 13;
const uint16_t _ANGLE_CDS_RUN_MODE = 0x0003 << 13;

/* Hard reset */
const uint16_t _ANGLE_HDR_RESET_0 = 0x0000 << 12;
const uint16_t _ANGLE_HDR_RESET_1 = 0x0001 << 12;

/* Soft Reset */
const uint16_t _ANGLE_SFR_RESET_0 = 0x0000 << 11;
const uint16_t _ANGLE_SFR_RESET_1 = 0x0001 << 11;

/* Clear Status registar */
const uint16_t _ANGLE_CSR_STA_0 = 0x0000 << 9;
const uint16_t _ANGLE_CSR_STA_1 = 0x0001 << 9;

/* Clear Extended Error register*/
const uint16_t _ANGLE_CXE_0 = 0x0000 << 8;
const uint16_t _ANGLE_CXE_1 = 0x0001 << 8;

/* Clear Error register */
const uint16_t _ANGLE_CER_0 = 0x0000 << 7;
const uint16_t _ANGLE_CER_1 = 0x0001 << 7;

/* -------------------------------------------- PRIVATE FUNCTION DECLARATIONS */

/**
 * @brief hal_i2cWrite
 *
 * @param[in] slaveAddress     7 bit slave addres without 0 bit (read/write bit)
 * @param[in] pBuf             pointer to data buffer
 * @param[in] nBytes           number of bytes for writing
 * @param[in] endMode          END_MODE_STOP / END_MODE_RESTART / END_MODE_NO
 *
 * @return    0                No Error
 *
 * Function should executes write sequence on I2C bus.
 *
 * |       8 Bits      | Write Sequence (n Bytes) | End Mode       |
 * |:-----------------:|:------------------------:|:--------------:|
 * | Slave Address + W | Data Bytes               | Stop / Restart |
 *
 * @note
 * This function alongside with hal_i2cStart represents complete write sequence.
 *
 * @warning
 * No End mode is not possible on all architectures.
 */
static int hal_i2cWrite(uint8_t slaveAddress, uint8_t *pBuf, uint16_t nBytes);

/**
 * @brief hal_i2cRead
 *
 * @param[in] slaveAddress     7 bit slave addres without 0 bit (read/write bit)
 * @param[in] pBuf             pointer to data buffer
 * @param[in] nBytes           number of bytes to read
 * @param[in] endMode          END_MODE_STOP / END_MODE_RESTART / END_MODE_NO
 *
 * @return    0                No Error
 *
 * Function should executes read sequence on I2C bus.
 *
 * |       8 Bits      | Read Sequence (n Bytes)  | End Mode       |
 * |:-----------------:|:------------------------:|:--------------:|
 * | Slave Address + R | Data Bytes + ACK/NACK    | Stop / Restart |
 *
 * @note
 * This function alongside with hal_i2cStart represents complete read sequence.
 *
 * @warning
 * No End mode is not possible on all architectures.
 */
static int hal_i2cRead(uint8_t i2cSlaveAddress, uint8_t* dataToReadBuffer,
		uint16_t dataToReadBufferSize, uint8_t* receiveDataBuffer,
		uint16_t receiveDataBufferSize);

/* --------------------------------------------- PRIVATE FUNCTION DEFINITIONS */
static int hal_i2cWrite(uint8_t slaveAddress, uint8_t *pBuf, uint16_t nBytes) {

	I2CM_XFER_T i2cData;

	// Prepare the i2cData register
	i2cData.slaveAddr = slaveAddress;
	i2cData.options = 0;
	i2cData.status = 0;
	i2cData.txBuff = pBuf;
	i2cData.txSz = nBytes;
	i2cData.rxBuff = 0;
	i2cData.rxSz = 0;

	/* Send the i2c data */
	if (Chip_I2CM_XferBlocking(LPC_I2C0, &i2cData) == 0) {
		return -1;
	}

	return 0;
}
static int hal_i2cRead(uint8_t i2cSlaveAddress, uint8_t* dataToReadBuffer,
		uint16_t dataToReadBufferSize, uint8_t* receiveDataBuffer,
		uint16_t receiveDataBufferSize) {
	I2CM_XFER_T i2cData;

	i2cData.slaveAddr = i2cSlaveAddress;
	i2cData.options = 0;
	i2cData.status = 0;
	i2cData.txBuff = dataToReadBuffer;
	i2cData.txSz = dataToReadBufferSize;
	i2cData.rxBuff = receiveDataBuffer;
	i2cData.rxSz = receiveDataBufferSize;

	if (Chip_I2CM_XferBlocking( LPC_I2C0, &i2cData) == 0) {
		return -1;
	}
	return 0;
}
/* --------------------------------------------------------- PUBLIC FUNCTIONS */

#ifdef   __ANGLE_DRV_I2C__

void angle_i2cDriverInit(uint32_t clkHz, uint8_t slave) {

	_slaveAddress = slave;
	Chip_SCU_I2C0PinConfig(I2C0_STANDARD_FAST_MODE);
	Chip_I2C_Init(I2C0);
	Chip_I2C_SetClockRate(I2C0, clkHz);
	Chip_I2C_SetMasterEventHandler(I2C0, Chip_I2C_EventHandlerPolling);
	StopWatch_Init();

}

#endif

/* ----------------------------------------------------------- IMPLEMENTATION */

uint16_t angle_getAngle() {
	uint8_t writeReg[1];
	uint8_t readReg[2];
	uint16_t angle;
	uint16_t AngleVal;

	writeReg[0] = 0x20;

	hal_i2cRead(_slaveAddress, writeReg, 1 , readReg, 2);

	angle = readReg[0];
	angle <<= 8;
	angle |= readReg[1];
	angle &= 0x0FFF;
	AngleVal = (uint16_t)(angle * (360 / 4096.0));

	return AngleVal;
}

uint16_t angle_getTemperature() {
	uint8_t writeReg[1];
	uint8_t readReg[2];
	uint16_t temp;
	uint16_t TempVal;

	writeReg[0] = 0x28;

	hal_i2cRead(_slaveAddress, writeReg, 1 , readReg, 2);

	temp = readReg[0];
	temp <<= 8;
	temp |= readReg[1];
	temp &= 0x0FFF;
	TempVal = (uint16_t) temp / 8 - 273;

	return TempVal;
}

uint16_t angle_getMagnetics() {
	uint8_t writeReg[1];
	uint8_t readReg[2];
	uint16_t magnet;
	uint16_t magnetVal;

	writeReg[0] = 0x2A;

	hal_i2cRead(_slaveAddress, writeReg, 1 , readReg, 2);

	magnet = readReg[0];
	magnet <<= 8;
	magnet |= readReg[1];
	magnet &= 0x0FFF;
	magnetVal = (uint16_t) magnet;

	return magnetVal;
}

uint16_t angle_getStatus(uint8_t reg) {
	uint8_t writeReg[1];
	uint8_t readReg[2];
	uint16_t status;

	writeReg[0] = reg;

	hal_i2cRead(_slaveAddress, writeReg, 1 , readReg, 2);

	status = readReg[0];
	status <<= 8;
	status |= readReg[1];
	status &= 0x0FFF;

	return status;
}

void angle_setConfig(uint16_t setValue) {
	uint8_t writeReg[3];
	writeReg[0] = 0x1E;
	writeReg[1] = setValue >> 8;
	writeReg[2] = setValue & 0x00FF;
	hal_i2cWrite(_slaveAddress, writeReg, 3);
	StopWatch_DelayMs(100);
}

/* -------------------------------------------------------------------------- */
/*
 __angle_driver.c

 Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.

 3. All advertising materials mentioning features or use of this software
 must display the following acknowledgement:
 This product includes software developed by the MikroElektonika.

 4. Neither the name of the MikroElektonika nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY MIKROELEKTRONIKA ''AS IS'' AND ANY
 EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL MIKROELEKTRONIKA BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ----------------------------------------------------------------------------- */
