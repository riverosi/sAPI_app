/*
    __angle_driver.h

-----------------------------------------------------------------------------

  This file is part of mikroSDK.
  
  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

----------------------------------------------------------------------------- */

/**
@file   __angle_driver.h
@brief    Angle Driver
@mainpage Angle Click
@{

@image html sch.jpg

@}

@defgroup   ANGLE
@brief      Angle Click Driver
@{

| Global Library Prefix | **ANGLE** |
|:---------------------:|:-----------------:|
| Version               | **1.0.0**    |
| Date                  | **Dec 2017.**      |
| Developer             | **MikroE Team**     |

*/
/* -------------------------------------------------------------------------- */

#include <stdint.h>

#ifndef _ANGLE_H_
#define _ANGLE_H_

/** 
 * @macro T_ANGLE_P
 * @brief Driver Abstract type 
 */
#define T_ANGLE_P    const uint8_t*

/** @defgroup ANGLE_COMPILE Compilation Config */              /** @{ */

   #define   __ANGLE_DRV_I2C__                            /**<     @macro __ANGLE_DRV_I2C__  @brief I2C driver selector */                                          

                                                                       /** @} */
/** @defgroup ANGLE_VAR Variables */                           /** @{ */

/* Error register status */
extern const uint8_t _ANGLE_ERROR_REG_MASK;
extern const uint8_t _ANGLE_EXTENDED_ERROR_REG_MASK;
extern const uint8_t _ANGLE_EXTENDED_ERROR_REG;
extern const uint8_t _ANGLE_ERROR_REG;
extern const uint8_t _ANGLE_STATUS_REG;

const uint8_t _ANGLE_SETTINGS_REG;

/* Change Processor State */
extern const uint16_t _ANGLE_CDS_NO_CHANGLE;
extern const uint16_t _ANGLE_CDS_IDLE_MODE;
extern const uint16_t _ANGLE_CDS_RUN_MODE;

/* Hard reset */
extern const uint16_t _ANGLE_HDR_RESET_0;
extern const uint16_t _ANGLE_HDR_RESET_1;

/* Soft Reset */
extern const uint16_t _ANGLE_SFR_RESET_0;
extern const uint16_t _ANGLE_SFR_RESET_1;

/* Clear Status registar */
extern const uint16_t _ANGLE_CSR_STA_0;
extern const uint16_t _ANGLE_CSR_STA_1;

/* Clear Extended Error register*/
extern const uint16_t _ANGLE_CXE_0;
extern const uint16_t _ANGLE_CXE_1;

/* Clear Error register */
extern const uint16_t _ANGLE_CER_0;
extern const uint16_t _ANGLE_CER_1;

                                                                       /** @} */
/** @defgroup ANGLE_TYPES Types */                             /** @{ */



                                                                       /** @} */
#ifdef __cplusplus
extern "C"{
#endif

/** @defgroup ANGLE_INIT Driver Initialization */              /** @{ */


#ifdef   __ANGLE_DRV_I2C__
void angle_i2cDriverInit(uint32_t clkHz, uint8_t slave);
#endif


                                                                       /** @} */
/** @defgroup ANGLE_FUNC Driver Functions */                   /** @{ */


/**
 * @brief Reads encoded Angle in degreeses
 */
uint16_t angle_getAngle();
/**
 * @brief Reads temperature in C
 */
uint16_t angle_getTemperature();
/**
 * @brief Magnetic data in gauss
 */
uint16_t angle_getMagnetics();
/**
 * @brief Writes configuration to CTRL register
 *
 * Configuration data (setValue) consists of multiple settings:
     Change Processor State, Hard reset, Soft Reset, Clear Status registar,
     Clear Extended Error register and Clear Error register.
 */
void angle_setConfig(uint16_t setValue);
/**
 * @brief Checks status state
 *
 * Function reads Status registers and returns value of this registers.
 */
uint16_t angle_getStatus( uint8_t reg );


                                                                       /** @} */
#ifdef __cplusplus
} // extern "C"
#endif
#endif

/*
  __angle_driver.h

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
