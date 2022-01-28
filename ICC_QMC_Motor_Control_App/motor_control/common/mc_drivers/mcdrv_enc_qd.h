/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2018 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef _MCDRV_ENC_QD_H_
#define _MCDRV_ENC_QD_H_

#include "mlib.h"
#include "mlib_types.h"
#include "fsl_device_registers.h"
#include "amclib.h"
#include "pmsm_control.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define MCDRV_QD (1)
     
typedef struct _mcdrv_qd_enc_t
{
    AMCLIB_TRACK_OBSRV_T_FLT        sTo;                                        /* tracking observer structure */
    ENC_Type                       *pui32QdBase;                                /* pointer to QD module base address*/  
    float_t                        *pfltSpdMeEst;                               /* pointer to measured mechanical speed  */    
    frac16_t                       *pf16PosElEst;                               /* pointer to measured electrical position */
    acc32_t                         a32PosErr;                                  /* position error to tracking observer  */    
    acc32_t                         a32PosMeGain;                               /* encoder pulses to mechanical position scale gain */ 
    float_t                         fltSpdMeEst;                                /* estimated speed calculated using tracking observer */            
    frac16_t                        f16PosMe;                                   /* mechanical position calculated using encoder edges */
    frac16_t                        f16PosMeEst;                                /* estimated position calculated using tracking observer */        
    uint16_t                        ui16Pp;                                     /* number of motor pole pairs */
    bool_t                          bDirection;                                 /* encoder direction */  
    float_t                         fltSpdEncMin;                               /* encoder minimal speed resolution */
    frac16_t                        f16PosErr;                                  /* poisition error to tracking observer  */   
    frac16_t                        f16PosMeGain;                               /* encoder pulses to mechanical position scale gain */ 
    int16_t                         i16PosMeGainSh;                             /* encoder pulses to mechanical position scale shift */ 
    acc32_t                         a32PosMeReal;                               /* real position (revolution counter + mechanical position) */
    uint32_t                        ui32RevCounter;                             /* revolution counter measured by periphery */
    
    uint16_t                        ui16OnLedPosition;
    uint16_t                        ui16OffLedDelta;
    uint16_t                        ui16OffLedPosition;
    
    uint16_t                        ui16OnLedPosition2;
    uint16_t                        ui16OffLedPosition2;
    
    bool_t                          bDemoOnOff;
    uint16_t                        ui16LPOSRegister;
    uint16_t                        ui16EncCtrl2DirRegister;
    uint16_t                        ui16EncLedDemoType;                         /* Type of Led demo flashing */
      
    acc32_t                         a32PosIndexOffset;                          /* position of index signal - offset between index and align position after startup */
    uint16_t                        ui16LedIndexOffset;                         /* position of index signal for Led */
    uint16_t                        ui16LedAlignOffset;                         /* position added to index position */
    
    GPIO_Type                       *pui32GpioLedBase;
    uint16_t                        ui16GpioLedPin;    
  
} mcdrv_qd_enc_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif
  
/*!
 * @brief Function returns actual position and speed
 *
 * @param this   Pointer to the current object
 *
 * @return boot_t true on success
 */
RAM_FUNC_CRITICAL
bool_t MCDRV_QdEncGet(mcdrv_qd_enc_t *this);


/*!
 * @brief Led flashing function
 *
 * @param this   Pointer to the current object
 *
 * @return boot_t true on success
 */
RAM_FUNC_CRITICAL
bool_t MCDRV_QdEncLedPosition(mcdrv_qd_enc_t *this);


/*!
 * @brief 
 *
 * @param this   Pointer to the current object
 *
 * @return boot_t true on success
 */
RAM_FUNC_CRITICAL
bool_t MCDRV_QdEncLedDemoType(mcdrv_qd_enc_t *this);

/*!
 * @brief Function returns index offset between index and align position of rotor
 *
 * @param this   Pointer to the current object
 *
 * @return boot_t true on success
 */
RAM_FUNC_CRITICAL
bool_t MCDRV_QdEncGetIndexOffset(mcdrv_qd_enc_t *this);

/*!
 * @brief Function clears internal variables and decoder counter
 *
 * @param this   Pointer to the current object
 *
 * @return boot_t true on success
 */
RAM_FUNC_CRITICAL
bool_t MCDRV_QdEncClear(mcdrv_qd_enc_t *this);

/*!
 * @brief Function se mechanical position of quadrature encoder
 *
 * @param this     Pointer to the current object
 *        f16PosMe Mechanical position
 *
 * @return boot_t true on success
 */
RAM_FUNC
bool_t MCDRV_QdEncSetPosMe(mcdrv_qd_enc_t *this, frac16_t f16PosMe);

/*!
 * @brief Function set direction of quadrature encoder
 *
 * @param this       Pointer to the current object
 *        bDirection Encoder direction
 *
 * @return boot_t true on success
 */
RAM_FUNC_CRITICAL
bool_t MCDRV_QdEncSetDirection(mcdrv_qd_enc_t *this);

/*!
 * @brief Function enable ENC index interrupt
 *
 * @param this   Pointer to the current object
 *
 * @return boot_t true on success
 */
RAM_FUNC
bool_t MCDRV_QdEncIndexIRQOn(mcdrv_qd_enc_t *this);


#ifdef __cplusplus
}
#endif

#endif /* _MCDRV_ENC_QD_H_ */

