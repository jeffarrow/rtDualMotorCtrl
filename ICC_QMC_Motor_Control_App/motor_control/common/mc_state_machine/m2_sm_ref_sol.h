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
#ifndef _M2_SM_REF_SOL_H_
#define _M2_SM_REF_SOL_H_

#include "sm_ref_sol_comm.h"
#include "m2_pmsm_appconfig.h"
#include "state_machine.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define M2_SPEED_CONV_SCALE 628.06F /* Speed conversion scale */

#define MCAT_SENSORLESS_CTRL 0 /* Sensorless control flag */
#define MCAT_ENC_CTRL 1 /* Position quadrature encoder control flag */

/*******************************************************************************
 * Variables
 ******************************************************************************/
extern bool_t g_bM2SwitchAppOnOff;
extern mcdef_pmsm_t g_sM2Drive;
extern sm_app_ctrl_t g_sM2Ctrl;
extern run_substate_t g_eM2StateRun;

extern volatile float g_fltM2voltageScale;
extern volatile float g_fltM2DCBvoltageScale;
extern volatile float g_fltM2currentScale;
extern volatile float g_fltM2speedScale;
extern volatile float g_fltM2speedAngularScale;
extern volatile float g_fltM2speedMechanicalScale;

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * API
 ******************************************************************************/

/*!
 * @brief Set application switch value to On or Off mode
 *
 * @param bValue  bool value, true - On of false - Off
 *
 * @return None
 */
//RAM_FUNC
void M2_SetAppSwitch(bool_t bValue);

/*!
 * @brief Get application switch value
 *
 * @param void  No input parameter
 *
 * @return bool_t Return bool value, true or false
 */
bool_t M2_GetAppSwitch(void);

/*!
 * @brief Get application state
 *
 * @param void  No input parameter
 *
 * @return uint16_t Return current application state
 */
uint16_t M2_GetAppState(void);

/*!
 * @brief Set spin speed of the motor in fractional value
 *
 * @param f16SpeedCmd  Speed command - set speed
 *
 * @return None
 */
//RAM_FUNC
void M2_SetSpeed(float_t fltSpeedCmd);

/*!
 * @brief Set position of the motor in acc value
 *
 * @param a32PositionCmd  Position command - set position
 *
 * @return None
 */
//RAM_FUNC
void M2_SetPosition(acc32_t a32PositionCmdDemo);

/*!
 * @brief Get spin required speed of the motor in fractional value
 *
 * @param void  No input parameter
 *
 * @return frac16_t Fractional value of the current speed
 */
float_t M2_GetReqSpeed(void);

#ifdef __cplusplus
}
#endif

#endif /* _M2_SM_REF_SOL_H_ */

