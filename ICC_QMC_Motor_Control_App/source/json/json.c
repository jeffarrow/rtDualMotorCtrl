/*
 * The Clear BSD License
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 *  that the following conditions are met:
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
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
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

#include "json.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
//int json_decode(uint8_t *jsonBuffer, mc_data_t *mcData);
//int json_encode(mc_data_t *mcData, uint32_t motorId, uint8_t *jsonBuffer);

/*******************************************************************************
 * Globals
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief json_decode function - parse JSON from string
 */
int json_decode(uint8_t *jsonBuffer, mc_command_data_t *mcCommandData)
{
  uint8_t *nextChar = NULL;
  uint32_t nextAddress;
  uint32_t motorId;
  
  assert(NULL != jsonBuffer);
  assert(NULL != mcCommandData);
  
  if (memcmp((jsonBuffer+16), "imxrt1050_icc_daughter_card", 27) != 0)
  {
    PRINTF("Data received are not correct: %s", jsonBuffer);
    return 0;
  }

#ifdef DEBUG
  //PRINTF("%s", jsonBuffer);
#endif 
  
  nextAddress = (uint32_t)jsonBuffer+88;
  motorId = strtol((uint8_t*)nextAddress, &nextChar, 10);
  nextAddress = (uint32_t)nextChar+29;
  mcCommandData[motorId].appStatus  = (mc_app_status_t)strtol((uint8_t*)nextAddress, &nextChar, 10);
  nextAddress = (uint32_t)nextChar+25;
  mcCommandData[motorId].controlMethodSel = (mc_method_selection_t)strtol((uint8_t*)nextAddress, &nextChar, 10);
  nextAddress = (uint32_t)nextChar+31;
  mcCommandData[motorId].bDemoModeSpeed = (bool)strtol((uint8_t*)nextAddress, &nextChar, 10);
  nextAddress = (uint32_t)nextChar+16;
  mcCommandData[motorId].f32Speed = (float)strtol((uint8_t*)nextAddress, &nextChar, 10);
  nextAddress = (uint32_t)nextChar+34;
  mcCommandData[motorId].bDemoModePosition = (bool)strtol((uint8_t*)nextAddress, &nextChar, 10);
  nextAddress = (uint32_t)nextChar+19;
  mcCommandData[motorId].u32Position = (uint32_t)strtol((uint8_t*)nextAddress, &nextChar, 10);

  return motorId;
}

/*!
 * @brief json_decode function - generated JSON fromat string
 */
int json_encode(mc_status_data_t *mcStatusData, uint32_t motorId, uint8_t *jsonBuffer)
{
  assert(NULL != jsonBuffer);
  assert(NULL != mcStatusData);
  
  sprintf(jsonBuffer,\
        JSON_MSG_MC_STATUS,\
        (uint32_t)motorId,                                       /* motor_id */
        (uint32_t)mcStatusData[motorId].appStatus,               /* app_status */
        (uint32_t)mcStatusData[motorId].motorState,              /* motor state */
        (uint32_t)mcStatusData[motorId].faultStatus,             /* fault */
        (uint32_t)mcStatusData[motorId].f32Speed,                /* speed */
        (uint32_t)mcStatusData[motorId].u32Position,             /* position */
        (uint32_t)(1000.0f*mcStatusData[motorId].f32Iq),         /* current_iq [mA] */
        (uint32_t)(1000.0f*mcStatusData[motorId].f32UDcBus)      /* voltage_vdc [mV] */
        );

#ifdef DEBUG
  //PRINTF("%s", jsonBuffer);
#endif 
  return 1;
}

