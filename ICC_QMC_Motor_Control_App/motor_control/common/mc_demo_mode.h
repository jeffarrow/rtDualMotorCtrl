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

#ifndef _MC_DEMO_MODE_H_
#define _MC_DEMO_MODE_H_

#include "m1_sm_ref_sol.h"
#include "m2_sm_ref_sol.h"
#include "m3_sm_ref_sol.h"
#include "m4_sm_ref_sol.h"

#include "mcdrv_enc_qd.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Speed demo mode enabled/disabled */
extern bool_t bDemoModeSpeed;

/* Position demo mode enabled/disabled */
extern bool_t bDemoModePosition;

extern bool_t bM1SpeedDemo; 
extern bool_t bM2SpeedDemo;
extern bool_t bM3SpeedDemo;
extern bool_t bM4SpeedDemo;

extern bool_t bM1PositionDemo; 
extern bool_t bM2PositionDemo;
extern bool_t bM3PositionDemo;
extern bool_t bM4PositionDemo;

extern bool_t bM2_SyncWithM1;
extern bool_t bM3_SyncWithM1;
extern bool_t bM4_SyncWithM1;
extern bool_t bM1_SyncWithM2;
extern bool_t bM3_SyncWithM2;
extern bool_t bM4_SyncWithM2;
extern bool_t bM1_SyncWithM3;
extern bool_t bM2_SyncWithM3;
extern bool_t bM4_SyncWithM3;
extern bool_t bM1_SyncWithM4;
extern bool_t bM2_SyncWithM4;
extern bool_t bM3_SyncWithM4;


extern mcdrv_qd_enc_t g_sM1Enc;
extern mcdrv_qd_enc_t g_sM2Enc;
extern mcdrv_qd_enc_t g_sM3Enc;
extern mcdrv_qd_enc_t g_sM4Enc;

/* Demo Speed Stimulator */
RAM_FUNC
void SpeedDemo(void);
/* Demo Position Stimulator */
RAM_FUNC
void PositionDemo(void);

/* Demo Speed Stimulator */
RAM_FUNC
void M1SpeedDemo(void);
RAM_FUNC
void M2SpeedDemo(void);
RAM_FUNC
void M3SpeedDemo(void);
RAM_FUNC
void M4SpeedDemo(void);
RAM_FUNC
void MotorSynchronisation(void);
   

#ifdef __cplusplus
}
#endif

#endif /* _MC_DEMO_MODE_H_  */