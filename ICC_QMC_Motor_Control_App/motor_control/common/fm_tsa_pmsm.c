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

#include "fm_tsa_pmsm.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/* global control variables */
extern bool_t           bDemoModeSpeed;
extern bool_t           bDemoModePosition;

extern bool_t           bM1SpeedDemo; 
extern bool_t           bM2SpeedDemo;
extern bool_t           bM3SpeedDemo;
extern bool_t           bM4SpeedDemo;
    
extern bool_t           bM1PositionDemo; 
extern bool_t           bM2PositionDemo;
extern bool_t           bM3PositionDemo;
extern bool_t           bM4PositionDemo;

extern bool_t           bM2_SyncWithM1;
extern bool_t           bM3_SyncWithM1;
extern bool_t           bM4_SyncWithM1;
extern bool_t           bM1_SyncWithM2;
extern bool_t           bM3_SyncWithM2;
extern bool_t           bM4_SyncWithM2;
extern bool_t           bM1_SyncWithM3;
extern bool_t           bM2_SyncWithM3;
extern bool_t           bM4_SyncWithM3;
extern bool_t           bM1_SyncWithM4;
extern bool_t           bM2_SyncWithM4;
extern bool_t           bM3_SyncWithM4;

/* global used misc variables */
extern uint32_t         g_ui32NumberOfCycles;
extern uint32_t         g_ui32MaxNumberOfCycles;
extern uint16_t         g_ui16DemoLedType;

/* Application and board ID  */
extern app_ver_t        g_sAppIdFM;

extern bool_t           g_bM1SwitchAppOnOff;
extern mcdef_pmsm_t     g_sM1Drive;
extern sm_app_ctrl_t    g_sM1Ctrl;
extern mcdrv_qd_enc_t   g_sM1Enc;

extern bool_t           g_bM2SwitchAppOnOff;
extern mcdef_pmsm_t     g_sM2Drive;
extern sm_app_ctrl_t    g_sM2Ctrl;
extern mcdrv_qd_enc_t   g_sM2Enc;

extern bool_t           g_bM3SwitchAppOnOff;
extern mcdef_pmsm_t     g_sM3Drive;
extern sm_app_ctrl_t    g_sM3Ctrl;
extern mcdrv_qd_enc_t   g_sM3Enc;

extern bool_t           g_bM4SwitchAppOnOff;
extern mcdef_pmsm_t     g_sM4Drive;
extern sm_app_ctrl_t    g_sM4Ctrl;
extern mcdrv_qd_enc_t   g_sM4Enc;

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief g_sM1Drive table structure
 *
 * @param None
 * 
 * @return None
 */
FMSTR_TSA_TABLE_BEGIN(gsM1Drive_table)

//LED DEMOS
    FMSTR_TSA_RW_VAR(g_sM1Enc.ui16EncLedDemoType,   FMSTR_TSA_UINT16)
    FMSTR_TSA_RW_VAR(g_sM2Enc.ui16EncLedDemoType,   FMSTR_TSA_UINT16)
    FMSTR_TSA_RW_VAR(g_sM3Enc.ui16EncLedDemoType,   FMSTR_TSA_UINT16)
    FMSTR_TSA_RW_VAR(g_sM4Enc.ui16EncLedDemoType,   FMSTR_TSA_UINT16)
//SYNCHRONISATION
    FMSTR_TSA_RW_VAR(bM2_SyncWithM1, FMSTR_TSA_UINT16)
    FMSTR_TSA_RW_VAR(bM3_SyncWithM1, FMSTR_TSA_UINT16)
    FMSTR_TSA_RW_VAR(bM4_SyncWithM1, FMSTR_TSA_UINT16)    
    FMSTR_TSA_RW_VAR(bM1_SyncWithM2, FMSTR_TSA_UINT16)
    FMSTR_TSA_RW_VAR(bM3_SyncWithM2, FMSTR_TSA_UINT16)
    FMSTR_TSA_RW_VAR(bM4_SyncWithM2, FMSTR_TSA_UINT16)     
    FMSTR_TSA_RW_VAR(bM1_SyncWithM3, FMSTR_TSA_UINT16)
    FMSTR_TSA_RW_VAR(bM2_SyncWithM3, FMSTR_TSA_UINT16)
    FMSTR_TSA_RW_VAR(bM4_SyncWithM3, FMSTR_TSA_UINT16)      
    FMSTR_TSA_RW_VAR(bM1_SyncWithM4, FMSTR_TSA_UINT16)
    FMSTR_TSA_RW_VAR(bM2_SyncWithM4, FMSTR_TSA_UINT16)
    FMSTR_TSA_RW_VAR(bM3_SyncWithM4, FMSTR_TSA_UINT16)
      
   
#if 1  // DEBUG
    FMSTR_TSA_RW_VAR(g_sM1Drive.sPosition.a32PositionAlignOffset,   FMSTR_TSA_FRAC32)   /* M1 Position Align Offset */ 
    FMSTR_TSA_RW_VAR(g_sM2Drive.sPosition.a32PositionAlignOffset,   FMSTR_TSA_FRAC32)   /* M2 Position Align Offset */ 
    FMSTR_TSA_RW_VAR(g_sM3Drive.sPosition.a32PositionAlignOffset,   FMSTR_TSA_FRAC32)   /* M3 Position Align Offset */ 
    FMSTR_TSA_RW_VAR(g_sM4Drive.sPosition.a32PositionAlignOffset,   FMSTR_TSA_FRAC32)   /* M4 Position Align Offset */  
            
    FMSTR_TSA_RW_VAR(g_sM1Enc.ui16OnLedPosition,   FMSTR_TSA_UINT16)       /* M1 Meassured Mechanical Position */
    FMSTR_TSA_RW_VAR(g_sM1Enc.ui16OffLedDelta,   FMSTR_TSA_UINT16)         /* M1 Meassured Mechanical Position */
             
    FMSTR_TSA_RW_VAR(g_sM2Enc.ui16OnLedPosition,   FMSTR_TSA_UINT16)       /* M2 Meassured Mechanical Position */
    FMSTR_TSA_RW_VAR(g_sM2Enc.ui16OffLedDelta,   FMSTR_TSA_UINT16)         /* M2 Meassured Mechanical Position */
         
    FMSTR_TSA_RW_VAR(g_sM3Enc.ui16OnLedPosition,   FMSTR_TSA_UINT16)       /* M3 Meassured Mechanical Position */
    FMSTR_TSA_RW_VAR(g_sM3Enc.ui16OffLedDelta,   FMSTR_TSA_UINT16)         /* M3 Meassured Mechanical Position */
               
    FMSTR_TSA_RW_VAR(g_sM4Enc.ui16OnLedPosition,   FMSTR_TSA_UINT16)       /* M4 Meassured Mechanical Position */
    FMSTR_TSA_RW_VAR(g_sM4Enc.ui16OffLedDelta,   FMSTR_TSA_UINT16)         /* M4 Meassured Mechanical Position */
      
    FMSTR_TSA_RW_VAR(g_sM1Enc.ui16LedAlignOffset,   FMSTR_TSA_UINT16)
    FMSTR_TSA_RW_VAR(g_sM2Enc.ui16LedAlignOffset,   FMSTR_TSA_UINT16)
    FMSTR_TSA_RW_VAR(g_sM3Enc.ui16LedAlignOffset,   FMSTR_TSA_UINT16)
    FMSTR_TSA_RW_VAR(g_sM4Enc.ui16LedAlignOffset,   FMSTR_TSA_UINT16)
         
#endif

    /* gsM1Drive structure definition */
    FMSTR_TSA_RW_VAR(g_sM1Drive.bFaultClearMan,   FMSTR_TSA_UINT16)         /* M1 Fault Clear */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sFaultIdCaptured,   FMSTR_TSA_UINT16)       /* M1 Captured Fault */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sFaultIdPending,   FMSTR_TSA_UINT16)        /* M1 Pending Fault */
    FMSTR_TSA_RW_VAR(g_sM1Drive.eControl,   FMSTR_TSA_UINT16)               /* M1 MCAT Control */
    FMSTR_TSA_RW_VAR(g_sM1Drive.ui16SlowCtrlLoopFreq,   FMSTR_TSA_UINT16)   /* M1 Slow Control Loop Frequency */
    FMSTR_TSA_RW_VAR(g_sM1Drive.ui16FastCtrlLoopFreq,   FMSTR_TSA_UINT16)   /* M1 Fast Control Loop Frequency */

    /* gsM1Drive.sSpeed structure definition */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sSpeed.fltSpeedFilt,   FMSTR_TSA_FLOAT)    /* M1 Speed filtered */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sSpeed.fltSpeed,   FMSTR_TSA_FLOAT)        /* M1 Speed Estimated */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sSpeed.fltSpeedRamp,   FMSTR_TSA_FLOAT)    /* M1 Speed Ramp */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sSpeed.fltSpeedCmd,   FMSTR_TSA_FLOAT)     /* M1 Speed Required */
      
    FMSTR_TSA_RW_VAR(g_sM1Drive.sSpeed.fltSpeedError, FMSTR_TSA_FLOAT)     /* M1 Speed Required */ 

    /* sSpeed.sSpeedFilter.sSpeedFilter definitions */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sSpeed.sSpeedFilter.sFltCoeff.fltA1,   FMSTR_TSA_FLOAT) /* M1 Speed Filter A1 */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sSpeed.sSpeedFilter.sFltCoeff.fltB0,   FMSTR_TSA_FLOAT) /* M1 Speed Filter B0 */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sSpeed.sSpeedFilter.sFltCoeff.fltB1,   FMSTR_TSA_FLOAT) /* M1 Speed Filter B1 */

    /* sSpeed.sSpeedFilter.sSpeedRampParams definitions */  
    FMSTR_TSA_RW_VAR(g_sM1Drive.sSpeed.sSpeedRampParams.fltRampDown,   FMSTR_TSA_FLOAT) /* M1 Speed Ramp Down */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sSpeed.sSpeedRampParams.fltRampUp,   FMSTR_TSA_FLOAT)   /* M1 Speed Ramp Up */

    /* sSpeed.sSpeedFilter.sSpeedRampParams definitions */  
    FMSTR_TSA_RW_VAR(g_sM1Drive.sSpeed.sSpeedPiParams.fltIGain,   FMSTR_TSA_FLOAT)      /* M1 Speed Loop Ki Gain */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sSpeed.sSpeedPiParams.fltPGain,   FMSTR_TSA_FLOAT)      /* M1 Speed Loop Kp Gain */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sSpeed.sSpeedPiParams.fltUpperLim,   FMSTR_TSA_FLOAT)   /* M1 Speed Loop Limit High */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sSpeed.sSpeedPiParams.fltLowerLim,   FMSTR_TSA_FLOAT)   /* M1 Speed Loop Limit Low */
      
    /* gsM1Drive.sPosition structure definition */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sPosition.f16PositionPGain,   FMSTR_TSA_FRAC16)         /* M1 Position P conroller P Gain */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sPosition.a32Position,   FMSTR_TSA_FRAC32)              /* M1 Position Actual */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sPosition.a32PositionError,   FMSTR_TSA_FRAC32)         /* M1 Position Error */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sPosition.a32PositionCmd,   FMSTR_TSA_FRAC32)           /* M1 Position Required */   

    /* sSpeed.sAlignment definitions */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sAlignment.ui16Time,   FMSTR_TSA_UINT16)    /* M1 Alignment Duration */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sAlignment.fltUdReq,   FMSTR_TSA_FLOAT)     /* M1 Alignment Voltage */

    /* gsM1Drive.sFocPMSM structure definition */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sFocPMSM.fltDutyCycleLimit,  FMSTR_TSA_FLOAT)   /* M1 Current Loop Limit */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sFocPMSM.fltUDcBus,   FMSTR_TSA_FLOAT)          /* M1 DCB Voltage */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sFocPMSM.fltUDcBusFilt,   FMSTR_TSA_FLOAT)      /* M1 DCB Voltage Filtered */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sFocPMSM.f16PosElExt,   FMSTR_TSA_UINT16)       /* M1 Posirtion External */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sFocPMSM.f16PosEl,   FMSTR_TSA_UINT16)          /* M1 Position Electrical */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sFocPMSM.f16PosElEst,   FMSTR_TSA_UINT16)       /* M1 Position Estimated */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sFocPMSM.ui16SectorSVM,   FMSTR_TSA_UINT16)     /* M1 SVM Sector */

    /* sFocPMSM.sIAlBe definitions */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sFocPMSM.sIAlBe.fltAlpha,   FMSTR_TSA_FLOAT)/* M1 I alpha */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sFocPMSM.sIAlBe.fltBeta,   FMSTR_TSA_FLOAT) /* M1 I beta */

    /* sFocPMSM.sIDQ definitions */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sFocPMSM.sIDQ.fltD,   FMSTR_TSA_FLOAT)      /* M1 Id */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sFocPMSM.sIDQ.fltQ,   FMSTR_TSA_FLOAT)      /* M1 Iq */

    /* sFocPMSM.sIDQReq definitions */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sFocPMSM.sIDQReq.fltD,   FMSTR_TSA_FLOAT)   /* M1 Id req */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sFocPMSM.sIDQReq.fltQ,   FMSTR_TSA_FLOAT)   /* M1 Iq req */

    /* sFocPMSM.sIDQReq definitions */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sFocPMSM.sUDQReq.fltD,   FMSTR_TSA_FLOAT)   /* M1 Ud req */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sFocPMSM.sUDQReq.fltQ,   FMSTR_TSA_FLOAT)   /* M1 Uq req */

    /* sFocPMSM.sIdPiParams definitions */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sFocPMSM.sIdPiParams.fltIGain,   FMSTR_TSA_FLOAT)        /* M1 Id Ki Gain */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sFocPMSM.sIdPiParams.fltPGain,   FMSTR_TSA_FLOAT)        /* M1 Id Kp Gain */

    /* sFocPMSM.sBemfObsrv definitions */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sFocPMSM.sBemfObsrv.fltEGain,   FMSTR_TSA_FLOAT)         /* M1 Obsrv E gain */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sFocPMSM.sBemfObsrv.fltIGain,   FMSTR_TSA_FLOAT)         /* M1 Obsrv I gain */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sFocPMSM.sBemfObsrv.sCtrl.fltIGain,   FMSTR_TSA_FLOAT)   /* M1 Obsrv Ki gain */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sFocPMSM.sBemfObsrv.sCtrl.fltPGain,   FMSTR_TSA_FLOAT)   /* M1 Obsrv Kp gain */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sFocPMSM.sBemfObsrv.fltUGain,   FMSTR_TSA_FLOAT)         /* M1 Obsrv U gain */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sFocPMSM.sBemfObsrv.fltWIGain,   FMSTR_TSA_FLOAT)        /* M1 Obsrv WI gain */

    /* sFocPMSM.sTo definitions */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sFocPMSM.sTo.fltIGain,   FMSTR_TSA_FLOAT)           /* M1 Obsrv To Ki gain */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sFocPMSM.sTo.fltPGain,   FMSTR_TSA_FLOAT)           /* M1 Obsrv To Kp gain */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sFocPMSM.sTo.fltThGain ,   FMSTR_TSA_FLOAT)         /* M1 Obsrv To Theta gain */

    /* sFocPMSM.sIqPiParams definitions */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sFocPMSM.sIqPiParams.fltIGain,   FMSTR_TSA_FLOAT)   /* M1 Iq Ki Gain */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sFocPMSM.sIqPiParams.fltPGain,   FMSTR_TSA_FLOAT)   /* M1 Iq Kp Gain */

    /* sFocPMSM.sIABC definitions */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sFocPMSM.sIABC.fltA,   FMSTR_TSA_FLOAT)        /* M1 Phase Current A */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sFocPMSM.sIABC.fltB,   FMSTR_TSA_FLOAT)        /* M1 Phase Current B */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sFocPMSM.sIABC.fltC,   FMSTR_TSA_FLOAT)        /* M1 Phase Current C */
      
    /* sFocPMSM.fltSpeedElEst definition */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sFocPMSM.fltSpeedElEst,   FMSTR_TSA_FLOAT)     /* M1 Speed Estimated */

    /* sFaultThresholds definitions */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sFaultThresholds.fltUqBemf,   FMSTR_TSA_FLOAT)         /* M1 Fault Threshold BemfBlocked */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sFaultThresholds.fltUDcBusOver,   FMSTR_TSA_FLOAT)     /* M1 Fault Threshold DcBusOver */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sFaultThresholds.fltUDcBusTrip,   FMSTR_TSA_FLOAT)     /* M1 Fault Threshold DcBusTrip */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sFaultThresholds.fltUDcBusUnder,   FMSTR_TSA_FLOAT)    /* M1 Fault Threshold DcBusUnder */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sFaultThresholds.fltSpeedMin,   FMSTR_TSA_FLOAT)       /* M1 Fault Threshold SpeedMin */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sFaultThresholds.fltSpeedNom,   FMSTR_TSA_FLOAT)       /* M1 Fault Threshold SpeedNom */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sFaultThresholds.fltSpeedOver,   FMSTR_TSA_FLOAT)      /* M1 Fault Threshold SpeedOver */

    /* sStartUp definitions */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sStartUp.f16CoeffMerging,   FMSTR_TSA_FRAC16)       /* M1 Merging Coefficient */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sStartUp.f16RatioMerging,   FMSTR_TSA_FRAC16)       /* M1 Merging Ratio */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sStartUp.fltSpeedCatchUp,   FMSTR_TSA_FLOAT)        /* M1 Merging Speed Catch Up */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sStartUp.f16PosGen,   FMSTR_TSA_FRAC16)             /* M1 Position Open Loop */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sStartUp.fltSpeedCatchUp,   FMSTR_TSA_FLOAT)        /* M1 Speed Merging Catch Up  */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sStartUp.fltSpeedRampOpenLoop,   FMSTR_TSA_FLOAT)   /* M1 Speed Ramp Open Loop  */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sStartUp.fltCurrentStartup,   FMSTR_TSA_FLOAT)      /* M1 Startup Current  */

    /* sStartUp.sSpeedRampOpenLoopParams definitions */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sStartUp.sSpeedRampOpenLoopParams.fltRampDown,   FMSTR_TSA_FLOAT)      /* M1 Startup Ramp Dec */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sStartUp.sSpeedRampOpenLoopParams.fltRampUp,   FMSTR_TSA_FLOAT)        /* M1 Startup Ramp Inc */

    /* sScalarCtrl definitions */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sScalarCtrl.f16PosElScalar,   FMSTR_TSA_FRAC16) /* M1 Position Electrical Scalar */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sScalarCtrl.fltFreqRamp,   FMSTR_TSA_FLOAT)     /* M1 Scalar Frequency Ramp */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sScalarCtrl.fltFreqCmd,   FMSTR_TSA_FLOAT)      /* M1 Scalar speed */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sScalarCtrl.fltVHzGain,   FMSTR_TSA_FLOAT)      /* M1 VHz Factor Gain */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sScalarCtrl.fltFreqMax,   FMSTR_TSA_FLOAT)      /* M1 FMSTR_M1_frequencyScale */      

    /* sScalarCtrl.sFreqRampParams definitions */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sScalarCtrl.sFreqRampParams.fltRampDown,   FMSTR_TSA_FLOAT)    /* M1 Scalar Ramp Down */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sScalarCtrl.sFreqRampParams.fltRampUp,   FMSTR_TSA_FLOAT)      /* M1 Scalar Ramp Up */

    /* sMCATctrl definitions */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sMCATctrl.ui16PospeSensor,   FMSTR_TSA_UINT16)      /* M1 MCAT POSPE Sensor */

    /* sMCATctrl.sIDQReqMCAT definitions */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sMCATctrl.sIDQReqMCAT.fltD,   FMSTR_TSA_FLOAT)     /* M1 MCAT Id Required */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sMCATctrl.sIDQReqMCAT.fltQ,   FMSTR_TSA_FLOAT)     /* M1 MCAT Iq Required */

    /* sMCATctrl.sUDQReqMCAT definitions */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sMCATctrl.sUDQReqMCAT.fltD,   FMSTR_TSA_FLOAT)     /* M1 MCAT Ud Required */
    FMSTR_TSA_RW_VAR(g_sM1Drive.sMCATctrl.sUDQReqMCAT.fltQ,   FMSTR_TSA_FLOAT)     /* M1 MCAT Uq Required */
      


FMSTR_TSA_TABLE_END()

/*!
 * @brief g_sM2Drive table structure
 *
 * @param None
 * 
 * @return None
 */
FMSTR_TSA_TABLE_BEGIN(gsM2Drive_table)

    /* gsM2Drive structure definition */
    FMSTR_TSA_RW_VAR(g_sM2Drive.bFaultClearMan,   FMSTR_TSA_UINT16)         /* M2 Fault Clear */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sFaultIdCaptured,   FMSTR_TSA_UINT16)       /* M2 Captured Fault */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sFaultIdPending,   FMSTR_TSA_UINT16)        /* M2 Pending Fault */
    FMSTR_TSA_RW_VAR(g_sM2Drive.eControl,   FMSTR_TSA_UINT16)               /* M2 MCAT Control */
    FMSTR_TSA_RW_VAR(g_sM2Drive.ui16SlowCtrlLoopFreq,   FMSTR_TSA_UINT16)   /* M2 Slow Control Loop Frequency */
    FMSTR_TSA_RW_VAR(g_sM2Drive.ui16FastCtrlLoopFreq,   FMSTR_TSA_UINT16)   /* M2 Fast Control Loop Frequency */

    /* gsM2Drive.sSpeed structure definition */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sSpeed.fltSpeedFilt,   FMSTR_TSA_FLOAT)    /* M2 Speed filtered */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sSpeed.fltSpeed,   FMSTR_TSA_FLOAT)        /* M2 Speed Estimated */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sSpeed.fltSpeedRamp,   FMSTR_TSA_FLOAT)    /* M2 Speed Ramp */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sSpeed.fltSpeedCmd,   FMSTR_TSA_FLOAT)     /* M2 Speed Required */

    /* sSpeed.sSpeedFilter.sSpeedFilter definitions */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sSpeed.sSpeedFilter.sFltCoeff.fltA1,   FMSTR_TSA_FLOAT) /* M2 Speed Filter A1 */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sSpeed.sSpeedFilter.sFltCoeff.fltB0,   FMSTR_TSA_FLOAT) /* M2 Speed Filter B0 */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sSpeed.sSpeedFilter.sFltCoeff.fltB1,   FMSTR_TSA_FLOAT) /* M2 Speed Filter B1 */

    /* sSpeed.sSpeedFilter.sSpeedRampParams definitions */  
    FMSTR_TSA_RW_VAR(g_sM2Drive.sSpeed.sSpeedRampParams.fltRampDown,   FMSTR_TSA_FLOAT) /* M2 Speed Ramp Down */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sSpeed.sSpeedRampParams.fltRampUp,   FMSTR_TSA_FLOAT)   /* M2 Speed Ramp Up */

    /* sSpeed.sSpeedFilter.sSpeedRampParams definitions */  
    FMSTR_TSA_RW_VAR(g_sM2Drive.sSpeed.sSpeedPiParams.fltIGain,   FMSTR_TSA_FLOAT)      /* M2 Speed Loop Ki Gain */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sSpeed.sSpeedPiParams.fltPGain,   FMSTR_TSA_FLOAT)      /* M2 Speed Loop Kp Gain */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sSpeed.sSpeedPiParams.fltUpperLim,   FMSTR_TSA_FLOAT)   /* M2 Speed Loop Limit High */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sSpeed.sSpeedPiParams.fltLowerLim,   FMSTR_TSA_FLOAT)   /* M2 Speed Loop Limit Low */
      
    /* gsM2Drive.sPosition structure definition */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sPosition.f16PositionPGain,   FMSTR_TSA_FRAC16)         /* M2 Position P conroller P Gain */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sPosition.a32Position,   FMSTR_TSA_FRAC32)              /* M2 Position Actual */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sPosition.a32PositionError,   FMSTR_TSA_FRAC32)         /* M2 Position Error */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sPosition.a32PositionCmd,   FMSTR_TSA_FRAC32)           /* M2 Position Required */         

    /* sSpeed.sAlignment definitions */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sAlignment.ui16Time,   FMSTR_TSA_UINT16)    /* M2 Alignment Duration */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sAlignment.fltUdReq,   FMSTR_TSA_FLOAT)     /* M2 Alignment Voltage */

    /* gsM2Drive.sFocPMSM structure definition */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sFocPMSM.fltDutyCycleLimit,  FMSTR_TSA_FLOAT)   /* M2 Current Loop Limit */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sFocPMSM.fltUDcBus,   FMSTR_TSA_FLOAT)          /* M2 DCB Voltage */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sFocPMSM.fltUDcBusFilt,   FMSTR_TSA_FLOAT)      /* M2 DCB Voltage Filtered */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sFocPMSM.f16PosElExt,   FMSTR_TSA_UINT16)       /* M2 Posirtion External */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sFocPMSM.f16PosEl,   FMSTR_TSA_UINT16)          /* M2 Position Electrical */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sFocPMSM.f16PosElEst,   FMSTR_TSA_UINT16)       /* M2 Position Estimated */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sFocPMSM.ui16SectorSVM,   FMSTR_TSA_UINT16)     /* M2 SVM Sector */

    /* sFocPMSM.sIAlBe definitions */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sFocPMSM.sIAlBe.fltAlpha,   FMSTR_TSA_FLOAT)/* M2 I alpha */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sFocPMSM.sIAlBe.fltBeta,   FMSTR_TSA_FLOAT) /* M2 I beta */

    /* sFocPMSM.sIDQ definitions */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sFocPMSM.sIDQ.fltD,   FMSTR_TSA_FLOAT)      /* M2 Id */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sFocPMSM.sIDQ.fltQ,   FMSTR_TSA_FLOAT)      /* M2 Iq */

    /* sFocPMSM.sIDQReq definitions */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sFocPMSM.sIDQReq.fltD,   FMSTR_TSA_FLOAT)   /* M2 Id req */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sFocPMSM.sIDQReq.fltQ,   FMSTR_TSA_FLOAT)   /* M2 Iq req */

    /* sFocPMSM.sIDQReq definitions */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sFocPMSM.sUDQReq.fltD,   FMSTR_TSA_FLOAT)   /* M2 Ud req */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sFocPMSM.sUDQReq.fltQ,   FMSTR_TSA_FLOAT)   /* M2 Uq req */

    /* sFocPMSM.sIdPiParams definitions */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sFocPMSM.sIdPiParams.fltIGain,   FMSTR_TSA_FLOAT)        /* M2 Id Ki Gain */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sFocPMSM.sIdPiParams.fltPGain,   FMSTR_TSA_FLOAT)        /* M2 Id Kp Gain */

    /* sFocPMSM.sBemfObsrv definitions */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sFocPMSM.sBemfObsrv.fltEGain,   FMSTR_TSA_FLOAT)         /* M2 Obsrv E gain */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sFocPMSM.sBemfObsrv.fltIGain,   FMSTR_TSA_FLOAT)         /* M2 Obsrv I gain */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sFocPMSM.sBemfObsrv.sCtrl.fltIGain,   FMSTR_TSA_FLOAT)   /* M2 Obsrv Ki gain */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sFocPMSM.sBemfObsrv.sCtrl.fltPGain,   FMSTR_TSA_FLOAT)   /* M2 Obsrv Kp gain */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sFocPMSM.sBemfObsrv.fltUGain,   FMSTR_TSA_FLOAT)         /* M2 Obsrv U gain */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sFocPMSM.sBemfObsrv.fltWIGain,   FMSTR_TSA_FLOAT)        /* M2 Obsrv WI gain */

    /* sFocPMSM.sTo definitions */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sFocPMSM.sTo.fltIGain,   FMSTR_TSA_FLOAT)           /* M2 Obsrv To Ki gain */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sFocPMSM.sTo.fltPGain,   FMSTR_TSA_FLOAT)           /* M2 Obsrv To Kp gain */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sFocPMSM.sTo.fltThGain ,   FMSTR_TSA_FLOAT)         /* M2 Obsrv To Theta gain */

    /* sFocPMSM.sIqPiParams definitions */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sFocPMSM.sIqPiParams.fltIGain,   FMSTR_TSA_FLOAT)   /* M2 Iq Ki Gain */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sFocPMSM.sIqPiParams.fltPGain,   FMSTR_TSA_FLOAT)   /* M2 Iq Kp Gain */

    /* sFocPMSM.sIABC definitions */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sFocPMSM.sIABC.fltA,   FMSTR_TSA_FLOAT)        /* M2 Phase Current A */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sFocPMSM.sIABC.fltB,   FMSTR_TSA_FLOAT)        /* M2 Phase Current B */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sFocPMSM.sIABC.fltC,   FMSTR_TSA_FLOAT)        /* M2 Phase Current C */
      
    /* sFocPMSM.fltSpeedElEst definition */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sFocPMSM.fltSpeedElEst,   FMSTR_TSA_FLOAT)     /* M2 Speed Estimated */

    /* sFaultThresholds definitions */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sFaultThresholds.fltUqBemf,   FMSTR_TSA_FLOAT)         /* M2 Fault Threshold BemfBlocked */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sFaultThresholds.fltUDcBusOver,   FMSTR_TSA_FLOAT)     /* M2 Fault Threshold DcBusOver */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sFaultThresholds.fltUDcBusTrip,   FMSTR_TSA_FLOAT)     /* M2 Fault Threshold DcBusTrip */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sFaultThresholds.fltUDcBusUnder,   FMSTR_TSA_FLOAT)    /* M2 Fault Threshold DcBusUnder */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sFaultThresholds.fltSpeedMin,   FMSTR_TSA_FLOAT)       /* M2 Fault Threshold SpeedMin */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sFaultThresholds.fltSpeedNom,   FMSTR_TSA_FLOAT)       /* M2 Fault Threshold SpeedNom */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sFaultThresholds.fltSpeedOver,   FMSTR_TSA_FLOAT)      /* M2 Fault Threshold SpeedOver */

    /* sStartUp definitions */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sStartUp.f16CoeffMerging,   FMSTR_TSA_FRAC16)       /* M2 Merging Coefficient */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sStartUp.f16RatioMerging,   FMSTR_TSA_FRAC16)       /* M2 Merging Ratio */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sStartUp.fltSpeedCatchUp,   FMSTR_TSA_FLOAT)        /* M2 Merging Speed Catch Up */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sStartUp.f16PosGen,   FMSTR_TSA_FRAC16)             /* M2 Position Open Loop */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sStartUp.fltSpeedCatchUp,   FMSTR_TSA_FLOAT)        /* M2 Speed Merging Catch Up  */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sStartUp.fltSpeedRampOpenLoop,   FMSTR_TSA_FLOAT)   /* M2 Speed Ramp Open Loop  */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sStartUp.fltCurrentStartup,   FMSTR_TSA_FLOAT)      /* M2 Startup Current  */

    /* sStartUp.sSpeedRampOpenLoopParams definitions */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sStartUp.sSpeedRampOpenLoopParams.fltRampDown,   FMSTR_TSA_FLOAT)      /* M2 Startup Ramp Dec */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sStartUp.sSpeedRampOpenLoopParams.fltRampUp,   FMSTR_TSA_FLOAT)        /* M2 Startup Ramp Inc */

    /* sScalarCtrl definitions */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sScalarCtrl.f16PosElScalar,   FMSTR_TSA_FRAC16) /* M2 Position Electrical Scalar */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sScalarCtrl.fltFreqRamp,   FMSTR_TSA_FLOAT)     /* M2 Scalar Frequency Ramp */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sScalarCtrl.fltFreqCmd,   FMSTR_TSA_FLOAT)      /* M2 Scalar speed */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sScalarCtrl.fltVHzGain,   FMSTR_TSA_FLOAT)      /* M2 VHz Factor Gain */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sScalarCtrl.fltFreqMax,   FMSTR_TSA_FLOAT)      /* M2 FMSTR_M2_frequencyScale */      

    /* sScalarCtrl.sFreqRampParams definitions */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sScalarCtrl.sFreqRampParams.fltRampDown,   FMSTR_TSA_FLOAT)    /* M2 Scalar Ramp Down */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sScalarCtrl.sFreqRampParams.fltRampUp,   FMSTR_TSA_FLOAT)      /* M2 Scalar Ramp Up */

    /* sMCATctrl definitions */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sMCATctrl.ui16PospeSensor,   FMSTR_TSA_UINT16)      /* M2 MCAT POSPE Sensor */

    /* sMCATctrl.sIDQReqMCAT definitions */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sMCATctrl.sIDQReqMCAT.fltD,   FMSTR_TSA_FLOAT)     /* M2 MCAT Id Required */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sMCATctrl.sIDQReqMCAT.fltQ,   FMSTR_TSA_FLOAT)     /* M2 MCAT Iq Required */

    /* sMCATctrl.sUDQReqMCAT definitions */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sMCATctrl.sUDQReqMCAT.fltD,   FMSTR_TSA_FLOAT)     /* M2 MCAT Ud Required */
    FMSTR_TSA_RW_VAR(g_sM2Drive.sMCATctrl.sUDQReqMCAT.fltQ,   FMSTR_TSA_FLOAT)     /* M2 MCAT Uq Required */
      
FMSTR_TSA_TABLE_END()


/*!
 * @brief g_sM3Drive table structure
 *
 * @param None
 * 
 * @return None
 */
FMSTR_TSA_TABLE_BEGIN(gsM3Drive_table)

    /* gsM1Drive structure definition */
    FMSTR_TSA_RW_VAR(g_sM3Drive.bFaultClearMan,   FMSTR_TSA_UINT16)         /* M3 Fault Clear */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sFaultIdCaptured,   FMSTR_TSA_UINT16)       /* M3 Captured Fault */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sFaultIdPending,   FMSTR_TSA_UINT16)        /* M3 Pending Fault */
    FMSTR_TSA_RW_VAR(g_sM3Drive.eControl,   FMSTR_TSA_UINT16)               /* M3 MCAT Control */
    FMSTR_TSA_RW_VAR(g_sM3Drive.ui16SlowCtrlLoopFreq,   FMSTR_TSA_UINT16)   /* M3 Slow Control Loop Frequency */
    FMSTR_TSA_RW_VAR(g_sM3Drive.ui16FastCtrlLoopFreq,   FMSTR_TSA_UINT16)   /* M3 Fast Control Loop Frequency */

    /* gsM3Drive.sSpeed structure definition */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sSpeed.fltSpeedFilt,   FMSTR_TSA_FLOAT)    /* M3 Speed filtered */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sSpeed.fltSpeed,   FMSTR_TSA_FLOAT)        /* M3 Speed Estimated */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sSpeed.fltSpeedRamp,   FMSTR_TSA_FLOAT)    /* M3 Speed Ramp */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sSpeed.fltSpeedCmd,   FMSTR_TSA_FLOAT)     /* M3 Speed Required */

    /* sSpeed.sSpeedFilter.sSpeedFilter definitions */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sSpeed.sSpeedFilter.sFltCoeff.fltA1,   FMSTR_TSA_FLOAT) /* M3 Speed Filter A1 */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sSpeed.sSpeedFilter.sFltCoeff.fltB0,   FMSTR_TSA_FLOAT) /* M3 Speed Filter B0 */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sSpeed.sSpeedFilter.sFltCoeff.fltB1,   FMSTR_TSA_FLOAT) /* M3 Speed Filter B1 */

    /* sSpeed.sSpeedFilter.sSpeedRampParams definitions */  
    FMSTR_TSA_RW_VAR(g_sM3Drive.sSpeed.sSpeedRampParams.fltRampDown,   FMSTR_TSA_FLOAT) /* M3 Speed Ramp Down */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sSpeed.sSpeedRampParams.fltRampUp,   FMSTR_TSA_FLOAT)   /* M3 Speed Ramp Up */

    /* sSpeed.sSpeedFilter.sSpeedRampParams definitions */  
    FMSTR_TSA_RW_VAR(g_sM3Drive.sSpeed.sSpeedPiParams.fltIGain,   FMSTR_TSA_FLOAT)      /* M3 Speed Loop Ki Gain */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sSpeed.sSpeedPiParams.fltPGain,   FMSTR_TSA_FLOAT)      /* M3 Speed Loop Kp Gain */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sSpeed.sSpeedPiParams.fltUpperLim,   FMSTR_TSA_FLOAT)   /* M3 Speed Loop Limit High */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sSpeed.sSpeedPiParams.fltLowerLim,   FMSTR_TSA_FLOAT)   /* M3 Speed Loop Limit Low */
      
    /* gsM3Drive.sPosition structure definition */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sPosition.f16PositionPGain,   FMSTR_TSA_FRAC16)         /* M3 Position P conroller P Gain */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sPosition.a32Position,   FMSTR_TSA_FRAC32)              /* M3 Position Actual */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sPosition.a32PositionError,   FMSTR_TSA_FRAC32)         /* M3 Position Error */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sPosition.a32PositionCmd,   FMSTR_TSA_FRAC32)           /* M3 Position Required */   

    /* sSpeed.sAlignment definitions */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sAlignment.ui16Time,   FMSTR_TSA_UINT16)    /* M3 Alignment Duration */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sAlignment.fltUdReq,   FMSTR_TSA_FLOAT)     /* M3 Alignment Voltage */

    /* gsM3Drive.sFocPMSM structure definition */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sFocPMSM.fltDutyCycleLimit,  FMSTR_TSA_FLOAT)   /* M3 Current Loop Limit */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sFocPMSM.fltUDcBus,   FMSTR_TSA_FLOAT)          /* M3 DCB Voltage */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sFocPMSM.fltUDcBusFilt,   FMSTR_TSA_FLOAT)      /* M3 DCB Voltage Filtered */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sFocPMSM.f16PosElExt,   FMSTR_TSA_UINT16)       /* M3 Posirtion External */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sFocPMSM.f16PosEl,   FMSTR_TSA_UINT16)          /* M3 Position Electrical */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sFocPMSM.f16PosElEst,   FMSTR_TSA_UINT16)       /* M3 Position Estimated */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sFocPMSM.ui16SectorSVM,   FMSTR_TSA_UINT16)     /* M3 SVM Sector */

    /* sFocPMSM.sIAlBe definitions */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sFocPMSM.sIAlBe.fltAlpha,   FMSTR_TSA_FLOAT)/* M3 I alpha */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sFocPMSM.sIAlBe.fltBeta,   FMSTR_TSA_FLOAT) /* M3 I beta */

    /* sFocPMSM.sIDQ definitions */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sFocPMSM.sIDQ.fltD,   FMSTR_TSA_FLOAT)      /* M3 Id */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sFocPMSM.sIDQ.fltQ,   FMSTR_TSA_FLOAT)      /* M3 Iq */

    /* sFocPMSM.sIDQReq definitions */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sFocPMSM.sIDQReq.fltD,   FMSTR_TSA_FLOAT)   /* M3 Id req */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sFocPMSM.sIDQReq.fltQ,   FMSTR_TSA_FLOAT)   /* M3 Iq req */

    /* sFocPMSM.sIDQReq definitions */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sFocPMSM.sUDQReq.fltD,   FMSTR_TSA_FLOAT)   /* M3 Ud req */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sFocPMSM.sUDQReq.fltQ,   FMSTR_TSA_FLOAT)   /* M3 Uq req */

    /* sFocPMSM.sIdPiParams definitions */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sFocPMSM.sIdPiParams.fltIGain,   FMSTR_TSA_FLOAT)        /* M3 Id Ki Gain */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sFocPMSM.sIdPiParams.fltPGain,   FMSTR_TSA_FLOAT)        /* M3 Id Kp Gain */

    /* sFocPMSM.sBemfObsrv definitions */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sFocPMSM.sBemfObsrv.fltEGain,   FMSTR_TSA_FLOAT)         /* M3 Obsrv E gain */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sFocPMSM.sBemfObsrv.fltIGain,   FMSTR_TSA_FLOAT)         /* M3 Obsrv I gain */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sFocPMSM.sBemfObsrv.sCtrl.fltIGain,   FMSTR_TSA_FLOAT)   /* M3 Obsrv Ki gain */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sFocPMSM.sBemfObsrv.sCtrl.fltPGain,   FMSTR_TSA_FLOAT)   /* M3 Obsrv Kp gain */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sFocPMSM.sBemfObsrv.fltUGain,   FMSTR_TSA_FLOAT)         /* M3 Obsrv U gain */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sFocPMSM.sBemfObsrv.fltWIGain,   FMSTR_TSA_FLOAT)        /* M3 Obsrv WI gain */

    /* sFocPMSM.sTo definitions */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sFocPMSM.sTo.fltIGain,   FMSTR_TSA_FLOAT)           /* M3 Obsrv To Ki gain */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sFocPMSM.sTo.fltPGain,   FMSTR_TSA_FLOAT)           /* M3 Obsrv To Kp gain */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sFocPMSM.sTo.fltThGain ,   FMSTR_TSA_FLOAT)         /* M3 Obsrv To Theta gain */

    /* sFocPMSM.sIqPiParams definitions */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sFocPMSM.sIqPiParams.fltIGain,   FMSTR_TSA_FLOAT)   /* M3 Iq Ki Gain */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sFocPMSM.sIqPiParams.fltPGain,   FMSTR_TSA_FLOAT)   /* M3 Iq Kp Gain */

    /* sFocPMSM.sIABC definitions */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sFocPMSM.sIABC.fltA,   FMSTR_TSA_FLOAT)        /* M3 Phase Current A */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sFocPMSM.sIABC.fltB,   FMSTR_TSA_FLOAT)        /* M3 Phase Current B */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sFocPMSM.sIABC.fltC,   FMSTR_TSA_FLOAT)        /* M3 Phase Current C */
      
    /* sFocPMSM.fltSpeedElEst definition */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sFocPMSM.fltSpeedElEst,   FMSTR_TSA_FLOAT)     /* M3 Speed Estimated */

    /* sFaultThresholds definitions */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sFaultThresholds.fltUqBemf,   FMSTR_TSA_FLOAT)         /* M3 Fault Threshold BemfBlocked */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sFaultThresholds.fltUDcBusOver,   FMSTR_TSA_FLOAT)     /* M3 Fault Threshold DcBusOver */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sFaultThresholds.fltUDcBusTrip,   FMSTR_TSA_FLOAT)     /* M3 Fault Threshold DcBusTrip */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sFaultThresholds.fltUDcBusUnder,   FMSTR_TSA_FLOAT)    /* M3 Fault Threshold DcBusUnder */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sFaultThresholds.fltSpeedMin,   FMSTR_TSA_FLOAT)       /* M3 Fault Threshold SpeedMin */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sFaultThresholds.fltSpeedNom,   FMSTR_TSA_FLOAT)       /* M3 Fault Threshold SpeedNom */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sFaultThresholds.fltSpeedOver,   FMSTR_TSA_FLOAT)      /* M3 Fault Threshold SpeedOver */

    /* sStartUp definitions */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sStartUp.f16CoeffMerging,   FMSTR_TSA_FRAC16)       /* M3 Merging Coefficient */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sStartUp.f16RatioMerging,   FMSTR_TSA_FRAC16)       /* M3 Merging Ratio */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sStartUp.fltSpeedCatchUp,   FMSTR_TSA_FLOAT)        /* M3 Merging Speed Catch Up */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sStartUp.f16PosGen,   FMSTR_TSA_FRAC16)             /* M3 Position Open Loop */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sStartUp.fltSpeedCatchUp,   FMSTR_TSA_FLOAT)        /* M3 Speed Merging Catch Up  */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sStartUp.fltSpeedRampOpenLoop,   FMSTR_TSA_FLOAT)   /* M3 Speed Ramp Open Loop  */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sStartUp.fltCurrentStartup,   FMSTR_TSA_FLOAT)      /* M3 Startup Current  */

    /* sStartUp.sSpeedRampOpenLoopParams definitions */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sStartUp.sSpeedRampOpenLoopParams.fltRampDown,   FMSTR_TSA_FLOAT)      /* M3 Startup Ramp Dec */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sStartUp.sSpeedRampOpenLoopParams.fltRampUp,   FMSTR_TSA_FLOAT)        /* M3 Startup Ramp Inc */

    /* sScalarCtrl definitions */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sScalarCtrl.f16PosElScalar,   FMSTR_TSA_FRAC16) /* M3 Position Electrical Scalar */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sScalarCtrl.fltFreqRamp,   FMSTR_TSA_FLOAT)     /* M3 Scalar Frequency Ramp */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sScalarCtrl.fltFreqCmd,   FMSTR_TSA_FLOAT)      /* M3 Scalar speed */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sScalarCtrl.fltVHzGain,   FMSTR_TSA_FLOAT)      /* M3 VHz Factor Gain */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sScalarCtrl.fltFreqMax,   FMSTR_TSA_FLOAT)      /* M3 FMSTR_M3_frequencyScale */      

    /* sScalarCtrl.sFreqRampParams definitions */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sScalarCtrl.sFreqRampParams.fltRampDown,   FMSTR_TSA_FLOAT)    /* M3 Scalar Ramp Down */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sScalarCtrl.sFreqRampParams.fltRampUp,   FMSTR_TSA_FLOAT)      /* M3 Scalar Ramp Up */

    /* sMCATctrl definitions */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sMCATctrl.ui16PospeSensor,   FMSTR_TSA_UINT16)      /* M3 MCAT POSPE Sensor */

    /* sMCATctrl.sIDQReqMCAT definitions */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sMCATctrl.sIDQReqMCAT.fltD,   FMSTR_TSA_FLOAT)     /* M3 MCAT Id Required */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sMCATctrl.sIDQReqMCAT.fltQ,   FMSTR_TSA_FLOAT)     /* M3 MCAT Iq Required */

    /* sMCATctrl.sUDQReqMCAT definitions */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sMCATctrl.sUDQReqMCAT.fltD,   FMSTR_TSA_FLOAT)     /* M3 MCAT Ud Required */
    FMSTR_TSA_RW_VAR(g_sM3Drive.sMCATctrl.sUDQReqMCAT.fltQ,   FMSTR_TSA_FLOAT)     /* M3 MCAT Uq Required */

FMSTR_TSA_TABLE_END()


/*!
 * @brief g_sM4Drive table structure
 *
 * @param None
 * 
 * @return None
 */
FMSTR_TSA_TABLE_BEGIN(gsM4Drive_table)

    /* gsM1Drive structure definition */
    FMSTR_TSA_RW_VAR(g_sM4Drive.bFaultClearMan,   FMSTR_TSA_UINT16)         /* M4 Fault Clear */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sFaultIdCaptured,   FMSTR_TSA_UINT16)       /* M4 Captured Fault */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sFaultIdPending,   FMSTR_TSA_UINT16)        /* M4 Pending Fault */
    FMSTR_TSA_RW_VAR(g_sM4Drive.eControl,   FMSTR_TSA_UINT16)               /* M4 MCAT Control */
    FMSTR_TSA_RW_VAR(g_sM4Drive.ui16SlowCtrlLoopFreq,   FMSTR_TSA_UINT16)   /* M4 Slow Control Loop Frequency */
    FMSTR_TSA_RW_VAR(g_sM4Drive.ui16FastCtrlLoopFreq,   FMSTR_TSA_UINT16)   /* M4 Fast Control Loop Frequency */

    /* gsM4Drive.sSpeed structure definition */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sSpeed.fltSpeedFilt,   FMSTR_TSA_FLOAT)    /* M4 Speed filtered */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sSpeed.fltSpeed,   FMSTR_TSA_FLOAT)        /* M4 Speed Estimated */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sSpeed.fltSpeedRamp,   FMSTR_TSA_FLOAT)    /* M4 Speed Ramp */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sSpeed.fltSpeedCmd,   FMSTR_TSA_FLOAT)     /* M4 Speed Required */
      
    FMSTR_TSA_RW_VAR(g_sM4Drive.sSpeed.fltSpeedError, FMSTR_TSA_FLOAT)     /* M1 Speed Required */ 

    /* sSpeed.sSpeedFilter.sSpeedFilter definitions */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sSpeed.sSpeedFilter.sFltCoeff.fltA1,   FMSTR_TSA_FLOAT) /* M4 Speed Filter A1 */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sSpeed.sSpeedFilter.sFltCoeff.fltB0,   FMSTR_TSA_FLOAT) /* M4 Speed Filter B0 */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sSpeed.sSpeedFilter.sFltCoeff.fltB1,   FMSTR_TSA_FLOAT) /* M4 Speed Filter B1 */

    /* sSpeed.sSpeedFilter.sSpeedRampParams definitions */  
    FMSTR_TSA_RW_VAR(g_sM4Drive.sSpeed.sSpeedRampParams.fltRampDown,   FMSTR_TSA_FLOAT) /* M4 Speed Ramp Down */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sSpeed.sSpeedRampParams.fltRampUp,   FMSTR_TSA_FLOAT)   /* M4 Speed Ramp Up */

    /* sSpeed.sSpeedFilter.sSpeedRampParams definitions */  
    FMSTR_TSA_RW_VAR(g_sM4Drive.sSpeed.sSpeedPiParams.fltIGain,   FMSTR_TSA_FLOAT)      /* M4 Speed Loop Ki Gain */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sSpeed.sSpeedPiParams.fltPGain,   FMSTR_TSA_FLOAT)      /* M4 Speed Loop Kp Gain */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sSpeed.sSpeedPiParams.fltUpperLim,   FMSTR_TSA_FLOAT)   /* M4 Speed Loop Limit High */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sSpeed.sSpeedPiParams.fltLowerLim,   FMSTR_TSA_FLOAT)   /* M4 Speed Loop Limit Low */
      
    /* gsM4Drive.sPosition structure definition */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sPosition.f16PositionPGain,   FMSTR_TSA_FRAC16)         /* M4 Position P conroller P Gain */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sPosition.a32Position,   FMSTR_TSA_FRAC32)              /* M4 Position Actual */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sPosition.a32PositionError,   FMSTR_TSA_FRAC32)         /* M4 Position Error */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sPosition.a32PositionCmd,   FMSTR_TSA_FRAC32)           /* M4 Position Required */ 
    FMSTR_TSA_RW_VAR(g_sM4Drive.sPosition.a32PositionIndexOffset,   FMSTR_TSA_FRAC32)   /* M4 Position Index Offset */ 
    FMSTR_TSA_RW_VAR(g_sM4Drive.sPosition.fltSpeedConvScale,  FMSTR_TSA_FRAC32)         /* M4 Position Align Offset */
      
    /* sSpeed.sAlignment definitions */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sAlignment.ui16Time,   FMSTR_TSA_UINT16)    /* M4 Alignment Duration */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sAlignment.fltUdReq,   FMSTR_TSA_FLOAT)     /* M4 Alignment Voltage */

    /* gsM4Drive.sFocPMSM structure definition */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sFocPMSM.fltDutyCycleLimit,  FMSTR_TSA_FLOAT)   /* M4 Current Loop Limit */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sFocPMSM.fltUDcBus,   FMSTR_TSA_FLOAT)          /* M4 DCB Voltage */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sFocPMSM.fltUDcBusFilt,   FMSTR_TSA_FLOAT)      /* M4 DCB Voltage Filtered */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sFocPMSM.f16PosElExt,   FMSTR_TSA_UINT16)       /* M4 Posirtion External */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sFocPMSM.f16PosEl,   FMSTR_TSA_UINT16)          /* M4 Position Electrical */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sFocPMSM.f16PosElEst,   FMSTR_TSA_UINT16)       /* M4 Position Estimated */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sFocPMSM.ui16SectorSVM,   FMSTR_TSA_UINT16)     /* M4 SVM Sector */

    /* sFocPMSM.sIAlBe definitions */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sFocPMSM.sIAlBe.fltAlpha,   FMSTR_TSA_FLOAT)/* M4 I alpha */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sFocPMSM.sIAlBe.fltBeta,   FMSTR_TSA_FLOAT) /* M4 I beta */

    /* sFocPMSM.sIDQ definitions */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sFocPMSM.sIDQ.fltD,   FMSTR_TSA_FLOAT)      /* M4 Id */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sFocPMSM.sIDQ.fltQ,   FMSTR_TSA_FLOAT)      /* M4 Iq */

    /* sFocPMSM.sIDQReq definitions */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sFocPMSM.sIDQReq.fltD,   FMSTR_TSA_FLOAT)   /* M4 Id req */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sFocPMSM.sIDQReq.fltQ,   FMSTR_TSA_FLOAT)   /* M4 Iq req */

    /* sFocPMSM.sIDQReq definitions */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sFocPMSM.sUDQReq.fltD,   FMSTR_TSA_FLOAT)   /* M4 Ud req */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sFocPMSM.sUDQReq.fltQ,   FMSTR_TSA_FLOAT)   /* M4 Uq req */

    /* sFocPMSM.sIdPiParams definitions */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sFocPMSM.sIdPiParams.fltIGain,   FMSTR_TSA_FLOAT)        /* M4 Id Ki Gain */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sFocPMSM.sIdPiParams.fltPGain,   FMSTR_TSA_FLOAT)        /* M4 Id Kp Gain */

    /* sFocPMSM.sBemfObsrv definitions */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sFocPMSM.sBemfObsrv.fltEGain,   FMSTR_TSA_FLOAT)         /* M4 Obsrv E gain */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sFocPMSM.sBemfObsrv.fltIGain,   FMSTR_TSA_FLOAT)         /* M4 Obsrv I gain */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sFocPMSM.sBemfObsrv.sCtrl.fltIGain,   FMSTR_TSA_FLOAT)   /* M4 Obsrv Ki gain */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sFocPMSM.sBemfObsrv.sCtrl.fltPGain,   FMSTR_TSA_FLOAT)   /* M4 Obsrv Kp gain */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sFocPMSM.sBemfObsrv.fltUGain,   FMSTR_TSA_FLOAT)         /* M4 Obsrv U gain */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sFocPMSM.sBemfObsrv.fltWIGain,   FMSTR_TSA_FLOAT)        /* M4 Obsrv WI gain */

    /* sFocPMSM.sTo definitions */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sFocPMSM.sTo.fltIGain,   FMSTR_TSA_FLOAT)           /* M4 Obsrv To Ki gain */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sFocPMSM.sTo.fltPGain,   FMSTR_TSA_FLOAT)           /* M4 Obsrv To Kp gain */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sFocPMSM.sTo.fltThGain ,   FMSTR_TSA_FLOAT)         /* M4 Obsrv To Theta gain */

    /* sFocPMSM.sIqPiParams definitions */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sFocPMSM.sIqPiParams.fltIGain,   FMSTR_TSA_FLOAT)   /* M4 Iq Ki Gain */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sFocPMSM.sIqPiParams.fltPGain,   FMSTR_TSA_FLOAT)   /* M4 Iq Kp Gain */

    /* sFocPMSM.sIABC definitions */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sFocPMSM.sIABC.fltA,   FMSTR_TSA_FLOAT)        /* M4 Phase Current A */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sFocPMSM.sIABC.fltB,   FMSTR_TSA_FLOAT)        /* M4 Phase Current B */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sFocPMSM.sIABC.fltC,   FMSTR_TSA_FLOAT)        /* M4 Phase Current C */
      
    /* sFocPMSM.fltSpeedElEst definition */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sFocPMSM.fltSpeedElEst,   FMSTR_TSA_FLOAT)     /* M4 Speed Estimated */

    /* sFaultThresholds definitions */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sFaultThresholds.fltUqBemf,   FMSTR_TSA_FLOAT)         /* M4 Fault Threshold BemfBlocked */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sFaultThresholds.fltUDcBusOver,   FMSTR_TSA_FLOAT)     /* M4 Fault Threshold DcBusOver */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sFaultThresholds.fltUDcBusTrip,   FMSTR_TSA_FLOAT)     /* M4 Fault Threshold DcBusTrip */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sFaultThresholds.fltUDcBusUnder,   FMSTR_TSA_FLOAT)    /* M4 Fault Threshold DcBusUnder */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sFaultThresholds.fltSpeedMin,   FMSTR_TSA_FLOAT)       /* M4 Fault Threshold SpeedMin */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sFaultThresholds.fltSpeedNom,   FMSTR_TSA_FLOAT)       /* M4 Fault Threshold SpeedNom */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sFaultThresholds.fltSpeedOver,   FMSTR_TSA_FLOAT)      /* M4 Fault Threshold SpeedOver */

    /* sStartUp definitions */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sStartUp.f16CoeffMerging,   FMSTR_TSA_FRAC16)       /* M4 Merging Coefficient */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sStartUp.f16RatioMerging,   FMSTR_TSA_FRAC16)       /* M4 Merging Ratio */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sStartUp.fltSpeedCatchUp,   FMSTR_TSA_FLOAT)        /* M4 Merging Speed Catch Up */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sStartUp.f16PosGen,   FMSTR_TSA_FRAC16)             /* M4 Position Open Loop */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sStartUp.fltSpeedCatchUp,   FMSTR_TSA_FLOAT)        /* M4 Speed Merging Catch Up  */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sStartUp.fltSpeedRampOpenLoop,   FMSTR_TSA_FLOAT)   /* M4 Speed Ramp Open Loop  */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sStartUp.fltCurrentStartup,   FMSTR_TSA_FLOAT)      /* M4 Startup Current  */

    /* sStartUp.sSpeedRampOpenLoopParams definitions */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sStartUp.sSpeedRampOpenLoopParams.fltRampDown,   FMSTR_TSA_FLOAT)      /* M4 Startup Ramp Dec */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sStartUp.sSpeedRampOpenLoopParams.fltRampUp,   FMSTR_TSA_FLOAT)        /* M4 Startup Ramp Inc */

    /* sScalarCtrl definitions */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sScalarCtrl.f16PosElScalar,   FMSTR_TSA_FRAC16) /* M4 Position Electrical Scalar */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sScalarCtrl.fltFreqRamp,   FMSTR_TSA_FLOAT)     /* M4 Scalar Frequency Ramp */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sScalarCtrl.fltFreqCmd,   FMSTR_TSA_FLOAT)      /* M4 Scalar speed */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sScalarCtrl.fltVHzGain,   FMSTR_TSA_FLOAT)      /* M4 VHz Factor Gain */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sScalarCtrl.fltFreqMax,   FMSTR_TSA_FLOAT)      /* M4 FMSTR_M4_frequencyScale */      

    /* sScalarCtrl.sFreqRampParams definitions */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sScalarCtrl.sFreqRampParams.fltRampDown,   FMSTR_TSA_FLOAT)    /* M4 Scalar Ramp Down */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sScalarCtrl.sFreqRampParams.fltRampUp,   FMSTR_TSA_FLOAT)      /* M4 Scalar Ramp Up */

    /* sMCATctrl definitions */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sMCATctrl.ui16PospeSensor,   FMSTR_TSA_UINT16)      /* M4 MCAT POSPE Sensor */

    /* sMCATctrl.sIDQReqMCAT definitions */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sMCATctrl.sIDQReqMCAT.fltD,   FMSTR_TSA_FLOAT)     /* M4 MCAT Id Required */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sMCATctrl.sIDQReqMCAT.fltQ,   FMSTR_TSA_FLOAT)     /* M4 MCAT Iq Required */

    /* sMCATctrl.sUDQReqMCAT definitions */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sMCATctrl.sUDQReqMCAT.fltD,   FMSTR_TSA_FLOAT)     /* M4 MCAT Ud Required */
    FMSTR_TSA_RW_VAR(g_sM4Drive.sMCATctrl.sUDQReqMCAT.fltQ,   FMSTR_TSA_FLOAT)     /* M4 MCAT Uq Required */

FMSTR_TSA_TABLE_END()




/*!
 * @brief MID table structure
 *
 * @param None
 *
 * @return None
 */

FMSTR_TSA_TABLE_BEGIN(sMID_table)

    /* sMIDAlignment structure definition */
    FMSTR_TSA_RW_VAR(g_sMID.sMIDAlignment.ui16Active,   FMSTR_TSA_UINT16)          /* MID Align Active */
    FMSTR_TSA_RW_VAR(g_sMID.sMIDAlignment.ui16AlignDuration,   FMSTR_TSA_UINT16)   /* MID Align AlignDuration */
    FMSTR_TSA_RW_VAR(g_sMID.sMIDAlignment.fltCurrentAlign,   FMSTR_TSA_FLOAT)      /* MID Align CurrentAlign */
 
    /* sMIDKe structure definition */
    FMSTR_TSA_RW_VAR(g_sMID.sMIDKe.ui16Active,   FMSTR_TSA_UINT16)          /* MID Ke Active */
    FMSTR_TSA_RW_VAR(g_sMID.sMIDKe.fltIdReqOpenLoop,   FMSTR_TSA_FLOAT)     /* MID Ke IdReqOpenLoop */
    FMSTR_TSA_RW_VAR(g_sMID.sMIDKe.fltKe,   FMSTR_TSA_FLOAT)                /* MID Ke Ke */
    FMSTR_TSA_RW_VAR(g_sMID.sMIDKe.ui16MCATObsrvDone,   FMSTR_TSA_UINT16)   /* MID Ke MCATObsrvDone */
    FMSTR_TSA_RW_VAR(g_sMID.sMIDKe.fltFreqElReq,   FMSTR_TSA_FLOAT)         /* MID Ke SpeedElReq */

    /* sMIDLs structure definition */
    FMSTR_TSA_RW_VAR(g_sMID.sMIDLs.ui16Active,   FMSTR_TSA_UINT16)          /* MID Ke Active */
    FMSTR_TSA_RW_VAR(g_sMID.sMIDLs.fltFreqMax,   FMSTR_TSA_FLOAT)           /* MID Ls FreqMax */
    FMSTR_TSA_RW_VAR(g_sMID.sMIDLs.fltFreqDecrement,   FMSTR_TSA_FLOAT)     /* MID Ls FreqDecrement */
    FMSTR_TSA_RW_VAR(g_sMID.sMIDLs.fltFreqMin,   FMSTR_TSA_FLOAT)           /* MID Ls FreqMin */
    FMSTR_TSA_RW_VAR(g_sMID.sMIDLs.fltFreqStart,   FMSTR_TSA_FLOAT)         /* MID Ls FreqStart */
    FMSTR_TSA_RW_VAR(g_sMID.sMIDLs.fltIdAmplitudeReq,   FMSTR_TSA_FLOAT)    /* MID Ls IdAmplitudeReq */
    FMSTR_TSA_RW_VAR(g_sMID.sMIDLs.fltLd,   FMSTR_TSA_FLOAT)                /* MID Ls Ld */
    FMSTR_TSA_RW_VAR(g_sMID.sMIDLs.fltLq,   FMSTR_TSA_FLOAT)                /* MID Ls Lq */
    FMSTR_TSA_RW_VAR(g_sMID.sMIDLs.fltLs,   FMSTR_TSA_FLOAT)                /* MID Ls Ls */
    FMSTR_TSA_RW_VAR(g_sMID.sMIDLs.fltRs,   FMSTR_TSA_FLOAT)                /* MID Ls Rs */
    FMSTR_TSA_RW_VAR(g_sMID.sMIDLs.fltUdIncrement,   FMSTR_TSA_FLOAT)       /* MID Ls UdIncrement */

    /* sMIDPp structure definition */
    FMSTR_TSA_RW_VAR(g_sMID.sMIDPp.ui16Active,   FMSTR_TSA_UINT16)          /* MID Pp Active */
    FMSTR_TSA_RW_VAR(g_sMID.sMIDPp.fltIdReqOpenLoop,   FMSTR_TSA_FLOAT)     /* MID Pp IdReqOpenLoop */
    FMSTR_TSA_RW_VAR(g_sMID.sMIDPp.ui16PpDetermined,   FMSTR_TSA_UINT16)    /* MID Pp PpDetermined */
    FMSTR_TSA_RW_VAR(g_sMID.sMIDPp.fltFreqElReq,   FMSTR_TSA_FLOAT)         /* MID Pp FreqElReq */
    FMSTR_TSA_RW_VAR(g_sMID.sMIDPp.fltFreqMax,   FMSTR_TSA_FLOAT)           /* MID Pp FreqMax */

    /* sMIDRs structure definition */
    FMSTR_TSA_RW_VAR(g_sMID.sMIDRs.ui16Active,   FMSTR_TSA_UINT16)  /* MID Rs Active */
    FMSTR_TSA_RW_VAR(g_sMID.sMIDRs.fltIdMeas,   FMSTR_TSA_FLOAT)    /* MID Rs IdMeas */
    FMSTR_TSA_RW_VAR(g_sMID.sMIDRs.fltRs,   FMSTR_TSA_FLOAT)        /* MID Rs Rs */

    /* sMIDKe structure definition */
    FMSTR_TSA_RW_VAR(g_sMID.sMIDPwrStgChar.ui16Active,   FMSTR_TSA_UINT16)      /* MID PwrStg Active */
    FMSTR_TSA_RW_VAR(g_sMID.sMIDPwrStgChar.fltIdCalib,   FMSTR_TSA_FLOAT)       /* MID PwrStg IdCalib */
    FMSTR_TSA_RW_VAR(g_sMID.sMIDPwrStgChar.fltIdIncrement,   FMSTR_TSA_FLOAT)   /* MID PwrStg IdIncrement */
    FMSTR_TSA_RW_VAR(g_sMID.sMIDPwrStgChar.fltRs,   FMSTR_TSA_FLOAT)            /* MID PwrStg Rs */
    FMSTR_TSA_RW_VAR(g_sMID.sMIDPwrStgChar.ui16NumOfChPnts,   FMSTR_TSA_UINT16) /* MID PwrStg Num of Points */
      
    /* Power Stage characteristic data */  
    FMSTR_TSA_RW_VAR(g_sM1Drive.sFocPMSM.fltPwrStgCharLinCoeff,   FMSTR_TSA_FLOAT)      /* MID PwrStg Char Linear Coefficient */  
    FMSTR_TSA_RW_VAR(g_sM1Drive.sFocPMSM.fltPwrStgCharIRange,   FMSTR_TSA_FLOAT)        /* MID PwrStg Char Range */   

    /* sMIDMech structure definition */
    FMSTR_TSA_RW_VAR(g_sMID.sMIDMech.ui16Active,   FMSTR_TSA_UINT16)      /* MID Mech Active */
    FMSTR_TSA_RW_VAR(g_sMID.sMIDMech.fltB,   FMSTR_TSA_FLOAT)        /* MID Mech B */    
    FMSTR_TSA_RW_VAR(g_sMID.sMIDMech.fltIqAccelerate,   FMSTR_TSA_FLOAT)        /* MID Mech IqAccelerate */  
    FMSTR_TSA_RW_VAR(g_sMID.sMIDMech.fltIqStartup,   FMSTR_TSA_FLOAT)        /* MID Mech IqStartup */  
    FMSTR_TSA_RW_VAR(g_sMID.sMIDMech.fltJ,   FMSTR_TSA_FLOAT)        /* MID Mech J */  
    FMSTR_TSA_RW_VAR(g_sMID.sMIDMech.fltKt,   FMSTR_TSA_FLOAT)        /* MID Mech Kt */  
    FMSTR_TSA_RW_VAR(g_sMID.sMIDMech.fltPp,   FMSTR_TSA_FLOAT)        /* MID Mech Pp */  
    FMSTR_TSA_RW_VAR(g_sMID.sMIDMech.fltSpeedThrsAccel,   FMSTR_TSA_FLOAT)        /* MID Mech SpeedAccelThreshold */  
    FMSTR_TSA_RW_VAR(g_sMID.sMIDMech.fltSpeedThrsDecel,   FMSTR_TSA_FLOAT)        /* MID Mech SpeedDecelThreshold */  
    FMSTR_TSA_RW_VAR(g_sMID.sMIDMech.fltSpeedThrsInteg,   FMSTR_TSA_FLOAT)        /* MID Mech SpeedIntegThreshold */  
    FMSTR_TSA_RW_VAR(g_sMID.sMIDMech.sStartup.sSpeedRampOpenLoopParams.fltRampDown,   FMSTR_TSA_FLOAT)        /* MID Mech StartupRampDown */  
    FMSTR_TSA_RW_VAR(g_sMID.sMIDMech.sStartup.sSpeedRampOpenLoopParams.fltRampUp,   FMSTR_TSA_FLOAT)        /* MID Mech StartupRampUp */  
    FMSTR_TSA_RW_VAR(g_sMID.sMIDMech.ui32TimeMeasMax,   FMSTR_TSA_FLOAT)        /* MID Mech TimeoutCount */  
    FMSTR_TSA_RW_VAR(g_sMID.sMIDMech.fltTauMech,   FMSTR_TSA_FLOAT)        /* MID Mech Tm */  
    
    /* MIDPwrStgChar Char ERROR */
    FMSTR_TSA_RW_MEM(g_sMID.sMIDPwrStgChar.fltUdErrorLookUp, FMSTR_TSA_FLOAT, &g_sMID.sMIDPwrStgChar.fltUdErrorLookUp[0], (65 << 2)) /* MID Ud Error Lookup */
      
    /* dead-time compensation voltage table */
    FMSTR_TSA_RW_MEM(pfltUDtComp, FMSTR_TSA_FLOAT, &pfltUDtComp[0],  (65 << 2)) /* pfltUDtComp[] */
      

FMSTR_TSA_TABLE_END()


/*!
 * @brief g_sM1Enc driver structure
 *
 * @param None
 *
 * @return None
 */
FMSTR_TSA_TABLE_BEGIN(gsM1Enc_table)

    /* gsM1Enc structure definition */
    FMSTR_TSA_RW_VAR(g_sM1Enc.fltSpdMeEst,   FMSTR_TSA_FLOAT)       /* M1 Measured Mechanical Speed */
    FMSTR_TSA_RW_VAR(g_sM1Enc.f16PosMe,   FMSTR_TSA_FRAC16)         /* M1 Meassured Mechanical Position */
    FMSTR_TSA_RW_VAR(g_sM1Enc.f16PosMeEst,   FMSTR_TSA_FRAC16)      /* M1 Position Encoder Mechanical */ 
    FMSTR_TSA_RW_VAR(g_sM1Enc.bDirection,   FMSTR_TSA_UINT16)       /* M1 Encoder direction */  
    FMSTR_TSA_RW_VAR(g_sM1Enc.sTo.fltThGain,   FMSTR_TSA_FLOAT)     /* M1 POSPE Integ Gain */
    FMSTR_TSA_RW_VAR(g_sM1Enc.sTo.fltIGain,   FMSTR_TSA_FLOAT)      /* M1 POSPE Ki Gain */
    FMSTR_TSA_RW_VAR(g_sM1Enc.sTo.fltPGain,   FMSTR_TSA_FLOAT)      /* M1 POSPE Kp Gain */
            
FMSTR_TSA_TABLE_END()


/*!
 * @brief g_sM2Enc driver structure
 *
 * @param None
 *
 * @return None
 */
FMSTR_TSA_TABLE_BEGIN(gsM2Enc_table)

    /* gsM1Enc structure definition */
    FMSTR_TSA_RW_VAR(g_sM2Enc.fltSpdMeEst,   FMSTR_TSA_FLOAT)       /* M2 Measured Mechanical Speed */
    FMSTR_TSA_RW_VAR(g_sM2Enc.f16PosMe,   FMSTR_TSA_FRAC16)         /* M2 Meassured Mechanical Position */
    FMSTR_TSA_RW_VAR(g_sM2Enc.f16PosMeEst,   FMSTR_TSA_FRAC16)      /* M2 Position Encoder Mechanical */ 
    FMSTR_TSA_RW_VAR(g_sM2Enc.bDirection,   FMSTR_TSA_UINT16)       /* M2 Encoder direction */  
    FMSTR_TSA_RW_VAR(g_sM2Enc.sTo.fltThGain,   FMSTR_TSA_FLOAT)     /* M2 POSPE Integ Gain */
    FMSTR_TSA_RW_VAR(g_sM2Enc.sTo.fltIGain,   FMSTR_TSA_FLOAT)      /* M2 POSPE Ki Gain */
    FMSTR_TSA_RW_VAR(g_sM2Enc.sTo.fltPGain,   FMSTR_TSA_FLOAT)      /* M2 POSPE Kp Gain */    
         
FMSTR_TSA_TABLE_END()


/*!
 * @brief g_sM1Enc driver structure
 *
 * @param None
 *
 * @return None
 */
FMSTR_TSA_TABLE_BEGIN(gsM3Enc_table)

    /* gsM1Enc structure definition */
    FMSTR_TSA_RW_VAR(g_sM3Enc.fltSpdMeEst,   FMSTR_TSA_FLOAT)       /* M3 Measured Mechanical Speed */
    FMSTR_TSA_RW_VAR(g_sM3Enc.f16PosMe,   FMSTR_TSA_FRAC16)         /* M3 Meassured Mechanical Position */
    FMSTR_TSA_RW_VAR(g_sM3Enc.f16PosMeEst,   FMSTR_TSA_FRAC16)      /* M3 Position Encoder Mechanical */ 
    FMSTR_TSA_RW_VAR(g_sM3Enc.bDirection,   FMSTR_TSA_UINT16)       /* M3 Encoder direction */  
    FMSTR_TSA_RW_VAR(g_sM3Enc.sTo.fltThGain,   FMSTR_TSA_FLOAT)     /* M3 POSPE Integ Gain */
    FMSTR_TSA_RW_VAR(g_sM3Enc.sTo.fltIGain,   FMSTR_TSA_FLOAT)      /* M3 POSPE Ki Gain */
    FMSTR_TSA_RW_VAR(g_sM3Enc.sTo.fltPGain,   FMSTR_TSA_FLOAT)      /* M3 POSPE Kp Gain */

FMSTR_TSA_TABLE_END()


/*!
 * @brief g_sM2Enc driver structure
 *
 * @param None
 *
 * @return None
 */
FMSTR_TSA_TABLE_BEGIN(gsM4Enc_table)

    /* gsM1Enc structure definition */
    FMSTR_TSA_RW_VAR(g_sM4Enc.fltSpdMeEst,   FMSTR_TSA_FLOAT)       /* M4 Measured Mechanical Speed */
    FMSTR_TSA_RW_VAR(g_sM4Enc.f16PosMe,   FMSTR_TSA_FRAC16)         /* M4 Meassured Mechanical Position */
    FMSTR_TSA_RW_VAR(g_sM4Enc.f16PosMeEst,   FMSTR_TSA_FRAC16)      /* M4 Position Encoder Mechanical */ 
    FMSTR_TSA_RW_VAR(g_sM4Enc.bDirection,   FMSTR_TSA_UINT16)       /* M4 Encoder direction */  
    FMSTR_TSA_RW_VAR(g_sM4Enc.sTo.fltThGain,   FMSTR_TSA_FLOAT)     /* M4 POSPE Integ Gain */
    FMSTR_TSA_RW_VAR(g_sM4Enc.sTo.fltIGain,   FMSTR_TSA_FLOAT)      /* M4 POSPE Ki Gain */
    FMSTR_TSA_RW_VAR(g_sM4Enc.sTo.fltPGain,   FMSTR_TSA_FLOAT)      /* M4 POSPE Kp Gain */

FMSTR_TSA_TABLE_END()


/*!
 * @brief Global table with global variables used in TSA
 *
 * @param None
 *
 * @return None
 */
FMSTR_TSA_TABLE_BEGIN(global_table)

    /* global variables & control */
    FMSTR_TSA_RW_VAR(g_ui32NumberOfCycles, FMSTR_TSA_UINT32)        /* Cycle Number */
    FMSTR_TSA_RW_VAR(g_ui32MaxNumberOfCycles, FMSTR_TSA_UINT32)     /* Cycle Number Maximum */
    FMSTR_TSA_RW_VAR(bDemoModeSpeed, FMSTR_TSA_UINT16)              /* Demo Mode Speed */
    FMSTR_TSA_RW_VAR(bM1SpeedDemo, FMSTR_TSA_UINT16)
    FMSTR_TSA_RW_VAR(bM2SpeedDemo, FMSTR_TSA_UINT16)
    FMSTR_TSA_RW_VAR(bM3SpeedDemo, FMSTR_TSA_UINT16)
    FMSTR_TSA_RW_VAR(bM4SpeedDemo, FMSTR_TSA_UINT16)
    FMSTR_TSA_RW_VAR(bDemoModePosition, FMSTR_TSA_UINT16)           /* Demo Mode Position */ 
    FMSTR_TSA_RW_VAR(bM1PositionDemo, FMSTR_TSA_UINT16)
    FMSTR_TSA_RW_VAR(bM2PositionDemo, FMSTR_TSA_UINT16)
    FMSTR_TSA_RW_VAR(bM3PositionDemo, FMSTR_TSA_UINT16)
    FMSTR_TSA_RW_VAR(bM4PositionDemo, FMSTR_TSA_UINT16)
    FMSTR_TSA_RW_VAR(g_ui16DemoLedType, FMSTR_TSA_UINT16)            /* Cycle Number */ 
      
FMSTR_TSA_TABLE_END()

/*!
 * @brief M1 Global table with global variables used in TSA
 *
 * @param None
 *
 * @return None
 */
FMSTR_TSA_TABLE_BEGIN(M1_global_table)

    /* global variables & control */ 
    FMSTR_TSA_RW_VAR(g_bM1SwitchAppOnOff, FMSTR_TSA_UINT16)         /* M1 Application Switch */
    FMSTR_TSA_RW_VAR(g_sM1Ctrl.eState, FMSTR_TSA_UINT16)            /* M1 Application State */

    /* global freemaster float variables */
    FMSTR_TSA_RW_VAR(g_fltM1currentScale, FMSTR_TSA_FLOAT)          /* FMSTR_M1_currentScale */
    FMSTR_TSA_RW_VAR(g_fltM1DCBvoltageScale, FMSTR_TSA_FLOAT)       /* FMSTR_M1_DCBvoltageScale */
    FMSTR_TSA_RW_VAR(g_fltM1speedScale, FMSTR_TSA_FLOAT)            /* FMSTR_M1_speedScale */
    FMSTR_TSA_RW_VAR(g_fltM1voltageScale, FMSTR_TSA_FLOAT)          /* FMSTR_M1_voltageScale */
    FMSTR_TSA_RW_VAR(g_fltM1speedAngularScale, FMSTR_TSA_FLOAT)     /* FMSTR_M1_speedAngularScale */ 
      
    FMSTR_TSA_RW_VAR(g_eM1StateRun, FMSTR_TSA_UINT16)               /* M1 State Run */

    /* global MCAT variables */
    FMSTR_TSA_RW_VAR(g_sMID.eMeasurementType, FMSTR_TSA_UINT16)         /* MID Calibration */
    FMSTR_TSA_RW_VAR(g_sMID.ui16EnableMeasurement, FMSTR_TSA_UINT16)    /* MID EnableMeasurement */
    FMSTR_TSA_RW_VAR(g_sMID.ui16FaultMID, FMSTR_TSA_UINT16)             /* MID FaultMID */
    FMSTR_TSA_RW_VAR(g_sMID.ui16WarnMID, FMSTR_TSA_UINT16)              /* MID WarnMID */

    /* MID variables */
    FMSTR_TSA_RW_VAR(g_sMIDCtrl.eState, FMSTR_TSA_UINT16)           /* MID State */

FMSTR_TSA_TABLE_END()

/*!
 * @brief M2 Global table with global variables used in TSA
 *
 * @param None
 *
 * @return None
 */
FMSTR_TSA_TABLE_BEGIN(M2_global_table)

    /* global variables & control */
    FMSTR_TSA_RW_VAR(g_bM2SwitchAppOnOff, FMSTR_TSA_UINT16)         /* M2 Application Switch */
    FMSTR_TSA_RW_VAR(g_sM2Ctrl.eState, FMSTR_TSA_UINT16)            /* M2 Application State */

    /* global freemaster float variables */
    FMSTR_TSA_RW_VAR(g_fltM2currentScale, FMSTR_TSA_FLOAT)          /* FMSTR_M2_currentScale */
    FMSTR_TSA_RW_VAR(g_fltM2DCBvoltageScale, FMSTR_TSA_FLOAT)       /* FMSTR_M2_DCBvoltageScale */
    FMSTR_TSA_RW_VAR(g_fltM2speedScale, FMSTR_TSA_FLOAT)            /* FMSTR_M2_speedScale */
    FMSTR_TSA_RW_VAR(g_fltM2voltageScale, FMSTR_TSA_FLOAT)          /* FMSTR_M2_voltageScale */
    FMSTR_TSA_RW_VAR(g_fltM2speedAngularScale, FMSTR_TSA_FLOAT)     /* FMSTR_M2_speedAngularScale */  
      
    FMSTR_TSA_RW_VAR(g_eM2StateRun, FMSTR_TSA_UINT16)               /* M2 State Run */

FMSTR_TSA_TABLE_END()


/*!
 * @brief M3 Global table with global variables used in TSA
 *
 * @param None
 *
 * @return None
 */
FMSTR_TSA_TABLE_BEGIN(M3_global_table)

    /* global variables & control */
    FMSTR_TSA_RW_VAR(g_bM3SwitchAppOnOff, FMSTR_TSA_UINT16)         /* M3 Application Switch */
    FMSTR_TSA_RW_VAR(g_sM3Ctrl.eState, FMSTR_TSA_UINT16)            /* M3 Application State */

    /* global freemaster float variables */
    FMSTR_TSA_RW_VAR(g_fltM3currentScale, FMSTR_TSA_FLOAT)          /* FMSTR_M3_currentScale */
    FMSTR_TSA_RW_VAR(g_fltM3DCBvoltageScale, FMSTR_TSA_FLOAT)       /* FMSTR_M3_DCBvoltageScale */
    FMSTR_TSA_RW_VAR(g_fltM3speedScale, FMSTR_TSA_FLOAT)            /* FMSTR_M3_speedScale */
    FMSTR_TSA_RW_VAR(g_fltM3voltageScale, FMSTR_TSA_FLOAT)          /* FMSTR_M3_voltageScale */
    FMSTR_TSA_RW_VAR(g_fltM3speedAngularScale, FMSTR_TSA_FLOAT)     /* FMSTR_M3_speedAngularScale */  
      
    FMSTR_TSA_RW_VAR(g_eM3StateRun, FMSTR_TSA_UINT16)               /* M3 State Run */

FMSTR_TSA_TABLE_END()


/*!
 * @brief M4 Global table with global variables used in TSA
 *
 * @param None
 *
 * @return None
 */
FMSTR_TSA_TABLE_BEGIN(M4_global_table)

    /* global variables & control */
    FMSTR_TSA_RW_VAR(g_bM4SwitchAppOnOff, FMSTR_TSA_UINT16)         /* M4 Application Switch */
    FMSTR_TSA_RW_VAR(g_sM4Ctrl.eState, FMSTR_TSA_UINT16)            /* M4 Application State */

    /* global freemaster float variables */
    FMSTR_TSA_RW_VAR(g_fltM4currentScale, FMSTR_TSA_FLOAT)          /* FMSTR_M4_currentScale */
    FMSTR_TSA_RW_VAR(g_fltM4DCBvoltageScale, FMSTR_TSA_FLOAT)       /* FMSTR_M4_DCBvoltageScale */
    FMSTR_TSA_RW_VAR(g_fltM4speedScale, FMSTR_TSA_FLOAT)            /* FMSTR_M4_speedScale */
    FMSTR_TSA_RW_VAR(g_fltM4voltageScale, FMSTR_TSA_FLOAT)          /* FMSTR_M4_voltageScale */
    FMSTR_TSA_RW_VAR(g_fltM4speedAngularScale, FMSTR_TSA_FLOAT)     /* FMSTR_M4_speedAngularScale */  
      
    FMSTR_TSA_RW_VAR(g_eM4StateRun, FMSTR_TSA_UINT16)               /* M4 State Run */

FMSTR_TSA_TABLE_END()


/*!
 * @brief Structure used in FM to get required ID's
 *
 * @param None
 *
 * @return None
 */
FMSTR_TSA_TABLE_BEGIN(sAppIdFM_table)

    /* Board ID structure definition */
    FMSTR_TSA_RO_MEM(g_sAppIdFM.cBoardID, FMSTR_TSA_UINT8, &g_sAppIdFM.cBoardID[0], 15)
    FMSTR_TSA_RO_MEM(g_sAppIdFM.cMotorType, FMSTR_TSA_UINT8, &g_sAppIdFM.cMotorType[0], 4)
    FMSTR_TSA_RO_MEM(g_sAppIdFM.cAppVer, FMSTR_TSA_UINT8, &g_sAppIdFM.cAppVer[0], 5)

FMSTR_TSA_TABLE_END()

/*!
 * @brief TSA Table list required if TSA macro is enabled
 *
 * @param None
 *
 * @return None
 */
FMSTR_TSA_TABLE_LIST_BEGIN()
    FMSTR_TSA_TABLE(sAppIdFM_table)
    FMSTR_TSA_TABLE(global_table)
      
    FMSTR_TSA_TABLE(M1_global_table)
    FMSTR_TSA_TABLE(M2_global_table)
    FMSTR_TSA_TABLE(M3_global_table)
    FMSTR_TSA_TABLE(M4_global_table)  
   
    FMSTR_TSA_TABLE(gsM1Drive_table)
    FMSTR_TSA_TABLE(gsM1Enc_table)
      
    FMSTR_TSA_TABLE(sMID_table)
      
    FMSTR_TSA_TABLE(gsM2Drive_table)
    FMSTR_TSA_TABLE(gsM2Enc_table)
      
    FMSTR_TSA_TABLE(gsM3Drive_table)
    FMSTR_TSA_TABLE(gsM3Enc_table)
      
    FMSTR_TSA_TABLE(gsM4Drive_table)
    FMSTR_TSA_TABLE(gsM4Enc_table)
      
FMSTR_TSA_TABLE_LIST_END()

