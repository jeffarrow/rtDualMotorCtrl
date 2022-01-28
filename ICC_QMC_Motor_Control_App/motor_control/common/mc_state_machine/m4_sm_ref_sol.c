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

#include "m4_sm_ref_sol.h"
#include "mcdrv.h"
#include "mid_sm_states.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define M4_SVM_SECTOR_DEFAULT (2) /* default SVM sector */
#define M4_BLOCK_ROT_FAULT_SH (5) /* filter window */

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

RAM_FUNC_CRITICAL 
static void M4_StateFaultFast(void);
RAM_FUNC_CRITICAL
static void M4_StateInitFast(void);
RAM_FUNC_CRITICAL
static void M4_StateStopFast(void);
RAM_FUNC_CRITICAL 
static void M4_StateRunFast(void);

RAM_FUNC 
static void M4_StateFaultSlow(void);
RAM_FUNC 
static void M4_StateInitSlow(void);
RAM_FUNC 
static void M4_StateStopSlow(void);
RAM_FUNC 
static void M4_StateRunSlow(void);

RAM_FUNC_CRITICAL  
static void M4_TransFaultStop(void);
RAM_FUNC_CRITICAL  
static void M4_TransInitFault(void);
RAM_FUNC_CRITICAL  
static void M4_TransInitStop(void);
RAM_FUNC_CRITICAL  
static void M4_TransStopFault(void);
RAM_FUNC_CRITICAL  
static void M4_TransStopRun(void);
RAM_FUNC_CRITICAL  
static void M4_TransRunFault(void);
RAM_FUNC_CRITICAL  
static void M4_TransRunStop(void);

RAM_FUNC_CRITICAL   
static void M4_StateRunCalibFast(void);
RAM_FUNC_CRITICAL   
static void M4_StateRunMeasureFast(void);
RAM_FUNC_CRITICAL   
static void M4_StateRunReadyFast(void);
RAM_FUNC_CRITICAL   
static void M4_StateRunAlignFast(void);
RAM_FUNC_CRITICAL   
static void M4_StateRunStartupFast(void);
RAM_FUNC_CRITICAL   
static void M4_StateRunSpinFast(void);
RAM_FUNC_CRITICAL   
static void M4_StateRunFreewheelFast(void);

RAM_FUNC 
static void M4_StateRunCalibSlow(void);
RAM_FUNC 
static void M4_StateRunMeasureSlow(void);
RAM_FUNC 
static void M4_StateRunReadySlow(void);
RAM_FUNC 
static void M4_StateRunAlignSlow(void);
RAM_FUNC 
static void M4_StateRunStartupSlow(void);
RAM_FUNC 
static void M4_StateRunSpinSlow(void);
RAM_FUNC 
static void M4_StateRunFreewheelSlow(void);

RAM_FUNC
static void M4_TransRunCalibReady(void);
RAM_FUNC 
static void M4_TransRunCalibMeasure(void);
RAM_FUNC_CRITICAL 
static void M4_TransRunMeasureReady(void);
RAM_FUNC_CRITICAL 
static void M4_TransRunReadyAlign(void);
RAM_FUNC_CRITICAL
static void M4_TransRunAlignStartup(void);
RAM_FUNC_CRITICAL 
static void M4_TransRunAlignReady(void);
RAM_FUNC_CRITICAL
static void M4_TransRunAlignSpin(void);
RAM_FUNC_CRITICAL
static void M4_TransRunStartupSpin(void);
RAM_FUNC_CRITICAL 
static void M4_TransRunStartupFreewheel(void);
RAM_FUNC_CRITICAL 
static void M4_TransRunSpinFreewheel(void);
RAM_FUNC
static void M4_TransRunFreewheelReady(void);

RAM_FUNC_CRITICAL 
static void M4_ClearFOCVariables(void);

RAM_FUNC_CRITICAL
static void M4_FaultDetection(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! @brief Main control structure */
mcdef_pmsm_t g_sM4Drive;

/*! @brief Main application switch */
bool_t g_bM4SwitchAppOnOff;

/*! @brief M4 structure */
run_substate_t g_eM4StateRun;

/*! @brief FreeMASTER scales */
/*! DO NOT USE THEM in the code to avoid float library include */
volatile float g_fltM4voltageScale;
volatile float g_fltM4DCBvoltageScale;
volatile float g_fltM4currentScale;
volatile float g_fltM4speedScale;
volatile float g_fltM4speedAngularScale;
volatile float g_fltM4speedMechanicalScale;

/*! @brief Application state machine table - fast */
const sm_app_state_fcn_t s_M4_STATE_FAST = {M4_StateFaultFast, M4_StateInitFast, M4_StateStopFast, M4_StateRunFast};

/*! @brief Application state machine table - slow */
const sm_app_state_fcn_t s_M4_STATE_SLOW = {M4_StateFaultSlow, M4_StateInitSlow, M4_StateStopSlow, M4_StateRunSlow};

/*! @brief Application sub-state function field - fast */
static const pfcn_void_void s_M4_STATE_RUN_TABLE_FAST[7] = {M4_StateRunCalibFast, M4_StateRunReadyFast,
                                                            M4_StateRunAlignFast, M4_StateRunStartupFast,
                                                            M4_StateRunSpinFast,  M4_StateRunFreewheelFast,
                                                            M4_StateRunMeasureFast};

/*! @brief Application sub-state function field - slow */
static const pfcn_void_void s_M4_STATE_RUN_TABLE_SLOW[7] = {M4_StateRunCalibSlow, M4_StateRunReadySlow,
                                                            M4_StateRunAlignSlow, M4_StateRunStartupSlow,
                                                            M4_StateRunSpinSlow,  M4_StateRunFreewheelSlow,
                                                            M4_StateRunMeasureSlow};

/*! @brief Application state-transition functions field  */
static const sm_app_trans_fcn_t s_TRANS = {M4_TransFaultStop, M4_TransInitFault, M4_TransInitStop, M4_TransStopFault,
                                           M4_TransStopRun,   M4_TransRunFault,  M4_TransRunStop};

/*! @brief  State machine structure declaration and initialization */
sm_app_ctrl_t g_sM4Ctrl = {
    /* g_sM4Ctrl.psState, User state functions  */
    &s_M4_STATE_FAST,

    /* g_sM4Ctrl.psState, User state functions  */
    &s_M4_STATE_SLOW,

    /* g_sM4Ctrl..psTrans, User state-transition functions */
    &s_TRANS,

    /* g_sM4Ctrl.uiCtrl, Default no control command */
    SM_CTRL_NONE,

    /* g_sM4Ctrl.eState, Default state after reset */
    kSM_AppInit};

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Fault state called in fast state machine
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M4_StateFaultFast(void)
{
    /* read ADC results (ADC triggered by HW trigger from PDB) */
    /* get all adc samples - DC-bus voltage, current, bemf and aux sample */
    M4_MCDRV_ADC_GET(&g_sM4AdcSensor);

    /* convert voltages from fractional measured values to float */
    g_sM4Drive.sFocPMSM.fltUDcBus = 
        MLIB_ConvSc_FLTsf(g_sM4Drive.sFocPMSM.f16UDcBus, g_fltM4DCBvoltageScale);

    /* Sampled DC-Bus voltage filter */
    g_sM4Drive.sFocPMSM.fltUDcBusFilt =
        GDFLIB_FilterIIR1_FLT(g_sM4Drive.sFocPMSM.fltUDcBus, &g_sM4Drive.sFocPMSM.sUDcBusFilter);

    /* Braking resistor control */
    if(g_sM4Drive.sFocPMSM.fltUDcBusFilt > g_sM4Drive.sFaultThresholds.fltUDcBusTrip)
        M4_BRAKE_SET();
    else
        M4_BRAKE_CLEAR();

    /* Disable user application switch */
    g_bM4SwitchAppOnOff = FALSE;

    /* clear fault state manually from FreeMASTER */
    if(g_sM4Drive.bFaultClearMan)
    {
        /* Clear fault state */
        g_sM4Ctrl.uiCtrl |= SM_CTRL_FAULT_CLEAR;
        g_sM4Drive.bFaultClearMan = FALSE;
    }

    /* PWM peripheral update */
    M4_MCDRV_PWM3PH_SET(&g_sM4Pwm3ph);

    /* Detects faults */
    M4_FaultDetection();
}

/*!
 * @brief State initialization routine called in fast state machine
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M4_StateInitFast(void)
{
    /* Type the code to do when in the INIT state */
    g_sM4Drive.sFocPMSM.sIdPiParams.fltPGain = MID_KP_GAIN;
    g_sM4Drive.sFocPMSM.sIdPiParams.fltIGain = MID_KI_GAIN;
    g_sM4Drive.sFocPMSM.sIdPiParams.fltInErrK_1 = 0.0F;
    g_sM4Drive.sFocPMSM.sIdPiParams.bLimFlag = FALSE;

    g_sM4Drive.sFocPMSM.sIqPiParams.fltPGain = MID_KP_GAIN;
    g_sM4Drive.sFocPMSM.sIqPiParams.fltIGain = MID_KI_GAIN;
    g_sM4Drive.sFocPMSM.sIqPiParams.fltInErrK_1 = 0.0F;
    g_sM4Drive.sFocPMSM.sIqPiParams.bLimFlag = FALSE;
  
    /* PMSM FOC params */
    g_sM4Drive.sFocPMSM.sIdPiParams.fltPGain = M4_D_KP_GAIN;
    g_sM4Drive.sFocPMSM.sIdPiParams.fltIGain = M4_D_KI_GAIN;
    g_sM4Drive.sFocPMSM.sIdPiParams.fltUpperLim = M4_U_MAX;
    g_sM4Drive.sFocPMSM.sIdPiParams.fltLowerLim = -M4_U_MAX;

    g_sM4Drive.sFocPMSM.sIqPiParams.fltPGain = M4_Q_KP_GAIN;
    g_sM4Drive.sFocPMSM.sIqPiParams.fltIGain = M4_Q_KI_GAIN;
    g_sM4Drive.sFocPMSM.sIqPiParams.fltUpperLim = M4_U_MAX;
    g_sM4Drive.sFocPMSM.sIqPiParams.fltLowerLim = -M4_U_MAX;

    g_sM4Drive.sFocPMSM.ui16SectorSVM = M4_SVM_SECTOR_DEFAULT;
    g_sM4Drive.sFocPMSM.fltDutyCycleLimit = M4_CLOOP_LIMIT;
    
    /* enable dead-time compensation */
    g_sM4Drive.sFocPMSM.bFlagDTComp = TRUE;

    g_sM4Drive.sFocPMSM.fltUDcBus = 0.0F;
    g_sM4Drive.sFocPMSM.fltUDcBusFilt = 0.0F;
    g_sM4Drive.sFocPMSM.sUDcBusFilter.sFltCoeff.fltB0 = M4_UDCB_IIR_B0;
    g_sM4Drive.sFocPMSM.sUDcBusFilter.sFltCoeff.fltB1 = M4_UDCB_IIR_B1;
    g_sM4Drive.sFocPMSM.sUDcBusFilter.sFltCoeff.fltA1 = M4_UDCB_IIR_A1;
    /* Filter init not to enter to fault */
    g_sM4Drive.sFocPMSM.sUDcBusFilter.fltFltBfrX[0] = 
        (M4_U_DCB_UNDERVOLTAGE / 2.0F) + (M4_U_DCB_OVERVOLTAGE / 2.0F);
    g_sM4Drive.sFocPMSM.sUDcBusFilter.fltFltBfrY[0] =
        (M4_U_DCB_UNDERVOLTAGE / 2.0F) + (M4_U_DCB_OVERVOLTAGE / 2.0F);


    g_sM4Drive.sAlignment.fltUdReq = M4_ALIGN_VOLTAGE;
    g_sM4Drive.sAlignment.ui16Time = M4_ALIGN_DURATION;

    /* Position and speed observer */
    g_sM4Drive.sFocPMSM.sTo.fltPGain = M4_TO_KP_GAIN;
    g_sM4Drive.sFocPMSM.sTo.fltIGain = M4_TO_KI_GAIN;
    g_sM4Drive.sFocPMSM.sTo.fltThGain = M4_TO_THETA_GAIN;

    g_sM4Drive.sFocPMSM.sBemfObsrv.fltIGain = M4_I_SCALE;
    g_sM4Drive.sFocPMSM.sBemfObsrv.fltUGain = M4_U_SCALE;
    g_sM4Drive.sFocPMSM.sBemfObsrv.fltEGain = M4_E_SCALE;
    g_sM4Drive.sFocPMSM.sBemfObsrv.fltWIGain = M4_WI_SCALE;
    g_sM4Drive.sFocPMSM.sBemfObsrv.sCtrl.fltPGain = M4_BEMF_DQ_KP_GAIN;
    g_sM4Drive.sFocPMSM.sBemfObsrv.sCtrl.fltIGain = M4_BEMF_DQ_KI_GAIN;

    g_sM4Drive.sFocPMSM.sSpeedElEstFilt.sFltCoeff.fltB0 = M4_TO_SPEED_IIR_B0;
    g_sM4Drive.sFocPMSM.sSpeedElEstFilt.sFltCoeff.fltB1 = M4_TO_SPEED_IIR_B1;
    g_sM4Drive.sFocPMSM.sSpeedElEstFilt.sFltCoeff.fltA1 = M4_TO_SPEED_IIR_A1;
    GDFLIB_FilterIIR1Init_FLT(&g_sM4Drive.sFocPMSM.sSpeedElEstFilt);
    
    /* Speed params */
    g_sM4Drive.sSpeed.sSpeedPiParams.fltPGain = M4_SPEED_PI_PROP_GAIN;
    g_sM4Drive.sSpeed.sSpeedPiParams.fltIGain = M4_SPEED_PI_INTEG_GAIN;
    g_sM4Drive.sSpeed.sSpeedPiParams.fltUpperLim = M4_SPEED_LOOP_HIGH_LIMIT;
    g_sM4Drive.sSpeed.sSpeedPiParams.fltLowerLim = M4_SPEED_LOOP_LOW_LIMIT;

    g_sM4Drive.sSpeed.sSpeedRampParams.fltRampUp = M4_SPEED_RAMP_UP;
    g_sM4Drive.sSpeed.sSpeedRampParams.fltRampDown = M4_SPEED_RAMP_DOWN;

    g_sM4Drive.sSpeed.sSpeedFilter.sFltCoeff.fltB0 = M4_SPEED_IIR_B0;
    g_sM4Drive.sSpeed.sSpeedFilter.sFltCoeff.fltB1 = M4_SPEED_IIR_B1;
    g_sM4Drive.sSpeed.sSpeedFilter.sFltCoeff.fltA1 = M4_SPEED_IIR_A1;

    g_sM4Drive.sSpeed.fltSpeedCmd = 0.0F;
    
    /* Position params */
    g_sM4Drive.sPosition.f16PositionPGain = M4_POS_P_PROP_GAIN;
    g_sM4Drive.sPosition.a32Position = 0;
    g_sM4Drive.sPosition.a32PositionIndexOffset = 0;
    g_sM4Drive.sPosition.a32PositionError = 0;
    g_sM4Drive.sPosition.a32PositionCmd = 0;
    g_sM4Drive.sPosition.f16SpeedReq = 0;
    g_sM4Drive.sPosition.fltSpeedConvScale = M4_SPEED_CONV_SCALE;
    g_sM4Drive.sPosition.a32PositionAlignOffset = 0;

    /* Scalar control params */
    g_sM4Drive.sScalarCtrl.fltVHzGain = M4_SCALAR_VHZ_FACTOR_GAIN;
    g_sM4Drive.sScalarCtrl.sFreqRampParams.fltRampUp = M4_SCALAR_RAMP_UP;
    g_sM4Drive.sScalarCtrl.sFreqRampParams.fltRampDown = M4_SCALAR_RAMP_DOWN;
    g_sM4Drive.sScalarCtrl.sFreqIntegrator.a32Gain = M4_SCALAR_INTEG_GAIN;
    g_sM4Drive.sScalarCtrl.fltFreqMax = M4_FREQ_MAX;
    
    /* Open loop start up */
    g_sM4Drive.sStartUp.sSpeedIntegrator.a32Gain = M4_SCALAR_INTEG_GAIN;
    g_sM4Drive.sStartUp.f16CoeffMerging = M4_MERG_COEFF;
    g_sM4Drive.sStartUp.fltSpeedCatchUp = M4_MERG_SPEED_TRH;
    g_sM4Drive.sStartUp.fltCurrentStartup = M4_OL_START_I;
    g_sM4Drive.sStartUp.sSpeedRampOpenLoopParams.fltRampUp = M4_OL_START_RAMP_INC;
    g_sM4Drive.sStartUp.sSpeedRampOpenLoopParams.fltRampDown = M4_OL_START_RAMP_INC;
    g_sM4Drive.sStartUp.fltSpeedMax = M4_N_MAX;
    g_sM4Drive.sStartUp.bOpenLoop = TRUE;

    /* MCAT cascade control variables */
    g_sM4Drive.sMCATctrl.sIDQReqMCAT.fltD = 0.0F;
    g_sM4Drive.sMCATctrl.sIDQReqMCAT.fltQ = 0.0F;
    g_sM4Drive.sMCATctrl.sUDQReqMCAT.fltD = 0.0F;
    g_sM4Drive.sMCATctrl.sUDQReqMCAT.fltQ = 0.0F;
    g_sM4Drive.sMCATctrl.ui16PospeSensor = MCAT_ENC_CTRL;

    /* Timing control and general variables */
    g_sM4Drive.ui16CounterState = 0U;
    g_sM4Drive.ui16TimeFullSpeedFreeWheel = M4_FREEWHEEL_DURATION;
    g_sM4Drive.ui16TimeCalibration = M4_CALIB_DURATION;
    g_sM4Drive.ui16TimeFaultRelease = M4_FAULT_DURATION;
    g_bM4SwitchAppOnOff = FALSE;
    /* Default MCAT control mode after reset */
    g_sM4Drive.eControl = kControlMode_SpeedFOC;

    /* fault set to init states */
    FAULT_CLEAR_ALL(g_sM4Drive.sFaultIdCaptured);
    FAULT_CLEAR_ALL(g_sM4Drive.sFaultIdPending);

    /* fault thresholds */
    g_sM4Drive.sFaultThresholds.fltUDcBusOver = M4_U_DCB_OVERVOLTAGE;
    g_sM4Drive.sFaultThresholds.fltUDcBusUnder = M4_U_DCB_UNDERVOLTAGE;
    g_sM4Drive.sFaultThresholds.fltUDcBusTrip = M4_U_DCB_TRIP;
    g_sM4Drive.sFaultThresholds.fltSpeedOver = M4_N_OVERSPEED;
    g_sM4Drive.sFaultThresholds.fltSpeedMin = M4_N_MIN;
    g_sM4Drive.sFaultThresholds.fltSpeedNom = M4_N_NOM;
    g_sM4Drive.sFaultThresholds.fltUqBemf = M4_E_BLOCK_TRH;
    g_sM4Drive.sFaultThresholds.ui16BlockedPerNum = M4_E_BLOCK_PER;

    /* fault blocked rotor filter */
    g_sM4Drive.msM4BlockedRotorUqFilt.fltLambda = M4_BLOCK_ROT_FAULT_SH;

    /* Defined scaling for FreeMASTER */
    g_fltM4voltageScale = M4_U_MAX;
    g_fltM4currentScale = M4_I_MAX;
    g_fltM4DCBvoltageScale = M4_U_DCB_MAX;
    g_fltM4speedScale = M4_N_MAX;
    g_fltM4speedAngularScale = (60.0F / (M4_MOTOR_PP * 2.0F * FLOAT_PI));
    g_fltM4speedMechanicalScale = (60.0F / (2.0F * FLOAT_PI));

    /* Application timing */
    g_sM4Drive.ui16FastCtrlLoopFreq = (g_sClockSetup.ui16M4PwmFreq/M4_FOC_FREQ_VS_PWM_FREQ);
    g_sM4Drive.ui16SlowCtrlLoopFreq = g_sClockSetup.ui16M4SpeedLoopFreq;
    
    /* Power Stage characteristic data */
    g_sM4Drive.sFocPMSM.fltPwrStgCharIRange = DTCOMP_I_RANGE;
    g_sM4Drive.sFocPMSM.fltPwrStgCharLinCoeff = DTCOMP_LINCOEFF;

    /* Clear rest of variables  */
    M4_ClearFOCVariables();

    /* Init sensors/actuators pointers */
    /* For PWM driver */
    g_sM4Pwm3ph.psUABC            = &(g_sM4Drive.sFocPMSM.sDutyABC);
    /* For ADC driver */
    g_sM4AdcSensor.pf16UDcBus     = &(g_sM4Drive.sFocPMSM.f16UDcBus);
    g_sM4AdcSensor.psIABC         = &(g_sM4Drive.sFocPMSM.sIABCFrac);
    g_sM4AdcSensor.pui16SVMSector = &(g_sM4Drive.sFocPMSM.ui16SectorSVM);
    g_sM4AdcSensor.pui16AuxChan   = &(g_sM4Drive.f16AdcAuxSample);

    /* For ENC driver */
    g_sM4Enc.pf16PosElEst    = &(g_sM4Drive.f16PosElEnc);
    g_sM4Enc.pfltSpdMeEst    = &(g_sM4Drive.fltSpeedEnc);

    /* INIT_DONE command */
    g_sM4Ctrl.uiCtrl |= SM_CTRL_INIT_DONE;
}

/*!
 * @brief Stop state routine called in fast state machine
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M4_StateStopFast(void)
{
    /* read ADC results (ADC triggered by HW trigger from PDB) */
    /* get all adc samples - DC-bus voltage, current, bemf and aux sample */
    M4_MCDRV_ADC_GET(&g_sM4AdcSensor);
    
    /* Set encoder direction */
    M4_MCDRV_QD_SET_DIRECTION(&g_sM4Enc);
    
    /* get position and speed from quadrature encoder sensor */
    M4_MCDRV_QD_GET(&g_sM4Enc);

    /* convert voltages from fractional measured values to float */
    g_sM4Drive.sFocPMSM.fltUDcBus = 
        MLIB_ConvSc_FLTsf(g_sM4Drive.sFocPMSM.f16UDcBus, g_fltM4DCBvoltageScale);

    /* Sampled DC-Bus voltage filter */
    g_sM4Drive.sFocPMSM.fltUDcBusFilt =
        GDFLIB_FilterIIR1_FLT(g_sM4Drive.sFocPMSM.fltUDcBus, &g_sM4Drive.sFocPMSM.sUDcBusFilter);

    /* If the user switches on or set non-zero speed*/
    if ((g_bM4SwitchAppOnOff != 0) || (g_sM4Drive.sSpeed.fltSpeedCmd != 0.0F))   
    {
        /* Set the switch on */
        g_bM4SwitchAppOnOff = TRUE;

        /* Start command */
        g_sM4Ctrl.uiCtrl |= SM_CTRL_START;
    }

    /* Braking resistor control */
    if(g_sM4Drive.sFocPMSM.fltUDcBusFilt > g_sM4Drive.sFaultThresholds.fltUDcBusTrip)
        M4_BRAKE_SET();
    else
        M4_BRAKE_CLEAR();

    M4_FaultDetection();

    /* If a fault occurred */
    if (g_sM4Drive.sFaultIdPending)
    {
        /* Switches to the FAULT state */
        g_sM4Ctrl.uiCtrl |= SM_CTRL_FAULT;
    }

    /* PWM peripheral update */
    M4_MCDRV_PWM3PH_SET(&g_sM4Pwm3ph);
}

/*!
 * @brief Run state routine called in fast state machine
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M4_StateRunFast(void)
{
    /* get all adc samples - DC-bus voltage, current, bemf and aux sample */
    M4_MCDRV_ADC_GET(&g_sM4AdcSensor);
    
    /* get position and speed from quadrature encoder sensor */
    M4_MCDRV_QD_GET(&g_sM4Enc);
        
    /* LED */
    M4_MCDRV_QD_LED(&g_sM4Enc);

    /* If the user switches off */
    if (!g_bM4SwitchAppOnOff)
    {
        /* Stop command */
        g_sM4Ctrl.uiCtrl |= SM_CTRL_STOP;
        
        g_sM4Drive.sPosition.a32PositionCmd = 0;
        g_sM4Drive.sPosition.a32Position = 0;
        g_sM4Drive.sPosition.a32PositionIndexOffset = 0;
            
    }

    /* detect fault */
    M4_FaultDetection();

    /* If a fault occurred */
    if (g_sM4Drive.sFaultIdPending != 0)
    {
        /* Switches to the FAULT state */
        g_sM4Ctrl.uiCtrl |= SM_CTRL_FAULT;
    }

    /* get all adc samples - DC-bus voltage, current, bemf and aux sample */
    M4_MCDRV_ADC_GET(&g_sM4AdcSensor);

    /* convert phase currents from fractional measured values to float */
    g_sM4Drive.sFocPMSM.sIABC.fltA = MLIB_ConvSc_FLTsf(g_sM4Drive.sFocPMSM.sIABCFrac.f16A, g_fltM4currentScale);
    g_sM4Drive.sFocPMSM.sIABC.fltB = MLIB_ConvSc_FLTsf(g_sM4Drive.sFocPMSM.sIABCFrac.f16B, g_fltM4currentScale);
    g_sM4Drive.sFocPMSM.sIABC.fltC = MLIB_ConvSc_FLTsf(g_sM4Drive.sFocPMSM.sIABCFrac.f16C, g_fltM4currentScale);

    /* convert voltages from fractional measured values to float */
    g_sM4Drive.sFocPMSM.fltUDcBus = 
        MLIB_ConvSc_FLTsf(g_sM4Drive.sFocPMSM.f16UDcBus, g_fltM4DCBvoltageScale);

    /* Sampled DC-Bus voltage filter */
    g_sM4Drive.sFocPMSM.fltUDcBusFilt =
        GDFLIB_FilterIIR1_FLT(g_sM4Drive.sFocPMSM.fltUDcBus, &g_sM4Drive.sFocPMSM.sUDcBusFilter);

    /* Braking resistor control */
    if(g_sM4Drive.sFocPMSM.fltUDcBusFilt > g_sM4Drive.sFaultThresholds.fltUDcBusTrip)
        M4_BRAKE_SET();
    else
        M4_BRAKE_CLEAR();

    /* Run sub-state function */
    s_M4_STATE_RUN_TABLE_FAST[g_eM4StateRun]();

    /* PWM peripheral update */
    M4_MCDRV_PWM3PH_SET(&g_sM4Pwm3ph);

    /* set current sensor for  sampling */
    M4_MCDRV_CURR_3PH_CHAN_ASSIGN(&g_sM4AdcSensor);
}

/*!
 * @brief Fault state routine called in slow state machine
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M4_StateFaultSlow(void)
{
    /* after fault condition ends wait defined time to clear fault state */
    if (!FAULT_ANY(g_sM4Drive.sFaultIdPending))
    {
        if (--g_sM4Drive.ui16CounterState == 0)
        {
            /* Clear fault state */
            g_sM4Ctrl.uiCtrl |= SM_CTRL_FAULT_CLEAR;
        }
    }
    else
    {
        g_sM4Drive.ui16CounterState = g_sM4Drive.ui16TimeFaultRelease;
    }
}

/*!
 * @brief Fault state routine called in slow state machine
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M4_StateInitSlow(void)
{
}

/*!
 * @brief Stop state routine called in slow state machine
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M4_StateStopSlow(void)
{
}

/*!
 * @brief Run state routine called in slow state machine
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M4_StateRunSlow(void)
{
    /* Run sub-state function */
    s_M4_STATE_RUN_TABLE_SLOW[g_eM4StateRun]();
}

/*!
 * @brief Transition from Fault to Stop state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M4_TransFaultStop(void)
{
    /* Type the code to do when going from the FAULT to the INIT state */
    FAULT_CLEAR_ALL(g_sM4Drive.sFaultIdCaptured);

    /* Clear all FOC variables, init filters, etc. */
    M4_ClearFOCVariables();
}

/*!
 * @brief Transition from Init to Fault state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M4_TransInitFault(void)
{
    /* type the code to do when going from the INIT to the FAULT state */
    /* disable PWM outputs */
    M4_MCDRV_PWM3PH_DIS(&g_sM4Pwm3ph);
    g_sM4Drive.ui16CounterState = g_sM4Drive.ui16TimeFaultRelease;

    g_sM4Drive.sSpeed.fltSpeedCmd = 0.0F;
}

/*!
 * @brief Transition from Init to Stop state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M4_TransInitStop(void)
{
    /* type the code to do when going from the INIT to the STOP state */
    /* disable PWM outputs */
    M4_MCDRV_PWM3PH_DIS(&g_sM4Pwm3ph);

    /* Enable Open loop start up */
    g_sM4Drive.sStartUp.bOpenLoop = TRUE;
}

/*!
 * @brief Transition from Stop to Fault state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M4_TransStopFault(void)
{
    /* type the code to do when going from the STOP to the FAULT state */
    /* load the fault release time to counter */
    g_sM4Drive.ui16CounterState = g_sM4Drive.ui16TimeFaultRelease;
}

/*!
 * @brief Transition from Stop to Run state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M4_TransStopRun(void)
{
    /* type the code to do when going from the STOP to the RUN state */
    /* 50% duty cycle */
    g_sM4Drive.sFocPMSM.sDutyABC.f16A = FRAC16(0.5);
    g_sM4Drive.sFocPMSM.sDutyABC.f16B = FRAC16(0.5);
    g_sM4Drive.sFocPMSM.sDutyABC.f16C = FRAC16(0.5);

    /* PWM duty cycles calculation and update */
    M4_MCDRV_PWM3PH_SET(&g_sM4Pwm3ph);

    /* Clear offset filters */
    M4_MCDRV_CURR_3PH_CALIB_INIT(&g_sM4AdcSensor);

    /* Enable PWM output */
    M4_MCDRV_PWM3PH_EN(&g_sM4Pwm3ph);

    /* pass calibration routine duration to state counter*/
    g_sM4Drive.ui16CounterState = g_sM4Drive.ui16TimeCalibration;

    /* Calibration sub-state when transition to RUN */
    g_eM4StateRun = kRunState_Calib;

    /* Acknowledge that the system can proceed into the RUN state */
    g_sM4Ctrl.uiCtrl |= SM_CTRL_RUN_ACK;
}

/*!
 * @brief Transition from Run to Fault state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M4_TransRunFault(void)
{
    /* type the code to do when going from the RUN to the FAULT state */
    /* disable PWM output */
    M4_MCDRV_PWM3PH_DIS(&g_sM4Pwm3ph);
    g_sM4Drive.ui16CounterState = g_sM4Drive.ui16TimeFaultRelease;

    /* clear over load flag */
    g_sM4Drive.sSpeed.bSpeedPiStopInteg = FALSE;

    g_sM4Drive.sSpeed.fltSpeedCmd = 0.0F;
    g_sM4Drive.sScalarCtrl.fltFreqCmd = 0.0F;
    g_sM4Drive.sScalarCtrl.sUDQReq.fltQ = 0.0F;
    g_sM4Drive.sMCATctrl.sUDQReqMCAT.fltQ = 0.0F;
    g_sM4Drive.sMCATctrl.sUDQReqMCAT.fltD = 0.0F;
    g_sM4Drive.sMCATctrl.sIDQReqMCAT.fltQ = 0.0F;
    g_sM4Drive.sMCATctrl.sIDQReqMCAT.fltD = 0.0F;

    /* Clear actual speed values */
    g_sM4Drive.sScalarCtrl.fltFreqRamp = 0.0F;
    g_sM4Drive.sSpeed.fltSpeed = 0.0F;
    g_sM4Drive.sSpeed.fltSpeedFilt = 0.0F;
}

/*!
 * @brief Transition from Run to Stop state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M4_TransRunStop(void)
{
    /* type the code to do when going from the RUN to the STOP state */
    /* disable PWM outputs */
    M4_MCDRV_PWM3PH_DIS(&g_sM4Pwm3ph);

    g_sM4Drive.sSpeed.fltSpeedCmd = 0.0F;
    g_sM4Drive.sScalarCtrl.fltFreqCmd = 0.0F;
    g_sM4Drive.sScalarCtrl.sUDQReq.fltQ = 0.0F;
    g_sM4Drive.sMCATctrl.sUDQReqMCAT.fltQ = 0.0F;
    g_sM4Drive.sMCATctrl.sUDQReqMCAT.fltD = 0.0F;
    g_sM4Drive.sMCATctrl.sIDQReqMCAT.fltQ = 0.0F;
    g_sM4Drive.sMCATctrl.sIDQReqMCAT.fltD = 0.0F;

    M4_ClearFOCVariables();

    /* Acknowledge that the system can proceed into the STOP state */
    g_sM4Ctrl.uiCtrl |= SM_CTRL_STOP_ACK;
}

/*!
 * @brief Calibration process called in fast state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M4_StateRunCalibFast(void)
{
    /* Type the code to do when in the RUN CALIB sub-state
       performing ADC offset calibration */

    /* call offset measurement */
    M4_MCDRV_CURR_3PH_CALIB(&g_sM4AdcSensor);

    /* change SVM sector in range <1;6> to measure all AD channel mapping combinations */
    if (++g_sM4Drive.sFocPMSM.ui16SectorSVM > 6)
        g_sM4Drive.sFocPMSM.ui16SectorSVM = 1;
}

/*!
 * @brief Motor identification process called in fast state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */

static void M4_StateRunMeasureFast(void)
{
    /* Set zero position at all measurements */
    if((g_sMIDCtrl.eState == kMID_Ld) || (g_sMIDCtrl.eState == kMID_Lq) || (g_sMIDCtrl.eState == kMID_Start) || (g_sMIDCtrl.eState == kMID_Rs) || (g_sMIDCtrl.eState == kMID_PwrStgCharact))
    {
        /* Zero position is needed for RL measurement */
        g_sM4Drive.sFocPMSM.f16PosEl = FRAC16(0.0);

        g_sM4Drive.sFocPMSM.sAnglePosEl.fltSin = 0.0F;
        g_sM4Drive.sFocPMSM.sAnglePosEl.fltCos = 1.0F;
    }
    
    /* turn on dead-time compensation in case of Rs measurement */
    g_sM4Drive.sFocPMSM.bFlagDTComp = (g_sMIDCtrl.eState == kMID_Rs);
    
    /* Perform current transformations if voltage control will be done.
     * At other measurements it is done in a current loop calculation */
    if((g_sMIDCtrl.eState == kMID_Ld) || (g_sMIDCtrl.eState == kMID_Lq))
    {
        /* Current transformations */
        GMCLIB_Clark_FLT(&g_sM4Drive.sFocPMSM.sIABC, &g_sM4Drive.sFocPMSM.sIAlBe);
        GMCLIB_Park_FLT(&g_sM4Drive.sFocPMSM.sIAlBe, &g_sM4Drive.sFocPMSM.sAnglePosEl, &g_sM4Drive.sFocPMSM.sIDQ);
    }
    
    /* If electrical parameters are being measured, put external position to FOC */
    if(g_sMIDCtrl.eState == kMID_Mech)
    {
        g_sM4Drive.sFocPMSM.bPosExtOn = (g_sMID.sMIDMech.eState == kMID_MechStartUp);
        g_sM4Drive.sFocPMSM.bOpenLoop = g_sMID.sMIDMech.sStartup.bOpenLoop;
    }
    else
        g_sM4Drive.sFocPMSM.bPosExtOn = TRUE;

    /* Motor parameters measurement state machine */
    MID_SM_StateMachine(&g_sMIDCtrl);

    /* Perform Current control if MID_START or MID_PWR_STG_CHARACT or MID_RS or MID_PP or MID_KE state */
    if((g_sMIDCtrl.eState == kMID_Start) || (g_sMIDCtrl.eState == kMID_PwrStgCharact) || (g_sMIDCtrl.eState == kMID_Rs) || (g_sMIDCtrl.eState == kMID_Pp) || (g_sMIDCtrl.eState == kMID_Ke) || (g_sMIDCtrl.eState == kMID_Mech))
    {    
        /* enable current control loop */
        g_sM4Drive.sFocPMSM.bCurrentLoopOn = TRUE;
    }
    
    /* Perform Voltage control if MID_LD or MID_LQ or START state*/
    else
    {
        /* disable current control loop */
        g_sM4Drive.sFocPMSM.bCurrentLoopOn = FALSE;
    }
    
    /* FOC */
    MCS_PMSMFocCtrl(&g_sM4Drive.sFocPMSM);

    /* Force sector to 4 to ensure that currents Ia, Ib will be sensed and Ic calculated */
    g_sM4Drive.sFocPMSM.ui16SectorSVM = 4U;

    /* When Measurement done go to RUN READY sub-state and then to STOP state and reset uw16Enable measurement */
    if(g_sMIDCtrl.uiCtrl & MID_SM_CTRL_STOP_ACK)
    {
        M4_TransRunMeasureReady();
        g_bM4SwitchAppOnOff = FALSE;
        g_sMID.ui16EnableMeasurement = 0U;
    }
}

/*!
 * @brief Ready state called in fast state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M4_StateRunReadyFast(void)
{
    /* Type the code to do when in the RUN READY sub-state */
    /* Clear actual speed values */
    g_sM4Drive.sScalarCtrl.fltFreqRamp = 0.0F;
    g_sM4Drive.sSpeed.fltSpeed = 0.0F;
    g_sM4Drive.sSpeed.fltSpeedFilt = 0.0F;
    g_sM4Drive.sFocPMSM.f16PosElEst = 0;
    g_sM4Drive.sFocPMSM.fltSpeedElEst = 0.0F;

    /* MCAT control structure switch */
    switch (g_sM4Drive.eControl)
    {
    case kControlMode_Scalar:
        if (!(g_sM4Drive.sScalarCtrl.fltFreqCmd == 0.0F))
        {
            g_sM4Drive.sScalarCtrl.fltFreqRamp = 0.0F;
            g_sM4Drive.sScalarCtrl.sUDQReq.fltQ = 0.0F;

            /* Transition to the RUN ALIGN sub-state */
            M4_TransRunReadyAlign();
        }
        break;

    case kControlMode_VoltageFOC:
        if (!(g_sM4Drive.sMCATctrl.sUDQReqMCAT.fltQ == 0.0F ))
        {
            if(g_sM4Drive.sMCATctrl.sUDQReqMCAT.fltQ > 0.0F)
                g_sM4Drive.sSpeed.fltSpeedCmd = g_sM4Drive.sStartUp.fltSpeedCatchUp * 2.0F;
            else
                g_sM4Drive.sSpeed.fltSpeedCmd = MLIB_Neg_FLT(g_sM4Drive.sStartUp.fltSpeedCatchUp * 2.0F);

            /* Transition to the RUN ALIGN sub-state */
            M4_TransRunReadyAlign();
        }
        break;

    case kControlMode_CurrentFOC:
        if (!(g_sM4Drive.sMCATctrl.sIDQReqMCAT.fltQ == 0.0F ))
        {
            if(g_sM4Drive.sMCATctrl.sIDQReqMCAT.fltQ > 0.0F)
                g_sM4Drive.sSpeed.fltSpeedCmd = g_sM4Drive.sStartUp.fltSpeedCatchUp * 2.0F;
            else
                g_sM4Drive.sSpeed.fltSpeedCmd = MLIB_Neg_FLT(g_sM4Drive.sStartUp.fltSpeedCatchUp * 2.0F);

            /* Transition to the RUN ALIGN sub-state */
            M4_TransRunReadyAlign();
        }
        break;

    case kControlMode_SpeedFOC:
               
    case kControlMode_PositionFOC:
      
    default: 
        if((g_sM4Drive.sMCATctrl.ui16PospeSensor==MCAT_ENC_CTRL) ||
            ((MLIB_Abs_FLT(g_sM4Drive.sSpeed.fltSpeedCmd) > g_sM4Drive.sFaultThresholds.fltSpeedMin) &&
            (MLIB_Abs_FLT(g_sM4Drive.sSpeed.fltSpeedCmd) <= g_sM4Drive.sFaultThresholds.fltSpeedNom)))
        {
            /* Transition to the RUN ALIGN sub-state */
            M4_TransRunReadyAlign();
        }
        else
        {
            g_sM4Drive.sSpeed.fltSpeedCmd = 0.0F;
        }
    }
}

/*!
 * @brief Alignment process called in fast state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M4_StateRunAlignFast(void)
{
    /* type the code to do when in the RUN ALIGN sub-state */
    /* When alignment elapsed go to Startup */
    if (--g_sM4Drive.ui16CounterState == 0U)
    {
        if((g_sM4Drive.sMCATctrl.ui16PospeSensor==MCAT_ENC_CTRL) && 
           (g_sM4Drive.eControl != kControlMode_Scalar))
        {
            /* Transition to the RUN kRunState_Spin sub-state */
            M4_TransRunAlignSpin();
        }
        else
        {    
            /* Transition to the RUN kRunState_Startup sub-state */
            M4_TransRunAlignStartup();
        }
    }

    /* If zero speed command go back to Ready */
    if((g_sM4Drive.sMCATctrl.ui16PospeSensor==MCAT_SENSORLESS_CTRL) && (g_sM4Drive.sSpeed.fltSpeedCmd == 0.0F) && (g_sM4Drive.sScalarCtrl.fltFreqCmd == 0.0F))
        M4_TransRunAlignReady();

    /* clear actual speed values */
    g_sM4Drive.sScalarCtrl.fltFreqRamp = 0.0F;
    g_sM4Drive.sSpeed.fltSpeed = 0.0F;
    g_sM4Drive.sSpeed.fltSpeedFilt = 0.0F;
    g_sM4Drive.sFocPMSM.f16PosElEst = 0;
    g_sM4Drive.sFocPMSM.fltSpeedElEst = 0.0F;

    MCS_PMSMAlignment(&g_sM4Drive.sAlignment);
    g_sM4Drive.sFocPMSM.f16PosElExt = g_sM4Drive.sAlignment.f16PosAlign;
    MCS_PMSMFocCtrl(&g_sM4Drive.sFocPMSM);
}

/*!
 * @brief Start-up process called in fast state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M4_StateRunStartupFast(void)
{
    /* If f16SpeedCmd = 0, go to Free-wheel state */
    if((g_sM4Drive.sSpeed.fltSpeedCmd==0) && (g_sM4Drive.eControl==kControlMode_SpeedFOC))
        M4_TransRunStartupFreewheel();

    /* Type the code to do when in the RUN STARTUP sub-state */
    /* pass actual estimation position to OL startup structure */
    g_sM4Drive.sStartUp.f16PosEst = g_sM4Drive.sFocPMSM.f16PosElEst;

    /*open loop startup */
    MCS_PMSMOpenLoopStartUp(&g_sM4Drive.sStartUp);

    /* Pass f16SpeedRampOpenloop to f16SpeedRamp*/
    g_sM4Drive.sSpeed.fltSpeedRamp = g_sM4Drive.sStartUp.fltSpeedRampOpenLoop;

    /* Position and speed for FOC */
    g_sM4Drive.sFocPMSM.f16PosElExt = g_sM4Drive.sStartUp.f16PosMerged;

    /* MCAT control structure switch */
    switch (g_sM4Drive.eControl)
    {
    case kControlMode_Scalar: 
        /* switch directly to SPIN state */
        M4_TransRunStartupSpin();
        break;

    case kControlMode_VoltageFOC:
        /* pass MCAT required values in run-time */
        g_sM4Drive.sFocPMSM.sUDQReq.fltD = g_sM4Drive.sMCATctrl.sUDQReqMCAT.fltD;
        g_sM4Drive.sFocPMSM.sUDQReq.fltQ = g_sM4Drive.sMCATctrl.sUDQReqMCAT.fltQ;
        /* FOC */
        g_sM4Drive.sFocPMSM.bCurrentLoopOn = FALSE;
        MCS_PMSMFocCtrl(&g_sM4Drive.sFocPMSM);
        break;  

    case kControlMode_CurrentFOC:
        /* FOC */
        g_sM4Drive.sFocPMSM.sIDQReq.fltD = g_sM4Drive.sMCATctrl.sIDQReqMCAT.fltD;
        g_sM4Drive.sFocPMSM.sIDQReq.fltQ = g_sM4Drive.sMCATctrl.sIDQReqMCAT.fltQ;
        g_sM4Drive.sFocPMSM.bCurrentLoopOn = TRUE;
        MCS_PMSMFocCtrl(&g_sM4Drive.sFocPMSM);
        break;

    case kControlMode_SpeedFOC:
      
    case kControlMode_PositionFOC:
        
    default:

        /* Current control loop */
        g_sM4Drive.sFocPMSM.sIDQReq.fltD = 0.0F;

        /* during the open loop start up the values of required Iq current are kept in pre-defined level*/
        if (g_sM4Drive.sSpeed.fltSpeedCmd > 0.0F)
            g_sM4Drive.sFocPMSM.sIDQReq.fltQ = g_sM4Drive.sStartUp.fltCurrentStartup;
        else
            g_sM4Drive.sFocPMSM.sIDQReq.fltQ = MLIB_Neg_FLT(g_sM4Drive.sStartUp.fltCurrentStartup);

        
        /* Init Bemf observer if open-loop speed is under SpeedCatchUp/2 */
        if (MLIB_Abs_FLT(g_sM4Drive.sStartUp.fltSpeedRampOpenLoop) <
            (g_sM4Drive.sStartUp.fltSpeedCatchUp / 2.0F))
        {
            AMCLIB_PMSMBemfObsrvDQInit_A32fff(&g_sM4Drive.sFocPMSM.sBemfObsrv);
            AMCLIB_TrackObsrvInit_A32af(ACC32(0.0), &g_sM4Drive.sFocPMSM.sTo);
        }

        /* FOC */
        g_sM4Drive.sFocPMSM.bCurrentLoopOn = TRUE;
        MCS_PMSMFocCtrl(&g_sM4Drive.sFocPMSM);
               
        /* select source of actual speed value */
        if(g_sM4Drive.sMCATctrl.ui16PospeSensor == MCAT_ENC_CTRL)
            /* pass encoder speed to actual speed value */
            g_sM4Drive.sSpeed.fltSpeed = g_sM4Drive.fltSpeedEnc * g_sM4Enc.ui16Pp;
        else
            /* pass estimated speed to actual speed value */
            g_sM4Drive.sSpeed.fltSpeed = g_sM4Drive.sFocPMSM.fltSpeedElEst;
        
        
        
    }

    /* switch to close loop  */
    if (!g_sM4Drive.sStartUp.bOpenLoop)
    {
        M4_TransRunStartupSpin();
    }
}

/*!
 * @brief Spin state called in fast state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M4_StateRunSpinFast(void)
{
    /* Type the code to do when in the RUN SPIN sub-state */
    /* MCAT control structure switch */
    switch (g_sM4Drive.eControl)
    {
    case kControlMode_Scalar:
        /* scalar control */
        MCS_PMSMScalarCtrl(&g_sM4Drive.sScalarCtrl);

        /* pass required voltages to Bemf Observer to work */
        g_sM4Drive.sFocPMSM.sUDQReq.fltQ = g_sM4Drive.sScalarCtrl.sUDQReq.fltQ;
        g_sM4Drive.sFocPMSM.sUDQReq.fltD = g_sM4Drive.sScalarCtrl.sUDQReq.fltD;
        g_sM4Drive.sFocPMSM.f16PosElExt = g_sM4Drive.sScalarCtrl.f16PosElScalar;
        
        /* call voltage FOC to calculate PWM duty cycles */
        MCS_PMSMFocCtrl(&g_sM4Drive.sFocPMSM);

        /* Sub-state RUN FREEWHEEL */
        if(g_sM4Drive.sScalarCtrl.fltFreqCmd==0.0F)
               M4_TransRunSpinFreewheel();
        break;

    case kControlMode_VoltageFOC:
        /* FOC */
        g_sM4Drive.sFocPMSM.sUDQReq.fltQ = g_sM4Drive.sMCATctrl.sUDQReqMCAT.fltQ;
        g_sM4Drive.sFocPMSM.sUDQReq.fltD = g_sM4Drive.sMCATctrl.sUDQReqMCAT.fltD;
        g_sM4Drive.sFocPMSM.bCurrentLoopOn = FALSE;
        
        /* Pass encoder position to FOC is enabled */
        if(g_sM4Drive.sMCATctrl.ui16PospeSensor==MCAT_ENC_CTRL)
        {
            g_sM4Drive.sFocPMSM.f16PosElExt = g_sM4Drive.f16PosElEnc;
            g_sM4Drive.sFocPMSM.bPosExtOn   = TRUE;
        }
        else
        {
            g_sM4Drive.sFocPMSM.bPosExtOn = FALSE;
        }
        
        MCS_PMSMFocCtrl(&g_sM4Drive.sFocPMSM);

        /* Sub-state RUN FREEWHEEL */
        if(g_sM4Drive.sMCATctrl.sUDQReqMCAT.fltQ==0.0F)
            M4_TransRunSpinFreewheel();
        break;

    case kControlMode_CurrentFOC: 
        /* current FOC */
        g_sM4Drive.sFocPMSM.sIDQReq.fltQ = g_sM4Drive.sMCATctrl.sIDQReqMCAT.fltQ;
        g_sM4Drive.sFocPMSM.sIDQReq.fltD = g_sM4Drive.sMCATctrl.sIDQReqMCAT.fltD;
        
        /* Pass encoder position to FOC is enabled */
        if(g_sM4Drive.sMCATctrl.ui16PospeSensor==MCAT_ENC_CTRL)
        {
            g_sM4Drive.sFocPMSM.f16PosElExt = g_sM4Drive.f16PosElEnc;
            g_sM4Drive.sFocPMSM.bPosExtOn   = TRUE;
        }
        else
        {
            g_sM4Drive.sFocPMSM.bPosExtOn = FALSE;
        }
        
        g_sM4Drive.sFocPMSM.bCurrentLoopOn = TRUE;
        MCS_PMSMFocCtrl(&g_sM4Drive.sFocPMSM);

        /* Sub-state RUN FREEWHEEL */
        if(g_sM4Drive.sMCATctrl.sIDQReqMCAT.fltQ==0.0F)
            M4_TransRunSpinFreewheel();
        break;

    case kControlMode_SpeedFOC:
    case kControlMode_PositionFOC:
    default: 
        if ((MLIB_Abs_FLT(g_sM4Drive.sSpeed.fltSpeedRamp) < 
            g_sM4Drive.sFaultThresholds.fltSpeedMin) &&
            (g_sM4Drive.sMCATctrl.ui16PospeSensor==MCAT_SENSORLESS_CTRL))
        {
            /* Sub-state RUN FREEWHEEL */
            M4_TransRunSpinFreewheel();
        }
  
        /* Pass encoder position to FOC is enabled */
        if(g_sM4Drive.sMCATctrl.ui16PospeSensor==MCAT_ENC_CTRL)
        {
            g_sM4Drive.sFocPMSM.f16PosElExt = g_sM4Drive.f16PosElEnc;
            g_sM4Drive.sFocPMSM.bPosExtOn   = TRUE;
        }
        else
        {
            g_sM4Drive.sFocPMSM.bPosExtOn = FALSE;
        }
        
        /* FOC */
        g_sM4Drive.sFocPMSM.bCurrentLoopOn = TRUE;
        MCS_PMSMFocCtrl(&g_sM4Drive.sFocPMSM);
        
        /* select source of actual speed value */
        if(g_sM4Drive.sMCATctrl.ui16PospeSensor==MCAT_ENC_CTRL)
            /* pass encoder speed to actual speed value */
            g_sM4Drive.sSpeed.fltSpeed = g_sM4Drive.fltSpeedEnc * g_sM4Enc.ui16Pp;
        else
            /* pass estimated speed to actual speed value */
            g_sM4Drive.sSpeed.fltSpeed = g_sM4Drive.sFocPMSM.fltSpeedElEst;
        
        break;
    }
}

/*!
 * @brief Free-wheel process called in fast state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M4_StateRunFreewheelFast(void)
{
    /* Type the code to do when in the RUN FREEWHEEL sub-state */

    /* clear actual speed values */
    g_sM4Drive.sScalarCtrl.fltFreqRamp = 0.0F;
    g_sM4Drive.sSpeed.fltSpeed = 0.0F;
    g_sM4Drive.sSpeed.fltSpeedFilt = 0.0F;
    g_sM4Drive.sSpeed.fltSpeedRamp = 0.0F;
}

/*!
 * @brief Calibration process called in slow state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M4_StateRunCalibSlow(void)
{
    if (--g_sM4Drive.ui16CounterState == 0U)
    {
        /* write calibrated offset values */
        M4_MCDRV_CURR_3PH_CALIB_SET(&g_sM4AdcSensor);

        if(g_sMID.ui16EnableMeasurement != 0U)
            /* To switch to the RUN MEASURE sub-state */
            M4_TransRunCalibMeasure();
        else
            /* To switch to the RUN READY sub-state */
            M4_TransRunCalibReady();
    }
}

/*!
 * @brief Measure state called in slow state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M4_StateRunMeasureSlow(void)
{
}

/*!
 * @brief Ready state called in slow state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M4_StateRunReadySlow(void)
{
}

/*!
 * @brief Alignment process called in slow state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M4_StateRunAlignSlow(void)
{
}

/*!
 * @brief Start-up process called in slow state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M4_StateRunStartupSlow(void)
{
    if(g_sM4Drive.eControl == kControlMode_SpeedFOC)
    {
        /* actual speed filter */
        g_sM4Drive.sSpeed.fltSpeedFilt = GDFLIB_FilterIIR1_FLT(g_sM4Drive.sSpeed.fltSpeed, &g_sM4Drive.sSpeed.sSpeedFilter);

        /* pass required speed values lower than nominal speed */
        if ((MLIB_Abs_FLT(g_sM4Drive.sSpeed.fltSpeedCmd) > g_sM4Drive.sFaultThresholds.fltSpeedNom))
        {
            /* set required speed to nominal speed if over speed command > speed nominal */
            if (g_sM4Drive.sSpeed.fltSpeedCmd > 0.0F)
                g_sM4Drive.sSpeed.fltSpeedCmd = g_sM4Drive.sFaultThresholds.fltSpeedNom;
            else
                g_sM4Drive.sSpeed.fltSpeedCmd = MLIB_Neg_FLT(g_sM4Drive.sFaultThresholds.fltSpeedNom);
        }
    }
}

/*!
 * @brief Spin state called in slow state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M4_StateRunSpinSlow(void)
{
    if(g_sM4Drive.eControl == kControlMode_SpeedFOC)
    {
        /* actual speed filter */
        g_sM4Drive.sSpeed.fltSpeedFilt = GDFLIB_FilterIIR1_FLT(g_sM4Drive.sSpeed.fltSpeed, &g_sM4Drive.sSpeed.sSpeedFilter);

        /* pass required speed values lower than nominal speed */
        if ((MLIB_Abs_FLT(g_sM4Drive.sSpeed.fltSpeedCmd) > g_sM4Drive.sFaultThresholds.fltSpeedNom))
        {
            /* set required speed to nominal speed if over speed command > speed nominal */
            if (g_sM4Drive.sSpeed.fltSpeedCmd > 0.0F)
                g_sM4Drive.sSpeed.fltSpeedCmd = g_sM4Drive.sFaultThresholds.fltSpeedNom;
            else
                g_sM4Drive.sSpeed.fltSpeedCmd = MLIB_Neg_FLT(g_sM4Drive.sFaultThresholds.fltSpeedNom);
        }

        if ((MLIB_Abs_FLT(g_sM4Drive.sSpeed.fltSpeedRamp) < g_sM4Drive.sFaultThresholds.fltSpeedMin)&&
           (g_sM4Drive.sMCATctrl.ui16PospeSensor==MCAT_SENSORLESS_CTRL))
            M4_TransRunSpinFreewheel();

        /* call PMSM speed control */
        g_sM4Drive.sSpeed.bIqPiLimFlag = g_sM4Drive.sFocPMSM.sIqPiParams.bLimFlag;
        MCS_PMSMFocCtrlSpeed(&g_sM4Drive.sSpeed);
        g_sM4Drive.sFocPMSM.sIDQReq.fltQ = g_sM4Drive.sSpeed.fltIqReq;
    }

    if(g_sM4Drive.eControl == kControlMode_PositionFOC)
    {
        /* actual speed filter */
        g_sM4Drive.sSpeed.fltSpeedFilt = GDFLIB_FilterIIR1_FLT(g_sM4Drive.sSpeed.fltSpeed, &g_sM4Drive.sSpeed.sSpeedFilter);
        
        /* pass required speed values lower than nominal speed */
        if ((MLIB_Abs_FLT(g_sM4Drive.sSpeed.fltSpeedCmd) > g_sM4Drive.sFaultThresholds.fltSpeedNom))
        {
            /* set required speed to nominal speed if over speed command > speed nominal */
            if (g_sM4Drive.sSpeed.fltSpeedCmd > 0.0F)
                g_sM4Drive.sSpeed.fltSpeedCmd = g_sM4Drive.sFaultThresholds.fltSpeedNom;
            else
                g_sM4Drive.sSpeed.fltSpeedCmd = MLIB_Neg_FLT(g_sM4Drive.sFaultThresholds.fltSpeedNom);
        }

        if ((MLIB_Abs_FLT(g_sM4Drive.sSpeed.fltSpeedRamp) < g_sM4Drive.sFaultThresholds.fltSpeedMin)&&
           (g_sM4Drive.sMCATctrl.ui16PospeSensor==MCAT_SENSORLESS_CTRL))
            M4_TransRunSpinFreewheel();      
          
        /* Actual position */                 
        g_sM4Drive.sPosition.a32Position = g_sM4Enc.a32PosMeReal - g_sM4Drive.sPosition.a32PositionIndexOffset - g_sM4Drive.sPosition.a32PositionAlignOffset;
        
        /* Call PMSM position control */
        MCS_PMSMFocCtrlPosition(&g_sM4Drive.sPosition);               
        
        /* Speed command is equal to position controller output */
        g_sM4Drive.sSpeed.fltSpeedCmd = MLIB_ConvSc_FLTsf(g_sM4Drive.sPosition.f16SpeedReq, g_sM4Drive.sPosition.fltSpeedConvScale);  
               
        /* Call PMSM speed control */
        g_sM4Drive.sSpeed.bIqPiLimFlag = g_sM4Drive.sFocPMSM.sIqPiParams.bLimFlag;
        MCS_PMSMFocCtrlSpeed(&g_sM4Drive.sSpeed);
        g_sM4Drive.sFocPMSM.sIDQReq.fltQ = g_sM4Drive.sSpeed.fltIqReq;
    }
 
}

/*!
 * @brief Free-wheel process called in slow state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M4_StateRunFreewheelSlow(void)
{
    /* wait until free-wheel time passes */
    if (--g_sM4Drive.ui16CounterState == 0)
    {
        /* switch to sub state READY */
        M4_TransRunFreewheelReady();
    }
}

/*!
 * @brief Transition from Calib to Ready state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M4_TransRunCalibReady(void)
{
    /* Type the code to do when going from the RUN CALIB to the RUN READY sub-state */

    /* set 50% PWM duty cycle */
    g_sM4Drive.sFocPMSM.sDutyABC.f16A = FRAC16(0.5);
    g_sM4Drive.sFocPMSM.sDutyABC.f16B = FRAC16(0.5);
    g_sM4Drive.sFocPMSM.sDutyABC.f16C = FRAC16(0.5);

    /* switch to sub state READY */
    g_eM4StateRun = kRunState_Ready;
}

/*!
 * @brief Transition from Calib to Measure state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M4_TransRunCalibMeasure(void)
{
    /* Type the code to do when going from the RUN CALIB to the RUN MEASURE sub-state */
    /* Initialise measurement */
  
    /* Set all measurement as inactive */
    g_sMID.sMIDAlignment.ui16Active     = FALSE;
    g_sMID.sMIDPwrStgChar.ui16Active    = FALSE;
    g_sMID.sMIDRs.ui16Active            = FALSE;
    g_sMID.sMIDLs.ui16Active            = FALSE;
    g_sMID.sMIDKe.ui16Active            = FALSE;
    g_sMID.sMIDPp.ui16Active            = FALSE;
    g_sMID.sMIDMech.ui16Active          = FALSE;
    
    /* I/O pointers */
    g_sMID.sIO.pf16PosElExt = &(g_sM4Drive.sFocPMSM.f16PosElExt);
    g_sMID.sIO.pfltId       = &(g_sM4Drive.sFocPMSM.sIDQ.fltD);
    g_sMID.sIO.pfltIq       = &(g_sM4Drive.sFocPMSM.sIDQ.fltQ);
    g_sMID.sIO.pfltIdReq    = &(g_sM4Drive.sFocPMSM.sIDQReq.fltD);
    g_sMID.sIO.pfltIqReq    = &(g_sM4Drive.sFocPMSM.sIDQReq.fltQ);
    g_sMID.sIO.pfltUdReq    = &(g_sM4Drive.sFocPMSM.sUDQReq.fltD);
    g_sMID.sIO.pfltUqReq    = &(g_sM4Drive.sFocPMSM.sUDQReq.fltQ);
    g_sMID.sIO.pfltUDCbus   = &(g_sM4Drive.sFocPMSM.fltUDcBusFilt);
    g_sMID.sIO.pfltEd       = &(g_sM4Drive.sFocPMSM.sBemfObsrv.sEObsrv.fltD);
    g_sMID.sIO.pfltEq       = &(g_sM4Drive.sFocPMSM.sBemfObsrv.sEObsrv.fltQ);
    g_sMID.sIO.pfltSpeedEst = &(g_sM4Drive.sFocPMSM.fltSpeedElEst);
    g_sMID.sIO.pf16PosElEst = &(g_sM4Drive.sFocPMSM.f16PosElEst);
    g_sMID.sIO.pf16PosElExt = &(g_sM4Drive.sFocPMSM.f16PosElExt);

    /* Ls measurement init */
    g_sMID.sMIDLs.fltUdMax   = MLIB_Mul_FLT(MID_K_MODULATION_RATIO, g_sM4Drive.sFocPMSM.fltUDcBusFilt);
    g_sMID.sMIDLs.fltFreqMax = (float_t)g_sM4Drive.ui16FastCtrlLoopFreq / 2U;

    /* Ke measurement init */
    g_sMID.sMIDKe.fltFreqMax = (float_t)g_sM4Drive.ui16FastCtrlLoopFreq / 2U;

    /* Pp measurement init */
    g_sMID.sMIDPp.fltFreqMax = (float_t)g_sM4Drive.ui16FastCtrlLoopFreq / 2U;
    
    /* PwrStg char init */
    g_sMID.sMIDPwrStgChar.ui16NumOfChPnts = MID_CHAR_CURRENT_POINT_NUMBERS;

    /* During the measurement motor is driven open-loop */
    g_sM4Drive.sFocPMSM.bOpenLoop = TRUE; 

    /* Reset DONE & ACK of all MID states */
    g_sMIDCtrl.uiCtrl = 0;

    /* First state in MID state machine will be kMID_Start */
    g_sMIDCtrl.eState = kMID_Start;

    /* Sub-state RUN MEASURE */
    g_eM4StateRun = kRunState_Measure;
}

/*!
 * @brief Transition from Measure to Ready state
 *
 * @param void  No input parameter
 *
 * @return None
 */

static void M4_TransRunMeasureReady(void)
{
    /* Type the code to do when going from the RUN CALIB to the RUN READY sub-state */
    /* Set off measurement */
    g_sMID.ui16EnableMeasurement = 0;

    /* set 50% PWM duty cycle */
    g_sM4Drive.sFocPMSM.sDutyABC.f16A = FRAC16(0.5);
    g_sM4Drive.sFocPMSM.sDutyABC.f16B = FRAC16(0.5);
    g_sM4Drive.sFocPMSM.sDutyABC.f16C = FRAC16(0.5);

    /* disable passing external electrical position to FOC */
    g_sM4Drive.sFocPMSM.bPosExtOn = FALSE;                                   

    /* swith to sub state READY */
    g_eM4StateRun = kRunState_Ready;
}

/*!
 * @brief Transition from Ready to Align state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M4_TransRunReadyAlign(void)
{
    /* Type the code to do when going from the RUN kRunState_Ready to the RUN kRunState_Align sub-state */
    /* Alignment duration set-up */
    g_sM4Drive.ui16CounterState = g_sM4Drive.sAlignment.ui16Time;
    /* Counter of half alignment duration */
    g_sM4Drive.sAlignment.ui16TimeHalf = MLIB_ShR_F16(g_sM4Drive.sAlignment.ui16Time, 1);

    /* set required alignment voltage to Ud */
    g_sM4Drive.sFocPMSM.sUDQReq.fltD = g_sM4Drive.sAlignment.fltUdReq;
    g_sM4Drive.sFocPMSM.sUDQReq.fltQ = 0.0F;

    /* enable passing required position to FOC */
    g_sM4Drive.sFocPMSM.bPosExtOn = TRUE;

    /* disable current FOC */
    g_sM4Drive.sFocPMSM.bCurrentLoopOn = FALSE;
    
    /* enable Open loop mode in main control structure */
    g_sM4Drive.sFocPMSM.bOpenLoop = TRUE;

    /* Enable PWM output */
    M4_MCDRV_PWM3PH_EN(&g_sM4Pwm3ph);

    /* Sub-state RUN ALIGN */
    g_eM4StateRun = kRunState_Align;
}

/*!
 * @brief Transition from Align to Startup state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M4_TransRunAlignStartup(void)
{
    /* Type the code to do when going from the RUN kRunState_Align to the RUN kRunState_Startup sub-state */
    /* initialize encoder driver */
    M4_MCDRV_QD_CLEAR(&g_sM4Enc);
  
    /* clear application parameters */
    M4_ClearFOCVariables();

    /* pass required speed to open loop start-up structure */
    if (g_sM4Drive.sSpeed.fltSpeedCmd > 0.0F)
        g_sM4Drive.sStartUp.fltSpeedReq = g_sM4Drive.sStartUp.fltSpeedCatchUp;
    else
        g_sM4Drive.sStartUp.fltSpeedReq = MLIB_Neg_FLT(g_sM4Drive.sStartUp.fltSpeedCatchUp);

    /* enable Open loop mode in main control structure */
    g_sM4Drive.sStartUp.bOpenLoop = TRUE;
    g_sM4Drive.sFocPMSM.bOpenLoop = TRUE;

    /* enable Open loop mode in FOC module */
    g_sM4Drive.sFocPMSM.bPosExtOn = TRUE;

    g_sM4Drive.sFocPMSM.ui16SectorSVM = M4_SVM_SECTOR_DEFAULT;
    GDFLIB_FilterIIR1Init_FLT(&g_sM4Drive.sSpeed.sSpeedFilter);

    /* Go to sub-state RUN STARTUP */
    g_eM4StateRun = kRunState_Startup;
}

/*!
 * @brief Transition from Align to Spin state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M4_TransRunAlignSpin(void)
{
    /* Type the code to do when going from the RUN STARTUP to the RUN SPIN sub-state */
    /* initialize encoder driver */
    M4_MCDRV_QD_CLEAR(&g_sM4Enc);
  
    g_sM4Drive.sFocPMSM.bPosExtOn = TRUE;                                        /* enable passing external electrical position from encoder to FOC */
    g_sM4Drive.sFocPMSM.bOpenLoop = FALSE;                                       /* disable parallel runnig openloop and estimator */
  
    g_sM4Drive.sFocPMSM.ui16SectorSVM    = M4_SVM_SECTOR_DEFAULT;
    g_sM4Drive.sFocPMSM.sIDQReq.fltD     = 0.0F;
    g_sM4Drive.sFocPMSM.sIDQReq.fltQ     = 0.0F;
    
    M4_ClearFOCVariables();

    /* Turn on ENC index interrupt */
    M4_MCDRV_QD_INDEX_IRQ_ON(&g_sM4Enc);
      
    /* To switch to the RUN SPIN sub-state */
    g_eM4StateRun = kRunState_Spin;
}

/*!
 * @brief Transition from Align to Ready state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M4_TransRunAlignReady(void)
{
    /* Type the code to do when going from the RUN kRunState_Align to the RUN kRunState_Ready sub-state */
    
    /* Clear FOC accumulators */
    M4_ClearFOCVariables();

    /* Go to sub-state RUN READY */
    g_eM4StateRun = kRunState_Ready;
}

/*!
 * @brief Transition from Startup to Spin state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M4_TransRunStartupSpin(void)
{
    /* Type the code to do when going from the RUN kRunState_Startup to the RUN kRunState_Spin sub-state */
    /* for FOC control switch open loop off in DQ observer */
    if(g_sM4Drive.eControl!=kControlMode_Scalar)
    {    
        g_sM4Drive.sFocPMSM.bPosExtOn = FALSE; /* disable passing external electrical position to FOC */
        g_sM4Drive.sFocPMSM.bOpenLoop = FALSE; /* disable parallel running open-loop and estimator */
    }

    g_sM4Drive.sSpeed.sSpeedPiParams.fltIAccK_1 = g_sM4Drive.sFocPMSM.sIDQReq.fltQ;
    g_sM4Drive.sSpeed.sSpeedRampParams.fltState = g_sM4Drive.sStartUp.fltSpeedRampOpenLoop;

    /* To switch to the RUN kRunState_Spin sub-state */
    g_eM4StateRun = kRunState_Spin;
}

/*!
 * @brief Transition from Startup to Free-wheel state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M4_TransRunStartupFreewheel(void)
{
    M4_MCDRV_PWM3PH_DIS(&g_sM4Pwm3ph);

    /* Free-wheel duration set-up */
    g_sM4Drive.ui16CounterState = g_sM4Drive.ui16TimeFullSpeedFreeWheel;

    /* enter FREEWHEEL sub-state */
    g_eM4StateRun = kRunState_Freewheel;
}

/*!
 * @brief Transition from Spin to Free-wheel state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M4_TransRunSpinFreewheel(void)
{
    /* Type the code to do when going from the RUN SPIN to the RUN FREEWHEEL sub-state */
    /* set 50% PWM duty cycle */
    g_sM4Drive.sFocPMSM.sDutyABC.f16A = FRAC16(0.5);
    g_sM4Drive.sFocPMSM.sDutyABC.f16B = FRAC16(0.5);
    g_sM4Drive.sFocPMSM.sDutyABC.f16C = FRAC16(0.5);

    g_sM4Drive.sFocPMSM.ui16SectorSVM = M4_SVM_SECTOR_DEFAULT;

    M4_MCDRV_PWM3PH_DIS(&g_sM4Pwm3ph);

    /* Generates a time gap before the alignment to assure the rotor is not rotating */
    g_sM4Drive.ui16CounterState = g_sM4Drive.ui16TimeFullSpeedFreeWheel;

    g_sM4Drive.sFocPMSM.sIDQReq.fltD = 0.0F;
    g_sM4Drive.sFocPMSM.sIDQReq.fltQ = 0.0F;

    g_sM4Drive.sFocPMSM.sUDQReq.fltD = 0.0F;
    g_sM4Drive.sFocPMSM.sUDQReq.fltQ = 0.0F;

    g_sM4Drive.sFocPMSM.sIAlBe.fltAlpha = 0.0F;
    g_sM4Drive.sFocPMSM.sIAlBe.fltBeta = 0.0F;
    g_sM4Drive.sFocPMSM.sUAlBeReq.fltAlpha = 0.0F;
    g_sM4Drive.sFocPMSM.sUAlBeReq.fltBeta = 0.0F;

    /* enter FREEWHEEL sub-state */
    g_eM4StateRun = kRunState_Freewheel;
}

/*!
 * @brief Transition from Free-wheel to Ready state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M4_TransRunFreewheelReady(void)
{
    /* Type the code to do when going from the RUN kRunState_FreeWheel to the RUN kRunState_Ready sub-state */
    /* clear application parameters */
    M4_ClearFOCVariables();

    M4_MCDRV_PWM3PH_EN(&g_sM4Pwm3ph);

    /* Sub-state RUN READY */
    g_eM4StateRun = kRunState_Ready;
}

/*!
 * @brief Clear FOc variables in global variable
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M4_ClearFOCVariables(void)
{
    g_sM4Drive.sAlignment.ui16TimeHalf = 0U;

    /* Clear FOC variables */
    g_sM4Drive.sFocPMSM.sIABC.fltA = 0.0F;
    g_sM4Drive.sFocPMSM.sIABC.fltB = 0.0F;
    g_sM4Drive.sFocPMSM.sIABC.fltC = 0.0F;
    g_sM4Drive.sFocPMSM.sIAlBe.fltAlpha = 0.0F;
    g_sM4Drive.sFocPMSM.sIAlBe.fltBeta = 0.0F;
    g_sM4Drive.sFocPMSM.sIDQ.fltD = 0.0F;
    g_sM4Drive.sFocPMSM.sIDQ.fltQ = 0.0F;
    g_sM4Drive.sFocPMSM.sIDQReq.fltD = 0.0F;
    g_sM4Drive.sFocPMSM.sIDQReq.fltQ = 0.0F;
    g_sM4Drive.sFocPMSM.sIDQError.fltD = 0.0F;
    g_sM4Drive.sFocPMSM.sIDQError.fltQ = 0.0F;
    g_sM4Drive.sFocPMSM.sDutyABC.f16A = FRAC16(0.5);
    g_sM4Drive.sFocPMSM.sDutyABC.f16B = FRAC16(0.5);
    g_sM4Drive.sFocPMSM.sDutyABC.f16C = FRAC16(0.5);
    g_sM4Drive.sFocPMSM.sUAlBeReq.fltAlpha = 0.0F;
    g_sM4Drive.sFocPMSM.sUAlBeReq.fltBeta = 0.0F;
    g_sM4Drive.sFocPMSM.sUDQReq.fltD = 0.0F;
    g_sM4Drive.sFocPMSM.sUDQReq.fltQ = 0.0F;
    g_sM4Drive.sFocPMSM.sAnglePosEl.fltSin = 0.0F;
    g_sM4Drive.sFocPMSM.sAnglePosEl.fltCos = 0.0F;
    g_sM4Drive.sFocPMSM.sAnglePosEl.fltSin = 0.0F;
    g_sM4Drive.sFocPMSM.sAnglePosEl.fltCos = 0.0F;
    g_sM4Drive.sFocPMSM.sIdPiParams.bLimFlag = FALSE;
    g_sM4Drive.sFocPMSM.sIqPiParams.bLimFlag = FALSE;
    g_sM4Drive.sFocPMSM.sIdPiParams.fltIAccK_1 = 0.0F;
    g_sM4Drive.sFocPMSM.sIdPiParams.fltIAccK_1 = 0.0F;
    g_sM4Drive.sFocPMSM.sIqPiParams.fltIAccK_1 = 0.0F;
    g_sM4Drive.sFocPMSM.sIqPiParams.fltIAccK_1 = 0.0F;
    GDFLIB_FilterIIR1Init_FLT(&g_sM4Drive.sFocPMSM.sSpeedElEstFilt);
    g_sM4Drive.sFocPMSM.bIdPiStopInteg = FALSE;
    g_sM4Drive.sFocPMSM.bIqPiStopInteg = FALSE;

    /* Clear Speed control state variables */
    g_sM4Drive.sSpeed.sSpeedRampParams.fltState = 0.0F;
    g_sM4Drive.sSpeed.fltSpeed = 0.0F;
    g_sM4Drive.sSpeed.fltSpeedFilt = 0.0F;
    g_sM4Drive.sSpeed.fltSpeedError = 0.0F;
    g_sM4Drive.sSpeed.fltSpeedRamp = 0.0F;
    g_sM4Drive.sSpeed.sSpeedPiParams.fltIAccK_1 = 0.0F;
    g_sM4Drive.sSpeed.sSpeedPiParams.bLimFlag = FALSE;
    g_sM4Drive.sSpeed.sSpeedFilter.fltFltBfrX[0] = 0.0F;
    g_sM4Drive.sSpeed.sSpeedFilter.fltFltBfrY[0] = 0.0F;
    g_sM4Drive.sSpeed.bSpeedPiStopInteg = FALSE;
    GDFLIB_FilterIIR1Init_FLT(&g_sM4Drive.sSpeed.sSpeedFilter);

    /* Init Blocked rotor filter */
    GDFLIB_FilterMAInit_FLT(0.0F, &g_sM4Drive.msM4BlockedRotorUqFilt);

    /* Clear Scalar control variables */
    g_sM4Drive.sScalarCtrl.fltFreqRamp = 0.0F;
    g_sM4Drive.sScalarCtrl.f16PosElScalar = 0.0F;
    g_sM4Drive.sScalarCtrl.sUDQReq.fltD = 0.0F;
    g_sM4Drive.sScalarCtrl.sUDQReq.fltQ = 0.0F;
    g_sM4Drive.sScalarCtrl.sFreqIntegrator.f32IAccK_1 = 0;
    g_sM4Drive.sScalarCtrl.sFreqIntegrator.f16InValK_1 = 0;
    g_sM4Drive.sScalarCtrl.sFreqRampParams.fltState = 0.0F;

    /* Clear Startup variables */
    g_sM4Drive.sStartUp.f16PosMerged = 0;
    g_sM4Drive.sStartUp.f16PosEst = 0;
    g_sM4Drive.sStartUp.f16PosGen = 0;
    g_sM4Drive.sStartUp.f16RatioMerging = 0;
    g_sM4Drive.sStartUp.fltSpeedRampOpenLoop = 0.0F;
    g_sM4Drive.sStartUp.fltSpeedReq = 0.0F;
    g_sM4Drive.sStartUp.sSpeedIntegrator.f32IAccK_1 = 0;
    g_sM4Drive.sStartUp.sSpeedIntegrator.f16InValK_1 = 0;
    g_sM4Drive.sStartUp.sSpeedRampOpenLoopParams.fltState = 0.0F;

    /* Clear BEMF and Tracking observers state variables */
    AMCLIB_PMSMBemfObsrvDQInit_A32fff(&g_sM4Drive.sFocPMSM.sBemfObsrv);
    AMCLIB_TrackObsrvInit_A32af(ACC32(0.0), &g_sM4Drive.sFocPMSM.sTo);
}

/*!
 * @brief Fault detention routine - check various faults
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M4_FaultDetection(void)
{
    /* Clearing actual faults before detecting them again  */
    /* Clear all faults */
    FAULT_CLEAR_ALL(g_sM4Drive.sFaultIdPending);

    /* Fault:   DC-bus over-current */
    if (M4_MCDRV_PWM3PH_FLT_GET(&g_sM4Pwm3ph))
        FAULT_SET(g_sM4Drive.sFaultIdPending, FAULT_I_DCBUS_OVER);

    /* Fault:   DC-bus over-voltage */
    if (g_sM4Drive.sFocPMSM.fltUDcBusFilt > g_sM4Drive.sFaultThresholds.fltUDcBusOver)
        FAULT_SET(g_sM4Drive.sFaultIdPending, FAULT_U_DCBUS_OVER);

    /* Fault:   DC-bus under-voltage */
    if (g_sM4Drive.sFocPMSM.fltUDcBusFilt < g_sM4Drive.sFaultThresholds.fltUDcBusUnder)
        FAULT_SET(g_sM4Drive.sFaultIdPending, FAULT_U_DCBUS_UNDER);

    /* Check only in SPEED_FOC control, RUN state, kRunState_Spin and kRunState_FreeWheel sub-states */
    if((g_sM4Drive.eControl==kControlMode_SpeedFOC) && 
       (g_sM4Ctrl.eState==kSM_AppRun) && 
       (g_eM4StateRun==kRunState_Spin || g_eM4StateRun==kRunState_Freewheel) &&
       (g_sM4Drive.sMCATctrl.ui16PospeSensor==MCAT_SENSORLESS_CTRL))
    {
        /* Fault: Overload  */
        float_t fltSpeedFiltAbs = MLIB_Abs_FLT(g_sM4Drive.sSpeed.fltSpeedFilt);
        float_t fltSpeedRampAbs = MLIB_Abs_FLT(g_sM4Drive.sSpeed.fltSpeedRamp);

        if ((fltSpeedFiltAbs < g_sM4Drive.sFaultThresholds.fltSpeedMin) &&
            (fltSpeedRampAbs > g_sM4Drive.sFaultThresholds.fltSpeedMin) &&
            (g_sM4Drive.sSpeed.bSpeedPiStopInteg == TRUE))
            FAULT_SET(g_sM4Drive.sFaultIdPending, FAULT_LOAD_OVER);

        /* Fault: Over-speed  */
        if ((MLIB_Abs_FLT(g_sM4Drive.sSpeed.fltSpeedFilt) > g_sM4Drive.sFaultThresholds.fltSpeedOver) &&
            (MLIB_Abs_FLT(g_sM4Drive.sSpeed.fltSpeedCmd) > g_sM4Drive.sFaultThresholds.fltSpeedMin))
            FAULT_SET(g_sM4Drive.sFaultIdPending, FAULT_SPEED_OVER);

        /* Fault: Blocked rotor detection */
        /* filter of bemf Uq voltage */
        g_sM4Drive. fltBemfUqAvg = GDFLIB_FilterMA_FLT(g_sM4Drive.sFocPMSM.sBemfObsrv.sEObsrv.fltQ,
                                                      &g_sM4Drive.msM4BlockedRotorUqFilt);
        /* check the bemf Uq voltage threshold only in kRunState_Spin - RUN state */
        if ((MLIB_Abs_FLT(g_sM4Drive.fltBemfUqAvg) < g_sM4Drive.sFaultThresholds.fltUqBemf) &&
            (g_eM4StateRun == kRunState_Spin))
            g_sM4Drive.ui16BlockRotorCnt++;
        else
            g_sM4Drive.ui16BlockRotorCnt = 0U;
        /* for bemf voltage detected above limit longer than defined period number set blocked rotor fault*/
        if (g_sM4Drive.ui16BlockRotorCnt > g_sM4Drive.sFaultThresholds.ui16BlockedPerNum)
        {
            FAULT_SET(g_sM4Drive.sFaultIdPending, FAULT_ROTOR_BLOCKED);
            g_sM4Drive.ui16BlockRotorCnt = 0U;
        }
    }
    /* pass fault to Fault ID Captured */
    g_sM4Drive.sFaultIdCaptured |= g_sM4Drive.sFaultIdPending;
}

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
void M4_SetAppSwitch(bool_t bValue)
{
    g_bM4SwitchAppOnOff = bValue;
}

/*!
 * @brief Get application switch value
 *
 * @param void  No input parameter
 *
 * @return bool_t Return bool value, true or false
 */
bool_t M4_GetAppSwitch(void)
{
    return (g_bM4SwitchAppOnOff);
}

/*!
 * @brief Get application state
 *
 * @param void  No input parameter
 *
 * @return uint16_t Return current application state
 */
uint16_t M4_GetAppState()
{
    return ((uint16_t)g_sM4Ctrl.eState);
}

/*!
 * @brief Set spin speed of the motor in fractional value
 *
 * @param f16SpeedCmd  Speed command - set speed
 *
 * @return None
 */
void M4_SetSpeed(float_t fltSpeedCmd)
{
    if (g_bM4SwitchAppOnOff)
    {
        /* Set speed */
        if (MLIB_Abs_FLT(fltSpeedCmd) < g_sM4Drive.sStartUp.fltSpeedCatchUp)
        {
            g_sM4Drive.sSpeed.fltSpeedCmd = 0.0F;
        }
        else if (MLIB_Abs_FLT(fltSpeedCmd) > M4_N_NOM)
        {
            g_sM4Drive.sSpeed.fltSpeedCmd = 0.0F;
        }
        else
        {
            g_sM4Drive.sSpeed.fltSpeedCmd = fltSpeedCmd;
        }
    }
    else
    {
        /* Set zero speed */
        g_sM4Drive.sSpeed.fltSpeedCmd = 0.0F;
    }
}


/*!
 * @brief Set spin speed of the motor in fractional value
 *
 * @param a32PositionCmdDemo  Position command - set position
 *
 * @return None
 */
void M4_SetPosition(acc32_t a32PositionCmdDemo)
{
    if (g_bM4SwitchAppOnOff)
    {
        /* Set position */
        g_sM4Drive.sPosition.a32PositionCmd = a32PositionCmdDemo;
    }
    else
    {
        /* Set zero speed */
        g_sM4Drive.sPosition.a32PositionCmd = 0U;
    }
}

/*!
 * @brief Get spin required speed of the motor in fractional value
 *
 * @param void  No input parameter
 *
 * @return frac16_t Fractional value of the current speed
 */
float_t M4_GetReqSpeed(void)
{
    /* return speed */
    return g_sM4Drive.sSpeed.fltSpeedCmd;
}

