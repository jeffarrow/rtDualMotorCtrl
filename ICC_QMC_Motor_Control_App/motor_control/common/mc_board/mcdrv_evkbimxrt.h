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

#ifndef _MCDRV_MIMXRT1050_EVK_H_
#define _MCDRV_MIMXRT1050_EVK_H_

#include "mcdrv_adc_imxrt.h"
#include "mcdrv_pwm3ph_pwma.h"
#include "mcdrv_enc_qd.h"
#include "m1_pmsm_appconfig.h"
#include "m2_pmsm_appconfig.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Version info */
#define MCRSP_VER       "0.0.1"        /* motor control package version */

/* Application info */
typedef struct _app_ver
{
    char    cBoardID[15];
    char    cMotorType[4];
    char    cAppVer[5];
}app_ver_t;

/* Structure used during clocks and modulo calculations */
typedef struct _clock_setup
{
    uint32_t ui32FastPeripheralClock;
    uint32_t ui32BusClock;
    uint32_t ui32SysPllClock;
    
    uint16_t ui16M1SpeedLoopFreq;
    uint16_t ui16M1SpeedLoopModulo;
    uint16_t ui16M1PwmFreq;
    uint16_t ui16M1PwmModulo;
    uint16_t ui16M1PwmDeadTime; 
    
    uint16_t ui16M2SpeedLoopFreq;
    uint16_t ui16M2SpeedLoopModulo;
    uint16_t ui16M2PwmFreq;
    uint16_t ui16M2PwmModulo;
    uint16_t ui16M2PwmDeadTime;
    
} clock_setup_t;

/******************************************************************************
 * Clock & PWM definition for motor 1
 ******************************************************************************/
#define M1_PWM_FREQ (10000)         /* PWM frequency - 10kHz */
#define M1_FOC_FREQ_VS_PWM_FREQ (1) /* FOC calculation is called every n-th PWM reload */
#define M1_SPEED_LOOP_FREQ (1000)   /* Speed loop frequency */
#define M1_PWM_DEADTIME (500)      /* Output PWM deadtime value in nanoseconds */

#define M1_FAST_LOOP_TS ((float_t)1.0/(float_t)(M1_PWM_FREQ / M1_FOC_FREQ_VS_PWM_FREQ))
#define M1_SLOW_LOOP_TS ((float_t)1.0/(float_t)(M1_SLOW_LOOP_FREQ))
#define M1_TIME_ONESEC_COUNT (M1_PWM_FREQ / M1_FOC_FREQ_VS_PWM_FREQ)

/* Assignment of eFlexPWM channels to motor 1 phases
 * 0 - PWM channels A0&B0 - sub-module 0
 * 1 - PWM channels A1&B1 - sub-module 1
 * 2 - PWM channels A2&B2 - sub-module 2
 */
#define M1_PWM_PAIR_PHA (0)
#define M1_PWM_PAIR_PHB (1)
#define M1_PWM_PAIR_PHC (2)

/* Over Current Fault detection */
#define M1_FAULT_NUM (0)   

/******************************************************************************
 * Clock & PWM definition for motor 2
 ******************************************************************************/
#define M2_PWM_FREQ (10000)         /* PWM frequency - 10kHz */
#define M2_FOC_FREQ_VS_PWM_FREQ (1) /* FOC calculation is called every n-th PWM reload */
#define M2_SPEED_LOOP_FREQ (1000)   /* Speed loop frequency */
#define M2_PWM_DEADTIME (500)      /* Output PWM deadtime value in nanoseconds */

#define M2_FAST_LOOP_TS ((float_t)1.0/(float_t)(M2_PWM_FREQ / M2_FOC_FREQ_VS_PWM_FREQ))
#define M2_SLOW_LOOP_TS ((float_t)1.0/(float_t)(M2_SLOW_LOOP_FREQ))
#define M2_TIME_ONESEC_COUNT (M2_PWM_FREQ / M2_FOC_FREQ_VS_PWM_FREQ)

/* Assignment of eFlexPWM channels to motor 2 phases
 * 0 - PWM channels A0&B0 - sub-module 0
 * 1 - PWM channels A1&B1 - sub-module 1
 * 2 - PWM channels A2&B2 - sub-module 2
 */
#define M2_PWM_PAIR_PHA (0)
#define M2_PWM_PAIR_PHB (1)
#define M2_PWM_PAIR_PHC (2)

/* Over Current Fault detection */
#define M2_FAULT_NUM (0)


/******************************************************************************
 * Define 3-ph PWM control functions for motors 1, 2, 3 & 4
 ******************************************************************************/
#define M1_MCDRV_PWM3PH_SET(par) (MCDRV_eFlexPwm3PhSet(par))
#define M1_MCDRV_PWM3PH_EN(par) (MCDRV_eFlexPwm3PhOutEn(par))
#define M1_MCDRV_PWM3PH_DIS(par) (MCDRV_eFlexPwm3PhOutDis(par))
#define M1_MCDRV_PWM3PH_FLT_GET(par) (MCDRV_eFlexPwm3PhFltGet(par))

#define M2_MCDRV_PWM3PH_SET(par) (MCDRV_eFlexPwm3PhSet(par))
#define M2_MCDRV_PWM3PH_EN(par) (MCDRV_eFlexPwm3PhOutEn(par))
#define M2_MCDRV_PWM3PH_DIS(par) (MCDRV_eFlexPwm3PhOutDis(par))
#define M2_MCDRV_PWM3PH_FLT_GET(par) (MCDRV_eFlexPwm3PhFltGet(par))


/******************************************************************************
 * ADC measurement definition for motors 1, 2, 3 & 4
 ******************************************************************************/
/* Configuration table of ADC channels according to the input pin signals
 *
 * Proper ADC channel assignment needs to follow these rules:
 *   - at least one phase current must be assigned to both ADC modules
 *   - two other phase current channels must be assigned to different ADC modules
 *   - Udcb and auxiliary channels must be assigned to different ADC modules
 */
   
/* Motor 1
 * Quantity     | Module 1 (ADC1)   | Module 2 (ADC2)
 * --------------------------------------------------------------------------
 * M1_I_A       | ADC1_IN10          | ADC2_IN10
 * M1_I_B       | ADC1_IN11          | ADC2_IN11
 * M1_I_C       | ADC1_IN13          | ADC2_IN13
 * M1_U_DCB     | ADC1_IN12          | ---------
 */
/* Phase current A assigned to ADC1 and ADC2 */
#define M1_ADC1_PH_A (10)
#define M1_ADC2_PH_A (10)
/* Phase current B assigned to ADC1 and ADC2 */
#define M1_ADC1_PH_B (11)
#define M1_ADC2_PH_B (11)
/* Phase current C assigned to ADC1 and ADC2 */
#define M1_ADC1_PH_C (13)
#define M1_ADC2_PH_C (13)
/* DCB assigned to ADC1 */
#define M1_ADC1_UDCB (12)
#define M1_ADC2_UDCB (MCDRV_CHAN_OFF)

/* Motor 2
 * Quantity     | Module 1 (ADC1)   | Module 2 (ADC2)
 * --------------------------------------------------------------------------
 * M2_I_A       | ADC1_IN1          | ADC2_IN01
 * M2_I_B       | ADC1_IN8          | ADC2_IN08
 * M2_I_C       | ADC1_IN9          | ADC2_IN09
 * M2_U_DCB     | ADC1_IN12         | ---------
*/ 
/* Phase current A assigned to ADC1 and ADC2 */
#define M2_ADC1_PH_A (14)
#define M2_ADC2_PH_A (14)
/* Phase current A assigned to ADC1 and ADC2 */
#define M2_ADC1_PH_B (8)
#define M2_ADC2_PH_B (8)
/* Phase current A assigned to ADC1 and ADC2 */
#define M2_ADC1_PH_C (9)
#define M2_ADC2_PH_C (9)
/* Phase current A assigned to ADC1 and ADC2 */
#define M2_ADC1_UDCB (12)
#define M2_ADC2_UDCB (MCDRV_CHAN_OFF)
     
/* offset measurement filter window */     
#define ADC_OFFSET_WINDOW (3)

/******************************************************************************
 * Define common ADC control functions for motors 1, 2, 3 & 4
 ******************************************************************************/
#define M1_MCDRV_ADC_GET(par) \
    MCDRV_Curr3Ph2ShGet(par); \
    MCDRV_VoltDcBusGet(par);  \
    MCDRV_AuxValGet(par);
#define M2_MCDRV_ADC_GET(par) \
    MCDRV_Curr3Ph2ShGet(par); \
    MCDRV_VoltDcBusGet(par);  \
    MCDRV_AuxValGet(par);
    
#define M1_MCDRV_CURR_3PH_CHAN_ASSIGN(par) (MCDRV_Curr3Ph2ShChanAssign(par))
#define M1_MCDRV_CURR_3PH_CALIB_INIT(par) (MCDRV_Curr3Ph2ShCalibInit(par))
#define M1_MCDRV_CURR_3PH_CALIB(par) (MCDRV_Curr3Ph2ShCalib(par))
#define M1_MCDRV_CURR_3PH_CALIB_SET(par) (MCDRV_Curr3Ph2ShCalibSet(par))

#define M2_MCDRV_CURR_3PH_CHAN_ASSIGN(par) (MCDRV_Curr3Ph2ShChanAssign(par))
#define M2_MCDRV_CURR_3PH_CALIB_INIT(par) (MCDRV_Curr3Ph2ShCalibInit(par))
#define M2_MCDRV_CURR_3PH_CALIB(par) (MCDRV_Curr3Ph2ShCalib(par))
#define M2_MCDRV_CURR_3PH_CALIB_SET(par) (MCDRV_Curr3Ph2ShCalibSet(par))

    
/******************************************************************************
 * Define motor 1 3-ph driver control functions for motors 1, 2, 3 & 4
 ******************************************************************************/
#define M1_MCDRV_DRV3PH_RD_OC(par) (MCDRV_Driver3PhReadOc(par))
#define M1_MCDRV_DRV3PH_RD_INT(par) (MCDRV_Driver3PhReadInt(par))
#define M1_MCDRV_DRV3PH_CLR_FLG(par) (MCDRV_Driver3PhClearFlags(par))
#define M1_MCDRV_DRV3PH_RD_S0(par) (MCDRV_Driver3PhGetSr0(par))
#define M1_MCDRV_DRV3PH_RD_S1(par) (MCDRV_Driver3PhGetSr1(par))
#define M1_MCDRV_DRV3PH_RD_S2(par) (MCDRV_Driver3PhGetSr2(par))
#define M1_MCDRV_DRV3PH_RD_S3(par) (MCDRV_Driver3PhGetSr3(par))

#define M2_MCDRV_DRV3PH_RD_OC(par) (MCDRV_Driver3PhReadOc(par))
#define M2_MCDRV_DRV3PH_RD_INT(par) (MCDRV_Driver3PhReadInt(par))
#define M2_MCDRV_DRV3PH_CLR_FLG(par) (MCDRV_Driver3PhClearFlags(par))
#define M2_MCDRV_DRV3PH_RD_S0(par) (MCDRV_Driver3PhGetSr0(par))
#define M2_MCDRV_DRV3PH_RD_S1(par) (MCDRV_Driver3PhGetSr1(par))
#define M2_MCDRV_DRV3PH_RD_S2(par) (MCDRV_Driver3PhGetSr2(par))
#define M2_MCDRV_DRV3PH_RD_S3(par) (MCDRV_Driver3PhGetSr3(par))
    

/******************************************************************************
 * Define position and speed sensor - quadrature encoder for motors 1, 2, 3 & 4
 ******************************************************************************/
#define M1_MCDRV_QD_GET(par) (MCDRV_QdEncGet(par))
#define M1_MCDRV_QD_LED(par) (MCDRV_QdEncLedPosition(par))
#define M1_MCDRV_QD_SET_DIRECTION(par) (MCDRV_QdEncSetDirection(par))
#define M1_MCDRV_QD_CLEAR(par) (MCDRV_QdEncClear(par))
#define M1_MCDRV_QD_INDEX_IRQ_ON(par) (MCDRV_QdEncIndexIRQOn(par))

#define M2_MCDRV_QD_GET(par) (MCDRV_QdEncGet(par))
#define M2_MCDRV_QD_LED(par) (MCDRV_QdEncLedPosition(par))
#define M2_MCDRV_QD_SET_DIRECTION(par) (MCDRV_QdEncSetDirection(par))
#define M2_MCDRV_QD_CLEAR(par) (MCDRV_QdEncClear(par))
#define M2_MCDRV_QD_INDEX_IRQ_ON(par) (MCDRV_QdEncIndexIRQOn(par))
    
/******************************************************************************
 * Brake resistor definitions
 ******************************************************************************/
#define M1_BRAKE_SET() //GPIO2->DR |= GPIO_DR_DR(1U << 18U)
#define M1_BRAKE_CLEAR() //GPIO2->DR &= ~(GPIO_DR_DR(1U << 18U))  
    
#define M2_BRAKE_SET() //GPIO2->DR |= GPIO_DR_DR(1U << 18U)
#define M2_BRAKE_CLEAR() //GPIO2->DR &= ~(GPIO_DR_DR(1U << 18U))
           
/******************************************************************************
 * Global variable definitions
 ******************************************************************************/
extern mcdrv_adc_t g_sM1AdcSensor;
extern mcdrv_pwm3ph_pwma_t g_sM1Pwm3ph;
extern mcdrv_qd_enc_t g_sM1Enc;

extern mcdrv_adc_t g_sM2AdcSensor;
extern mcdrv_pwm3ph_pwma_t g_sM2Pwm3ph;
extern mcdrv_qd_enc_t g_sM2Enc;

extern clock_setup_t g_sClockSetup;

/*******************************************************************************
 * API
 ******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

void MCDRV_Init(void);
void InitClock(void);
void InitXBAR(void);
void InitADC(void);
void InitADC_ETC(void);
void InitTMR1(void);
void InitPIT(void);
void InitLED(void);

void M1_InitPWM(void);
void M1_InitQD(void);

void M2_InitPWM(void);
void M2_InitQD(void);

#ifdef __cplusplus
}
#endif

#endif /* _MCDRV_MIMXRT1050_EVK_H_  */
