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

#include "mcdrv_evkbimxrt.h"
#include "fsl_common.h"
#include "fsl_xbara.h"
#include "fsl_aoi.h"
#include "fsl_adc_etc.h"
#include "fsl_gpio.h"
#include "mcdrv_enc_qd.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/

/* Motor 1 */
/* Structure for current and voltage measurement */
mcdrv_adc_t g_sM1AdcSensor;
/* Structure for 3-phase PWM MC driver */
mcdrv_pwm3ph_pwma_t g_sM1Pwm3ph;
/* Structure for Encoder driver */
mcdrv_qd_enc_t g_sM1Enc;


/* Motor 2 */
/* Structure for current and voltage measurement */
mcdrv_adc_t g_sM2AdcSensor;
/* Structure for 3-phase PWM MC driver */
mcdrv_pwm3ph_pwma_t g_sM2Pwm3ph;
/* Structure for Encoder driver */
mcdrv_qd_enc_t g_sM2Enc;

/* Clock setup structure */
clock_setup_t g_sClockSetup;

//extern const clock_arm_pll_config_t armPllConfig;

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
* @brief   void MCDRV_Init(void)
*           - Motor control driver main initialization
*           - Calls initialization functions of peripherals required for motor
*             control functionality
*
* @param   void
*
* @return  none
*/
void MCDRV_Init(void)
{    
    /* Init application clock dependent variables */
    InitClock();
    
    /* Init ADC for M1, M2, M3 & M4*/
    InitADC();
        
    /* Init XBAR */
    InitXBAR();
    
    /* Init ADC_ETC */
    InitADC_ETC();
    
    /* Init TMR1 (slow loop counter) */
    InitTMR1();
        
    /* 6-channel PWM peripheral init for M1, M2, M3 & M4 */
    M1_InitPWM();
    M2_InitPWM();
    
    /* Qudrature decoder peripheral init */
//    M1_InitQD();
//    M2_InitQD();
    
    /* Init PIT for CPU Load measuring */
    InitPIT();
    
    /* Init LED  */
    InitLED();
    
    //InitCMP();
}

void InitCMP(void)
{
    /* Enable clock for CMP module */
    CLOCK_EnableClock(kCLOCK_Acmp3);

    /* Filter - 4 consecutive samples must agree */
    CMP3->CR0 = CMP_CR0_FILTER_CNT(4);

    /* DAC output set to 3.197V ~ 7.73A (for 8.25A scale) */
    /* Reference voltage will be VDD */
    /* Enable DAC */
    CMP3->DACCR = CMP_DACCR_VOSEL(60) | CMP_DACCR_VRSEL_MASK | CMP_DACCR_DACEN_MASK;

    /* Plus is CMP3_IN3 ~ overcurrent pin */
    /* Minus is CMP3_IN7 ~ 6bit reference */
    CMP3->MUXCR = CMP_MUXCR_PSEL(3) | CMP_MUXCR_MSEL(7);

    /* Enable analog comparator */
    CMP3->CR1 = CMP_CR1_EN_MASK;

    /* Enable clock for CMP module */
    CLOCK_EnableClock(kCLOCK_Acmp4);

    /* Filter - 4 consecutive samples must agree */
    CMP3->CR0 = CMP_CR0_FILTER_CNT(4);

    /* DAC output set to 3.197V ~ 7.73A (for 8.25A scale) */
    /* Reference voltage will be VDD */
    /* Enable DAC */
    CMP3->DACCR = CMP_DACCR_VOSEL(60) | CMP_DACCR_VRSEL_MASK | CMP_DACCR_DACEN_MASK;

    /* Plus is CMP3_IN3 ~ overcurrent pin */
    /* Minus is CMP3_IN7 ~ 6bit reference */
    CMP3->MUXCR = CMP_MUXCR_PSEL(3) | CMP_MUXCR_MSEL(7);

    /* Enable analog comparator */
    CMP3->CR1 = CMP_CR1_EN_MASK;
}

/*!
* @brief      Core, bus, flash clock setup
*
* @param      void
*
* @return     none
*/
void InitClock(void)
{  
    uint32_t ui32CyclesNumber = 0U;
    /* Calculate clock dependant variables for PMSM control algorithm */
    g_sClockSetup.ui32FastPeripheralClock = CLOCK_GetFreq(kCLOCK_IpgClk);
    g_sClockSetup.ui32SysPllClock = CLOCK_GetFreq(kCLOCK_SysPllClk);

    /* Parameters for motor M1 */
    g_sClockSetup.ui16M1PwmFreq = M1_PWM_FREQ; /* 10 kHz */
    g_sClockSetup.ui16M1PwmModulo = g_sClockSetup.ui32FastPeripheralClock / g_sClockSetup.ui16M1PwmFreq;
    ui32CyclesNumber = ((M1_PWM_DEADTIME * (g_sClockSetup.ui32FastPeripheralClock / 1000000U))/1000U); /* max 2047 cycles */
    g_sClockSetup.ui16M1PwmDeadTime = ui32CyclesNumber;
    g_sClockSetup.ui16M1SpeedLoopFreq = M1_SPEED_LOOP_FREQ; /* 1kHz */   
    
    /* Parameters for motor M2 */
    g_sClockSetup.ui16M2PwmFreq = M2_PWM_FREQ; /* 10 kHz */
    g_sClockSetup.ui16M2PwmModulo = g_sClockSetup.ui32FastPeripheralClock / g_sClockSetup.ui16M2PwmFreq;
    ui32CyclesNumber = ((M2_PWM_DEADTIME * (g_sClockSetup.ui32FastPeripheralClock / 1000000U))/1000U); /* max 2047 cycles */
    g_sClockSetup.ui16M2PwmDeadTime = ui32CyclesNumber;
    g_sClockSetup.ui16M2SpeedLoopFreq = M2_SPEED_LOOP_FREQ; /* 1kHz */   
    
}

/*!
* @brief   void InitADC(void)
*           - Initialization of the ADC peripheral for both motors
*          
* @param   void
*
* @return  none
*/
void InitADC(void)
{       
    /* Enable clock to ADC module 1 */
    CLOCK_EnableClock(kCLOCK_Adc1); 
    /* Enable clock to ADC module 2 */
    CLOCK_EnableClock(kCLOCK_Adc2);
                     
    /* Single-ended 12-bit conversion (MODE = 0x1) */
    /* Set divide ratio to 1 (ADIV = 0x0) */
    /* 4 samples averaged (AVGS = 0x0) */
    /* IPG clock select (ADICLK = 0x0) */
    ADC1->CFG = ( ADC_CFG_ADICLK(0U) | ADC_CFG_MODE(1U) | ADC_CFG_ADIV(1U) | ADC_CFG_AVGS(0U) | ADC_CFG_ADTRG(0U) );
    ADC2->CFG = ( ADC_CFG_ADICLK(0U) | ADC_CFG_MODE(1U) | ADC_CFG_ADIV(1U) | ADC_CFG_AVGS(0U) | ADC_CFG_ADTRG(0U) );

    /* HW averaging disabled (AVGE = 0) */
    /* One conversion or one set of conversion (ADCO = 0) */ 
    ADC1->GC = (ADC_GC_AVGE(0U) | ADC_GC_ADCO(0U));
    ADC2->GC = (ADC_GC_AVGE(0U) | ADC_GC_ADCO(0U));
    
    /* Asynchronous clock output disabled */
    ADC1->GC |= ADC_GC_ADACKEN(0U); 
    ADC2->GC |= ADC_GC_ADACKEN(0U);
    
    /* ------- ADC self calibration procedure start ----- */ 
    /* Starting the calibration of ADC1 */
    /* Clear the CALF and launch the calibration. */
    ADC1->GS = ADC_GS_CALF_MASK; /* Clear the CALF. */
    ADC1->GC |= ADC_GC_CAL_MASK; /* Launch the calibration. */
      
    /* Check the status of CALF bit in ADC_GS and the CAL bit in ADC_GC. */
    while (0U != (ADC1->GC & ADC_GC_CAL_MASK))
    {
        /* Check the CALF when the calibration is active. */
        if (0U != (ADC1->GS & ADC_GS_CALF_MASK))
        {
            /* Breakpoint. */
            __asm("BKPT 1");
            break;
        }
    }
    
    /* Starting the calibration of ADC2 */
    /* Clear the CALF and launch the calibration. */
    ADC2->GS = ADC_GS_CALF_MASK; /* Clear the CALF. */
    ADC2->GC |= ADC_GC_CAL_MASK; /* Launch the calibration. */
      
    /* Check the status of CALF bit in ADC_GS and the CAL bit in ADC_GC. */
    while (0U != (ADC2->GC & ADC_GC_CAL_MASK))
    {
        /* Check the CALF when the calibration is active. */
        if (0U != (ADC2->GS & ADC_GS_CALF_MASK))
        {
            /* Breakpoint. */
            __asm("BKPT 1");
            break;
        }
    }
    
    /* ------- ADC self calibration procedure end ----- */   
    /* hardware trigger selected */
    ADC1->CFG |= ADC_CFG_ADTRG(1U);
    /* hardware trigger selected */
    ADC2->CFG |= ADC_CFG_ADTRG(1U);
    
    /* Set ADC channels for ADC_ETC */
    /* Withouth this setting ADC_ETC not working properly!!! */  
    ADC1->HC[0] = ADC_HC_ADCH(0x01U);
    ADC1->HC[1] = ADC_HC_ADCH(0x01U);
    ADC1->HC[2] = ADC_HC_ADCH(0x01U);
    ADC1->HC[3] = ADC_HC_ADCH(0x01U);
    ADC1->HC[4] = ADC_HC_ADCH(0x01U);
    ADC1->HC[5] = ADC_HC_ADCH(0x01U);
    ADC1->HC[5] |= ADC_HC_AIEN(1U); // AIEN = 1
    
    ADC2->HC[0] = ADC_HC_ADCH(0x02U);
    ADC2->HC[1] = ADC_HC_ADCH(0x02U);
    ADC2->HC[2] = ADC_HC_ADCH(0x02U);
    ADC2->HC[3] = ADC_HC_ADCH(0x02U);
    ADC2->HC[4] = ADC_HC_ADCH(0x02U);
    ADC2->HC[5] = ADC_HC_ADCH(0x02U);
     
    /**************************************/
    /* motor M1 ADC driver initialization */
    /**************************************/
    /* offset filter window */
    g_sM1AdcSensor.ui16OffsetFiltWindow = ADC_OFFSET_WINDOW;      
    /* Phase current measurement */
    /* Sector 1,6 - measured currents Ic & Ib */
    /* ADC1, channel Ic = M1_ADC1_PH_C, , HCRegister (Result)  = 0 */
    g_sM1AdcSensor.sCurrSec16.pAdcBasePhaC = (ADC_Type *)ADC1;
    g_sM1AdcSensor.sCurrSec16.ui16ChanNumPhaC = M1_ADC1_PH_C;
    g_sM1AdcSensor.sCurrSec16.ui16HCRegPhaC = 0U;
    /* ADC2, channel Ib = M1_ADC2_PH_B, HCRegister (Result)  = 0 */
    g_sM1AdcSensor.sCurrSec16.pAdcBasePhaB = (ADC_Type *)ADC2;
    g_sM1AdcSensor.sCurrSec16.ui16ChanNumPhaB = M1_ADC2_PH_B;
    g_sM1AdcSensor.sCurrSec16.ui16HCRegPhaB = 0U;
    /* Sector 2,3 - measured currents Ic & Ia*/
    /* ADC1, channel Ic = M1_ADC1_PH_C, HCRegister (Result)  = 0 */
    g_sM1AdcSensor.sCurrSec23.pAdcBasePhaC = (ADC_Type *)ADC1;
    g_sM1AdcSensor.sCurrSec23.ui16ChanNumPhaC = M1_ADC1_PH_C;
    g_sM1AdcSensor.sCurrSec23.ui16HCRegPhaC = 0U;
    /* ADC2, channel Ia = M1_ADC2_PH_A, HCRegister (Result)  = 0 */
    g_sM1AdcSensor.sCurrSec23.pAdcBasePhaA = (ADC_Type *)ADC2;
    g_sM1AdcSensor.sCurrSec23.ui16ChanNumPhaA = M1_ADC2_PH_A;
    g_sM1AdcSensor.sCurrSec23.ui16HCRegPhaA = 0U;
    /* Sector 4,5 - measured currents Ia & Ib */
    /* ADC1, channel Ia = M1_ADC1_PH_A, HCRegister (Result)  = 0 */
    g_sM1AdcSensor.sCurrSec45.pAdcBasePhaA = (ADC_Type *)ADC1;
    g_sM1AdcSensor.sCurrSec45.ui16ChanNumPhaA = M1_ADC1_PH_A;
    g_sM1AdcSensor.sCurrSec45.ui16HCRegPhaA = 0U;
    /* ADC2, channel Ib = M1_ADC2_PH_B, HCRegister (Result)  = 0 */
    g_sM1AdcSensor.sCurrSec45.pAdcBasePhaB = (ADC_Type *)ADC2;
    g_sM1AdcSensor.sCurrSec45.ui16ChanNumPhaB = M1_ADC2_PH_B;
    g_sM1AdcSensor.sCurrSec45.ui16HCRegPhaB = 0U;    
    /* UDCbus channel measurement */
    /* ADC1, channel Udcb = M1_ADC1_UDCB, HCRegister (Result)  = 2 */
    g_sM1AdcSensor.pui32UdcbAdcBase = (ADC_Type *)ADC1;
    g_sM1AdcSensor.ui16ChanNumVDcb = M1_ADC1_UDCB;
    g_sM1AdcSensor.ui16HCRegVDcb = 2U;     
    /* Assign channels and init all pointers */
    MCDRV_Curr3Ph2ShChanAssignInit(&g_sM1AdcSensor);
  
    /**************************************/
    /* motor M2 ADC driver initialization */
    /**************************************/
    /* offset filter window */
    g_sM2AdcSensor.ui16OffsetFiltWindow = ADC_OFFSET_WINDOW;
    /* Phase current measurement */
    /* Sector 1,6 - measured currents Ic & Ib */
    /* ADC1, channel Ic = M2_ADC1_PH_C, HCRegister (Result)  = 0 */
    g_sM2AdcSensor.sCurrSec16.pAdcBasePhaC = (ADC_Type *)ADC1;
    g_sM2AdcSensor.sCurrSec16.ui16ChanNumPhaC = M2_ADC1_PH_C;
    g_sM2AdcSensor.sCurrSec16.ui16HCRegPhaC = 3U;
    /* ADC2, channel Ib = M2_ADC2_PH_B, HCRegister (Result)  = 0 */
    g_sM2AdcSensor.sCurrSec16.pAdcBasePhaB = (ADC_Type *)ADC2;
    g_sM2AdcSensor.sCurrSec16.ui16ChanNumPhaB = M2_ADC2_PH_B;
    g_sM2AdcSensor.sCurrSec16.ui16HCRegPhaB = 3U;
    /* Sector 2,3 - measured currents Ic & Ia*/
    /* ADC1, channel Ic = M2_ADC1_PH_C, HCRegister (Result)  = 0 */
    g_sM2AdcSensor.sCurrSec23.pAdcBasePhaC = (ADC_Type *)ADC1;
    g_sM2AdcSensor.sCurrSec23.ui16ChanNumPhaC = M2_ADC1_PH_C;
    g_sM2AdcSensor.sCurrSec23.ui16HCRegPhaC = 3U;
    /* ADC2, channel Ia = M2_ADC2_PH_A, HCRegister (Result)  = 0 */
    g_sM2AdcSensor.sCurrSec23.pAdcBasePhaA = (ADC_Type *)ADC2;
    g_sM2AdcSensor.sCurrSec23.ui16ChanNumPhaA = M2_ADC2_PH_A;
    g_sM2AdcSensor.sCurrSec23.ui16HCRegPhaA = 3U;
    /* Sector 4,5 - measured currents Ia & Ib */
    /* ADC1, channel Ia = M2_ADC1_PH_A, HCRegister (Result)  = 0 */
    g_sM2AdcSensor.sCurrSec45.pAdcBasePhaA = (ADC_Type *)ADC1;
    g_sM2AdcSensor.sCurrSec45.ui16ChanNumPhaA = M2_ADC1_PH_A;
    g_sM2AdcSensor.sCurrSec45.ui16HCRegPhaA = 3U;
    /* ADC2, channel Ib = M2_ADC2_PH_B, HCRegister (Result)  = 0 */
    g_sM2AdcSensor.sCurrSec45.pAdcBasePhaB = (ADC_Type *)ADC2;
    g_sM2AdcSensor.sCurrSec45.ui16ChanNumPhaB = M2_ADC2_PH_B;
    g_sM2AdcSensor.sCurrSec45.ui16HCRegPhaB = 3U;    
    /* UDCbus channel measurement */
    /* ADC2, channel Udcb = M2_ADC2_UDCB, HCRegister (Result)  = 2 */
    g_sM2AdcSensor.pui32UdcbAdcBase = (ADC_Type *)ADC1;
    g_sM2AdcSensor.ui16ChanNumVDcb = M2_ADC1_UDCB;
    g_sM2AdcSensor.ui16HCRegVDcb = 5U;    
    /* Assign channels and init all pointers */
    MCDRV_Curr3Ph2ShChanAssignInit(&g_sM2AdcSensor); 

    /* Enable & setup interrupt from ADC1 */
    EnableIRQ(ADC1_IRQn);
    NVIC_SetPriority(ADC1_IRQn, 1U); 
         
}

/*!
* @brief   void InitTMR1(void)
*           - Initialization of the TMR1 peripheral
*           - Performs slow control loop counter
*
* @param   void
*
* @return  none
*/
void InitTMR1(void)
{    
    uint16_t    ui16SpeedLoopFreq = g_sClockSetup.ui16M1SpeedLoopFreq;
    uint32_t    ui32FastPeripheralClock = g_sClockSetup.ui32FastPeripheralClock;
    uint16_t    ui16CompareReg = (ui32FastPeripheralClock/(16U * ui16SpeedLoopFreq)); 
  
    /* Enable clock to TMR1 module */
    CLOCK_EnableClock(kCLOCK_Timer1);
    
    /* TMR0_CTRL: CM=0,PCS=0,SCS=0,ONCE=0,LENGTH=1,DIR=0,COINIT=0,OUTMODE=0 */
    /* Stop all functions of the timer */
    TMR1->CHANNEL[0].CTRL = 0x20;
    
    /* TMR0_SCTRL: TCF=0,TCFIE=0,TOF=0,TOFIE=0,IEF=0,IEFIE=0,IPS=0,INPUT=0,
    Capture_Mode=0,MSTR=0,EEOF=0,VAL=0,FORCE=0,OPS=0,OEN=0 */
    TMR1->CHANNEL[0].SCTRL = 0x00;    
    TMR1->CHANNEL[0].LOAD = 0x00; /* Reset load register */
    
    TMR1->CHANNEL[0].COMP1 = ui16CompareReg; /* Set up compare 1 register */
    TMR1->CHANNEL[0].CMPLD1 = ui16CompareReg; /* Also set the compare preload register */
    
    /* TMR0_CSCTRL: DBG_EN=0,FAULT=0,ALT_LOAD=0,ROC=0,TCI=0,UP=0,OFLAG=0,TCF2EN=0,TCF1EN=1,
    TCF2=0,TCF1=0,CL2=0,CL1=1 */
    /* Enable compare 1 interrupt and compare 1 preload*/
    TMR1->CHANNEL[0].CSCTRL = 0x41; 
    
    /* Primary Count Source to IP_bus_clk */
    TMR1->CHANNEL[0].CTRL |= TMR_CTRL_PCS(0x0C);  /* Frequency = IP_bus clock/PCS */

    /* Reset counter register */
    TMR1->CHANNEL[0].CNTR = 0x00;
    
    /* Run counter */
    TMR1->CHANNEL[0].CTRL |= TMR_CTRL_CM(0x01);
     
    /* Enable & setup interrupt from TMR1 */
    EnableIRQ(TMR1_IRQn);   
    NVIC_SetPriority(TMR1_IRQn, 3U);
}

/*!
* @brief   void InitPWM_M1(void)
*           - Initialization of the eFlexPWMA peripheral for motor M1
*           - 3-phase center-aligned PWM
*           - Top signals have negative polarity due to MC33937
*
* @param   void
*
* @return  none
*/
void M1_InitPWM(void)
{ 
    /* PWM base pointer (affects the entire initialization) */
    PWM_Type *PWMBase = (PWM_Type *)PWM2;
  
    /* PWM clock gating register: enabled */
    CCM->CCGR4 = (CCM->CCGR4 & ~(CCM_CCGR4_CG9_MASK)) | CCM_CCGR4_CG9(0x3);
  
    /* Full cycle reload */
    PWMBase->SM[0].CTRL |= PWM_CTRL_FULL_MASK;
    PWMBase->SM[1].CTRL |= PWM_CTRL_FULL_MASK;
    PWMBase->SM[2].CTRL |= PWM_CTRL_FULL_MASK;

    /* Value register initial values, duty cycle 50% */
    PWMBase->SM[0].INIT = PWM_INIT_INIT((uint16_t)(-(g_sClockSetup.ui16M1PwmModulo / 2U)));
    PWMBase->SM[1].INIT = PWM_INIT_INIT((uint16_t)(-(g_sClockSetup.ui16M1PwmModulo / 2U)));
    PWMBase->SM[2].INIT = PWM_INIT_INIT((uint16_t)(-(g_sClockSetup.ui16M1PwmModulo / 2U)));

    PWMBase->SM[0].VAL0 = PWM_VAL0_VAL0((uint16_t)(0U));
    PWMBase->SM[1].VAL0 = PWM_VAL0_VAL0((uint16_t)(0U));
    PWMBase->SM[2].VAL0 = PWM_VAL0_VAL0((uint16_t)(0U));
    
    PWMBase->SM[0].VAL1 = PWM_VAL1_VAL1((uint16_t)((g_sClockSetup.ui16M1PwmModulo / 2U) - 1U));
    PWMBase->SM[1].VAL1 = PWM_VAL1_VAL1((uint16_t)((g_sClockSetup.ui16M1PwmModulo / 2U) - 1U));
    PWMBase->SM[2].VAL1 = PWM_VAL1_VAL1((uint16_t)((g_sClockSetup.ui16M1PwmModulo / 2U) - 1U));
   
    PWMBase->SM[0].VAL2 = PWM_VAL2_VAL2((uint16_t)(-(g_sClockSetup.ui16M1PwmModulo / 4U)));
    PWMBase->SM[1].VAL2 = PWM_VAL2_VAL2((uint16_t)(-(g_sClockSetup.ui16M1PwmModulo / 4U)));
    PWMBase->SM[2].VAL2 = PWM_VAL2_VAL2((uint16_t)(-(g_sClockSetup.ui16M1PwmModulo / 4U)));
 
    PWMBase->SM[0].VAL3 = PWM_VAL3_VAL3((uint16_t)(g_sClockSetup.ui16M1PwmModulo / 4U));
    PWMBase->SM[1].VAL3 = PWM_VAL3_VAL3((uint16_t)(g_sClockSetup.ui16M1PwmModulo / 4U));
    PWMBase->SM[2].VAL3 = PWM_VAL3_VAL3((uint16_t)(g_sClockSetup.ui16M1PwmModulo / 4U));

    PWMBase->SM[0].VAL4 = PWM_VAL4_VAL4((uint16_t)((-(g_sClockSetup.ui16M1PwmModulo / 2U) + 10U)));
    PWMBase->SM[1].VAL4 = PWM_VAL4_VAL4((uint16_t)((-(g_sClockSetup.ui16M1PwmModulo / 2U) + 10U)));
    PWMBase->SM[2].VAL4 = PWM_VAL4_VAL4((uint16_t)(0U));

    PWMBase->SM[0].VAL5 = PWM_VAL5_VAL5((uint16_t)(0U));
    PWMBase->SM[1].VAL5 = PWM_VAL5_VAL5((uint16_t)(0U));
    PWMBase->SM[2].VAL5 = PWM_VAL5_VAL5((uint16_t)(0U));

    /* PWM2 module 0 trigger on VAL4 enabled for ADC2 synchronization */
    PWMBase->SM[0].TCTRL |= PWM_TCTRL_OUT_TRIG_EN(1 << 4); // kXBARA1_InputFlexpwm2Pwm1OutTrig01 ADC TRIGGER
        
    /* Set dead-time register */
    PWMBase->SM[0].DTCNT0 = PWM_DTCNT0_DTCNT0(g_sClockSetup.ui16M1PwmDeadTime);
    PWMBase->SM[1].DTCNT0 = PWM_DTCNT0_DTCNT0(g_sClockSetup.ui16M1PwmDeadTime);
    PWMBase->SM[2].DTCNT0 = PWM_DTCNT0_DTCNT0(g_sClockSetup.ui16M1PwmDeadTime);
    PWMBase->SM[0].DTCNT1 = PWM_DTCNT1_DTCNT1(g_sClockSetup.ui16M1PwmDeadTime);
    PWMBase->SM[1].DTCNT1 = PWM_DTCNT1_DTCNT1(g_sClockSetup.ui16M1PwmDeadTime);
    PWMBase->SM[2].DTCNT1 = PWM_DTCNT1_DTCNT1(g_sClockSetup.ui16M1PwmDeadTime);

    /* Channels A and B disabled when fault 0 occurs */
    PWMBase->SM[0].DISMAP[0] = ((PWMBase->SM[0].DISMAP[0] & ~PWM_DISMAP_DIS0A_MASK) | PWM_DISMAP_DIS0A(0x1));
    PWMBase->SM[1].DISMAP[0] = ((PWMBase->SM[0].DISMAP[0] & ~PWM_DISMAP_DIS0A_MASK) | PWM_DISMAP_DIS0A(0x1));
    PWMBase->SM[2].DISMAP[0] = ((PWMBase->SM[0].DISMAP[0] & ~PWM_DISMAP_DIS0A_MASK) | PWM_DISMAP_DIS0A(0x1));
    PWMBase->SM[0].DISMAP[0] = ((PWMBase->SM[0].DISMAP[0] & ~PWM_DISMAP_DIS0B_MASK) | PWM_DISMAP_DIS0B(0x1));
    PWMBase->SM[1].DISMAP[0] = ((PWMBase->SM[0].DISMAP[0] & ~PWM_DISMAP_DIS0B_MASK) | PWM_DISMAP_DIS0B(0x1));
    PWMBase->SM[2].DISMAP[0] = ((PWMBase->SM[0].DISMAP[0] & ~PWM_DISMAP_DIS0B_MASK) | PWM_DISMAP_DIS0B(0x1));

    /* Modules one and two gets clock from module zero */
    PWMBase->SM[1].CTRL2 = (PWMBase->SM[1].CTRL2 & ~PWM_CTRL2_CLK_SEL_MASK) | PWM_CTRL2_CLK_SEL(0x2);
    PWMBase->SM[2].CTRL2 = (PWMBase->SM[2].CTRL2 & ~PWM_CTRL2_CLK_SEL_MASK) | PWM_CTRL2_CLK_SEL(0x2);

    /* Master reload active for modules one and two*/
    PWMBase->SM[1].CTRL2 |= PWM_CTRL2_RELOAD_SEL_MASK;
    PWMBase->SM[2].CTRL2 |= PWM_CTRL2_RELOAD_SEL_MASK;

    /* Master reload is generated every PWM opportunity */
    PWMBase->SM[0].CTRL = (PWMBase->SM[0].CTRL & ~PWM_CTRL_LDFQ_MASK) | PWM_CTRL_LDFQ(M1_FOC_FREQ_VS_PWM_FREQ - 1);

    /* Master sync active for modules one and two*/
    PWMBase->SM[1].CTRL2 = (PWMBase->SM[1].CTRL2 & ~PWM_CTRL2_INIT_SEL_MASK) | PWM_CTRL2_INIT_SEL(0x2);
    PWMBase->SM[2].CTRL2 = (PWMBase->SM[2].CTRL2 & ~PWM_CTRL2_INIT_SEL_MASK) | PWM_CTRL2_INIT_SEL(0x2);

    /* Fault 0 active in logic level one, automatic clearing */
    PWMBase->FCTRL = (PWMBase->FCTRL & ~PWM_FCTRL_FLVL_MASK) | PWM_FCTRL_FLVL(0x1);
    PWMBase->FCTRL = (PWMBase->FCTRL & ~PWM_FCTRL_FAUTO_MASK) | PWM_FCTRL_FAUTO(0x1);

    /* Clear fault flags */
    PWMBase->FSTS = (PWMBase->FCTRL & ~PWM_FSTS_FFLAG_MASK) | PWM_FSTS_FFLAG(0xF);

    /* PWMs are re-enabled at PWM full cycle */
    PWMBase->FSTS = (PWMBase->FSTS & ~PWM_FSTS_FFULL_MASK) | PWM_FSTS_FFULL(0x1);

    /* PWM fault filter - 5 Fast peripheral clocks sample rate, 5 agreeing
       samples to activate */
    PWMBase->FFILT = (PWMBase->FFILT & ~PWM_FFILT_FILT_PER_MASK) | PWM_FFILT_FILT_PER(5U);
    PWMBase->FFILT = (PWMBase->FFILT & ~PWM_FFILT_FILT_CNT_MASK) | PWM_FFILT_FILT_CNT(5U);

    /* Start PWMs (set load OK flags and run) */
    PWMBase->MCTRL = (PWMBase->MCTRL & ~PWM_MCTRL_CLDOK_MASK) | PWM_MCTRL_CLDOK(0xF);
    PWMBase->MCTRL = (PWMBase->MCTRL & ~PWM_MCTRL_LDOK_MASK) | PWM_MCTRL_LDOK(0xF);
    PWMBase->MCTRL = (PWMBase->MCTRL & ~PWM_MCTRL_RUN_MASK) | PWM_MCTRL_RUN(0xF);
    
    /* Initialize MC driver */
    g_sM1Pwm3ph.pui32PwmBaseAddress = (PWM_Type *)PWMBase;

    g_sM1Pwm3ph.ui16PhASubNum = M1_PWM_PAIR_PHA; /* PWMA phase A sub-module number */
    g_sM1Pwm3ph.ui16PhBSubNum = M1_PWM_PAIR_PHB; /* PWMA phase B sub-module number */
    g_sM1Pwm3ph.ui16PhCSubNum = M1_PWM_PAIR_PHC; /* PWMA phase C sub-module number */

    g_sM1Pwm3ph.ui16FaultFixNum = M1_FAULT_NUM; /* PWMA fixed-value over-current fault number */
    g_sM1Pwm3ph.ui16FaultAdjNum = M1_FAULT_NUM; /* PWMA adjustable over-current fault number */
}

/*!
* @brief   void InitPWM_M2(void)
*           - Initialization of the eFlexPWMA peripheral for motor M2
*           - 3-phase center-aligned PWM
*           - Top signals have negative polarity due to MC33937
*
* @param   void
*
* @return  none
*/
void M2_InitPWM(void)
{    
    /* PWM base pointer (affects the entire initialization) */
    PWM_Type *PWMBase = (PWM_Type *)PWM1;

    /* PWM clock gating register: enabled */
    CCM->CCGR4 = (CCM->CCGR4 & ~(CCM_CCGR4_CG8_MASK)) | CCM_CCGR4_CG8(0x3);
  
    /* Full cycle reload */
    PWMBase->SM[0].CTRL |= PWM_CTRL_FULL_MASK;
    PWMBase->SM[1].CTRL |= PWM_CTRL_FULL_MASK;
    PWMBase->SM[2].CTRL |= PWM_CTRL_FULL_MASK;

    /* Value register initial values, duty cycle 50% */
    PWMBase->SM[0].INIT = PWM_INIT_INIT((uint16_t)(-(g_sClockSetup.ui16M2PwmModulo / 2U)));
    PWMBase->SM[1].INIT = PWM_INIT_INIT((uint16_t)(-(g_sClockSetup.ui16M2PwmModulo / 2U)));
    PWMBase->SM[2].INIT = PWM_INIT_INIT((uint16_t)(-(g_sClockSetup.ui16M2PwmModulo / 2U)));

    PWMBase->SM[0].VAL0 = PWM_VAL0_VAL0((uint16_t)(0U));
    PWMBase->SM[1].VAL0 = PWM_VAL0_VAL0((uint16_t)(0U));
    PWMBase->SM[2].VAL0 = PWM_VAL0_VAL0((uint16_t)(0U));

    PWMBase->SM[0].VAL1 = PWM_VAL1_VAL1((uint16_t)((g_sClockSetup.ui16M2PwmModulo / 2U) - 1U));
    PWMBase->SM[1].VAL1 = PWM_VAL1_VAL1((uint16_t)((g_sClockSetup.ui16M2PwmModulo / 2U) - 1U));
    PWMBase->SM[2].VAL1 = PWM_VAL1_VAL1((uint16_t)((g_sClockSetup.ui16M2PwmModulo / 2U) - 1U));

    PWMBase->SM[0].VAL2 = PWM_VAL2_VAL2((uint16_t)(-(g_sClockSetup.ui16M2PwmModulo / 4U)));
    PWMBase->SM[1].VAL2 = PWM_VAL2_VAL2((uint16_t)(-(g_sClockSetup.ui16M2PwmModulo / 4U)));
    PWMBase->SM[2].VAL2 = PWM_VAL2_VAL2((uint16_t)(-(g_sClockSetup.ui16M2PwmModulo / 4U)));

    PWMBase->SM[0].VAL3 = PWM_VAL3_VAL3((uint16_t)(g_sClockSetup.ui16M2PwmModulo / 4U));
    PWMBase->SM[1].VAL3 = PWM_VAL3_VAL3((uint16_t)(g_sClockSetup.ui16M2PwmModulo / 4U));
    PWMBase->SM[2].VAL3 = PWM_VAL3_VAL3((uint16_t)(g_sClockSetup.ui16M2PwmModulo / 4U));

    PWMBase->SM[0].VAL4 = PWM_VAL4_VAL4((uint16_t)((-(g_sClockSetup.ui16M2PwmModulo / 2U) + 10U)));
    PWMBase->SM[1].VAL4 = PWM_VAL4_VAL4((uint16_t)(0U));
    PWMBase->SM[2].VAL4 = PWM_VAL4_VAL4((uint16_t)(0U));

    PWMBase->SM[0].VAL5 = PWM_VAL5_VAL5((uint16_t)(0U));
    PWMBase->SM[1].VAL5 = PWM_VAL5_VAL5((uint16_t)(0U));
    PWMBase->SM[2].VAL5 = PWM_VAL5_VAL5((uint16_t)(0U));

    /* PWM1 module 0 trigger on VAL4 enabled for ADC synchronization */
    PWMBase->SM[0].TCTRL |= PWM_TCTRL_OUT_TRIG_EN(1 << 4); /// kXBARA1_InputFlexpwm1Pwm1OutTrig01 - ADC TRIGGER & SYNCHRONIZATION FOR PWM3
    /* PWM1 module 1 trigger on VAL0 enabled for PWM synchronization */
    PWMBase->SM[1].TCTRL |= PWM_TCTRL_OUT_TRIG_EN(1 << 0); /// kXBARA1_InputFlexpwm1Pwm2OutTrig01 - SYNCHRONIZATION FOR PWM2 and PWM4

    /* Set dead-time register */
    PWMBase->SM[0].DTCNT0 = PWM_DTCNT0_DTCNT0(g_sClockSetup.ui16M2PwmDeadTime);
    PWMBase->SM[1].DTCNT0 = PWM_DTCNT0_DTCNT0(g_sClockSetup.ui16M2PwmDeadTime);
    PWMBase->SM[2].DTCNT0 = PWM_DTCNT0_DTCNT0(g_sClockSetup.ui16M2PwmDeadTime);
    PWMBase->SM[0].DTCNT1 = PWM_DTCNT1_DTCNT1(g_sClockSetup.ui16M2PwmDeadTime);
    PWMBase->SM[1].DTCNT1 = PWM_DTCNT1_DTCNT1(g_sClockSetup.ui16M2PwmDeadTime);
    PWMBase->SM[2].DTCNT1 = PWM_DTCNT1_DTCNT1(g_sClockSetup.ui16M2PwmDeadTime);

    /* Channels A and B disabled when fault 0 occurs */
    PWMBase->SM[0].DISMAP[0] = ((PWMBase->SM[0].DISMAP[0] & ~PWM_DISMAP_DIS0A_MASK) | PWM_DISMAP_DIS0A(0x1));
    PWMBase->SM[1].DISMAP[0] = ((PWMBase->SM[0].DISMAP[0] & ~PWM_DISMAP_DIS0A_MASK) | PWM_DISMAP_DIS0A(0x1));
    PWMBase->SM[2].DISMAP[0] = ((PWMBase->SM[0].DISMAP[0] & ~PWM_DISMAP_DIS0A_MASK) | PWM_DISMAP_DIS0A(0x1));
    PWMBase->SM[0].DISMAP[0] = ((PWMBase->SM[0].DISMAP[0] & ~PWM_DISMAP_DIS0B_MASK) | PWM_DISMAP_DIS0B(0x1));
    PWMBase->SM[1].DISMAP[0] = ((PWMBase->SM[0].DISMAP[0] & ~PWM_DISMAP_DIS0B_MASK) | PWM_DISMAP_DIS0B(0x1));
    PWMBase->SM[2].DISMAP[0] = ((PWMBase->SM[0].DISMAP[0] & ~PWM_DISMAP_DIS0B_MASK) | PWM_DISMAP_DIS0B(0x1));

    /* Modules one and two gets clock from module zero */
    PWMBase->SM[1].CTRL2 = (PWMBase->SM[1].CTRL2 & ~PWM_CTRL2_CLK_SEL_MASK) | PWM_CTRL2_CLK_SEL(0x2);
    PWMBase->SM[2].CTRL2 = (PWMBase->SM[2].CTRL2 & ~PWM_CTRL2_CLK_SEL_MASK) | PWM_CTRL2_CLK_SEL(0x2);

    /* Master reload active for modules one and two*/
    PWMBase->SM[1].CTRL2 |= PWM_CTRL2_RELOAD_SEL_MASK;
    PWMBase->SM[2].CTRL2 |= PWM_CTRL2_RELOAD_SEL_MASK;

    /* Master reload is generated every PWM opportunity */
    PWMBase->SM[0].CTRL = (PWMBase->SM[0].CTRL & ~PWM_CTRL_LDFQ_MASK) | PWM_CTRL_LDFQ(M1_FOC_FREQ_VS_PWM_FREQ - 1);
    
    /* External synchronization for submodule 0 */
    PWMBase->SM[0].CTRL2 = (PWMBase->SM[0].CTRL2 & ~PWM_CTRL2_INIT_SEL_MASK) | PWM_CTRL2_INIT_SEL(0x3); //  EXTERNAL SYNCHRONIZATION FROM PWM1 

    /* Master sync active for modules one and two*/
    PWMBase->SM[1].CTRL2 = (PWMBase->SM[1].CTRL2 & ~PWM_CTRL2_INIT_SEL_MASK) | PWM_CTRL2_INIT_SEL(0x2);
    PWMBase->SM[2].CTRL2 = (PWMBase->SM[2].CTRL2 & ~PWM_CTRL2_INIT_SEL_MASK) | PWM_CTRL2_INIT_SEL(0x2);
  
    /* Fault 0 active in logic level one, automatic clearing */
    PWMBase->FCTRL = (PWMBase->FCTRL & ~PWM_FCTRL_FLVL_MASK) | PWM_FCTRL_FLVL(0x1);
    PWMBase->FCTRL = (PWMBase->FCTRL & ~PWM_FCTRL_FAUTO_MASK) | PWM_FCTRL_FAUTO(0x1);

    /* Clear fault flags */
    PWMBase->FSTS = (PWMBase->FCTRL & ~PWM_FSTS_FFLAG_MASK) | PWM_FSTS_FFLAG(0xF);

    /* PWMs are re-enabled at PWM full cycle */
    PWMBase->FSTS = (PWMBase->FSTS & ~PWM_FSTS_FFULL_MASK) | PWM_FSTS_FFULL(0x1);

    /* PWM fault filter - 5 Fast peripheral clocks sample rate, 5 agreeing samples to activate */
    PWMBase->FFILT = (PWMBase->FFILT & ~PWM_FFILT_FILT_PER_MASK) | PWM_FFILT_FILT_PER(5U);
    PWMBase->FFILT = (PWMBase->FFILT & ~PWM_FFILT_FILT_CNT_MASK) | PWM_FFILT_FILT_CNT(5U);

    /* Start PWMs (set load OK flags and run) */
    PWMBase->MCTRL = (PWMBase->MCTRL & ~PWM_MCTRL_CLDOK_MASK) | PWM_MCTRL_CLDOK(0xF);
    PWMBase->MCTRL = (PWMBase->MCTRL & ~PWM_MCTRL_LDOK_MASK) | PWM_MCTRL_LDOK(0xF);
    PWMBase->MCTRL = (PWMBase->MCTRL & ~PWM_MCTRL_RUN_MASK) | PWM_MCTRL_RUN(0xF);
    
    /* Initialize MC driver */
    g_sM2Pwm3ph.pui32PwmBaseAddress = (PWM_Type *)PWMBase;

    g_sM2Pwm3ph.ui16PhASubNum = M2_PWM_PAIR_PHA; /* PWMA phase A sub-module number */
    g_sM2Pwm3ph.ui16PhBSubNum = M2_PWM_PAIR_PHB; /* PWMA phase B sub-module number */
    g_sM2Pwm3ph.ui16PhCSubNum = M2_PWM_PAIR_PHC; /* PWMA phase C sub-module number */

    g_sM2Pwm3ph.ui16FaultFixNum = M2_FAULT_NUM; /* PWMA fixed-value over-current fault number */
    g_sM2Pwm3ph.ui16FaultAdjNum = M2_FAULT_NUM; /* PWMA adjustable over-current fault number */
}


/*!
* @brief   void InitXBARA(void)
*           - Initialization of the XBARA peripheral
*          
* @param   void
*
* @return  none
*/
void InitXBAR(void)
{
    /* Init xbara module. */
    XBARA_Init(XBARA);
    
    /* PWM synchronization - From PWM1 SM[1] (VAL4) to PWM2 */ 
    /* FLEXPWM1_PWM2_OUT_TRIG0_1 ==> FLEXPWM2_EXT_SYNC0 */
    XBARA_SetSignalsConnection(XBARA, kXBARA1_InputFlexpwm1Pwm2OutTrig01, kXBARA1_OutputFlexpwm2ExtSync0);

    /* ADC TRIGGER - trigger from PWM1 SM[0] (VAL4) to ADC_ETC trigger 0 */
    /* FLEXPWM1_PWM1_OUT_TRIG0_1 ==> ADC_ETC */
    XBARA_SetSignalsConnection(XBARA, kXBARA1_InputFlexpwm2Pwm1OutTrig01, kXBARA1_OutputAdcEtcTrig00);
    /* ADC TRIGGER - trigger from PWM2 SM[0] (VAL4) to ADC_ETC trigger 1 */
    /* FLEXPWM2_PWM1_OUT_TRIG0_1 ==> ADC_ETC */
    XBARA_SetSignalsConnection(XBARA, kXBARA1_InputFlexpwm1Pwm1OutTrig01, kXBARA1_OutputAdcEtcTrig01);

//    XBARA_SetSignalsConnection(XBARA1, kXBARA1_InputIomuxXbarInout18, kXBARA1_OutputEnc1PhaseAInput); /* IOMUX_XBAR_INOUT18 output assigned to XBARA1_IN18 input is connected to XBARA1_OUT66 output assigned to ENC1_PHASE_A_INPUT */
//    XBARA_SetSignalsConnection(XBARA1, kXBARA1_InputIomuxXbarIn22, kXBARA1_OutputEnc1PhaseBInput); /* IOMUX_XBAR_IN22 output assigned to XBARA1_IN22 input is connected to XBARA1_OUT67 output assigned to ENC1_PHASE_B_INPUT */
//    XBARA_SetSignalsConnection(XBARA1, kXBARA1_InputIomuxXbarIn23, kXBARA1_OutputEnc1Index); /* IOMUX_XBAR_IN23 output assigned to XBARA1_IN23 input is connected to XBARA1_OUT68 output assigned to ENC1_INDEX */
//    XBARA_SetSignalsConnection(XBARA1, kXBARA1_InputIomuxXbarIn25, kXBARA1_OutputEnc2PhaseAInput); /* IOMUX_XBAR_IN25 output assigned to XBARA1_IN25 input is connected to XBARA1_OUT71 output assigned to ENC2_PHASE_A_INPUT */
//    XBARA_SetSignalsConnection(XBARA1, kXBARA1_InputIomuxXbarInout19, kXBARA1_OutputEnc2PhaseBInput); /* IOMUX_XBAR_INOUT19 output assigned to XBARA1_IN19 input is connected to XBARA1_OUT72 output assigned to ENC2_PHASE_B_INPUT */
//    XBARA_SetSignalsConnection(XBARA1, kXBARA1_InputIomuxXbarInout13, kXBARA1_OutputEnc2Index); /* IOMUX_XBAR_INOUT13 output assigned to XBARA1_IN13 input is connected to XBARA1_OUT73 output assigned to ENC2_INDEX */
//    XBARA_SetSignalsConnection(XBARA1, kXBARA1_InputIomuxXbarIn20, kXBARA1_OutputEnc3PhaseAInput); /* IOMUX_XBAR_IN20 output assigned to XBARA1_IN20 input is connected to XBARA1_OUT76 output assigned to ENC3_PHASE_A_INPUT */
//    XBARA_SetSignalsConnection(XBARA1, kXBARA1_InputIomuxXbarIn21, kXBARA1_OutputEnc3PhaseBInput); /* IOMUX_XBAR_IN21 output assigned to XBARA1_IN21 input is connected to XBARA1_OUT77 output assigned to ENC3_PHASE_B_INPUT */
//    XBARA_SetSignalsConnection(XBARA1, kXBARA1_InputIomuxXbarInout14, kXBARA1_OutputEnc3Index); /* IOMUX_XBAR_INOUT14 output assigned to XBARA1_IN14 input is connected to XBARA1_OUT78 output assigned to ENC3_INDEX */
//    XBARA_SetSignalsConnection(XBARA1, kXBARA1_InputIomuxXbarInout15, kXBARA1_OutputEnc4PhaseAInput); /* IOMUX_XBAR_INOUT15 output assigned to XBARA1_IN15 input is connected to XBARA1_OUT81 output assigned to ENC4_PHASE_A_INPUT */
//    XBARA_SetSignalsConnection(XBARA1, kXBARA1_InputIomuxXbarInout16, kXBARA1_OutputEnc4PhaseBInput); /* IOMUX_XBAR_INOUT16 output assigned to XBARA1_IN16 input is connected to XBARA1_OUT82 output assigned to ENC4_PHASE_B_INPUT */
//    XBARA_SetSignalsConnection(XBARA1, kXBARA1_InputIomuxXbarInout17, kXBARA1_OutputEnc4Index); /* IOMUX_XBAR_INOUT17 output assigned to XBARA1_IN17 input is connected to XBARA1_OUT83 output assigned to ENC4_INDEX */  
    
}

/*!
* @brief   void InitADC_ETC(void)
*           - Initialization of the ADC_ETC peripheral
*          
* @param   void
*
* @return  none
*/
void InitADC_ETC(void)
{   
    adc_etc_config_t adcEtcConfig;
    adc_etc_trigger_config_t adcEtcTriggerConfig;
    adc_etc_trigger_chain_config_t adcEtcTriggerChainConfig;  
    
    /* Initialize the ADC_ETC. */
    ADC_ETC_GetDefaultConfig(&adcEtcConfig);
    adcEtcConfig.XBARtriggerMask = 0x3U; /* 0000011 - 0x3 - enabled trig0-0 and trig0-1; Enable the external XBAR(from PWM) trigger0 (ADC1) (ADC2 is in sync mode). */
    ADC_ETC_Init(ADC_ETC, &adcEtcConfig);
  
    /* trigger0 is synchronized with trigger4 (sync mode have to be enabled just in trigger0 !) */
    /* Set the external ADC_ETC trigger0 configuration. */
    adcEtcTriggerConfig.enableSyncMode = true;
    adcEtcTriggerConfig.enableSWTriggerMode = false;
    adcEtcTriggerConfig.triggerChainLength = 2U; 
    adcEtcTriggerConfig.triggerPriority = 0U;
    adcEtcTriggerConfig.sampleIntervalDelay = 0U;
    adcEtcTriggerConfig.initialDelay = 0U;
    
    ADC_ETC_SetTriggerConfig(ADC_ETC, 0U, &adcEtcTriggerConfig);

    adcEtcTriggerChainConfig.enableB2BMode = true;  
       
    adcEtcTriggerChainConfig.ADCHCRegisterSelect = 1U << 0U; /* Select ADC_HC0 register to trigger. */
    adcEtcTriggerChainConfig.InterruptEnable = kADC_ETC_InterruptDisable; /* Enable the Done0 interrupt. */   
    ADC_ETC_SetTriggerChainConfig(ADC_ETC, 0U, 0U, &adcEtcTriggerChainConfig); /* Configure the trigger0 chain0. */
       
    adcEtcTriggerChainConfig.ADCHCRegisterSelect = 1U << 1U; /* Select ADC_HC1 register to trigger. */
    adcEtcTriggerChainConfig.InterruptEnable = kADC_ETC_InterruptDisable; /* Enable the Done1 interrupt. */
    ADC_ETC_SetTriggerChainConfig(ADC_ETC, 0U, 1U, &adcEtcTriggerChainConfig); /* Configure the trigger0 chain1. */
    
    adcEtcTriggerChainConfig.ADCHCRegisterSelect = 1U << 2U; /* Select ADC_HC2 register to trigger. */
    adcEtcTriggerChainConfig.InterruptEnable = kADC_ETC_InterruptDisable; /* Enable the Done1 interrupt. */
    ADC_ETC_SetTriggerChainConfig(ADC_ETC, 0U, 2U, &adcEtcTriggerChainConfig); /* Configure the trigger0 chain2. */    
    
    /* Set the external ADC_ETC trigger4 configuration. */
    adcEtcTriggerConfig.enableSyncMode = false; 
    adcEtcTriggerConfig.enableSWTriggerMode = false;
    adcEtcTriggerConfig.triggerChainLength = 2U;
    adcEtcTriggerConfig.triggerPriority = 0U;
    adcEtcTriggerConfig.sampleIntervalDelay = 0U;
    adcEtcTriggerConfig.initialDelay = 0U;
    
    ADC_ETC_SetTriggerConfig(ADC_ETC, 4U, &adcEtcTriggerConfig);
    
    adcEtcTriggerChainConfig.enableB2BMode = true;
 
    adcEtcTriggerChainConfig.ADCHCRegisterSelect = 1U << 0U; /* Select ADC_HC0 register to trigger. */
    adcEtcTriggerChainConfig.InterruptEnable = kADC_ETC_InterruptDisable; /* Enable the Done0 interrupt. */   
    ADC_ETC_SetTriggerChainConfig(ADC_ETC, 4U, 0U, &adcEtcTriggerChainConfig); /* Configure the trigger4 chain0. */
     
    adcEtcTriggerChainConfig.ADCHCRegisterSelect = 1U << 1U; /* Select ADC_HC1 register to trigger. */
    adcEtcTriggerChainConfig.InterruptEnable = kADC_ETC_InterruptDisable; /* Enable the Done1 interrupt. */
    ADC_ETC_SetTriggerChainConfig(ADC_ETC, 4U, 1U, &adcEtcTriggerChainConfig); /* Configure the trigger4 chain1. */
    
    adcEtcTriggerChainConfig.ADCHCRegisterSelect = 1U << 2U; /* Select ADC_HC2 register to trigger. */
    adcEtcTriggerChainConfig.InterruptEnable = kADC_ETC_InterruptDisable; /* Enable the Done1 interrupt. */
    ADC_ETC_SetTriggerChainConfig(ADC_ETC, 4U, 2U, &adcEtcTriggerChainConfig); /* Configure the trigger4 chain2. */
    
    
    /* trigger1 is synchronized with trigger5 (sync mode have to be enabled just in trigger1 !) */
    /* Set the external ADC_ETC trigger2 configuration. */
    adcEtcTriggerConfig.enableSyncMode = true; 
    adcEtcTriggerConfig.enableSWTriggerMode = false;
    adcEtcTriggerConfig.triggerChainLength = 2U; 
    adcEtcTriggerConfig.triggerPriority = 0U;
    adcEtcTriggerConfig.sampleIntervalDelay = 0U;
    adcEtcTriggerConfig.initialDelay = 0U;
    
    ADC_ETC_SetTriggerConfig(ADC_ETC, 1U, &adcEtcTriggerConfig);
    
    adcEtcTriggerChainConfig.enableB2BMode = true;
 
    adcEtcTriggerChainConfig.ADCHCRegisterSelect = 1U << 3U; /* Select ADC_HC3 register to trigger. */
    adcEtcTriggerChainConfig.InterruptEnable = kADC_ETC_InterruptDisable; /* Enable the Done0 interrupt. */   
    ADC_ETC_SetTriggerChainConfig(ADC_ETC, 1U, 0U, &adcEtcTriggerChainConfig); /* Configure the trigger1 chain0. */
     
    adcEtcTriggerChainConfig.ADCHCRegisterSelect = 1U << 4U; /* Select ADC_HC4 register to trigger. */
    adcEtcTriggerChainConfig.InterruptEnable = kADC_ETC_InterruptDisable; /* Enable the Done1 interrupt. */
    ADC_ETC_SetTriggerChainConfig(ADC_ETC, 1U, 1U, &adcEtcTriggerChainConfig); /* Configure the trigger1 chain1. */
    
    adcEtcTriggerChainConfig.ADCHCRegisterSelect = 1U << 5U; /* Select ADC_HC5 register to trigger. */
    adcEtcTriggerChainConfig.InterruptEnable = kADC_ETC_InterruptDisable; /* Enable the Done1 interrupt. */
    ADC_ETC_SetTriggerChainConfig(ADC_ETC, 1U, 2U, &adcEtcTriggerChainConfig); /* Configure the trigger1 chain2. */
      
    /* Set the external ADC_ETC trigger5 configuration. */
    adcEtcTriggerConfig.enableSyncMode = false; 
    adcEtcTriggerConfig.enableSWTriggerMode = false;
    adcEtcTriggerConfig.triggerChainLength = 2U; 
    adcEtcTriggerConfig.triggerPriority = 0U;
    adcEtcTriggerConfig.sampleIntervalDelay = 0U;
    adcEtcTriggerConfig.initialDelay = 0U;
    
    ADC_ETC_SetTriggerConfig(ADC_ETC, 5U, &adcEtcTriggerConfig);
    
    adcEtcTriggerChainConfig.enableB2BMode = true;
 
    adcEtcTriggerChainConfig.ADCHCRegisterSelect = 1U << 3U; /* Select ADC_HC3 register to trigger. */
    adcEtcTriggerChainConfig.InterruptEnable = kADC_ETC_InterruptDisable; /* Enable the Done0 interrupt. */   
    ADC_ETC_SetTriggerChainConfig(ADC_ETC, 5U, 0U, &adcEtcTriggerChainConfig); /* Configure the trigger5 chain0. */
     
    adcEtcTriggerChainConfig.ADCHCRegisterSelect = 1U << 4U; /* Select ADC_HC4 register to trigger. */
    adcEtcTriggerChainConfig.InterruptEnable = kADC_ETC_InterruptDisable; /* Enable the Done1 interrupt. */
    ADC_ETC_SetTriggerChainConfig(ADC_ETC, 5U, 1U, &adcEtcTriggerChainConfig); /* Configure the trigger5 chain1. */
    
    adcEtcTriggerChainConfig.ADCHCRegisterSelect = 1U << 5U; /* Select ADC_HC5 register to trigger. */
    adcEtcTriggerChainConfig.InterruptEnable = kADC_ETC_InterruptDisable; /* Enable the Done1 interrupt. */
    ADC_ETC_SetTriggerChainConfig(ADC_ETC, 5U, 2U, &adcEtcTriggerChainConfig); /* Configure the trigger5 chain2. */
      
}

/*!
* @brief   void InitQD(void)
*           - Initialization of the Quadrature Encoder peripheral
*           - performs speed and position sensor processing
*
* @param   void
*
* @return  none
*/
void M1_InitQD(void)
{
    /* Enable clock to ENC modules */
    CLOCK_EnableClock(kCLOCK_Enc1);
    
    /* Initialization modulo counter to encoder number of pulses * 4 - 1 */
    ENC1->LMOD = (M1_POSPE_ENC_PULSES * 4U) - 1U;
    
    /* Enable modulo counting and revolution counter increment on roll-over */
    ENC1->CTRL2 = (ENC_CTRL2_MOD_MASK | ENC_CTRL2_REVMOD_MASK);
    
    /* Pass initialization data into encoder driver structure */     
    /* encoder position and speed measurement */
    g_sM1Enc.pui32QdBase = (ENC_Type *) ENC1;
    g_sM1Enc.sTo.fltPGain = M1_POSPE_KP_GAIN;
    g_sM1Enc.sTo.fltIGain = M1_POSPE_KI_GAIN;
    g_sM1Enc.sTo.fltThGain = M1_POSPE_INTEG_GAIN;
    g_sM1Enc.a32PosMeGain = M1_POSPE_MECH_POS_GAIN;
    g_sM1Enc.ui16Pp = M1_MOTOR_PP;
    g_sM1Enc.bDirection = M1_POSPE_ENC_DIRECTION;
    g_sM1Enc.fltSpdEncMin = M1_POSPE_ENC_N_MIN;
    g_sM1Enc.ui16EncLedDemoType = 0U;
    
    g_sM1Enc.ui16OnLedPosition = 40U;
    g_sM1Enc.ui16OffLedDelta = 80U;
    g_sM1Enc.ui16LPOSRegister = 0U;
    g_sM1Enc.ui16LedIndexOffset = 0U;
    g_sM1Enc.ui16LedAlignOffset = 1800U;
    
    g_sM1Enc.pui32GpioLedBase = (GPIO_Type *) GPIO2; //ENC LED GPIO BASE
    g_sM1Enc.ui16GpioLedPin = 13U; //ENC LED GPIO PIN

    MCDRV_QdEncSetDirection(&g_sM1Enc); 
           
    /* Enable & setup interrupt from ENC2 */
    EnableIRQ(ENC1_IRQn);   
    NVIC_SetPriority(ENC1_IRQn, 0U);
    
}

/*!
* @brief   void InitQD(void)
*           - Initialization of the Quadrature Encoder peripheral
*           - performs speed and position sensor processing
*
* @param   void
*
* @return  none
*/
void M2_InitQD(void)
{
    /* Enable clock to ENC modules */
    CLOCK_EnableClock(kCLOCK_Enc2);
    
    /* Initialization modulo counter to encoder number of pulses * 4 - 1 */
    ENC2->LMOD = (M2_POSPE_ENC_PULSES * 4U) - 1U;
    
    /* Enable modulo counting and revolution counter increment on roll-over */
    ENC2->CTRL2 = (ENC_CTRL2_MOD_MASK | ENC_CTRL2_REVMOD_MASK);
    
    /* Pass initialization data into encoder driver structure */     
    /* encoder position and speed measurement */
    g_sM2Enc.pui32QdBase = (ENC_Type *) ENC2;
    g_sM2Enc.sTo.fltPGain = M2_POSPE_KP_GAIN;
    g_sM2Enc.sTo.fltIGain = M2_POSPE_KI_GAIN;
    g_sM2Enc.sTo.fltThGain = M2_POSPE_INTEG_GAIN;
    g_sM2Enc.a32PosMeGain = M2_POSPE_MECH_POS_GAIN;
    g_sM2Enc.ui16Pp = M2_MOTOR_PP;
    g_sM2Enc.bDirection = M2_POSPE_ENC_DIRECTION;
    g_sM2Enc.fltSpdEncMin = M2_POSPE_ENC_N_MIN;
    g_sM2Enc.ui16EncLedDemoType = 0U;
    
    g_sM2Enc.ui16OnLedPosition = 40U;
    g_sM2Enc.ui16OffLedDelta = 80U;
    g_sM2Enc.ui16LPOSRegister = 0U;
    g_sM2Enc.ui16LedIndexOffset = 0U;
    g_sM2Enc.ui16LedAlignOffset = 980U;
    
    g_sM2Enc.pui32GpioLedBase = (GPIO_Type *) GPIO2; //ENC LED GPIO BASE
    g_sM2Enc.ui16GpioLedPin = 14U; //ENC LED GPIO PIN
    
    MCDRV_QdEncSetDirection(&g_sM2Enc);
    
    /* Enable & setup interrupt from ENC2 */
    EnableIRQ(ENC2_IRQn);   
    NVIC_SetPriority(ENC2_IRQn, 0U);
  
}

/*!
* @brief   void InitQD(void)
*           - Initialization of the Quadrature Encoder peripheral
*           - performs speed and position sensor processing
*
* @param   void
*
* @return  none
*/
void M3_InitQD(void)
{
    /* Enable clock to ENC modules */
//    CLOCK_EnableClock(kCLOCK_Enc3);
    
    /* Initialization modulo counter to encoder number of pulses * 4 - 1 */
//    ENC3->LMOD = (M3_POSPE_ENC_PULSES * 4U) - 1U;
    
    /* Enable modulo counting and revolution counter increment on roll-over */
//    ENC3->CTRL2 = (ENC_CTRL2_MOD_MASK | ENC_CTRL2_REVMOD_MASK);
    
    /* Pass initialization data into encoder driver structure */     
    /* encoder position and speed measurement */
/*
	g_sM3Enc.pui32QdBase = (ENC_Type *) ENC3;
    g_sM3Enc.sTo.fltPGain = M3_POSPE_KP_GAIN;
    g_sM3Enc.sTo.fltIGain = M3_POSPE_KI_GAIN;
    g_sM3Enc.sTo.fltThGain = M3_POSPE_INTEG_GAIN;
    g_sM3Enc.a32PosMeGain = M3_POSPE_MECH_POS_GAIN;
    g_sM3Enc.ui16Pp = M3_MOTOR_PP;
    g_sM3Enc.bDirection = M3_POSPE_ENC_DIRECTION;
    g_sM3Enc.fltSpdEncMin = M3_POSPE_ENC_N_MIN;
    g_sM3Enc.ui16EncLedDemoType = 0U;
    
    g_sM3Enc.ui16OnLedPosition = 40U;
    g_sM3Enc.ui16OffLedDelta = 80U;
    g_sM3Enc.ui16LPOSRegister = 0U;
    g_sM3Enc.ui16LedIndexOffset = 0U;
    g_sM3Enc.ui16LedAlignOffset = 3340U;
    
    g_sM3Enc.pui32GpioLedBase = (GPIO_Type *) GPIO4; //ENC LED GPIO BASE
    g_sM3Enc.ui16GpioLedPin = 10U; //ENC LED GPIO PIN
    
    MCDRV_QdEncSetDirection(&g_sM3Enc);
*/
    /* Enable & setup interrupt from ENC3 */
//    EnableIRQ(ENC3_IRQn);
//    NVIC_SetPriority(ENC3_IRQn, 0U);
  
}

/*!
* @brief   void InitQD(void)
*           - Initialization of the Quadrature Encoder peripheral
*           - performs speed and position sensor processing
*
* @param   void
*
* @return  none
*/
void M4_InitQD(void)
{
    /* Enable clock to ENC modules */
//    CLOCK_EnableClock(kCLOCK_Enc4);
    
    /* Initialization modulo counter to encoder number of pulses * 4 - 1 */
//    ENC4->LMOD = (M4_POSPE_ENC_PULSES * 4U) - 1U;
    
    /* Enable modulo counting and revolution counter increment on roll-over */
//    ENC4->CTRL2 = (ENC_CTRL2_MOD_MASK | ENC_CTRL2_REVMOD_MASK);
    
    /* Pass initialization data into encoder driver structure */     
    /* encoder position and speed measurement */
/*
	g_sM4Enc.pui32QdBase = (ENC_Type *) ENC4;
    g_sM4Enc.sTo.fltPGain = M4_POSPE_KP_GAIN;
    g_sM4Enc.sTo.fltIGain = M4_POSPE_KI_GAIN;
    g_sM4Enc.sTo.fltThGain = M4_POSPE_INTEG_GAIN;
    g_sM4Enc.a32PosMeGain = M4_POSPE_MECH_POS_GAIN;
    g_sM4Enc.ui16Pp = M4_MOTOR_PP;
    g_sM4Enc.bDirection = M4_POSPE_ENC_DIRECTION;
    g_sM4Enc.fltSpdEncMin = M4_POSPE_ENC_N_MIN;
    g_sM4Enc.ui16EncLedDemoType = 0U;
    
    g_sM4Enc.ui16OnLedPosition = 40U;
    g_sM4Enc.ui16OffLedDelta = 80U;
    g_sM4Enc.ui16LPOSRegister = 0U;
    g_sM4Enc.ui16LedIndexOffset = 0U;
    g_sM4Enc.ui16LedAlignOffset = 2880U;
    
    g_sM4Enc.pui32GpioLedBase = (GPIO_Type *) GPIO3; //ENC LED GPIO BASE
    g_sM4Enc.ui16GpioLedPin = 27U; //ENC LED GPIO PIN
    
    MCDRV_QdEncSetDirection(&g_sM4Enc);
*/
    /* Enable & setup interrupt from ENC4 */
//    EnableIRQ(ENC4_IRQn);
//    NVIC_SetPriority(ENC4_IRQn, 0U);
  
}

/*!
* @brief   void InitPIT(void)
*           - Initialization of the PIT
*           - performs CPU cycle measuring
*
* @param   void
*
* @return  none
*/
void InitPIT(void)
{
    /* Enable clock to ENC modules */
    CLOCK_EnableClock(kCLOCK_Pit);
    
    /* Turn on PIT */
    PIT->MCR = 0x0;
    
    /* Timer Load Value Register */
    PIT->CHANNEL[0].LDVAL = 0xFFFFFFFF;

}


/*!
* @brief   void InitLED(void)
*
* @param   void
*
* @return  none
*/
void InitLED(void)
{
	// No LEDs available for use on EVK with motor control
}
