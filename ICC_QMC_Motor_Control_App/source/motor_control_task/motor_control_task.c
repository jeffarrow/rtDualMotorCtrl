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

#include "motor_control_task.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/* ADC interrupt handler */
RAM_FUNC_CRITICAL
void ADC1_IRQHandler(void);
/* TMR1 interrupt handler */
RAM_FUNC
void TMR1_IRQHandler(void);
/* LPUART6 interrupt handler */
RAM_FUNC
void LPUART6_IRQHandler(void);
/* ENC interrupt handlers */
RAM_FUNC
void ENC1_IRQHandler(void);
RAM_FUNC
void ENC2_IRQHandler(void);
RAM_FUNC
void ENC3_IRQHandler(void);
RAM_FUNC
void ENC4_IRQHandler(void);

/*******************************************************************************
 * Globals
 ******************************************************************************/
/* CPU load measurement using PIT*/
uint32_t g_ui32NumberOfCycles = 0U;
uint32_t g_ui32MaxNumberOfCycles = 0U;
uint32_t PIT_cval_1 = 0U;
uint32_t PIT_cval_2 = 0U;

/* PWM2 submodule 0 compare event has occurred for VAL4 - trigger for M1 ADC conversion */
uint16_t ui16M1AdcConv = 0U;
/* PWM3 submodule 0 compare event has occurred for VAL4 - trigger for M2 ADC conversion */
uint16_t ui16M2AdcConv = 0U;

uint16_t g_ui16DemoLedType = 1U;

/* Application and board ID  */
app_ver_t   g_sAppId = {
    "evk_imxrt1050",    /* board id */
    "pmsm",             /* motor type */
    MCRSP_VER,          /* sw version */
};
/* Structure used in FM to get required ID's */
app_ver_t   g_sAppIdFM;

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief motor_control_init_task function
 */
void motor_control_init_task(void *pvParameters)
{
  PRINTF("motor_control_init_task entered.\r\n");
    
  uint32_t ui32PrimaskReg;
  /* Disable all interrupts before peripherals are initialized */
  ui32PrimaskReg = DisableGlobalIRQ();
  /* Init peripheral motor control driver for motor M1, M2, M3 & M4 */
  MCDRV_Init();   
  /* Init UART for FreeMaster communication */ 
  BOARD_InitDebugConsole(); //BOARD_InitUART(BOARD_FMSTR_UART_BAUDRATE);
  /* FreeMaster init */
  FMSTR_Init();

  /* Turn off motor control application */
  M1_SetAppSwitch(0);
  M2_SetAppSwitch(0);
/*
  USER_LED1_ON();
  USER_LED2_ON();
  USER_LED3_ON();
  USER_LED4_ON();
*/
  /* Pass actual demo id and board info to FM */
  g_sAppIdFM = g_sAppId;
  
  /* Enable interrupts */
  EnableGlobalIRQ(ui32PrimaskReg);
  vTaskDelete(NULL);
}

/* ADC interrupt perform motor control fast loop */
//RAM_FUNC_CRITICAL
void ADC1_IRQHandler(void)
{
      
    /* Check PWM compare flag for M1 - PWM1 Sub0 VAL4  */
    ui16M1AdcConv =  (((PWM1->SM[0].STS & PWM_STS_CMPF_MASK)& 0x8U) >> 3U); 
    
    if(ui16M1AdcConv == 1U)
    { 
        /* Start measuring CPU Load */
        PIT->CHANNEL[0].TCTRL = PIT_TCTRL_TEN(1U);
        PIT_cval_1 = PIT->CHANNEL[0].CVAL;
      
        /* M1 State machine */
        SM_StateMachineFast(&g_sM1Ctrl);
       
        /* M3 State machine */
//        SM_StateMachineFast(&g_sM3Ctrl);
        
        /* Clear PWM STS compare flags for M1 */
        PWM1->SM[0].STS |= PWM_STS_CMPF_MASK; 
        ui16M1AdcConv =  0U; 
        
        /* Stop CPU tick number couting and store actual and maximum ticks */
        PIT_cval_1 = (PIT_cval_1 - PIT->CHANNEL[0].CVAL);
        PIT->CHANNEL[0].TCTRL = PIT_TCTRL_TEN(0U); 
        
    }     
    
    /* Check PWM compare flag for M3 - PWM3 Sub0 VAL4  */
//    ui16M2AdcConv =  (((PWM3->SM[0].STS & PWM_STS_CMPF_MASK) & 0x8U) >> 3U);

    /* Check PWM compare flag for M2 - PWM2 Sub0 VAL4  */
    ui16M2AdcConv =  (((PWM2->SM[0].STS & PWM_STS_CMPF_MASK) & 0x8U) >> 3U);
       
    if(ui16M2AdcConv == 1U)
    {   
        /* Start measuring CPU Load */
        PIT->CHANNEL[0].TCTRL = PIT_TCTRL_TEN(1U);
        PIT_cval_2 = PIT->CHANNEL[0].CVAL;
      
        /* M2 State machine */
        SM_StateMachineFast(&g_sM2Ctrl);
        
        /* M4 State machine */
//        SM_StateMachineFast(&g_sM4Ctrl);
                
        /* Clear PWM STS compare flags for M2 */
        PWM2->SM[0].STS |= PWM_STS_CMPF_MASK;
        ui16M2AdcConv =  0U; 
        
        /* Stop CPU tick number couting and store actual and maximum ticks */
        PIT_cval_2 = (PIT_cval_2 - PIT->CHANNEL[0].CVAL);
        PIT->CHANNEL[0].TCTRL = PIT_TCTRL_TEN(0U); 
        
    }
           
    g_ui32NumberOfCycles = PIT_cval_1 + PIT_cval_2;
    g_ui32MaxNumberOfCycles = g_ui32NumberOfCycles>g_ui32MaxNumberOfCycles ? g_ui32NumberOfCycles : g_ui32MaxNumberOfCycles;
    
    /* Call FreeMASTER recorder */
    FMSTR_Recorder(); 
    
    /* Add empty instructions for correct interrupt flag clearing */
    M1_END_OF_ISR;

}


/* TMR1 reload ISR called with 1ms period and processes motor M1, M2, M3 and M4 slow application machine function */
RAM_FUNC
void TMR1_IRQHandler(void)
{     
    /* M1 Slow StateMachine call */
    SM_StateMachineSlow(&g_sM1Ctrl);
    
    /* M2 Slow StateMachine call */
    SM_StateMachineSlow(&g_sM2Ctrl);
    
    /* M3 Slow StateMachine call */
//    SM_StateMachineSlow(&g_sM3Ctrl);
    
    /* M4 Slow StateMachine call */
//    SM_StateMachineSlow(&g_sM4Ctrl);
    
    /* Demo speed stimulator */
    SpeedDemo();
    
    /* Demo position stimulator */
    PositionDemo();
    
    /* Motor synchronisation */
    MotorSynchronisation();
    
    MCDRV_QdEncLedDemoType(&g_sM1Enc);
    
    MCDRV_QdEncLedDemoType(&g_sM2Enc);
    
//    MCDRV_QdEncLedDemoType(&g_sM3Enc);
    
//    MCDRV_QdEncLedDemoType(&g_sM4Enc);
    
    /* Clear the CSCTRL0[TCF1] flag */  
    TMR1->CHANNEL[0].CSCTRL |= TMR_CSCTRL_TCF1(0x00);
    TMR1->CHANNEL[0].CSCTRL  &= ~(TMR_CSCTRL_TCF1_MASK);
    
    /* Clear the CSCTRL0[TCF] flag */
    TMR1->CHANNEL[0].SCTRL  &= ~(TMR_SCTRL_TCF_MASK); 
    
    /* Add empty instructions for correct interrupt flag clearing */
    M1_END_OF_ISR;
    
}


/* ISR called when encoder 1 index signal is detected */ 
RAM_FUNC
void ENC1_IRQHandler(void)
{   
    
    /* Read offset between Index position and position of the shaft after align*/
    MCDRV_QdEncGetIndexOffset(&g_sM1Enc);
    g_sM1Drive.sPosition.a32PositionIndexOffset = g_sM1Enc.a32PosIndexOffset;
    g_sM1Drive.sPosition.a32PositionAlignOffset = ACC32(-0.05*2);
    
    /* Clear interrupt flag */
    ENC1->CTRL = (ENC1->CTRL & (uint16_t)(~((ENC_CTRL_HIRQ_MASK | ENC_CTRL_XIRQ_MASK | ENC_CTRL_DIRQ_MASK | ENC_CTRL_CMPIRQ_MASK)))) | ENC_CTRL_XIRQ_MASK;
    /* Do not execuite next ENC interrupt */ 
    ENC1->CTRL = 0x0;
    
    /* Add empty instructions for correct interrupt flag clearing */
    M1_END_OF_ISR;

}


/* ISR called when encoder 2 index signal is detected */
RAM_FUNC
void ENC2_IRQHandler(void)
{   
    
    /* Read offset between Index position and position of the shaft after align*/
    MCDRV_QdEncGetIndexOffset(&g_sM2Enc);
    g_sM2Drive.sPosition.a32PositionIndexOffset = g_sM2Enc.a32PosIndexOffset;
    g_sM2Drive.sPosition.a32PositionAlignOffset = ACC32(-0.25*2);
    
    /* Clear interrupt flag */
    ENC2->CTRL = (ENC2->CTRL & (uint16_t)(~((ENC_CTRL_HIRQ_MASK | ENC_CTRL_XIRQ_MASK | ENC_CTRL_DIRQ_MASK | ENC_CTRL_CMPIRQ_MASK)))) | ENC_CTRL_XIRQ_MASK;
    /* Do not execuite next ENC interrupt */ 
    ENC2->CTRL = 0x0;
    
    /* Add empty instructions for correct interrupt flag clearing */
    M1_END_OF_ISR;

}


/* ISR called when encoder 3 index signal is detected */
RAM_FUNC
void ENC3_IRQHandler(void)
{   
    M1_END_OF_ISR;

}


/* ISR called when encoder 4 index signal is detected */
RAM_FUNC
void ENC4_IRQHandler(void)
{   
    /* Add empty instructions for correct interrupt flag clearing */
    M1_END_OF_ISR;

}


/* LPUART6 interrupt, call freemaster functions */
RAM_FUNC
void LPUART6_IRQHandler(void)
{   
    /* Service Routine handles the SCI interrupts for the FreeMASTER */
    FMSTR_Isr(); 
    /* Add empty instructions for correct interrupt flag clearing */
    M1_END_OF_ISR;

}
