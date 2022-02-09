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

#include "mc_demo_mode.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* Speed demo mode enabled/disabled */
bool_t bDemoModeSpeed = FALSE;

/* Position demo mode enabled/disabled */
bool_t bDemoModePosition = FALSE;

bool_t bM1SpeedDemo = FALSE; 
bool_t bM2SpeedDemo = FALSE;

bool_t bM1PositionDemo = FALSE; 
bool_t bM2PositionDemo = FALSE;

bool_t bM2_SyncWithM1 = FALSE;
bool_t bM1_SyncWithM2 = FALSE;
   
/* Counter used for demo mode */
static uint32_t ui32M1SpeedDemoCnt = 0U;
static uint32_t ui32M2SpeedDemoCnt = 0U;

/* Counter used for demo mode */
static uint32_t ui32M1PositionDemoCnt = 0U;
static uint32_t ui32M2PositionDemoCnt = 0U;

/*******************************************************************************
 * Code
 ******************************************************************************/

/* Demo Speed Stimulator */
RAM_FUNC
void SpeedDemo(void)
{  
    static bool_t bSpeedDemoOff = 0U;
  
    if (bDemoModeSpeed)
    {  
        bSpeedDemoOff = 1U;
  
        bM1SpeedDemo = TRUE; 
        bM2SpeedDemo = TRUE;
        
        bDemoModePosition = FALSE;
        bM1PositionDemo = FALSE; 
        bM2PositionDemo = FALSE;
    }
    else
    {     
        if(bSpeedDemoOff == 1U)
        {
            bSpeedDemoOff = 0U;
            bM1SpeedDemo = FALSE; 
            bM2SpeedDemo = FALSE;
        }            
    }
    
    /* M1 speed demo */
    M1SpeedDemo();  
    /* M2 speed demo */
    M2SpeedDemo(); 
}

/* Demo Position Stimulator */
RAM_FUNC
void PositionDemo(void)
{
  
    static bool_t bPosDemoOff = 0U;
  
    if (bDemoModePosition)
    {     
        bPosDemoOff = 1U;
        
        bDemoModeSpeed = FALSE;
        bM1SpeedDemo = FALSE; 
        bM2SpeedDemo = FALSE;
        
        bM1PositionDemo = TRUE; 
        bM2PositionDemo = TRUE;
        
    }
    else
    {    
        if(bPosDemoOff == 1U)
        {
            bPosDemoOff = 0U;
            bM1PositionDemo = FALSE; 
            bM2PositionDemo = FALSE;
        }   
    }
    /* Position demo M1 */
    if (bM1PositionDemo)
    {
        ui32M1PositionDemoCnt++;
        switch (ui32M1PositionDemoCnt)
        {
            case 1:
                g_sM1Drive.eControl = kControlMode_PositionFOC;
                g_sM1Drive.sMCATctrl.ui16PospeSensor = MCAT_ENC_CTRL;  
                g_sM1Enc.ui16EncLedDemoType = 1U;
                break;
            case 2:    
                M1_SetAppSwitch(1); 
                M1_SetPosition(ACC32(2*2));
                break;        
            case 5000:
                M1_SetPosition(ACC32(5*2));
                break; 
            case 6500:
                M1_SetPosition(ACC32(400*2));
                break;    
            case 21500:
                M1_SetPosition(ACC32(0));
                break;
            case 36500:
                ui32M1PositionDemoCnt = 1;
                g_sM1Enc.ui16EncLedDemoType++;
                
                if(g_sM1Enc.ui16EncLedDemoType>3U)
                    g_sM1Enc.ui16EncLedDemoType = 1U;
                
                break;
            default:
                break;    
        }
        
        if(g_sM1Drive.sPosition.a32Position > ACC32(50*2))
        {
            g_sM1Drive.sPosition.fltSpeedConvScale = (2000.0F)/g_fltM1speedAngularScale;
        }
        if(g_sM1Drive.sPosition.a32Position > ACC32(100*2))
        {
            g_sM1Drive.sPosition.fltSpeedConvScale = (2500.0F)/g_fltM1speedAngularScale;
        }
        if(g_sM1Drive.sPosition.a32Position > ACC32(150*2))
        {
            g_sM1Drive.sPosition.fltSpeedConvScale = (3000.0F)/g_fltM1speedAngularScale;
        }
        if(g_sM1Drive.sPosition.a32Position > ACC32(200*2))
        {
            g_sM1Drive.sPosition.fltSpeedConvScale = (2500.0F)/g_fltM1speedAngularScale;
        }   
        if(g_sM1Drive.sPosition.a32Position > ACC32(250*2))
        {
            g_sM1Drive.sPosition.fltSpeedConvScale = (2000.0F)/g_fltM1speedAngularScale;
        }        
      
    }
    else
    {
        if(ui32M1PositionDemoCnt != 0U)
        {
            M1_SetAppSwitch(0);
            g_sM1Drive.sPosition.fltSpeedConvScale = (2000.0F)/g_fltM1speedAngularScale;
            g_sM1Enc.ui16EncLedDemoType = 0U;
        }     
        ui32M1PositionDemoCnt = 0U;     
    }
        
    /* Position demo M2 */
    if (bM2PositionDemo)
    {
        ui32M2PositionDemoCnt++;
        switch (ui32M2PositionDemoCnt)
        {
            case 1:
                g_sM2Drive.eControl = kControlMode_PositionFOC;
                g_sM2Drive.sMCATctrl.ui16PospeSensor = MCAT_ENC_CTRL;  
                g_sM2Enc.ui16EncLedDemoType = 1U;
                break;
            case 2:      
                M2_SetAppSwitch(1); 
                M2_SetPosition(ACC32(2*2));
                break;        
            case 5000:
                M2_SetPosition(ACC32(5*2));
                break; 
            case 6500:
                M2_SetPosition(ACC32(400*2));
                break;    
            case 21500:
                M2_SetPosition(ACC32(0));
                break;
            case 36500:
                ui32M2PositionDemoCnt = 1;
                g_sM2Enc.ui16EncLedDemoType++;
                
                if(g_sM2Enc.ui16EncLedDemoType>3U)
                    g_sM2Enc.ui16EncLedDemoType = 1U;
                
                break;
            default:
                break;    
        }
        
        if(g_sM2Drive.sPosition.a32Position > ACC32(50*2))
        {
            g_sM2Drive.sPosition.fltSpeedConvScale = (2000.0F)/g_fltM2speedAngularScale;
        }
        if(g_sM2Drive.sPosition.a32Position > ACC32(100*2))
        {
            g_sM2Drive.sPosition.fltSpeedConvScale = (2500.0F)/g_fltM2speedAngularScale;
        }
        if(g_sM2Drive.sPosition.a32Position > ACC32(150*2))
        {
            g_sM2Drive.sPosition.fltSpeedConvScale = (3000.0F)/g_fltM2speedAngularScale;
        }
        if(g_sM2Drive.sPosition.a32Position > ACC32(200*2))
        {
            g_sM2Drive.sPosition.fltSpeedConvScale = (2500.0F)/g_fltM2speedAngularScale;
        }   
        if(g_sM2Drive.sPosition.a32Position > ACC32(250*2))
        {
            g_sM2Drive.sPosition.fltSpeedConvScale = (2000.0F)/g_fltM2speedAngularScale;
        }        
      
    }
    else
    {
        if(ui32M2PositionDemoCnt != 0U)
        {
            M2_SetAppSwitch(0);
            g_sM2Drive.sPosition.fltSpeedConvScale = (2000.0F)/g_fltM2speedAngularScale;
            g_sM2Enc.ui16EncLedDemoType = 0U;
        }     
        ui32M2PositionDemoCnt = 0U;     
    }
}
   
/* Motor 1 Speed Demo */
float_t demoSpeed=500.0;
float_t prevDemoSpeed=500.0;
RAM_FUNC
void M1SpeedDemo(void)
{
    if (bM1SpeedDemo)
    {
        ui32M1SpeedDemoCnt++;
        switch (ui32M1SpeedDemoCnt)
        {     
            case 1:
                break;
            case 2:
                g_sM1Drive.eControl = kControlMode_SpeedFOC;
                M1_SetAppSwitch(1);
                g_sM1Enc.ui16EncLedDemoType = 1U;
                break;    
            case 10:
                M1_SetSpeed((demoSpeed)/g_fltM1speedAngularScale);
                break;
//            case 2000:
//                M1_SetSpeed((2000.0F)/g_fltM1speedAngularScale);
//                break;
//            case 4000:
//                M1_SetSpeed((3500.0F)/g_fltM1speedAngularScale);
//                break;
//            case 6000:
//                M1_SetSpeed((2000.0F)/g_fltM1speedAngularScale);
//                break;
//            case 8000:
//                M1_SetSpeed((1000.0F)/g_fltM1speedAngularScale);
//                break;
//            case 10000:
//                M1_SetSpeed((-1000.0F)/g_fltM1speedAngularScale);
//                break;
//            case 12000:
//                M1_SetSpeed((-2000.0F)/g_fltM1speedAngularScale);
//                break;
//            case 14000:
//                M1_SetSpeed((-3500.0F)/g_fltM1speedAngularScale);
//                break;
//            case 16000:
//                M1_SetSpeed((-2000.0F)/g_fltM1speedAngularScale);
//                break;
//            case 18000:
//                M1_SetSpeed((-1000.0F)/g_fltM1speedAngularScale);
//                break;
//            case 20000:
//                M1_SetSpeed((0.0F)/g_fltM1speedAngularScale);
//                break;
//            case 20100:
//                ui32M1SpeedDemoCnt = 1;
//                break;
            default:
                if (prevDemoSpeed != demoSpeed){
                	M1_SetSpeed((demoSpeed)/g_fltM1speedAngularScale);
					prevDemoSpeed = demoSpeed;
                }
        }
    }
    else
    {
        if(ui32M1SpeedDemoCnt != 0U)
        {
            M1_SetAppSwitch(0);
            g_sM1Enc.ui16EncLedDemoType = 0U;
        }
        ui32M1SpeedDemoCnt = 0U;
    }
}

/* Motor 2 Speed Demo */
RAM_FUNC
void M2SpeedDemo(void)
{
	return;
    if (bM2SpeedDemo)
    {
        ui32M2SpeedDemoCnt++;
        switch (ui32M2SpeedDemoCnt)
        {                        
            case 1:
                break;
            case 2:
                g_sM2Drive.eControl = kControlMode_SpeedFOC;
                M2_SetAppSwitch(1);
                g_sM2Enc.ui16EncLedDemoType = 1U;
                break;    
            case 10:
                M2_SetSpeed((demoSpeed)/g_fltM2speedAngularScale);
                break;      
//            case 2000:
//                M2_SetSpeed((2000.0F)/g_fltM2speedAngularScale);
//                break;
//            case 4000:
//                M2_SetSpeed((3500.0F)/g_fltM2speedAngularScale);
//                break;
//            case 6000:
//                M2_SetSpeed((2000.0F)/g_fltM2speedAngularScale);
//                break;
//            case 8000:
//                M2_SetSpeed((1000.0F)/g_fltM2speedAngularScale);
//                break;
//            case 10000:
//                M2_SetSpeed((-1000.0F)/g_fltM2speedAngularScale);
//                break;
//            case 12000:
//                M2_SetSpeed((-2000.0F)/g_fltM2speedAngularScale);
//                break;
//            case 14000:
//                M2_SetSpeed((-3500.0F)/g_fltM2speedAngularScale);
//                break;
//            case 16000:
//                M2_SetSpeed((-2000.0F)/g_fltM2speedAngularScale);
//                break;
//            case 18000:
//                M2_SetSpeed((-1000.0F)/g_fltM2speedAngularScale);
//                break;
//            case 20000:
//                M2_SetSpeed((0.0F)/g_fltM2speedAngularScale);
//                break;
//            case 20100:
//                ui32M2SpeedDemoCnt = 1;
//                break;
            default:
                if (prevDemoSpeed != demoSpeed){
                	M2_SetSpeed((demoSpeed)/g_fltM2speedAngularScale);
                }
        }
    }
    else
    {
        if(ui32M2SpeedDemoCnt != 0U)
        {
            M2_SetAppSwitch(0);
            g_sM2Enc.ui16EncLedDemoType = 0U;
        }
          
        ui32M2SpeedDemoCnt = 0U;
    }
}

RAM_FUNC
void MotorSynchronisation(void)
{  
  
  /* Synchronisation with M1 */
  if(bM2_SyncWithM1)
  {
        g_bM2SwitchAppOnOff = g_bM1SwitchAppOnOff;
        g_sM2Drive.eControl = g_sM1Drive.eControl;
        g_sM2Enc.ui16EncLedDemoType = g_sM1Enc.ui16EncLedDemoType;
        bM1_SyncWithM2 = 0;
     
        if(g_sM2Drive.eControl == 3)
        {
            bM2SpeedDemo = FALSE;
            g_sM2Drive.sSpeed.fltSpeedCmd = g_sM1Drive.sSpeed.fltSpeedCmd;      
        }
        else if(g_sM2Drive.eControl == 4)
        {
            bM2PositionDemo = FALSE;
            g_sM2Drive.sPosition.a32PositionCmd = g_sM1Drive.sPosition.a32PositionCmd;
            g_sM2Drive.sPosition.fltSpeedConvScale = g_sM1Drive.sPosition.fltSpeedConvScale;
        }        
  } 
  
  /* Synchronisation with M2 */
  if(bM1_SyncWithM2)
  {
        g_bM1SwitchAppOnOff = g_bM2SwitchAppOnOff;
        g_sM1Drive.eControl = g_sM2Drive.eControl;
        g_sM1Enc.ui16EncLedDemoType = g_sM2Enc.ui16EncLedDemoType;
        bM2_SyncWithM1 = 0;
           
        if(g_sM1Drive.eControl == 3)
        {
            bM1SpeedDemo = FALSE;
            g_sM1Drive.sSpeed.fltSpeedCmd = g_sM2Drive.sSpeed.fltSpeedCmd;      
        }
        else if(g_sM1Drive.eControl == 4)
        {
            bM1PositionDemo = FALSE;
            g_sM1Drive.sPosition.a32PositionCmd = g_sM2Drive.sPosition.a32PositionCmd;
            g_sM1Drive.sPosition.fltSpeedConvScale = g_sM2Drive.sPosition.fltSpeedConvScale;
        }        
  } 

}
