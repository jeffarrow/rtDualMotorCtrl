/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
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
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
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
#ifndef __main_h_
#define __main_h_

#include <stdio.h>
#include <stdlib.h>

//-----------------------------------------------------------------------
// KSDK Includes
//-----------------------------------------------------------------------
#include "fsl_device_registers.h"
#include "fsl_common.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"
#include "event_groups.h"

#include "fsl_debug_console.h"
#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"

#include "fsl_gpio.h"
#include "fsl_iomuxc.h"
#include "fsl_lpuart.h"
#include "fsl_pwm.h"
#include "fsl_xbara.h"
#include "fsl_adc.h"
#include "fsl_enc.h"
//#include "fsl_elcdif.h"

//#include "lwip_task.h"
#include "json.h"
//#include "lcd_task.h"
//#include "nfc_task.h"
//#include "motor_control_task.h"

#include "freemaster.h"
#include "mcdrv.h"
#include "fm_tsa_pmsm.h"
#include "m1_sm_ref_sol.h"
#include "m2_sm_ref_sol.h"
#include "m3_sm_ref_sol.h"
#include "m4_sm_ref_sol.h"

#include "mc_demo_mode.h"

//------------------------------------------------------------------------------
// Enums
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Typedef
//------------------------------------------------------------------------------

/*!
 * @brief Application status.
 */
typedef enum _mc_app_status
{
  kMC_App_Off = 0,
  kMC_App_Auto = 1,
  kMC_App_Manual = 2,
} mc_app_status_t;

/*!
 * @brief Motor control method selection.
 */
typedef enum _mc_method_selection
{
  kMC_ScalarControl = 0,
  kMC_FOC_Voltage = 1,
  kMC_FOC_Current = 2,
  kMC_FOC_Speed = 3,
  kMC_PositionControl = 4,  
} mc_method_selection_t;

/*!
 * @brief Motor state.
 */
typedef enum _mc_state
{
  kMC_Fault = 0,
  kMC_Init = 1,
  kMC_Stop = 2,
  kMC_Run = 3,
} mc_state_t;

/*!
 * @brief Motor fault status.
 */
typedef enum _mc_fault
{
  kMC_NoFault = 0,
  kMC_OverCurrent = (1<<0),
  kMC_UnderDcBusVoltage = (1<<1),
  kMC_OverDcBusVoltage = (1<<2),
  kMC_OverLoad = (1<<3),
  kMC_OverSpeed = (1<<4),
  kMC_RotorBlocked = (1<<5),
} mc_fault_t;

/*!
 * @brief Motor control status data (for message exchange).
 */
typedef struct _mc_status_data{
  mc_app_status_t appStatus;            /* Motor state OFF/AUTO/MANUAL */
  mc_state_t motorState;                /* Motor state FAULT/INIT/STOP/RUN */
  mc_fault_t faultStatus;               /* Motor control fault status */ 
  float f32Speed;                       /* Motor speed in rpm*/
  union {
    struct {
      uint16_t u16NumofTurns;             /* Number of motor turns */
      uint16_t u16RotorPosition;          /* position of motors in 0 - 65535    \
                                            (180degrees = 65535*x/360 = 32767)*/
    };
    uint32_t u32Position;
  };
  float f32Iq;                          /* current in Q axis (Torq equivalent) */
  float f32UDcBus;                      /* Position demo mode enabled/disabled */
} mc_status_data_t;

/*!
 * @brief Motor control command data (for message exchange).
 */
typedef struct _mc_command_data{
  mc_app_status_t appStatus;                 /* Motor x off/auto/manual */
  mc_method_selection_t controlMethodSel;    /* Selection of control method */
  bool bDemoModeSpeed;                       /* Speed demo mode enabled/disabled */
  float f32Speed;                            /* Speed command in rpm */
  bool bDemoModePosition;                    /* Position demo mode enabled/disabled */
  union {
    struct {
      uint16_t u16NumofTurns;                /* Command for number of motor turns */
      uint16_t u16RotorPosition;             /* Command for motor position in 0 - 65535\
                                                (180degrees = 65535*x/360 = 32767)*/
    };
    uint32_t u32Position;
  };
} mc_command_data_t;   

//------------------------------------------------------------------------------
// Defines
//------------------------------------------------------------------------------
#define EVENT_NFC_ID_IGNORE             (1<<0)
#define EVENT_NFC_ID_LCD_SUCCESS        (1<<1)
#define EVENT_MOTOR_ID_SUCCESS          (1<<2)
#define EVENT_NFC_ID_LCD_FAIL           (1<<3)
#define EVENT_LCD_TOUCH                 (1<<4)
#define EVENT_LCD_UPDATE                (1<<5)
#define EVENT_ETHERNET_RX               (1<<6)
#define EVENT_ETHERNET_TX               (1<<7)
#define EVENT_PERIODIC_1S               (1<<8)

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

//------------------------------------------------------------------------------     
// Function Prototypes                                                          
//------------------------------------------------------------------------------
extern void user_button_callback(void);
extern void lwip_udp_init_task(void *pvParameters);
extern void lcd_init_task(void *pvParameters);
extern void lcd_touch_int_callback(void);
extern void elcdif_frame_done_callback(void);
extern void nfc_task(void *pvParameters);
extern void motor_control_init_task(void *pvParameters);
extern int json_decode(uint8_t *jsonBuffer, mc_command_data_t *mcCommandData);
extern int json_encode(mc_status_data_t *mcStatusData, uint32_t motorId, uint8_t *jsonBuffer);

//------------------------------------------------------------------------------
// Global Variables
//------------------------------------------------------------------------------
extern QueueHandle_t queueEthernetMotorControlCommandData;
extern QueueHandle_t queueLcdMotorControlCommandData;
extern QueueHandle_t queueEthernetMotorControlStatusData;
extern QueueHandle_t queueLcdMotorControlStatusData;
extern EventGroupHandle_t eventGroupMotorControlData;
extern TaskHandle_t xNfcTaskHandle;
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
#endif /* __main_h_ */

