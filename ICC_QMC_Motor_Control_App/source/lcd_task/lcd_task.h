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
#ifndef __lcd_task_h_
#define __lcd_task_h_

#include "main.h"

//------------------------------------------------------------------------------
// Enums
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Typedef
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Defines
//------------------------------------------------------------------------------
#define ELCDIF_RGB_MODE_CONFIG                                                  \
{                                                                               \
  .panelWidth = (3*480),                                                        \
  .panelHeight = 272,                                                           \
  .hsw = 41,                                                                    \
  .hfp = 12,                                                                    \
  .hbp = 108,                                                                   \
  .vsw = 10,                                                                    \
  .vfp = 3,                                                                     \
  .vbp = 12,                                                                    \
  .polarityFlags = (kELCDIF_DataEnableActiveHigh | kELCDIF_VsyncActiveLow |     \
                    kELCDIF_HsyncActiveLow | kELCDIF_DriveDataOnRisingClkEdge), \
  .bufferAddr = (uint32_t)&initial_page_buffer[0],                              \
  .pixelFormat = kELCDIF_PixelFormatRAW8,                                       \
  .dataBus = kELCDIF_DataBus8Bit,                                               \
}
//------------------------------------------------------------------------------     
// Function Prototypes                                                          
//------------------------------------------------------------------------------
extern void lcd_init_task(void *pvParameters);
extern void elcdif_frame_done_callback(void);
extern void lcd_touch_int_callback(void);
//------------------------------------------------------------------------------
// Global Variables
//------------------------------------------------------------------------------

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

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
#endif /* __lcd_task_h_ */

