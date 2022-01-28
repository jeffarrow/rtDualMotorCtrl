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
 /**************************************************************************//*!
 * @file      interrupts.c
 * @author    
 * @version   0.0.1.0
 * @brief     
 ******************************************************************************/

// Standard C Included Files
#include <stdint.h>
#include <stdbool.h>
#include "main.h"

/* module definitions                                                         */

/* module function prototypes                                                 */

/* module variables                                                           */
   
/* module Function definitions                                                */

/* module IRQ Handlers                                                        */
   
/***************************************************************************//*!
 * @brief   NMI exception handler
 * @details It will stop application in debugg mode with software breakpoint
 *          when hard fault event occur.
 ******************************************************************************/
void NMI_Handler(void)
{
#ifdef DEBUG
  __asm("BKPT #0x01");
#endif
}

/***************************************************************************//*!
 * @brief   Hardfault exception handler
 * @details It will stop application in debugg mode with software breakpoint
 *          when hard fault event occur.
 ******************************************************************************/
void HardFault_Handler(void)
{
#ifdef DEBUG
  __asm("BKPT #0x02");
#endif
}

/***************************************************************************//*!
 * @brief   BOARD_USER_BUTTON_1_IRQ_HANDLER interrupt handler
 * @details Generate an interrupt on rising edge on USER_BUTTON_1 or USER_BUTTON_2.
 ******************************************************************************/
void BOARD_USER_BUTTON_1_IRQ_HANDLER(void)
{
  if(GPIO_GetPinsInterruptFlags(BOARD_USER_BUTTON_1_GPIO)&(1U << BOARD_USER_BUTTON_1_GPIO_PIN))
  {
    user_button_callback();
    GPIO_PortClearInterruptFlags(BOARD_USER_BUTTON_1_GPIO, 1U << BOARD_USER_BUTTON_1_GPIO_PIN);
  }
  if(GPIO_GetPinsInterruptFlags(BOARD_USER_BUTTON_2_GPIO)&(1U << BOARD_USER_BUTTON_2_GPIO_PIN))
  {
    user_button_callback();
    GPIO_PortClearInterruptFlags(BOARD_USER_BUTTON_2_GPIO, 1U << BOARD_USER_BUTTON_2_GPIO_PIN);
  }
  if(GPIO_GetPinsInterruptFlags(BOARD_LCD_TOUCH_INT_GPIO)&(1U << BOARD_LCD_TOUCH_INT_GPIO_PIN))
  {
    lcd_touch_int_callback();
    GPIO_PortClearInterruptFlags(BOARD_LCD_TOUCH_INT_GPIO, 1U << BOARD_LCD_TOUCH_INT_GPIO_PIN);
  }  
}

/***************************************************************************//*!
 * @brief   LCDIF_IRQHandler interrupt handler
 * @details LCD frame done interrupt.
 ******************************************************************************/
void LCDIF_IRQHandler(void)
{
  uint32_t intStatus;

  intStatus = ELCDIF_GetInterruptStatus(LCDIF);
  ELCDIF_ClearInterruptStatus(LCDIF, intStatus);
  if (intStatus & kELCDIF_CurFrameDone)
  {
    elcdif_frame_done_callback();
  }
}