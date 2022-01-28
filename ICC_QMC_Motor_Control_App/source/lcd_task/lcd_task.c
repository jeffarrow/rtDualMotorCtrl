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

#include "lcd_task.h"
#include "lcd_480x272_rgb888_frames.h"
#include "fsl_lpi2c.h"
#include "fsl_ft5406_rt.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void lcd_update_nfc_task(void *pvParameters);
static void lcd_update_task(void *pvParameters);
static void lcd_touch_task(void *pvParameters);

void BOARD_InitLcdifPixelClock(void);
void BOARD_InitLcd(void);

static void LcdTouchInit(void);
void BOARD_InitI2cTouchClock(void);

/*******************************************************************************
 * Globals
 ******************************************************************************/
const elcdif_rgb_mode_config_t elcdifRgbModeConfig = ELCDIF_RGB_MODE_CONFIG;
static volatile ft5406_rt_handle_t touchHandle;
static volatile int touch_x = 0, touch_y = 0;
const uint32_t buttonBufferAddresses[16] = {
  (uint32_t)button_0000_buffer, (uint32_t)button_0001_buffer, (uint32_t)button_0010_buffer, (uint32_t)button_0011_buffer,\
  (uint32_t)button_0100_buffer, (uint32_t)button_0101_buffer, (uint32_t)button_0110_buffer, (uint32_t)button_0111_buffer,\
  (uint32_t)button_1000_buffer, (uint32_t)button_1001_buffer, (uint32_t)button_1010_buffer, (uint32_t)button_1011_buffer,\
  (uint32_t)button_1100_buffer, (uint32_t)button_1101_buffer, (uint32_t)button_1110_buffer, (uint32_t)button_1111_buffer};
static uint32_t currButtonBufferAddressSel = 0;
static uint8_t numOfTouchedButton = 0; 

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief lcd_init_task function
 */
void lcd_init_task(void *pvParameters)
{
  BOARD_InitLcdifPixelClock();
  BOARD_InitI2cTouchClock();
  
  gpio_pin_config_t lcdTouchIntConfig = {kGPIO_DigitalInput, 0, kGPIO_IntFallingEdge};
  GPIO_PinInit(BOARD_LCD_TOUCH_INT_GPIO, BOARD_LCD_TOUCH_INT_GPIO_PIN, &lcdTouchIntConfig);
  GPIO_SetPinInterruptConfig(BOARD_LCD_TOUCH_INT_GPIO, 1U << BOARD_LCD_TOUCH_INT_GPIO_PIN, kGPIO_IntFallingEdge);
  GPIO_PortEnableInterrupts(BOARD_LCD_TOUCH_INT_GPIO, 1U << BOARD_LCD_TOUCH_INT_GPIO_PIN);
  LcdTouchInit();
  
  ELCDIF_RgbModeInit(LCDIF, &elcdifRgbModeConfig);
  EnableIRQ(LCDIF_IRQn);    
  ELCDIF_EnableInterrupts(LCDIF, kELCDIF_CurFrameDoneInterruptEnable);
  ELCDIF_RgbModeStart(LCDIF);
  
  if (xTaskCreate(lcd_update_nfc_task, "LCD_UPDATE_NFC_TASK", configMINIMAL_STACK_SIZE+100, NULL, tskIDLE_PRIORITY+4, NULL) !=
      pdPASS)
  {
      PRINTF("Task creation failed!.\r\n");
      while (1);
  }
  vTaskDelete(NULL);
}

/*!
 * @brief lcd_update_nfc_task function
 */
static void lcd_update_nfc_task(void *pvParameters)
{
  EventBits_t eventNfcTagDiscoverBit;
  
  while (1)
  {
    eventNfcTagDiscoverBit = xEventGroupWaitBits(eventGroupMotorControlData,
                                           EVENT_NFC_ID_LCD_FAIL|EVENT_NFC_ID_LCD_SUCCESS,
                                           pdTRUE,
                                           pdFALSE,
                                           pdMS_TO_TICKS(3000));
//                                           portMAX_DELAY);
    if (!(eventNfcTagDiscoverBit & (EVENT_NFC_ID_LCD_FAIL|EVENT_NFC_ID_LCD_SUCCESS)))
    {
      LCDIF->CUR_BUF = (uint32_t)&initial_page_buffer[0];
      LCDIF->NEXT_BUF = (uint32_t)&initial_page_buffer[0];      
    }
    if (eventNfcTagDiscoverBit & EVENT_NFC_ID_LCD_FAIL)
    {
      LCDIF->CUR_BUF = (uint32_t)&nfc_identification_fail_page[0];
      LCDIF->NEXT_BUF = (uint32_t)&nfc_identification_fail_page[0];
//      vTaskDelay(pdMS_TO_TICKS(3000));
//      LCDIF->CUR_BUF = (uint32_t)&initial_page_buffer[0];
//      LCDIF->NEXT_BUF = (uint32_t)&initial_page_buffer[0];
    }
    if (eventNfcTagDiscoverBit & EVENT_NFC_ID_LCD_SUCCESS)
    {
      LCDIF->CUR_BUF = (uint32_t)&nfc_identification_success_page[0];
      LCDIF->NEXT_BUF = (uint32_t)&nfc_identification_success_page[0];
      vTaskDelay(pdMS_TO_TICKS(3000));
      if (xTaskCreate(lcd_update_task, "LCD_UPDATE_TASK", configMINIMAL_STACK_SIZE+100, NULL, tskIDLE_PRIORITY+4, NULL) !=
          pdPASS)
      {
          PRINTF("Task creation failed!.\r\n");
          while (1);
      }
      if (xTaskCreate(lcd_touch_task, "LCD_TOUCH_TASK", configMINIMAL_STACK_SIZE+100, NULL, tskIDLE_PRIORITY+4, NULL) !=
          pdPASS)
      {
          PRINTF("Task creation failed!.\r\n");
          while (1);
      }
      if (xEventGroupSetBits(eventGroupMotorControlData,EVENT_MOTOR_ID_SUCCESS) == pdPASS);
      vTaskDelete(NULL);
    }
  }
}

/*!
 * @brief lcd_update_task function
 */
static void lcd_update_task(void *pvParameters)
{
  mc_status_data_t lcdMotorControlStatusData[4];  // may be better to put it on heap
  EventBits_t eventLcdUpdateBit;
  
  PRINTF("lcd_update_task entered.\r\n");
  
  LCDIF->CUR_BUF = (uint32_t)&button_0000_buffer[0];
  LCDIF->NEXT_BUF = (uint32_t)&button_0000_buffer[0];
  
  while (1)
  {
    eventLcdUpdateBit = xEventGroupWaitBits(eventGroupMotorControlData,
                                           EVENT_LCD_UPDATE,
                                           pdTRUE,
                                           pdFALSE,
                                           portMAX_DELAY);
    if ((eventLcdUpdateBit&EVENT_LCD_UPDATE) != 0)
    {
      if (xQueueReceive(queueLcdMotorControlStatusData, (void*)&lcdMotorControlStatusData, 0) == pdPASS)
      {
        currButtonBufferAddressSel = 0x0F&(bM1PositionDemo | (bM2PositionDemo<<1) | (bM3PositionDemo<<2) | (bM4PositionDemo<<3));
        LCDIF->CUR_BUF = buttonBufferAddresses[currButtonBufferAddressSel];
        LCDIF->NEXT_BUF = buttonBufferAddresses[currButtonBufferAddressSel];
      }
    }
  }
}

/*!
 * @brief lcd_touch_task function
 */
static void lcd_touch_task(void *pvParameters)
{
  //mc_command_data_t lcdMotorControlCommandData[4]; // may be better to put it on heap
  EventBits_t eventLcdBit;
  
  PRINTF("lcd_touch_task entered.\r\n");
  
  while (1)
  {
    eventLcdBit = xEventGroupWaitBits(eventGroupMotorControlData,
                                  EVENT_LCD_TOUCH,
                                  pdTRUE,
                                  pdFALSE,
                                  portMAX_DELAY);
    if (eventLcdBit & EVENT_LCD_TOUCH)
    {
      if (currButtonBufferAddressSel&(1<<(numOfTouchedButton-1)))
      {
        switch (numOfTouchedButton)
        {
        case 1: bM1PositionDemo = 0; break;
        case 2: bM2PositionDemo = 0; break;
        case 3: bM3PositionDemo = 0; break;
        case 4: bM4PositionDemo = 0; break;
        default: break;
        }
      }
      else
      {
        switch (numOfTouchedButton)
        {
        case 1: g_bM1SwitchAppOnOff = 0; bM1PositionDemo = 1; break;
        case 2: g_bM2SwitchAppOnOff = 0; bM2PositionDemo = 1; break;
        case 3: g_bM3SwitchAppOnOff = 0; bM3PositionDemo = 1; break;
        case 4: g_bM4SwitchAppOnOff = 0; bM4PositionDemo = 1; break;
        default: break;
        }
      }
    currButtonBufferAddressSel ^= (1<<(numOfTouchedButton-1));
    LCDIF->CUR_BUF = buttonBufferAddresses[currButtonBufferAddressSel];
    LCDIF->NEXT_BUF = buttonBufferAddresses[currButtonBufferAddressSel];
    numOfTouchedButton = 0;
    }
  }
}

void BOARD_InitLcdifPixelClock(void)
{
    clock_video_pll_config_t config = {
        .loopDivider = 34, .postDivider = 8, .numerator = 0, .denominator = 0,
    };
    CLOCK_InitVideoPll(&config);
    CLOCK_SetMux(kCLOCK_LcdifPreMux, 2);
    CLOCK_SetDiv(kCLOCK_LcdifPreDiv, 1);
    CLOCK_SetDiv(kCLOCK_LcdifDiv, 1);
}

void BOARD_InitI2cTouchClock(void)
{
    CLOCK_SetMux(kCLOCK_Lpi2cMux, 0U);
    CLOCK_SetDiv(kCLOCK_Lpi2cDiv, 5U);
}

void BOARD_InitLcd(void)
{
    volatile uint32_t i = 0x100U;

    gpio_pin_config_t config = {kGPIO_DigitalOutput, 0};
    GPIO_PinInit(BOARD_LCD_RST_GPIO, BOARD_LCD_RST_GPIO_PIN, &config);
    GPIO_PinWrite(BOARD_LCD_RST_GPIO, BOARD_LCD_RST_GPIO_PIN, 0);
    while (i--);
    GPIO_PinWrite(BOARD_LCD_RST_GPIO, BOARD_LCD_RST_GPIO_PIN, 1);
}

static void LcdTouchInit(void)
{
    lpi2c_master_config_t masterConfig = {0};
    LPI2C_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Hz = BOARD_TOUCH_I2C_BAUDRATE;
    LPI2C_MasterInit(BOARD_TOUCH_I2C_BASEADDR, &masterConfig, BOARD_TOUCH_I2C_CLK_FREQ);
    FT5406_RT_Init(&touchHandle, BOARD_TOUCH_I2C_BASEADDR);
}

void elcdif_frame_done_callback(void)
{
  //elcdifFrameDoneFlag = true;
}

void lcd_touch_int_callback(void)
{
  touch_event_t touch_event;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (kStatus_Success != FT5406_RT_GetSingleTouch(&touchHandle, &touch_event, &touch_x, &touch_y))
  {
      return;
  }
  else
  {
    switch(touch_event)
    {
    case kTouch_Down: 
      break;
    case kTouch_Up:
      if (xEventGroupSetBitsFromISR(eventGroupMotorControlData,EVENT_LCD_TOUCH,&xHigherPriorityTaskWoken) == pdPASS);
      if (((touch_y > 80) && (touch_y < 160)) && ((touch_x > 20) && (touch_x < 100)))
      {
        numOfTouchedButton = 1;
      }
      if (((touch_y > 80) && (touch_y < 160)) && ((touch_x > 140) && (touch_x < 220)))
      {
        numOfTouchedButton = 2;
      }
      if (((touch_y > 320) && (touch_y < 400)) && ((touch_x > 20) && (touch_x < 100)))
      {
        numOfTouchedButton = 3;
      }
      if (((touch_y > 320) && (touch_y < 400)) && ((touch_x > 140) && (touch_x < 220)))
      {
        numOfTouchedButton = 4;
      }
      break;
    case kTouch_Contact:
      break;
    default: break;
    }
  }
}
