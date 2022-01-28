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

/* FreeRTOS kernel includes. */
#include "main.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
#if defined(__CC_ARM) || defined(__GNUC__)
  __attribute__((used)) __attribute__((section (".ramfunc.$SRAM_ITC")))
#elif defined(__ICCARM__)
  __ramfunc
#endif
  void SoftwareHandler(void);
#if defined(__CC_ARM) || defined(__GNUC__)
  __attribute__((used)) __attribute__((section (".ramfunc.$SRAM_ITC")))
#elif defined(__ICCARM__)
  __ramfunc
#endif
  uint32_t move_data(const uint32_t cData);
  static void motor_nfc_identification_task(void *pvParameters);
  static void button_task(void *pvParameters);
  static void periodic_1s_task(void *pvParameters);

/*******************************************************************************
 * Globals
 ******************************************************************************/
QueueHandle_t queueEthernetMotorControlCommandData = NULL;
QueueHandle_t queueLcdMotorControlCommandData = NULL;
QueueHandle_t queueEthernetMotorControlStatusData = NULL;
QueueHandle_t queueLcdMotorControlStatusData = NULL;
EventGroupHandle_t eventGroupMotorControlData;
TaskHandle_t xNfcTaskHandle;
static mc_status_data_t appMotorControlStatusData[4];
static mc_command_data_t appMotorControlCommandData[4];

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */
int main(void)
{
  BOARD_ConfigMPU();
  BOARD_InitBootPins();
  BOARD_BootClockRUN();
  BOARD_InitDebugConsole();
  /* do not place a printf here*/
  
  BOARD_LCD_BACKLIGHT_INIT(1U);
  
  InstallIRQHandler(Reserved144_IRQn, (uint32_t)SoftwareHandler); // check whether this function does not enable irq at the end
  
  SoftwareHandler();

  PRINTF("This is FreeRTOS based ICC 4xMC demo (MCUXpresso).\r\n");
  
  queueEthernetMotorControlCommandData = xQueueCreate(1, (sizeof(mc_command_data_t)<<2));
  queueLcdMotorControlCommandData = xQueueCreate(1, (sizeof(mc_command_data_t)<<2));
  queueEthernetMotorControlStatusData = xQueueCreate(1, (sizeof(mc_status_data_t)<<2));
  queueLcdMotorControlStatusData = xQueueCreate(1, (sizeof(mc_status_data_t)<<2));

  eventGroupMotorControlData = xEventGroupCreate();

  if (xTaskCreate(lcd_init_task, "LCD_INIT_TASK", configMINIMAL_STACK_SIZE+100, NULL, tskIDLE_PRIORITY+3, NULL) !=
      pdPASS)
  {
      PRINTF("Task creation failed!.\r\n");
      while (1);
  }
  if (xTaskCreate(nfc_task, "NFC_TASK", 1024, NULL, tskIDLE_PRIORITY+3, &xNfcTaskHandle) !=
      pdPASS)
  {
      PRINTF("Task creation failed!.\r\n");
      while (1);
  }
  if (xTaskCreate(motor_nfc_identification_task, "MOTOR_NFC_IDENTIFICATION_TASK", 1024, NULL, tskIDLE_PRIORITY+3, NULL) !=
      pdPASS)
  {
      PRINTF("Task creation failed!.\r\n");
      while (1);
  }
  if (xTaskCreate(button_task, "BUTTON_TASK", configMINIMAL_STACK_SIZE+100, NULL, tskIDLE_PRIORITY+4, NULL) !=
      pdPASS)
  {
      PRINTF("Task creation failed!.\r\n");
      while (1);
  }
  vTaskStartScheduler();
  return 0;
}

/*!
 * @brief motor_nfc_identificaiton_task function
 */
static void motor_nfc_identification_task(void *pvParameters)
{
  EventBits_t eventMotorIdentificationBit;
  
  PRINTF("motor_nfc_identification_task entered.\r\n");

  while(1)
  {                            
    eventMotorIdentificationBit = xEventGroupWaitBits(eventGroupMotorControlData,
                                                    EVENT_MOTOR_ID_SUCCESS,
                                                    pdTRUE,
                                                    pdFALSE,
                                                    portMAX_DELAY);
    if (eventMotorIdentificationBit & EVENT_MOTOR_ID_SUCCESS)
    {
      PRINTF("Motor identification succesfull.\r\n");
      if (xTaskCreate(motor_control_init_task, "MOTOR_CONTROL_INIT_TASK", 1024, NULL, tskIDLE_PRIORITY+5, NULL) !=
          pdPASS)
      {
          PRINTF("Task creation failed!.\r\n");
          while (1);
      }
      if (xTaskCreate(periodic_1s_task, "PERIODIC_1S_TASK", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+4, NULL) !=
          pdPASS)
      {
          PRINTF("Task creation failed!.\r\n");
          while (1);
      }
      if (xTaskCreate(lwip_udp_init_task, "LWIP_UDP_INIT_TASK", 1024, NULL, tskIDLE_PRIORITY+4, NULL) !=
          pdPASS)
      {
          PRINTF("Task creation failed!.\r\n");
          while (1);
      }
      vTaskDelete(NULL);
    }
    else
    {
      PRINTF("Motor identification fail.\r\n");
    }
  }
}

/*!
 * @brief button_task function
 */
static void button_task(void *pvParameters)
{
  gpio_pin_config_t buttonConfig = {kGPIO_DigitalInput, 0, kGPIO_IntFallingEdge};
  
  PRINTF("button_task entered.\r\n");

  GPIO_PinInit(BOARD_USER_BUTTON_1_GPIO, BOARD_USER_BUTTON_1_GPIO_PIN, &buttonConfig);
  GPIO_PortEnableInterrupts(BOARD_USER_BUTTON_1_GPIO, 1U << BOARD_USER_BUTTON_1_GPIO_PIN);
  GPIO_PinInit(BOARD_USER_BUTTON_2_GPIO, BOARD_USER_BUTTON_2_GPIO_PIN, &buttonConfig);
  GPIO_PortEnableInterrupts(BOARD_USER_BUTTON_2_GPIO, 1U << BOARD_USER_BUTTON_2_GPIO_PIN);
  GPIO_PortClearInterruptFlags(BOARD_USER_BUTTON_1_GPIO, 1U << BOARD_USER_BUTTON_1_GPIO_PIN);
  GPIO_PortClearInterruptFlags(BOARD_USER_BUTTON_2_GPIO, 1U << BOARD_USER_BUTTON_2_GPIO_PIN);
  NVIC_EnableIRQ(BOARD_USER_BUTTON_1_IRQ);
  NVIC_SetPriority(BOARD_USER_BUTTON_1_IRQ, configLIBRARY_LOWEST_INTERRUPT_PRIORITY); // lower priority 15

  vTaskDelete(NULL);
}

/*!
 * @brief periodic_1s_task function
 */
static void periodic_1s_task(void *pvParameters)
{
  TickType_t xLastWakeTime;
//  EventBits_t eventPeriodicBit;
  
  PRINTF("periodic_1s_task entered.\r\n");
  
  xLastWakeTime = xTaskGetTickCount();
  
  while (1)
  {
    // handle APP data update
    // Motor 1 status data
    appMotorControlStatusData[0].appStatus = (mc_app_status_t)g_bM1SwitchAppOnOff;
    appMotorControlStatusData[0].motorState = (mc_state_t)g_sM1Ctrl.eState;
    appMotorControlStatusData[0].faultStatus = (mc_fault_t)g_sM1Drive.sFaultIdCaptured;
    appMotorControlStatusData[0].f32Speed = g_sM1Drive.sSpeed.fltSpeedFilt;
    appMotorControlStatusData[0].u32Position = g_sM1Drive.sPosition.a32Position;
    appMotorControlStatusData[0].f32Iq = g_sM1Drive.sFocPMSM.sIDQ.fltQ;
    appMotorControlStatusData[0].f32UDcBus = g_sM1Drive.sFocPMSM.fltUDcBusFilt;
    // Motor 2 status data
    appMotorControlStatusData[1].appStatus = (mc_app_status_t)g_bM2SwitchAppOnOff;
    appMotorControlStatusData[1].motorState = (mc_state_t)g_sM2Ctrl.eState;
    appMotorControlStatusData[1].faultStatus = (mc_fault_t)g_sM2Drive.sFaultIdCaptured;
    appMotorControlStatusData[1].f32Speed = g_sM2Drive.sSpeed.fltSpeedFilt;
    appMotorControlStatusData[1].u32Position = g_sM2Drive.sPosition.a32Position;
    appMotorControlStatusData[1].f32Iq = g_sM2Drive.sFocPMSM.sIDQ.fltQ;
    appMotorControlStatusData[1].f32UDcBus = g_sM2Drive.sFocPMSM.fltUDcBusFilt;
    // Motor 3 status data
    appMotorControlStatusData[2].appStatus = (mc_app_status_t)g_bM3SwitchAppOnOff;
    appMotorControlStatusData[2].motorState = (mc_state_t)g_sM3Ctrl.eState;
    appMotorControlStatusData[2].faultStatus = (mc_fault_t)g_sM3Drive.sFaultIdCaptured;
    appMotorControlStatusData[2].f32Speed = g_sM3Drive.sSpeed.fltSpeedFilt;
    appMotorControlStatusData[2].u32Position = g_sM3Drive.sPosition.a32Position;
    appMotorControlStatusData[2].f32Iq = g_sM3Drive.sFocPMSM.sIDQ.fltQ;
    appMotorControlStatusData[2].f32UDcBus = g_sM3Drive.sFocPMSM.fltUDcBusFilt;
    // Motor 4 status data
    appMotorControlStatusData[3].appStatus = (mc_app_status_t)g_bM4SwitchAppOnOff;
    appMotorControlStatusData[3].motorState = (mc_state_t)g_sM4Ctrl.eState;
    appMotorControlStatusData[3].faultStatus = (mc_fault_t)g_sM4Drive.sFaultIdCaptured;
    appMotorControlStatusData[3].f32Speed = g_sM4Drive.sSpeed.fltSpeedFilt;
    appMotorControlStatusData[3].u32Position = g_sM4Drive.sPosition.a32Position;
    appMotorControlStatusData[3].f32Iq = g_sM4Drive.sFocPMSM.sIDQ.fltQ;
    appMotorControlStatusData[3].f32UDcBus = g_sM4Drive.sFocPMSM.fltUDcBusFilt;
    // Send data into the ethernet queue
    if (xQueueSend(queueEthernetMotorControlStatusData, (void*)&appMotorControlStatusData, NULL) != pdPASS )
    {
      //PRINTF("Could not send a App Queue because it is already full.\r\n");
    }
    if (xQueueSend(queueLcdMotorControlStatusData, (void*)&appMotorControlStatusData, NULL) != pdPASS )
    {
      //PRINTF("Could not send a App Queue because it is already full.\r\n");
    }
    if (xEventGroupSetBits(eventGroupMotorControlData,(EVENT_PERIODIC_1S|EVENT_ETHERNET_TX|EVENT_LCD_UPDATE)) == pdPASS);
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000)); // temporarily 1s
  }
}

/*!
 * @brief user_button_callback function
 */
void user_button_callback(void)
{  
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if(eTaskGetState(xNfcTaskHandle) == eRunning)
  {
    vTaskDelete(xNfcTaskHandle);
    PRINTF("\nMotor identification bypassed.\n");
    if (xEventGroupSetBitsFromISR(eventGroupMotorControlData,EVENT_NFC_ID_LCD_SUCCESS,&xHigherPriorityTaskWoken) == pdPASS);
    BOARD_DeinitNfcPins();
  }
  if (xQueueSendFromISR(queueEthernetMotorControlStatusData, (void*)&appMotorControlStatusData, NULL) != pdPASS )
  {
#ifdef DEBUG
    PRINTF("Could not send a Ethernet Queue because it is already full.\r\n");
#endif  
  }
}

void SoftwareHandler(void)
{
//#ifdef DEBUG
//  __asm("BKPT #0x03");
//#endif  
}
