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

#include "lwip/opt.h"

#if LWIP_NETCONN

#include "lwip/api.h"
#include "lwip/sys.h"
#include "lwip/pbuf.h"

#include "lwip_task.h"

#include "json.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void lwip_udp_send_packet_task(void *pvParameters);
static void lwip_udp_receive_packet_task(void *pvParameters);
static void BOARD_InitModuleClock(void);
static void delay(void);
   
/*******************************************************************************
 * Globals
 ******************************************************************************/
struct netconn *netConn;
struct netbuf volatile *netBuffer;
static uint16_t netBufferPort;
static uint8_t jsonStringRxBuffer[512];
static uint8_t jsonStringTxBuffer[512];

//static mc_data_t ethernetMotorControlData;

static ip4_addr_t fsl_netif0_ipaddr, fsl_netif0_netmask, fsl_netif0_gw;
   
/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief lwip_udp_init_task function
 */
void lwip_udp_init_task(void *pvParameters)
{
  gpio_pin_config_t gpio_config = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};
  static struct netif fsl_netif0;
  
  BOARD_InitModuleClock();
  IOMUXC_EnableMode(IOMUXC_GPR, kIOMUXC_GPR_ENET1TxClkOutputDir, true);

  GPIO_PinInit(GPIO1, 9, &gpio_config);
  GPIO_PinInit(GPIO1, 10, &gpio_config);
  GPIO_WritePinOutput(BOARD_ENET_INT_GPIO, BOARD_ENET_INT_GPIO_PIN, 1);
  GPIO_WritePinOutput(BOARD_ENET_RST_GPIO, BOARD_ENET_RST_GPIO_PIN, 0);
  delay();
  GPIO_WritePinOutput(BOARD_ENET_RST_GPIO, BOARD_ENET_RST_GPIO_PIN, 1);
  
  ethernetif_config_t fsl_enet_config0 = {
      .phyAddress = BOARD_ENET0_PHY_ADDRESS,
      .clockName = kCLOCK_CoreSysClk,
      .macAddress = configMAC_ADDR,
  };
  IP4_ADDR(&fsl_netif0_ipaddr, configIP_ADDR0, configIP_ADDR1, configIP_ADDR2, configIP_ADDR3);
  IP4_ADDR(&fsl_netif0_netmask, configNET_MASK0, configNET_MASK1, configNET_MASK2, configNET_MASK3);
  IP4_ADDR(&fsl_netif0_gw, configGW_ADDR0, configGW_ADDR1, configGW_ADDR2, configGW_ADDR3);
  
  tcpip_init(NULL, NULL);
  
  netif_add(&fsl_netif0, &fsl_netif0_ipaddr, &fsl_netif0_netmask, &fsl_netif0_gw,
          &fsl_enet_config0, ethernetif0_init, tcpip_input);
  netif_set_default(&fsl_netif0);
  netif_set_up(&fsl_netif0);

  PRINTF("************************************************\r\n");
  PRINTF(" IPv4 Address     : %u.%u.%u.%u\r\n", ((u8_t *)&fsl_netif0_ipaddr)[0], ((u8_t *)&fsl_netif0_ipaddr)[1],
         ((u8_t *)&fsl_netif0_ipaddr)[2], ((u8_t *)&fsl_netif0_ipaddr)[3]);
  PRINTF(" IPv4 Subnet mask : %u.%u.%u.%u\r\n", ((u8_t *)&fsl_netif0_netmask)[0], ((u8_t *)&fsl_netif0_netmask)[1],
         ((u8_t *)&fsl_netif0_netmask)[2], ((u8_t *)&fsl_netif0_netmask)[3]);
  PRINTF(" IPv4 Gateway     : %u.%u.%u.%u\r\n", ((u8_t *)&fsl_netif0_gw)[0], ((u8_t *)&fsl_netif0_gw)[1],
         ((u8_t *)&fsl_netif0_gw)[2], ((u8_t *)&fsl_netif0_gw)[3]);
  PRINTF("************************************************\r\n");

  netConn = netconn_new(NETCONN_UDP);
  netconn_bind(netConn, IP_ADDR_ANY, PORT);
  LWIP_ERROR("udpecho: invalid netConn", (netConn != NULL), return;);
  
  if (xTaskCreate(lwip_udp_send_packet_task, "LWIP_UDP_SEND_PACKET_TASK", 3000, NULL, tskIDLE_PRIORITY+5, NULL) !=
    pdPASS)
  {
      PRINTF("Task creation failed!.\r\n");
      while (1);
  }
  if (xTaskCreate(lwip_udp_receive_packet_task, "LWIP_UDP_RECEIVE_PACKET_TASK", 3000, NULL, tskIDLE_PRIORITY+6, NULL) !=
      pdPASS)
  {
      PRINTF("Task creation failed!.\r\n");
      while (1);
  }
  
  vTaskDelete(NULL);
}

/*!
 * @brief lwip_udp_receive_packet_task function
 */
static void lwip_udp_receive_packet_task(void *pvParameters)
{
  mc_command_data_t ethernetMotorControlCommandData[4];
  err_t netConnError;
  uint16_t motorId;
  
  PRINTF("lwip_udp_receive_packet_task entered.\r\n");
  vTaskDelay(pdMS_TO_TICKS(1000));
  
  while(1)
  {
    netConnError = netconn_recv(netConn, &netBuffer);
    if (netConnError == ERR_OK)
    {
      netBufferPort = netBuffer->port;
      if(netbuf_copy(netBuffer, jsonStringRxBuffer, sizeof(jsonStringRxBuffer)) != netBuffer->p->tot_len)
      {
        PRINTF("Received message fail/corrupted.\r\n");
      }
      else
      {
        netbuf_delete(netBuffer);
        motorId = json_decode(jsonStringRxBuffer, &ethernetMotorControlCommandData);
        switch(motorId)
        {
        case 0: 
          {
            // Motor 1 
            g_bM1SwitchAppOnOff = ethernetMotorControlCommandData[0].appStatus;
            g_sM1Drive.eControl = (mcs_ctrl_mode_t)ethernetMotorControlCommandData[0].controlMethodSel;
            g_sM1Drive.sSpeed.fltSpeedCmd = ethernetMotorControlCommandData[0].f32Speed;
            g_sM1Drive.sPosition.a32PositionCmd = ethernetMotorControlCommandData[0].u32Position;
            bM1SpeedDemo  = ethernetMotorControlCommandData[0].bDemoModeSpeed;
            bM1PositionDemo = ethernetMotorControlCommandData[0].bDemoModePosition;            
          }
          break;
        case 1: 
          {
            // Motor 2
            g_bM2SwitchAppOnOff = ethernetMotorControlCommandData[1].appStatus;
            g_sM2Drive.eControl = (mcs_ctrl_mode_t)ethernetMotorControlCommandData[1].controlMethodSel;
            g_sM2Drive.sSpeed.fltSpeedCmd = ethernetMotorControlCommandData[1].f32Speed;
            g_sM2Drive.sPosition.a32PositionCmd = ethernetMotorControlCommandData[1].u32Position;
            bM2SpeedDemo = ethernetMotorControlCommandData[1].bDemoModeSpeed;        
            bM2PositionDemo = ethernetMotorControlCommandData[1].bDemoModePosition;           
          }
          break;
        case 2: 
          {
            // Motor 3
            g_bM3SwitchAppOnOff = ethernetMotorControlCommandData[2].appStatus;
            g_sM3Drive.eControl = (mcs_ctrl_mode_t)ethernetMotorControlCommandData[2].controlMethodSel;
            g_sM3Drive.sSpeed.fltSpeedCmd = ethernetMotorControlCommandData[2].f32Speed;
            g_sM3Drive.sPosition.a32PositionCmd = ethernetMotorControlCommandData[2].u32Position;  
            bM3SpeedDemo = ethernetMotorControlCommandData[2].bDemoModeSpeed;
            bM3PositionDemo = ethernetMotorControlCommandData[2].bDemoModePosition;          
          }
          break;
        case 3: 
          {
            // Motor 4       
            g_bM4SwitchAppOnOff = ethernetMotorControlCommandData[3].appStatus;
            g_sM4Drive.eControl = (mcs_ctrl_mode_t)ethernetMotorControlCommandData[3].controlMethodSel;
            g_sM4Drive.sSpeed.fltSpeedCmd = ethernetMotorControlCommandData[3].f32Speed;
            g_sM4Drive.sPosition.a32PositionCmd = ethernetMotorControlCommandData[3].u32Position;   
            bM4SpeedDemo = ethernetMotorControlCommandData[3].bDemoModeSpeed;
            bM4PositionDemo = ethernetMotorControlCommandData[3].bDemoModePosition;        
          }
          break;
        default: break;
        }
        // Send message to console 
        PRINTF("Received message: %s\r\n", jsonStringRxBuffer);
      }
    }
  }
}

/*!
 * @brief lwip_udp_send_packet_task function
 */
static void lwip_udp_send_packet_task(void *pvParameters)
{
  mc_status_data_t ethernetMotorControlStatusData[4];
  err_t netConnError;
  EventBits_t eventEthernetTxBit;
  static uint8_t motorId = 0;
  
  PRINTF("lwip_udp_send_packet_task entered.\r\n");
  vTaskDelay(pdMS_TO_TICKS(1000));
  
  while(1)
  {
    eventEthernetTxBit = xEventGroupWaitBits(eventGroupMotorControlData,
                                             EVENT_ETHERNET_TX,
                                             pdTRUE,
                                             pdFALSE,
                                             portMAX_DELAY);
    if ((eventEthernetTxBit&EVENT_ETHERNET_TX) != 0)
    {
      if(xQueueReceive(queueEthernetMotorControlStatusData, (void*)&ethernetMotorControlStatusData, 0) == pdPASS)
      {
        json_encode(&ethernetMotorControlStatusData, motorId, jsonStringTxBuffer);
        motorId++;
        motorId &= 0x3;
      }
      netBuffer = netbuf_new();
      netBuffer->addr = fsl_netif0_gw;
      netBuffer->p->type_internal = 3;
      netBuffer->p->flags = 0;
      netBuffer->p->ref = 1;
      netbuf_ref(netBuffer, (void*)jsonStringTxBuffer, sizeof(jsonStringTxBuffer));
      netBuffer->port = netBufferPort;
      netConnError = netconn_send(netConn, netBuffer);
      if(netConnError != ERR_OK) 
      {
        LWIP_DEBUGF(LWIP_DBG_ON, ("netconn_send failed: %d\n", (int)netConnError));
      }
      else
      {
        //PRINTF("Message sent (UDP in JSON)\r\n%s");
      }
      netbuf_delete(netBuffer);
    }
  }
}

/*!
 * @brief BOARD_InitModuleClock to initialize enet_pll
 */
static void BOARD_InitModuleClock(void)
{
    const clock_enet_pll_config_t config = {true, false, 1};
    CLOCK_InitEnetPll(&config);
}

/*!
 * @brief Simple NOP based delay function
 */
static void delay(void)
{
    volatile uint32_t i = 0;
    for (i = 0; i < 1000000; ++i)
    {
        __asm("NOP");
    }
}

#endif
