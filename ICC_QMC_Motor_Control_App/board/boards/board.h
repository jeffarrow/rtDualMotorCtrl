/*
 * The Clear BSD License
 * Copyright 2017 NXP
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

#ifndef _BOARD_H_
#define _BOARD_H_

#include "clock_config.h"
#include "fsl_common.h"
#include "fsl_gpio.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief The board name */
#define BOARD_NAME "RT1052-SD200"

/* The UART to use for debug messages. */
#define BOARD_DEBUG_UART_TYPE DEBUG_CONSOLE_DEVICE_TYPE_LPUART
#define BOARD_DEBUG_UART_BASEADDR (uint32_t) LPUART6
#define BOARD_DEBUG_UART_INSTANCE 6U

#define BOARD_DEBUG_UART_CLK_FREQ BOARD_DebugConsoleSrcFreq()

#define BOARD_UART_IRQ LPUART6_IRQn
#define BOARD_UART_IRQ_HANDLER LPUART6_IRQHandler

#ifndef BOARD_DEBUG_UART_BAUDRATE
#define BOARD_DEBUG_UART_BAUDRATE (115200U)
#endif /* BOARD_DEBUG_UART_BAUDRATE */

/*! @brief The UART to use for FreeMASTER communication */
#define BOARD_FMSTR_LPUART (6) 
#define BOARD_FMSTR_UART_PORT LPUART6
#define BOARD_FMSTR_UART_BAUDRATE 115200U
#define BOARD_FMSTR_UART_TYPE BOARD_FMSTR_LPUART 
#define BOARD_FMSTR_USE_TSA 1 

/*! @brief The USER_LED used for board */
#define LOGIC_LED_ON (0U)
#define LOGIC_LED_OFF (1U)
#ifndef BOARD_USER_LED1_GPIO
#define BOARD_USER_LED1_GPIO GPIO2
#endif
#ifndef BOARD_USER_LED1_GPIO_PIN
#define BOARD_USER_LED1_GPIO_PIN (13U)
#endif
#ifndef BOARD_USER_LED2_GPIO
#define BOARD_USER_LED2_GPIO GPIO2
#endif
#ifndef BOARD_USER_LED2_GPIO_PIN
#define BOARD_USER_LED2_GPIO_PIN (14U)
#endif
#ifndef BOARD_USER_LED3_GPIO
#define BOARD_USER_LED3_GPIO GPIO4
#endif
#ifndef BOARD_USER_LED3_GPIO_PIN
#define BOARD_USER_LED3_GPIO_PIN (10U)
#endif
#ifndef BOARD_USER_LED4_GPIO
#define BOARD_USER_LED4_GPIO GPIO3
#endif
#ifndef BOARD_USER_LED4_GPIO_PIN
#define BOARD_USER_LED4_GPIO_PIN (27U)
#endif

#define USER_LED1_INIT(output)                                            \
    GPIO_PinWrite(BOARD_USER_LED1_GPIO, BOARD_USER_LED1_GPIO_PIN, output); \
    BOARD_USER_LED1_GPIO->GDIR |= (1U << BOARD_USER_LED1_GPIO_PIN) /*!< Enable target USER_LED1 */
#define USER_LED1_ON() \
    GPIO_PortClear(BOARD_USER_LED1_GPIO, 1U << BOARD_USER_LED1_GPIO_PIN)                  /*!< Turn off target USER_LED1 */
#define USER_LED1_OFF() GPIO_PortSet(BOARD_USER_LED1_GPIO, 1U << BOARD_USER_LED1_GPIO_PIN) /*!<Turn on target USER_LED1 */
#define USER_LED1_TOGGLE()                                       \
    GPIO_PinWrite(BOARD_USER_LED1_GPIO, BOARD_USER_LED1_GPIO_PIN, \
                  0x1 ^ GPIO_PinRead(BOARD_USER_LED1_GPIO, BOARD_USER_LED1_GPIO_PIN)) /*!< Toggle target USER_LED1 */

#define USER_LED2_INIT(output)                                            \
    GPIO_PinWrite(BOARD_USER_LED2_GPIO, BOARD_USER_LED2_GPIO_PIN, output); \
    BOARD_USER_LED2_GPIO->GDIR |= (1U << BOARD_USER_LED2_GPIO_PIN) /*!< Enable target USER_LED2 */
#define USER_LED2_ON() \
    GPIO_PortClear(BOARD_USER_LED2_GPIO, 1U << BOARD_USER_LED2_GPIO_PIN)                  /*!< Turn off target USER_LED2 */
#define USER_LED2_OFF() GPIO_PortSet(BOARD_USER_LED2_GPIO, 1U << BOARD_USER_LED2_GPIO_PIN) /*!<Turn on target USER_LED2 */
#define USER_LED2_TOGGLE()                                       \
    GPIO_PinWrite(BOARD_USER_LED2_GPIO, BOARD_USER_LED2_GPIO_PIN, \
                  0x1 ^ GPIO_PinRead(BOARD_USER_LED2_GPIO, BOARD_USER_LED2_GPIO_PIN)) /*!< Toggle target USER_LED2 */
      
#define USER_LED3_INIT(output)                                            \
    GPIO_PinWrite(BOARD_USER_LED3_GPIO, BOARD_USER_LED3_GPIO_PIN, output); \
    BOARD_USER_LED3_GPIO->GDIR |= (1U << BOARD_USER_LED3_GPIO_PIN) /*!< Enable target USER_LED3 */
#define USER_LED3_ON() \
    GPIO_PortClear(BOARD_USER_LED3_GPIO, 1U << BOARD_USER_LED3_GPIO_PIN)                  /*!< Turn off target USER_LED3 */
#define USER_LED3_OFF() GPIO_PortSet(BOARD_USER_LED3_GPIO, 1U << BOARD_USER_LED3_GPIO_PIN) /*!<Turn on target USER_LED3 */
#define USER_LED3_TOGGLE()                                       \
    GPIO_PinWrite(BOARD_USER_LED3_GPIO, BOARD_USER_LED3_GPIO_PIN, \
                  0x1 ^ GPIO_PinRead(BOARD_USER_LED3_GPIO, BOARD_USER_LED3_GPIO_PIN)) /*!< Toggle target USER_LED3 */
      
#define USER_LED4_INIT(output)                                            \
    GPIO_PinWrite(BOARD_USER_LED4_GPIO, BOARD_USER_LED4_GPIO_PIN, output); \
    BOARD_USER_LED4_GPIO->GDIR |= (1U << BOARD_USER_LED4_GPIO_PIN) /*!< Enable target USER_LED4 */
#define USER_LED4_ON() \
    GPIO_PortClear(BOARD_USER_LED4_GPIO, 1U << BOARD_USER_LED4_GPIO_PIN)                  /*!< Turn off target USER_LED4 */
#define USER_LED4_OFF() GPIO_PortSet(BOARD_USER_LED4_GPIO, 1U << BOARD_USER_LED4_GPIO_PIN) /*!<Turn on target USER_LED4 */
#define USER_LED4_TOGGLE()                                       \
    GPIO_PinWrite(BOARD_USER_LED4_GPIO, BOARD_USER_LED4_GPIO_PIN, \
                  0x1 ^ GPIO_PinRead(BOARD_USER_LED4_GPIO, BOARD_USER_LED4_GPIO_PIN)) /*!< Toggle target USER_LED4 */

/*! @brief Define the port interrupt number for the board switches */
#ifndef BOARD_USER_BUTTON_1_GPIO
#define BOARD_USER_BUTTON_1_GPIO GPIO1
#endif
#ifndef BOARD_USER_BUTTON_1_GPIO_PIN
#define BOARD_USER_BUTTON_1_GPIO_PIN (4U)
#endif
#define BOARD_USER_BUTTON_1_IRQ GPIO1_Combined_0_15_IRQn
#define BOARD_USER_BUTTON_1_IRQ_HANDLER GPIO1_Combined_0_15_IRQHandler
#define BOARD_USER_BUTTON_1_NAME "SW240"
      
#ifndef BOARD_USER_BUTTON_2_GPIO
#define BOARD_USER_BUTTON_2_GPIO GPIO1
#endif
#ifndef BOARD_USER_BUTTON_2_GPIO_PIN
#define BOARD_USER_BUTTON_2_GPIO_PIN (5U)
#endif
#define BOARD_USER_BUTTON_2_IRQ GPIO1_Combined_0_15_IRQn
#define BOARD_USER_BUTTON_2_IRQ_HANDLER GPIO1_Combined_0_15_IRQHandler
#define BOARD_USER_BUTTON_2_NAME "SW241"
      
/*! @brief LCD RST */
#ifndef BOARD_LCD_RST_GPIO
#define BOARD_LCD_RST_GPIO GPIO3
#endif
#ifndef BOARD_LCD_RST_GPIO_PIN
#define BOARD_LCD_RST_GPIO_PIN (26U)
#endif

      
/* The LCD Touch I2C */
#define BOARD_TOUCH_I2C_BASEADDR LPI2C4
#define BOARD_TOUCH_I2C_INSTANCE 4U
#define BOARD_TOUCH_I2C_CLK_FREQ ((CLOCK_GetFreq(kCLOCK_Usb1PllClk) / 8) / (5U + 1U))
#define BOARD_TOUCH_I2C_BAUDRATE (100000U)
      
/*! @brief LCD TOUCH */
#ifndef BOARD_LCD_TOUCH_INT_GPIO
#define BOARD_LCD_TOUCH_INT_GPIO GPIO1
#endif
#ifndef BOARD_LCD_TOUCH_INT_GPIO_PIN
#define BOARD_LCD_TOUCH_INT_GPIO_PIN (11U)
#endif
#define BOARD_LCD_TOUCH_INT_IRQ GPIO1_Combined_0_15_IRQn
#define BOARD_LCD_TOUCH_INT_IRQ_HANDLER GPIO1_Combined_0_15_IRQHandler
#define BOARD_LCD_TOUCH_INT_NAME "LCD_TOUCH_INT"
      
/*! @brief LCD BACKLIGHT */
#ifndef BOARD_LCD_BACKLIGHT_GPIO
#define BOARD_LCD_BACKLIGHT_GPIO GPIO1
#endif
#ifndef BOARD_LCD_BACKLIGHT_GPIO_PIN
#define BOARD_LCD_BACKLIGHT_GPIO_PIN (8U)
#endif

#define  BOARD_LCD_BACKLIGHT_INIT(output)                                            \
    GPIO_PinWrite( BOARD_LCD_BACKLIGHT_GPIO,  BOARD_LCD_BACKLIGHT_GPIO_PIN, output); \
     BOARD_LCD_BACKLIGHT_GPIO->GDIR |= (1U <<  BOARD_LCD_BACKLIGHT_GPIO_PIN) /*!< Enable target LCD_BACKLIGHT */
#define  BOARD_LCD_BACKLIGHT_ON() \
    GPIO_PortClear( BOARD_LCD_BACKLIGHT_GPIO, 1U <<  BOARD_LCD_BACKLIGHT_GPIO_PIN)                  /*!< Turn off target LCD_BACKLIGHT */
#define  BOARD_LCD_BACKLIGHT_OFF() GPIO_PortSet( BOARD_LCD_BACKLIGHT_GPIO, 1U <<  BOARD_LCD_BACKLIGHT_GPIO_PIN) /*!<Turn on target LCD_BACKLIGHT */
#define   BOARD_LCD_BACKLIGHT_TOGGLE()                                       \
    GPIO_PinWrite( BOARD_LCD_BACKLIGHT_GPIO,  BOARD_LCD_BACKLIGHT_GPIO_PIN, \
                  0x1 ^ GPIO_PinRead( BOARD_LCD_BACKLIGHT_GPIO,  BOARD_LCD_BACKLIGHT_GPIO_PIN)) /*!< Toggle target LCD_BACKLIGHT */      
      
/*! @brief The qspi flash size */
#define BOARD_FLASH_A_SIZE    (0x800000U)
#define BOARD_FLASH_B_SIZE    (0x800000U)
#define BOARD_FLASH_SIZE      BOARD_FLASH_A_SIZE

/*! @brief The Enet instance used for board. */
#define BOARD_ENET_BASEADDR ENET

/*! @brief The ENET PHY address. */
#define BOARD_ENET0_PHY_ADDRESS (0x02U) /* Phy address of enet port 0. */
      
/*! @brief ENET RST */
#ifndef BOARD_ENET_RST_GPIO
#define BOARD_ENET_RST_GPIO GPIO1
#endif
#ifndef BOARD_ENET_RST_GPIO_PIN
#define BOARD_ENET_RST_GPIO_PIN (9U)
#endif

/*! @brief ENET INT */
#ifndef BOARD_ENET_INT_GPIO
#define BOARD_ENET_INT_GPIO GPIO1
#endif
#ifndef BOARD_ENET_INT_GPIO_PIN
#define BOARD_ENET_INT_GPIO_PIN (10U)
#endif
      
/* USB PHY condfiguration */
#define BOARD_USB_PHY_D_CAL (0x0CU)
#define BOARD_USB_PHY_TXCAL45DP (0x06U)
#define BOARD_USB_PHY_TXCAL45DM (0x06U)

#define BOARD_USDHC1_BASEADDR USDHC1
#define BOARD_USDHC_CD_GPIO_BASE GPIO2
#define BOARD_USDHC_CD_GPIO_PIN 28
#define BOARD_USDHC_CD_PORT_IRQ GPIO2_Combined_16_31_IRQn
#define BOARD_USDHC_CD_PORT_IRQ_HANDLER GPIO2_Combined_16_31_IRQHandler

#define BOARD_USDHC_CD_STATUS() (GPIO_PinRead(BOARD_USDHC_CD_GPIO_BASE, BOARD_USDHC_CD_GPIO_PIN))

#define BOARD_USDHC_CD_INTERRUPT_STATUS() (GPIO_PortGetInterruptFlags(BOARD_USDHC_CD_GPIO_BASE))
#define BOARD_USDHC_CD_CLEAR_INTERRUPT(flag) (GPIO_PortClearInterruptFlags(BOARD_USDHC_CD_GPIO_BASE, flag))

#define BOARD_USDHC_CD_GPIO_INIT()                                                          \
    {                                                                                       \
        gpio_pin_config_t sw_config = {                                                     \
            kGPIO_DigitalInput, 0, kGPIO_IntFallingEdge,                                    \
        };                                                                                  \
        GPIO_PinInit(BOARD_USDHC_CD_GPIO_BASE, BOARD_USDHC_CD_GPIO_PIN, &sw_config);        \
        GPIO_PortEnableInterrupts(BOARD_USDHC_CD_GPIO_BASE, 1U << BOARD_USDHC_CD_GPIO_PIN); \
        GPIO_PortClearInterruptFlags(BOARD_USDHC_CD_GPIO_BASE, ~0);                         \
    }

#define BOARD_SD_POWER_RESET_GPIO (GPIO1)
#define BOARD_SD_POWER_RESET_GPIO_PIN (5U)

#define BOARD_USDHC_CARD_INSERT_CD_LEVEL (0U)

#define BOARD_USDHC_SDCARD_POWER_CONTROL_INIT()                                             \
    {                                                                                       \
        gpio_pin_config_t sw_config = {                                                     \
            kGPIO_DigitalOutput, 0, kGPIO_NoIntmode,                                        \
        };                                                                                  \
        GPIO_PinInit(BOARD_SD_POWER_RESET_GPIO, BOARD_SD_POWER_RESET_GPIO_PIN, &sw_config); \
    }

#define BOARD_USDHC_SDCARD_POWER_CONTROL(state) \
    (GPIO_PinWrite(BOARD_SD_POWER_RESET_GPIO, BOARD_SD_POWER_RESET_GPIO_PIN, state))

#define BOARD_USDHC1_CLK_FREQ (CLOCK_GetSysPfdFreq(kCLOCK_Pfd0) / (CLOCK_GetDiv(kCLOCK_Usdhc1Div) + 1U))

#define BOARD_SD_HOST_BASEADDR BOARD_USDHC1_BASEADDR
#define BOARD_SD_HOST_CLK_FREQ BOARD_USDHC1_CLK_FREQ
#define BOARD_SD_HOST_IRQ USDHC1_IRQn
/* we are using the BB SD socket to DEMO the MMC example,but the
* SD socket provide 4bit bus only, so we define this macro to avoid
* 8bit data bus test
*/

#define BOARD_SD_HOST_SUPPORT_SDR104_FREQ (200000000U)
#define BOARD_SD_HOST_SUPPORT_HS200_FREQ (180000000U)
/* define for SD/MMC config IO driver strength dynamic */
#define BOARD_SD_PIN_CONFIG(speed, strength)                                                      \
    {                                                                                             \
        IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_00_USDHC1_CMD,                                      \
                            IOMUXC_SW_PAD_CTL_PAD_SPEED(speed) | IOMUXC_SW_PAD_CTL_PAD_SRE_MASK | \
                                IOMUXC_SW_PAD_CTL_PAD_PKE_MASK | IOMUXC_SW_PAD_CTL_PAD_PUE_MASK | \
                                IOMUXC_SW_PAD_CTL_PAD_HYS_MASK | IOMUXC_SW_PAD_CTL_PAD_PUS(1) |   \
                                IOMUXC_SW_PAD_CTL_PAD_DSE(strength));                             \
        IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_01_USDHC1_CLK,                                      \
                            IOMUXC_SW_PAD_CTL_PAD_SPEED(speed) | IOMUXC_SW_PAD_CTL_PAD_SRE_MASK | \
                                IOMUXC_SW_PAD_CTL_PAD_HYS_MASK | IOMUXC_SW_PAD_CTL_PAD_PUS(0) |   \
                                IOMUXC_SW_PAD_CTL_PAD_DSE(strength));                             \
        IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_02_USDHC1_DATA0,                                    \
                            IOMUXC_SW_PAD_CTL_PAD_SPEED(speed) | IOMUXC_SW_PAD_CTL_PAD_SRE_MASK | \
                                IOMUXC_SW_PAD_CTL_PAD_PKE_MASK | IOMUXC_SW_PAD_CTL_PAD_PUE_MASK | \
                                IOMUXC_SW_PAD_CTL_PAD_HYS_MASK | IOMUXC_SW_PAD_CTL_PAD_PUS(1) |   \
                                IOMUXC_SW_PAD_CTL_PAD_DSE(strength));                             \
        IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_03_USDHC1_DATA1,                                    \
                            IOMUXC_SW_PAD_CTL_PAD_SPEED(speed) | IOMUXC_SW_PAD_CTL_PAD_SRE_MASK | \
                                IOMUXC_SW_PAD_CTL_PAD_PKE_MASK | IOMUXC_SW_PAD_CTL_PAD_PUE_MASK | \
                                IOMUXC_SW_PAD_CTL_PAD_HYS_MASK | IOMUXC_SW_PAD_CTL_PAD_PUS(1) |   \
                                IOMUXC_SW_PAD_CTL_PAD_DSE(strength));                             \
        IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_04_USDHC1_DATA2,                                    \
                            IOMUXC_SW_PAD_CTL_PAD_SPEED(speed) | IOMUXC_SW_PAD_CTL_PAD_SRE_MASK | \
                                IOMUXC_SW_PAD_CTL_PAD_PKE_MASK | IOMUXC_SW_PAD_CTL_PAD_PUE_MASK | \
                                IOMUXC_SW_PAD_CTL_PAD_HYS_MASK | IOMUXC_SW_PAD_CTL_PAD_PUS(1) |   \
                                IOMUXC_SW_PAD_CTL_PAD_DSE(strength));                             \
        IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_05_USDHC1_DATA3,                                    \
                            IOMUXC_SW_PAD_CTL_PAD_SPEED(speed) | IOMUXC_SW_PAD_CTL_PAD_SRE_MASK | \
                                IOMUXC_SW_PAD_CTL_PAD_PKE_MASK | IOMUXC_SW_PAD_CTL_PAD_PUE_MASK | \
                                IOMUXC_SW_PAD_CTL_PAD_HYS_MASK | IOMUXC_SW_PAD_CTL_PAD_PUS(1) |   \
                                IOMUXC_SW_PAD_CTL_PAD_DSE(strength));                             \
    }

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*******************************************************************************
 * API
 ******************************************************************************/
uint32_t BOARD_DebugConsoleSrcFreq(void);

void BOARD_InitDebugConsole(void);

void BOARD_InitUART(uint32_t u32BaudRate);

void BOARD_ConfigMPU(void);

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* _BOARD_H_ */
