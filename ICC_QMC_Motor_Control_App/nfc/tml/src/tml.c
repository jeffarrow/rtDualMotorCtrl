/*
*         Copyright (c), NXP Semiconductors Caen / France
*
*                     (C)NXP Semiconductors
*       All rights are reserved. Reproduction in whole or in part is
*      prohibited without the written consent of the copyright owner.
*  NXP reserves the right to make changes without notice at any time.
* NXP makes no warranty, expressed, implied or statutory, including but
* not limited to any implied warranty of merchantability or fitness for any
*particular purpose, or that the use will not infringe any third party patent,
* copyright or trademark. NXP must not be liable for any loss or damage
*                          arising from its use.
*/

#include <tool.h>
#include "fsl_gpio.h"
#include "fsl_common.h"
#include "board.h"
#include "fsl_lpi2c.h"
#include "pin_mux.h"

#define I2C_MASTER_BASE (LPI2C2_BASE)
#define I2C_MASTER ((LPI2C_Type *)I2C_MASTER_BASE)
#define I2C_MASTER_SLAVE_ADDR_7BIT (0x28U)
#define I2C_BAUDRATE 100000U

/* Select USB1 PLL (480 MHz) as master lpi2c clock source */
#define LPI2C_CLOCK_SOURCE_SELECT (0U)
/* Clock divider for master lpi2c clock source */
#define LPI2C_CLOCK_SOURCE_DIVIDER (5U)
/* Get frequency of lpi2c clock */
#define LPI2C_CLOCK_FREQUENCY ((CLOCK_GetFreq(kCLOCK_Usb1PllClk) / 8) / (LPI2C_CLOCK_SOURCE_DIVIDER + 1U))

lpi2c_master_handle_t g_m_handle;
lpi2c_master_transfer_t masterXfer = {0};
volatile bool g_MasterCompletionFlag = false;

static void lpi2c_master_callback(LPI2C_Type *base, lpi2c_master_handle_t *handle, status_t status, void *userData)
{
    /* Signal transfer success when received success status. */
    if (status == kStatus_Success)
    {
        g_MasterCompletionFlag = true;
    }
}

static status_t I2C_WRITE(uint8_t *pBuff, uint16_t buffLen)
{
	status_t reVal;
	int8_t to = 100;

	masterXfer.slaveAddress = I2C_MASTER_SLAVE_ADDR_7BIT;
    masterXfer.direction = kLPI2C_Write;
    masterXfer.subaddressSize = 0;
    masterXfer.data = pBuff;
    masterXfer.dataSize = buffLen;
    masterXfer.flags = kLPI2C_TransferDefaultFlag;

    /*  Reset master completion flag to false. */
    g_MasterCompletionFlag = false;

    /* Send master non-blocking data to slave */
    reVal = LPI2C_MasterTransferNonBlocking(I2C_MASTER, &g_m_handle, &masterXfer);

    if (reVal != kStatus_Success)
    {
        return kStatus_Fail;
    }

    /* wait for master complete */
    while (!g_MasterCompletionFlag)
    {
    	Sleep(1);
    	if(to-- <= 0) return kStatus_Fail;
    }
    g_MasterCompletionFlag = false;

    return kStatus_Success;
}

static status_t I2C_READ(uint8_t *pBuff, uint16_t buffLen)
{
	status_t reVal;
        uint32_t u32timeOut;

	masterXfer.slaveAddress = I2C_MASTER_SLAVE_ADDR_7BIT;
    masterXfer.direction = kLPI2C_Read;
    masterXfer.subaddressSize = 0;
    masterXfer.data = pBuff;
    masterXfer.dataSize = buffLen;
    masterXfer.flags = kLPI2C_TransferDefaultFlag;

    /* Reset master completion flag to false. */
    g_MasterCompletionFlag = false;

    reVal = LPI2C_MasterTransferNonBlocking(I2C_MASTER, &g_m_handle, &masterXfer);
    if (reVal != kStatus_Success)
    {
        return kStatus_Fail;
    }

    /* wait for master complete */
    u32timeOut = 0xFFFFFFFF;
    while ((!g_MasterCompletionFlag) || (!u32timeOut))
    {
      u32timeOut--;
    }

    g_MasterCompletionFlag = false;

    return kStatus_Success;
}

static Status tml_Init(void)
{
    lpi2c_master_config_t masterConfig;

    gpio_pin_config_t irq_config = {kGPIO_DigitalInput, 0, kGPIO_IntRisingEdge,};
    gpio_pin_config_t ven_config = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};

    /* Clock setting for LPI2C */
    CLOCK_SetMux(kCLOCK_Lpi2cMux, LPI2C_CLOCK_SOURCE_SELECT);
    CLOCK_SetDiv(kCLOCK_Lpi2cDiv, LPI2C_CLOCK_SOURCE_DIVIDER);

    EnableIRQ(GPIO2_Combined_0_15_IRQn);
    GPIO_PinInit(BOARD_INITPINS_NFC_IRQ_GPIO, BOARD_INITPINS_NFC_IRQ_GPIO_PIN, &irq_config);
    GPIO_PinInit(BOARD_INITPINS_NFC_VEN_GPIO, BOARD_INITPINS_NFC_VEN_GPIO_PIN, &ven_config);

    /*
     * masterConfig.debugEnable = false;
     * masterConfig.ignoreAck = false;
     * masterConfig.pinConfig = kLPI2C_2PinOpenDrain;
     * masterConfig.baudRate_Hz = 100000U;
     * masterConfig.busIdleTimeout_ns = 0;
     * masterConfig.pinLowTimeout_ns = 0;
     * masterConfig.sdaGlitchFilterWidth_ns = 0;
     * masterConfig.sclGlitchFilterWidth_ns = 0;
     */
    LPI2C_MasterGetDefaultConfig(&masterConfig);

    /* Change the default baudrate configuration */
    masterConfig.baudRate_Hz = I2C_BAUDRATE;

    /* Initialize the LPI2C master peripheral */
    LPI2C_MasterInit(I2C_MASTER, &masterConfig, LPI2C_CLOCK_FREQUENCY);

    /* Create the LPI2C handle for the non-blocking transfer */
    LPI2C_MasterTransferCreateHandle(I2C_MASTER, &g_m_handle, lpi2c_master_callback, NULL);

    return SUCCESS;
}

static Status tml_DeInit(void) {
    GPIO_PinWrite(BOARD_INITPINS_NFC_VEN_GPIO, BOARD_INITPINS_NFC_VEN_GPIO_PIN, 0U);
    return SUCCESS;
}

static Status tml_Reset(void) {
    GPIO_PinWrite(BOARD_INITPINS_NFC_VEN_GPIO, BOARD_INITPINS_NFC_VEN_GPIO_PIN, 0U);
	Sleep(10);
    GPIO_PinWrite(BOARD_INITPINS_NFC_VEN_GPIO, BOARD_INITPINS_NFC_VEN_GPIO_PIN, 1U);
	Sleep(10);
	return SUCCESS;
}

static Status tml_Tx(uint8_t *pBuff, uint16_t buffLen) {
    if (I2C_WRITE(pBuff, buffLen) != kStatus_Success)
    {
    	Sleep(10);
    	if(I2C_WRITE(pBuff, buffLen) != kStatus_Success)
    	{
    		return ERROR;
    	}
    }
	return SUCCESS;
}

static Status tml_Rx(uint8_t *pBuff, uint16_t buffLen, uint16_t *pBytesRead) {
    if(I2C_READ(pBuff, 3) == kStatus_Success)
    {
    	if ((pBuff[2] + 3) <= buffLen)
    	{
			if (pBuff[2] > 0)
			{
				if(I2C_READ(&pBuff[3], pBuff[2]) == kStatus_Success)
				{
					*pBytesRead = pBuff[2] + 3;
				}
				else return ERROR;
			} else
			{
				*pBytesRead = 3;
			}
    	}
		else return ERROR;
   }
    else return ERROR;

	return SUCCESS;
}

static Status tml_WaitForRx(uint32_t timeout) {
	if (timeout == 0) {
		while (!GPIO_PinRead(BOARD_INITPINS_NFC_IRQ_GPIO, BOARD_INITPINS_NFC_IRQ_GPIO_PIN));
	} else {
		int16_t to = timeout;
		while (!GPIO_PinRead(BOARD_INITPINS_NFC_IRQ_GPIO, BOARD_INITPINS_NFC_IRQ_GPIO_PIN)) {
			Sleep(10);
			to -= 10;
			if (to <= 0) return ERROR;
		}
	}
	return SUCCESS;
}

void tml_Connect(void) {
	tml_Init();
	tml_Reset();
}

void tml_Disconnect(void) {
	tml_DeInit();
}

void tml_Send(uint8_t *pBuffer, uint16_t BufferLen, uint16_t *pBytesSent) {
	if(tml_Tx(pBuffer, BufferLen) == ERROR) *pBytesSent = 0;
	else *pBytesSent = BufferLen;
}

void tml_Receive(uint8_t *pBuffer, uint16_t BufferLen, uint16_t *pBytes, uint16_t timeout) {
	if (tml_WaitForRx(timeout) == ERROR) *pBytes = 0;
	else tml_Rx(pBuffer, BufferLen, pBytes);
}
