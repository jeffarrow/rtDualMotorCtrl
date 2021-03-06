/*
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_common.h"
#include "mc_periph_init.h"
#include "freemaster.h"
#include "pin_mux.h"
#include "peripherals.h"
#include "fsl_gpio.h"
#include "fsl_lpuart.h"
#include "m1_sm_snsless_enc.h"

#include "freemaster_serial_lpuart.h"
#include "board.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Version info */
#define MCRSP_VER "2.0.0" /* motor control package version */

#define BOARD_USER_BUTTON_PRIORITY 4

/* CPU load measurement SysTick START / STOP macros */
#define SYSTICK_START_COUNT() (SysTick->VAL = SysTick->LOAD)
#define SYSTICK_STOP_COUNT(par1)   \
    uint32_t val  = SysTick->VAL;  \
    uint32_t load = SysTick->LOAD; \
    par1          = load - val

/* Three instruction added after interrupt flag clearing as required */
#define M1_END_OF_ISR \
    {                 \
        __DSB();      \
        __ISB();      \
    }

/* Init SDK HW */
static void BOARD_Init(void);
/* ADC COCO interrupt */
void ADC1_IRQHandler(void);
/* TMR1 reload ISR called with 1ms period */
void TMR1_IRQHandler(void);
/* SW8 Button interrupt handler */
void GPIO5_Combined_0_15_IRQHandler(void);
/* Demo Speed Stimulator */
static void DemoSpeedStimulator(void);
/* Demo Position Stimulator */
static void DemoPositionStimulator(void);

static void BOARD_InitSysTick(void);
static void BOARD_InitGPIO(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/* CPU load measurement using Systick */
uint32_t g_ui32NumberOfCycles    = 0U;
uint32_t g_ui32MaxNumberOfCycles = 0U;

/* Demo mode enabled/disabled */
bool_t bDemoModeSpeed    = FALSE;
bool_t bDemoModePosition = FALSE;

/* Counters used for demo mode */
static uint32_t ui32SpeedStimulatorCnt    = 0U;
static uint32_t ui32PositionStimulatorCnt = 0U;

/* Counter for button pressing */
static uint32_t ui32ButtonFilter = 0U;

/* Application and board ID  */
app_ver_t g_sAppId = {
    "evk-imxrt1024", /* board id */
    "pmsm",          /* motor type */
    MCRSP_VER,       /* sw version */
};

/* Structure used in FM to get required ID's */
app_ver_t g_sAppIdFM = {
	"",
	"",
	MCRSP_VER,
};

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Main function
 */
__RAMFUNC(SRAM_ITC_cm7) int main(void)
{
    uint32_t ui32PrimaskReg;

    /* Disable all interrupts before peripherals are initialized */
    ui32PrimaskReg = DisableGlobalIRQ();

    /* Disable demo mode after reset */
    bDemoModeSpeed    = FALSE;
    bDemoModePosition = FALSE;

    /* Pass actual demo id and board info to FM */
    g_sAppIdFM = g_sAppId;

    /* Init board hardware. */
    BOARD_Init();

    /* SysTick initialization for CPU load measurement */
    BOARD_InitSysTick();

    /* Init peripheral motor control driver for motor M1 */
    MCDRV_Init_M1();

    /* Turn off application */
    M1_SetAppSwitch(FALSE);

    /* Enable interrupts */
    EnableGlobalIRQ(ui32PrimaskReg);

    /* Infinite loop */
    while (1)
    {
        /* FreeMASTER Polling function */
        FMSTR_Poll();
    }
}
__RAMFUNC(SRAM_ITC_cm7) void PWM2_0_IRQHandler(void)
{
	// clear flag
	PWM2->SM[0].STS  = PWM_STS_CMPF(1<<4);

	// toggle test IO
	GPIO_PortToggle(BOARD_INITPINS_TP_PORT, 1<<BOARD_INITPINS_TP_PIN);

	__DSB();
}
/*!
 * @brief   ADC conversion complete ISR called with 100us period processes
 *           - motor M1 fast application machine function
 *
 * @param   void
 *
 * @return  none
 */
__RAMFUNC(SRAM_ITC_cm7) void ADC1_IRQHandler(void)
{
    /* Start CPU tick number couting */
    SYSTICK_START_COUNT();

    /* M1 State machine */
    SM_StateMachineFast(&g_sM1Ctrl);

    /* Call FreeMASTER recorder */
    FMSTR_Recorder(0);

    /* Stop CPU tick number couting and store actual and maximum ticks */
    SYSTICK_STOP_COUNT(g_ui32NumberOfCycles);
    g_ui32MaxNumberOfCycles =
        g_ui32NumberOfCycles > g_ui32MaxNumberOfCycles ? g_ui32NumberOfCycles : g_ui32MaxNumberOfCycles;



    /* Add empty instructions for correct interrupt flag clearing */
    M1_END_OF_ISR;
}

/*!
 * @brief   TMR1 reload ISR called with 1ms period and processes following functions:
 *           - motor M1 slow application machine function
 *
 * @param   void
 *
 * @return  none
 */
__RAMFUNC(SRAM_ITC_cm7) void TMR1_IRQHandler(void)
{
    /* M1 Slow StateMachine call */
    SM_StateMachineSlow(&g_sM1Ctrl);

    /* In FAULT is demo Mode switched off */
    if (M1_GetAppState() == 0)
    {
        bDemoModeSpeed    = FALSE;
        bDemoModePosition = FALSE;
    }
    
    /* Demo speed stimulator */
    DemoSpeedStimulator();

    /* Demo position stimulator */
    DemoPositionStimulator();

    /* Clear the CSCTRL0[TCF1] flag */
    TMR1->CHANNEL[0].CSCTRL |= TMR_CSCTRL_TCF1(0x00);
    TMR1->CHANNEL[0].CSCTRL &= ~(TMR_CSCTRL_TCF1_MASK);

    /* Clear the CSCTRL0[TCF] flag */
    TMR1->CHANNEL[0].SCTRL &= ~(TMR_SCTRL_TCF_MASK);

    /* Add empty instructions for correct interrupt flag clearing */
    M1_END_OF_ISR;
}

/*!
 * @brief   SW8 Button interrupt handler
 *
 * @param   void
 *
 * @return  none
 */
void GPIO5_Combined_0_15_IRQHandler(void)
{
    if (GPIO5->ISR & GPIO_ICR1_ICR0_MASK)
    {
        /* Clear the flag */
        GPIO5->ISR |= GPIO_ISR_ISR(1);

        /* Proceed only if pressing longer than timeout */
        if (ui32ButtonFilter > 300)
        {
            ui32ButtonFilter = 0;

            /* Speed demo */
            if (bDemoModeSpeed)
            {
                /* Stop application */
                M1_SetSpeed(0);
                M1_SetAppSwitch(0);
                bDemoModeSpeed = FALSE;
            }
            else
            {
                /* Start application */
                M1_SetAppSwitch(1);
                bDemoModeSpeed         = TRUE;
                ui32SpeedStimulatorCnt = 0;
            }
        }
    }

    /* Add empty instructions for correct interrupt flag clearing */
    M1_END_OF_ISR;
}

/*!
 * @brief   DemoSpeedStimulator
 *           - When demo mode is enabled it changes the required speed according
 *             to predefined profile
 *
 * @param   void
 *
 * @return  none
 */
__RAMFUNC(SRAM_ITC_cm7) static void DemoSpeedStimulator(void)
{
    /* Increase push button pressing counter  */
    if (ui32ButtonFilter < 1000)
    {
    	ui32ButtonFilter++;
    }

    if (bDemoModeSpeed)
    {
        ui32SpeedStimulatorCnt++;
        switch (ui32SpeedStimulatorCnt)
        {
            case 10:
                M1_SetAppSwitch(0);
                break;
            case 20:
                g_sM1Drive.eControl                  = kControlMode_SpeedFOC;
                g_sM1Drive.sMCATctrl.ui16PospeSensor = MCAT_SENSORLESS_CTRL;
                M1_SetAppSwitch(1);
                break;
            case 1000:
                M1_SetSpeed(1000.0F);
                break;
            case 5000:
                M1_SetSpeed(2000.0F);
                break;
            case 10000:
                M1_SetSpeed(-1000.0F);
                break;
            case 15000:
                M1_SetSpeed(-2000.0F);
                break;
            case 19800:
                M1_SetSpeed(0.0F);
                break;
            case 20000:
                ui32SpeedStimulatorCnt = 0;
                M1_SetAppSwitch(0);
                break;
            default:
            	;
            	break;
        }
    }
}

/*!
 * @brief   DemoPositionStimulator
 *           - When demo mode is enabled it changes the required position according
 *             to predefined profile
 *
 * @param   void
 *
 * @return  none
 */
__RAMFUNC(SRAM_ITC_cm7) static void DemoPositionStimulator(void)
{
    if (bDemoModePosition)
    {
        ui32PositionStimulatorCnt++;
        switch (ui32PositionStimulatorCnt)
        {
            case 1:
                M1_SetAppSwitch(0);
                break;
            case 2:
                g_sM1Drive.eControl                  = kControlMode_PositionFOC;
                g_sM1Drive.sMCATctrl.ui16PospeSensor = MCAT_ENC_CTRL;
                M1_SetAppSwitch(1);
                break;
            case 10:
                M1_SetPosition(ACC32(20));
                break;
            case 3000:
                M1_SetPosition(ACC32(10));
                break;
            case 6000:
                M1_SetPosition(ACC32(20));
                break;
            case 9000:
                M1_SetPosition(ACC32(0));
                break;
            case 12000:
                M1_SetPosition(ACC32(20));
                break;
            case 15000:
                M1_SetPosition(ACC32(0));
                break;
            case 18000:
                ui32PositionStimulatorCnt = 0;
                M1_SetAppSwitch(0);
                break;
            default:
			    ;
            	break;
        }
    }
}

/*!
 *@brief      Initialization of the Clocks and Pins
 *
 *@param      none
 *
 *@return     none
 */
static void BOARD_Init(void)
{
    /* Init board hardware. */
    BOARD_InitBootPins();
    /* Initialize clock configuration */
    BOARD_InitBootClocks();
    /* Init peripherals set in peripherals file */
    BOARD_InitBootPeripherals();
    /* Init GPIO pins */
    BOARD_InitGPIO();
}

/*!
 * @brief   static void BOARD_InitGPIO(void)
 *           - Initialization of the GPIO peripherals
 *
 * @param   void
 *
 * @return  none
 */
static void BOARD_InitGPIO(void)
{
    /* SW4 Button configuration */
    const gpio_pin_config_t user_button_config = {
        kGPIO_DigitalInput,  /* Set pin as digital input */
        0,                   /* Set default output logic, which has no use in input  */
        kGPIO_IntFallingEdge /* Set pin interrupt mode: falling-edge sensitive */
    };

    /* Enable GPIO pin interrupt for SW4 button */
    EnableIRQ(BOARD_USER_BUTTON_IRQ);
    GPIO_PinInit(BOARD_USER_BUTTON_GPIO, BOARD_USER_BUTTON_GPIO_PIN, &user_button_config);
    GPIO_PortEnableInterrupts(BOARD_USER_BUTTON_GPIO, 1U << BOARD_USER_BUTTON_GPIO_PIN);
    NVIC_SetPriority(BOARD_USER_BUTTON_IRQ, BOARD_USER_BUTTON_PRIORITY);
}


/*!
 *@brief      SysTick initialization for CPU cycle measurement
 *
 *@param      none
 *
 *@return     none
 */
static void BOARD_InitSysTick(void)
{
    /* Initialize SysTick core timer to run free */
    /* Set period to maximum value 2^24*/
    SysTick->LOAD = 0xFFFFFF;

    /*Clock source - System Clock*/
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;

    /*Start Sys Timer*/
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}
