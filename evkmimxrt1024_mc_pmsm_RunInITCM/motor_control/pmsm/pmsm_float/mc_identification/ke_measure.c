/*
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "ke_measure.h"

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Ke measurement routine
 *
 * @param *sKeMeasFcn   input structure of type #mid_get_ke_t for passing
 *                      all necessary parameters.
 *
 * @return None
 */
__RAMFUNC(SRAM_ITC_cm7) void MID_getKe(mid_get_ke_t *sKeMeasFcn)
{
    float_t fltEdFilt, fltEqFilt;
    float_t fltEdFiltSquare, fltEqFiltSquare;
    float_t fltEtotal;

    /* Initialisation */
    if (sKeMeasFcn->bActive == FALSE)
    {
        sKeMeasFcn->bActive                   = TRUE;
        sKeMeasFcn->ui16LoopCounter              = 0U;
        sKeMeasFcn->fltFreqElRamp                = 0.0F;
        sKeMeasFcn->sFreqElRampParam.fltRampUp   = sKeMeasFcn->fltFreqElReq / MID_SPEED_RAMP_TIME / 10000.0F;
        sKeMeasFcn->sFreqElRampParam.fltRampDown = sKeMeasFcn->fltFreqElReq / MID_SPEED_RAMP_TIME / 10000.0F;
        sKeMeasFcn->sEdMA32Filter.fltLambda      = 1.0F / 10.0F;
        GDFLIB_FilterMAInit_FLT(0.0, &sKeMeasFcn->sEdMA32Filter);
        sKeMeasFcn->sEqMA32Filter.fltLambda = 1.0F / 10.0F;
        GDFLIB_FilterMAInit_FLT(0.0, &sKeMeasFcn->sEqMA32Filter);
        sKeMeasFcn->sFreqIntegrator.a32Gain = ACC32(1.0F * sKeMeasFcn->fltFreqMax / 10000.0F * 2.0F);
        GFLIB_IntegratorInit_F16(0, &sKeMeasFcn->sFreqIntegrator);
        GFLIB_RampInit_FLT(0.0, &sKeMeasFcn->sFreqElRampParam);
    }
    /* Set Id required */
    *(sKeMeasFcn->pfltIdReq) = sKeMeasFcn->fltIdReqOpenLoop;
    /* Ramp electrical speed */
    sKeMeasFcn->fltFreqElRamp = GFLIB_Ramp_FLT(sKeMeasFcn->fltFreqElReq, &sKeMeasFcn->sFreqElRampParam);
    /* Integrate electrical speed to get electrical position */
    *sKeMeasFcn->pf16PosEl = GFLIB_Integrator_F16(MLIB_ConvSc_F16ff(sKeMeasFcn->fltFreqElRamp, sKeMeasFcn->fltFreqMax),
                                                  &sKeMeasFcn->sFreqIntegrator);

    /* Bemf filtering */
    fltEdFilt = GDFLIB_FilterMA_FLT(*(sKeMeasFcn->pfltEd), &sKeMeasFcn->sEdMA32Filter);
    fltEqFilt = GDFLIB_FilterMA_FLT(*(sKeMeasFcn->pfltEq), &sKeMeasFcn->sEqMA32Filter);

    if (sKeMeasFcn->fltFreqElRamp == sKeMeasFcn->fltFreqElReq)
    {
        sKeMeasFcn->ui16LoopCounter++;

        if (sKeMeasFcn->ui16LoopCounter > (uint16_t)MID_TIME_2400MS)
        {
            /* Total Bemf calculation */
            fltEdFiltSquare = MLIB_Mul_FLT(fltEdFilt, fltEdFilt);
            fltEqFiltSquare = MLIB_Mul_FLT(fltEqFilt, fltEqFilt);
            fltEtotal       = GFLIB_Sqrt_FLT(MLIB_Add_FLT(fltEdFiltSquare, fltEqFiltSquare));

            /* Ke calculation */
            sKeMeasFcn->fltKe = MLIB_Div_FLT(fltEtotal, sKeMeasFcn->fltFreqElReq);

            /* Check Faults */
            /* Check if Ke is negative or saturated*/
            if (sKeMeasFcn->fltKe < 0.0F)
            {
                g_sMID.ui16WarnMID |= MID_WARN_KE_OUT_OF_RANGE;
            }

            /* When finished exit the function */
            sKeMeasFcn->bActive = FALSE;
        }
    }
}
