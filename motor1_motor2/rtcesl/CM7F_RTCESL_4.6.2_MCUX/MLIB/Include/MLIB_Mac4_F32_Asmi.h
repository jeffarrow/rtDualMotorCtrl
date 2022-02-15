/*******************************************************************************
*
 * Copyright (c) 2013 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
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
* 
*
****************************************************************************//*!
*
* @brief  Multiply accumulate of four inputs
* 
*******************************************************************************/
#ifndef _MLIB_MAC4_F32_ASM_H_
#define _MLIB_MAC4_F32_ASM_H_

#if defined(__cplusplus)
extern "C" {
#endif

/*******************************************************************************
* Includes
*******************************************************************************/
#include "mlib_types.h"

/*******************************************************************************
* Macros
*******************************************************************************/  
#define MLIB_Mac4Sat_F32ssss_Asmi(f16Add1Mul1, f16Add1Mul2, f16Add2Mul1, f16Add2Mul2)  \
        MLIB_Mac4Sat_F32ssss_FAsmi(f16Add1Mul1, f16Add1Mul2, f16Add2Mul1, f16Add2Mul2)

/***************************************************************************//*!
*
* f32Out = (f16Add1Mul1 * f16Add1Mul2) + (f16Add2Mul1 * f16Add2Mul2)
* With saturation
*******************************************************************************/
RTCESL_INLINE_OPTIM_SAVE
RTCESL_INLINE_OPTIM_SET 
RAM_FUNC_LIB 
RTCESL_INLINE static inline frac32_t MLIB_Mac4Sat_F32ssss_FAsmi(register frac16_t f16Add1Mul1, register frac16_t f16Add1Mul2,
                                                  register frac16_t f16Add2Mul1, register frac16_t f16Add2Mul2)
{
    register frac32_t f32Out=0;
    #if defined(__CC_ARM)                                            /* For ARM Compiler */
        __asm volatile{ smulbb f16Add1Mul1, f16Add1Mul1, f16Add1Mul2 /* f16Add1Mul1 * f16Add1Mul2 */
                        smulbb f32Out, f16Add2Mul1, f16Add2Mul2      /* f16Add2Mul1 * f16Add2Mul2 */
                        qadd f32Out, f32Out, f16Add1Mul1             /* Addition with saturation */
                        qadd f32Out, f32Out, f32Out };               /* Addition with saturation */
    #else
        __asm volatile( 
                        #if defined(__GNUC__) && !defined(__ARMCC_VERSION)            /* For GCC compiler */
                            ".syntax unified \n"                     /* Using unified asm syntax */
                        #endif
                        "smulbb %1, %1, %2 \n"                       /* f16Add1Mul1 * f16Add1Mul2 */
                        "smulbb %2, %3, %4 \n"                       /* f16Add2Mul1 * f16Add2Mul2 */
                        "qadd %2, %2, %1 \n"                         /* Addition with saturation */
                        "qadd %0, %2, %2 \n"                         /* Addition with saturation */
                        #if defined(__GNUC__) && !defined(__ARMCC_VERSION)            /* For GCC compiler */
                            ".syntax divided \n"
                        #endif
                        : "=l"(f32Out), "+l"(f16Add1Mul1), "+l"(f16Add1Mul2): "l"(f16Add2Mul1), "l"(f16Add2Mul2));
    #endif

    return f32Out;
}
RTCESL_INLINE_OPTIM_RESTORE 

#if defined(__cplusplus)
}
#endif

#endif /* _MLIB_MAC4_F32_ASM_H_ */
