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
#ifndef __lcd_480x272_rgb888_frames_h_
#define __lcd_480x272_rgb888_frames_h_

#include "fsl_common.h"

#if defined(XIP_BOOT_HEADER_ENABLE) && (XIP_BOOT_HEADER_ENABLE == 1)

#if 0

#if defined(__ICCARM__)
#pragma data_alignment=4
extern const uint8_t high_tatras_buffer[3*480*272];
#elif defined(__CC_ARM) || defined(__ARMCC_VERSION)
extern const uint8_t high_tatras_buffer[3*480*272] __attribute__((aligned(4)));
#elif defined(__GNUC__)
extern const uint8_t high_tatras_buffer[3*480*272] __attribute__ ((aligned(4)));
#endif

#if defined(__ICCARM__)
#pragma data_alignment=4
extern const uint8_t nxp_logo_buffer[3*480*272];
#elif defined(__CC_ARM) || defined(__ARMCC_VERSION)
extern const uint8_t nxp_logo_buffer[3*480*272] __attribute__((aligned(4)));
#elif defined(__GNUC__)
extern const uint8_t nxp_logo_buffer[3*480*272] __attribute__ ((aligned(4)));
#endif

#endif

#if defined(__ICCARM__)
#pragma data_alignment=4
extern const uint8_t initial_page_buffer[3*480*272];
#elif defined(__CC_ARM) || defined(__ARMCC_VERSION)
extern const uint8_t initial_page_buffer[3*480*272] __attribute__((aligned(4)));
#elif defined(__GNUC__)
extern const uint8_t initial_page_buffer[3*480*272] __attribute__ ((aligned(4)));
#endif

#if defined(__ICCARM__)
#pragma data_alignment=4
extern const uint8_t nfc_identification_fail_page[3*480*272];
#elif defined(__CC_ARM) || defined(__ARMCC_VERSION)
extern const uint8_t nfc_identification_fail_page[3*480*272] __attribute__((aligned(4)));
#elif defined(__GNUC__)
extern const uint8_t nfc_identification_fail_page[3*480*272] __attribute__ ((aligned(4)));
#endif

#if defined(__ICCARM__)
#pragma data_alignment=4
extern const uint8_t nfc_identification_success_page[3*480*272];
#elif defined(__CC_ARM) || defined(__ARMCC_VERSION)
extern const uint8_t nfc_identification_success_page[3*480*272] __attribute__((aligned(4)));
#elif defined(__GNUC__)
extern const uint8_t nfc_identification_success_page[3*480*272] __attribute__ ((aligned(4)));
#endif

#if defined(__ICCARM__)
#pragma data_alignment=4
extern const uint8_t button_0000_buffer[3*480*272];
#elif defined(__CC_ARM) || defined(__ARMCC_VERSION)
extern const uint8_t button_0000_buffer[3*480*272] __attribute__((aligned(4)));
#elif defined(__GNUC__)
extern const uint8_t button_0000_buffer[3*480*272] __attribute__ ((aligned(4)));
#endif

#if defined(__ICCARM__)
#pragma data_alignment=4
extern const uint8_t button_0001_buffer[3*480*272];
#elif defined(__CC_ARM) || defined(__ARMCC_VERSION)
extern const uint8_t button_0001_buffer[3*480*272] __attribute__((aligned(4)));
#elif defined(__GNUC__)
extern const uint8_t button_0001_buffer[3*480*272] __attribute__ ((aligned(4)));
#endif

#if defined(__ICCARM__)
#pragma data_alignment=4
extern const uint8_t button_0010_buffer[3*480*272];
#elif defined(__CC_ARM) || defined(__ARMCC_VERSION)
extern const uint8_t button_0010_buffer[3*480*272] __attribute__((aligned(4)));
#elif defined(__GNUC__)
extern const uint8_t button_0010_buffer[3*480*272] __attribute__ ((aligned(4)));
#endif

#if defined(__ICCARM__)
#pragma data_alignment=4
extern const uint8_t button_0011_buffer[3*480*272];
#elif defined(__CC_ARM) || defined(__ARMCC_VERSION)
extern const uint8_t button_0011_buffer[3*480*272] __attribute__((aligned(4)));
#elif defined(__GNUC__)
extern const uint8_t button_0011_buffer[3*480*272] __attribute__ ((aligned(4)));
#endif

#if defined(__ICCARM__)
#pragma data_alignment=4
extern const uint8_t button_0100_buffer[3*480*272];
#elif defined(__CC_ARM) || defined(__ARMCC_VERSION)
extern const uint8_t button_0100_buffer[3*480*272] __attribute__((aligned(4)));
#elif defined(__GNUC__)
extern const uint8_t button_0100_buffer[3*480*272] __attribute__ ((aligned(4)));
#endif

#if defined(__ICCARM__)
#pragma data_alignment=4
extern const uint8_t button_0101_buffer[3*480*272];
#elif defined(__CC_ARM) || defined(__ARMCC_VERSION)
extern const uint8_t button_0101_buffer[3*480*272] __attribute__((aligned(4)));
#elif defined(__GNUC__)
extern const uint8_t button_0101_buffer[3*480*272] __attribute__ ((aligned(4)));
#endif

#if defined(__ICCARM__)
#pragma data_alignment=4
extern const uint8_t button_0110_buffer[3*480*272];
#elif defined(__CC_ARM) || defined(__ARMCC_VERSION)
extern const uint8_t button_0110_buffer[3*480*272] __attribute__((aligned(4)));
#elif defined(__GNUC__)
extern const uint8_t button_0110_buffer[3*480*272] __attribute__ ((aligned(4)));
#endif

#if defined(__ICCARM__)
#pragma data_alignment=4
extern const uint8_t button_0111_buffer[3*480*272];
#elif defined(__CC_ARM) || defined(__ARMCC_VERSION)
extern const uint8_t button_0111_buffer[3*480*272] __attribute__((aligned(4)));
#elif defined(__GNUC__)
extern const uint8_t button_0111_buffer[3*480*272];// __attribute__ ((aligned(4)));
#endif

#if defined(__ICCARM__)
#pragma data_alignment=4
extern const uint8_t button_1000_buffer[3*480*272];
#elif defined(__CC_ARM) || defined(__ARMCC_VERSION)
extern const uint8_t button_1000_buffer[3*480*272] __attribute__((aligned(4)));
#elif defined(__GNUC__)
extern const uint8_t button_1000_buffer[3*480*272] __attribute__ ((aligned(4)));
#endif

#if defined(__ICCARM__)
#pragma data_alignment=4
extern const uint8_t button_1001_buffer[3*480*272];
#elif defined(__CC_ARM) || defined(__ARMCC_VERSION)
extern const uint8_t button_1001_buffer[3*480*272] __attribute__((aligned(4)));
#elif defined(__GNUC__)
extern const uint8_t button_1001_buffer[3*480*272] __attribute__ ((aligned(4)));
#endif

#if defined(__ICCARM__)
#pragma data_alignment=4
extern const uint8_t button_1010_buffer[3*480*272];
#elif defined(__CC_ARM) || defined(__ARMCC_VERSION)
extern const uint8_t button_1010_buffer[3*480*272] __attribute__((aligned(4)));
#elif defined(__GNUC__)
extern const uint8_t button_1010_buffer[3*480*272] __attribute__ ((aligned(4)));
#endif

#if defined(__ICCARM__)
#pragma data_alignment=4
extern const uint8_t button_1011_buffer[3*480*272];
#elif defined(__CC_ARM) || defined(__ARMCC_VERSION)
extern const uint8_t button_1011_buffer[3*480*272] __attribute__((aligned(4)));
#elif defined(__GNUC__)
extern const uint8_t button_1011_buffer[3*480*272] __attribute__ ((aligned(4)));
#endif

#if defined(__ICCARM__)
#pragma data_alignment=4
extern const uint8_t button_1100_buffer[3*480*272];
#elif defined(__CC_ARM) || defined(__ARMCC_VERSION)
extern const uint8_t button_1100_buffer[3*480*272] __attribute__((aligned(4)));
#elif defined(__GNUC__)
extern const uint8_t button_1100_buffer[3*480*272] __attribute__ ((aligned(4)));
#endif

#if defined(__ICCARM__)
#pragma data_alignment=4
extern const uint8_t button_1101_buffer[3*480*272];
#elif defined(__CC_ARM) || defined(__ARMCC_VERSION)
extern const uint8_t button_1101_buffer[3*480*272] __attribute__((aligned(4)));
#elif defined(__GNUC__)
extern const uint8_t button_1101_buffer[3*480*272] __attribute__ ((aligned(4)));
#endif

#if defined(__ICCARM__)
#pragma data_alignment=4
extern const uint8_t button_1110_buffer[3*480*272];
#elif defined(__CC_ARM) || defined(__ARMCC_VERSION)
extern const uint8_t button_1110_buffer[3*480*272] __attribute__((aligned(4)));
#elif defined(__GNUC__)
extern const uint8_t button_1110_buffer[3*480*272] __attribute__ ((aligned(4)));
#endif

#if defined(__ICCARM__)
#pragma data_alignment=4
extern const uint8_t button_1111_buffer[3*480*272];
#elif defined(__CC_ARM) || defined(__ARMCC_VERSION)
extern const uint8_t button_1111_buffer[3*480*272] __attribute__((aligned(4)));
#elif defined(__GNUC__)
extern const uint8_t button_1111_buffer[3*480*272] __attribute__ ((aligned(4)));
#endif

#endif

#endif /* __lcd_480x272_rgb888_frames_h_ */
 
