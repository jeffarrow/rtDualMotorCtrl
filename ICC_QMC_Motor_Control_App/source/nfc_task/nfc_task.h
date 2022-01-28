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
#include "main.h"

#define NFC_MOTOR_ID1   {0x04, 0x7A, 0x19, 0xEA, 0xFC, 0x38, 0x80, 0x00, 0x00, 0xA5}
#define NFC_MOTOR_ID2   {0x04, 0xEF, 0x33, 0x6A, 0x64, 0x34, 0x80, 0x00, 0x00, 0xA5}
#define NFC_MOTOR_ID3   {0x04, 0x83, 0x51, 0xd2, 0x9C, 0x39, 0x80, 0x00, 0x00, 0xA5}

void task_nfc(void);
