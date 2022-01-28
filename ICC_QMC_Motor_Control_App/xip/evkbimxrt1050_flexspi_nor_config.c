/*
 * Copyright 2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "evkbimxrt1050_flexspi_nor_config.h"

/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.xip_board"
#endif

/*******************************************************************************
 * Code
 ******************************************************************************/
#if defined(XIP_BOOT_HEADER_ENABLE) && (XIP_BOOT_HEADER_ENABLE == 1)
#if defined(__CC_ARM) || defined(__ARMCC_VERSION) || defined(__GNUC__)
__attribute__((section(".boot_hdr.conf")))
#elif defined(__ICCARM__)
#pragma location = ".boot_hdr.conf"
#endif

#if 1
const flexspi_nor_config_t qspiflash_config = {
    .memConfig =
        {
            .tag = FLEXSPI_CFG_BLK_TAG,
            .version = FLEXSPI_CFG_BLK_VERSION,
            .readSampleClkSrc = kFlexSPIReadSampleClk_LoopbackInternally,
            .csHoldTime = 3u,
            .csSetupTime = 3u,
            // Enable DDR mode, Wordaddassable, Safe configuration, Differential clock
            .sflashPadType = kSerialFlash_4Pads,
            .serialClkFreq = kFlexSpiSerialClk_100MHz,
            .sflashA1Size = 8u * 1024u * 1024u,
            .lookupTable =
                {
                    // Read LUTs
                    FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0xEB, RADDR_SDR, FLEXSPI_4PAD, 0x18),
                    FLEXSPI_LUT_SEQ(DUMMY_SDR, FLEXSPI_4PAD, 0x06, READ_SDR, FLEXSPI_4PAD, 0x04),
                    //FLEXSPI_LUT_SEQ(DUMMY_SDR, FLEXSPI_4PAD, 0x09, READ_SDR, FLEXSPI_4PAD, 0x04), // @133MHz the dummy cyles must be set to 9
                },
        },
    .pageSize = 256u,
    .sectorSize = 4u * 1024u,
    .blockSize = 256u * 1024u,
    .isUniformBlockSize = false,
};

#else

const flexspi_nor_config_t qspiflash_config = {
    .memConfig =
        {
          .tag = FLEXSPI_CFG_BLK_TAG,
          .version = FLEXSPI_CFG_BLK_VERSION,
          .readSampleClkSrc = 0u,
          .csHoldTime = 3u,
          .csSetupTime = 3u,
          .columnAddressWidth = 0u,
          .deviceModeCfgEnable = 0u,
          .deviceModeType = 0u,
          .waitTimeCfgCommands = 0u,
          .deviceModeSeq =
            {
              .seqNum = 0u,
              .seqId = 0u,
            },
          .deviceModeArg = 0u,
          .configCmdEnable = 0u,
          .controllerMiscOption = 0u,
          .deviceType = kFlexSpiDeviceType_SerialNOR,
          .sflashPadType = kSerialFlash_4Pads,
          .serialClkFreq = kFlexSpiSerialClk_100MHz,
          .lutCustomSeqEnable = 0u,
          .sflashA1Size = 8u*1024u*1024u, /* 8MB/64Mb */
          .sflashA2Size = 0u,
          .sflashB1Size = 8u*1024u*1024u, /* 8MB/64Mb */
          .sflashB2Size = 0u,
          .csPadSettingOverride = 0,
          .sclkPadSettingOverride = 0,
          .dataPadSettingOverride = 0,
          .dqsPadSettingOverride =0,
          .timeoutInMs = 0u,
          .commandInterval = 0u,
          .busyOffset = 0u,
          .busyBitPolarity = 0u,
          .lookupTable =
              {
                [0] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0xEB, RADDR_SDR, FLEXSPI_4PAD, 0x18),
                [1] = FLEXSPI_LUT_SEQ(DUMMY_SDR, FLEXSPI_4PAD, 0x6, READ_SDR , FLEXSPI_4PAD, 0x4),
                [2] = FLEXSPI_LUT_SEQ(STOP , FLEXSPI_1PAD, 0x0, STOP , FLEXSPI_1PAD, 0x0),
                [3] = FLEXSPI_LUT_SEQ(STOP , FLEXSPI_1PAD, 0x0, STOP , FLEXSPI_1PAD, 0x0),
                [4] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x5, READ_SDR , FLEXSPI_1PAD, 0x4),
                [5] = FLEXSPI_LUT_SEQ(STOP , FLEXSPI_1PAD, 0x0, STOP , FLEXSPI_1PAD, 0x0),
                [12] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x6, STOP , FLEXSPI_1PAD, 0x0),
                [13] = FLEXSPI_LUT_SEQ(STOP , FLEXSPI_1PAD, 0x0, STOP , FLEXSPI_1PAD, 0x0),
                [20] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x20, RADDR_SDR, FLEXSPI_1PAD, 0x18),
                [21] = FLEXSPI_LUT_SEQ(STOP , FLEXSPI_1PAD, 0x0, STOP , FLEXSPI_1PAD, 0x0),
                [32] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0xD8, RADDR_SDR, FLEXSPI_1PAD, 0x18),
                [33] = FLEXSPI_LUT_SEQ(STOP , FLEXSPI_1PAD, 0x0, STOP , FLEXSPI_1PAD, 0x0),
                [36] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x2, RADDR_SDR, FLEXSPI_1PAD, 0x18),
                [37] = FLEXSPI_LUT_SEQ(WRITE_SDR, FLEXSPI_1PAD, 0x4, STOP , FLEXSPI_1PAD, 0x0),
                [38] = FLEXSPI_LUT_SEQ(STOP , FLEXSPI_1PAD, 0x0, STOP , FLEXSPI_1PAD, 0x0),
                [44] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x60, STOP , FLEXSPI_1PAD, 0x0),
                [45] = FLEXSPI_LUT_SEQ(STOP , FLEXSPI_1PAD, 0x0, STOP , FLEXSPI_1PAD, 0x0),
              },
        },
    .pageSize = 256,
    .sectorSize = 4096,
    .ipcmdSerialClkFreq = 1, /* keep default */
    .isUniformBlockSize = false,
    .serialNorType = 0,
    .needExitNoCmdMode = 0,
    .halfClkForNonReadCmd = 0,
    .needRestoreNoCmdMode = 0,
    .blockSize = 0x00010000,
};

#endif

#endif /* XIP_BOOT_HEADER_ENABLE */
