/*
 * GENERATED FILE - DO NOT EDIT
 * Copyright (c) 2008 - 2013 Code Red Technologies Ltd, 
 * Copyright 2015, 2018 NXP
 * (c) NXP Semiconductors 2013-2019
 * Generated linker script file for MIMXRT1052xxxxB
 * Created from memory.ldt by FMCreateLinkMemory
 * Using Freemarker v2.3.23
 * MCUXpresso IDE v10.3.0 [Build 2200] [2018-12-03] on Jul 11, 2019 1:14:46 PM
 */

MEMORY
{
  /* Define each memory region */
  BOARD_FLASH (rx) : ORIGIN = 0x60000000, LENGTH = 0x800000 /* 8M bytes (alias Flash) */  
  SRAM_ITC (rwx) : ORIGIN = 0x400, LENGTH = 0x17C00 /* 95K bytes (alias RAM) */  
  SRAM_DTC (rwx) : ORIGIN = 0x20000000, LENGTH = 0x60000 /* 352K bytes (alias RAM2) */  
  SRAM_OC (rwx) : ORIGIN = 0x20200000, LENGTH = 0x10000 /* 64K bytes (alias RAM3) */  
}

  /* Define a symbol for the top of each memory region */
  __base_BOARD_FLASH = 0x60000000  ; /* BOARD_FLASH */  
  __base_Flash = 0x60000000 ; /* Flash */  
  __top_BOARD_FLASH = 0x60000000 + 0x800000 ; /* 8M bytes */  
  __top_Flash = 0x60000000 + 0x800000 ; /* 8M bytes */  
  __base_SRAM_ITC = 0x400  ; /* SRAM_ITC */  
  __base_RAM = 0x400 ; /* RAM */  
  __top_SRAM_ITC = 0x400 + 0x17C00 ; /* 95K bytes */  
  __top_RAM = 0x400 + 0x17C00 ; /* 95K bytes */  
  __base_SRAM_DTC = 0x20000000  ; /* SRAM_DTC */  
  __base_RAM2 = 0x20000000 ; /* RAM2 */  
  __top_SRAM_DTC = 0x20000000 + 0x60000 ; /* 384K bytes */  
  __top_RAM2 = 0x20000000 + 0x60000 ; /* 384K bytes */  
  __base_SRAM_OC = 0x20200000  ; /* SRAM_OC */  
  __base_RAM3 = 0x20200000 ; /* RAM3 */  
  __top_SRAM_OC = 0x20200000 + 0x10000 ; /* 64K bytes */  
  __top_RAM3 = 0x20200000 + 0x10000 ; /* 64K bytes */  
  
  __VECTOR_TABLE = 0x60002000;
  __VECTOR_RAM = 0x00000000;
  __RAM_VECTOR_TABLE_SIZE_BYTES = 0x400;
