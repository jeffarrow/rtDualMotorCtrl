/*
 * GENERATED FILE - DO NOT EDIT
 * Copyright (c) 2008 - 2013 Code Red Technologies Ltd, 
 * Copyright 2015, 2018 NXP
 * (c) NXP Semiconductors 2013-2019
 * Generated linker script file for MIMXRT1052xxxxB
 * Created from linkscript.ldt by FMCreateLinkLibraries
 * Using Freemarker v2.3.23
 * MCUXpresso IDE v10.3.0 [Build 2200] [2018-12-03] on Jul 24, 2019 1:57:18 PM
 */

INCLUDE "ICC_QMC_Motor_Control_App_library.ld"
INCLUDE "ICC_QMC_Motor_Control_App_memory.ld"

ENTRY(ResetISR)

SECTIONS
{
    /* Image Vector Table and Boot Data for booting from external flash */
    .boot_hdr : ALIGN(4)
    {
        FILL(0xff)
        __boot_hdr_start__ = ABSOLUTE(.) ;
        KEEP(*(.boot_hdr.conf))
        . = 0x1000 ;
        KEEP(*(.boot_hdr.ivt))
        . = 0x1020 ;
        KEEP(*(.boot_hdr.boot_data))
        . = 0x1030 ;
        KEEP(*(.boot_hdr.dcd_data))
        __boot_hdr_end__ = ABSOLUTE(.) ;
        . = 0x2000 ;
    } >BOARD_FLASH

    /* MAIN TEXT SECTION */
    .text : ALIGN(4)
    {
        FILL(0xff)
        __vectors_start__ = ABSOLUTE(.) ;
        KEEP(*(.isr_vector))
        /* Global Section Table */
        . = ALIGN(4) ;
        __section_table_start = .;
        __data_section_table = .;
        LONG(LOADADDR(.data_RAM));
        LONG(    ADDR(.data_RAM));
        LONG(  SIZEOF(.data_RAM));
        LONG(LOADADDR(.data));
        LONG(    ADDR(.data));
        LONG(  SIZEOF(.data));
        LONG(LOADADDR(.data_RAM3));
        LONG(    ADDR(.data_RAM3));
        LONG(  SIZEOF(.data_RAM3));
        __data_section_table_end = .;
        __bss_section_table = .;
        LONG(    ADDR(.bss_RAM));
        LONG(  SIZEOF(.bss_RAM));
        LONG(    ADDR(.bss));
        LONG(  SIZEOF(.bss));
        LONG(    ADDR(.bss_RAM3));
        LONG(  SIZEOF(.bss_RAM3));
        __bss_section_table_end = .;
        __section_table_end = . ;
        /* End of Global Section Table */

        *(.after_vectors*)

    } > BOARD_FLASH

    .text : ALIGN(4)
    {
       *(.text*)
       *(.rodata .rodata.* .constdata .constdata.*)
       . = ALIGN(4);
    } > BOARD_FLASH
    /*
     * for exception handling/unwind - some Newlib functions (in common
     * with C++ and STDC++) use this. 
     */
    .ARM.extab : ALIGN(4) 
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > BOARD_FLASH

    __exidx_start = .;

    .ARM.exidx : ALIGN(4)
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > BOARD_FLASH
    __exidx_end = .;
 
    _etext = .;
        

    /* Reserve and place Stack within memory map */
    _StackSize = 0x4000;
    .stack :  ALIGN(4)
    {
        _vStackBase = .;
        . += _StackSize;
        . = ALIGN(4);
        _vStackTop = .;
    } > SRAM_DTC
    
	/* DATA section for SRAM_ITC */
    .data_RAM : ALIGN(4)
    {
        FILL(0xff)
        PROVIDE(__start_data_RAM = .) ;
        *(.ramfunc.$RAM)
        *(.ramfunc.$SRAM_ITC)
        *(.data.$RAM*)
        *(.data.$SRAM_ITC*)
        . = ALIGN(4) ;
        PROVIDE(__end_data_RAM = .) ;
     } > SRAM_ITC AT>BOARD_FLASH
    /* DATA section for SRAM_OC */

    .data_RAM3 : ALIGN(4)
    {
        FILL(0xff)
        PROVIDE(__start_data_RAM3 = .) ;
        *(.ramfunc.$RAM3)
        *(.ramfunc.$SRAM_OC)
        *(.data.$RAM3*)
        *(.data.$SRAM_OC*)
        . = ALIGN(4) ;
        PROVIDE(__end_data_RAM3 = .) ;
     } > SRAM_OC AT>BOARD_FLASH
    /* MAIN DATA SECTION */
    .uninit_RESERVED : ALIGN(4)
    {
        KEEP(*(.bss.$RESERVED*))
        . = ALIGN(4) ;
        _end_uninit_RESERVED = .;
    } > SRAM_DTC

    /* Main DATA section (SRAM_DTC) */
    .data : ALIGN(4)
    {
       FILL(0xff)
       _data = . ;
       *(vtable)
       *(.ramfunc*)
       *(.data*)
       . = ALIGN(4) ;
       _edata = . ;
    } > SRAM_DTC AT>BOARD_FLASH

    /* BSS section for SRAM_ITC */
    .bss_RAM : ALIGN(4)
    {
       PROVIDE(__start_bss_RAM = .) ;
       *(.bss.$RAM*)
       *(.bss.$SRAM_ITC*)
       . = ALIGN (. != 0 ? 4 : 1) ; /* avoid empty segment */
       PROVIDE(__end_bss_RAM = .) ;
    } > SRAM_ITC 

    /* BSS section for SRAM_OC */
    .bss_RAM3 : ALIGN(4)
    {
       PROVIDE(__start_bss_RAM3 = .) ;
       *(.bss.$RAM3*)
       *(.bss.$SRAM_OC*)
       . = ALIGN (. != 0 ? 4 : 1) ; /* avoid empty segment */
       PROVIDE(__end_bss_RAM3 = .) ;
    } > SRAM_OC 

    /* MAIN BSS SECTION */
    .bss : ALIGN(4)
    {
        _bss = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4) ;
        _ebss = .;
        PROVIDE(end = .);
    } > SRAM_DTC

    /* NOINIT section for SRAM_ITC */
    .noinit_RAM (NOLOAD) : ALIGN(4)
    {
       *(.noinit.$RAM*)
       *(.noinit.$SRAM_ITC*)
       . = ALIGN(4) ;
    } > SRAM_ITC 

    /* NOINIT section for SRAM_OC */
    .noinit_RAM3 (NOLOAD) : ALIGN(4)
    {
       *(.noinit.$RAM3*)
       *(.noinit.$SRAM_OC*)
       . = ALIGN(4) ;
    } > SRAM_OC 

    /* DEFAULT NOINIT SECTION */
    .noinit (NOLOAD): ALIGN(4)
    {
        _noinit = .;
        *(.noinit*) 
         . = ALIGN(4) ;
        _end_noinit = .;
    } > SRAM_DTC

    /* Reserve and place Heap within memory map */
    _HeapSize = 0x10000;
    .heap :  ALIGN(4)
    {
        _pvHeapStart = .;
        . += _HeapSize;
        . = ALIGN(4);
        _pvHeapLimit = .;
    } > SRAM_DTC

    /* Provide basic symbols giving location and size of main text
     * block, including initial values of RW data sections. Note that
     * these will need extending to give a complete picture with
     * complex images (e.g multiple Flash banks).
     */
    _image_start = LOADADDR(.text);
    _image_end = LOADADDR(.data) + SIZEOF(.data);
    _image_size = _image_end - _image_start;
}