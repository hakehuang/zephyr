/*
 * Copyright 2025 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Relocatable linker script for boot loader loading
 * Code will be loaded by boot loader to specified addresses
 */

OUTPUT_ARCH(xtensa)

#include <soc/memory.h>
#include <xtensa/config/core-isa.h>

#include <zephyr/linker/linker-defs.h>
#include <zephyr/linker/linker-tool.h>
#include <zephyr/linker/sections.h>

/* Define relocation addresses for boot loader loading */
#define RELOC_TEXT_BASE    0x80000000  /* VMA base address for text sections */
#define RELOC_DATA_BASE    0x90000000  /* VMA base address for data sections */
#define FLASH_LMA_BASE     0x60000000   /* LMA base address in flash */

#define RAMABLE_REGION adsp_data :adsp_data_phdr
#define ROMABLE_REGION adsp_data :adsp_data_phdr

MEMORY
{
  /* Vector sections - LMA addresses in flash */
  vector_reset_text_lma :
    org = FLASH_LMA_BASE,
    len = MEM_RESET_TEXT_SIZE
  vector_reset_text :
    org = XCHAL_RESET_VECTOR0_PADDR_IRAM,
    len = MEM_RESET_TEXT_SIZE
  vector_reset_lit_lma :
    org = FLASH_LMA_BASE + MEM_RESET_TEXT_SIZE,
    len = MEM_RESET_LIT_SIZE
  vector_reset_lit :
    org = XCHAL_RESET_VECTOR0_PADDR_IRAM + MEM_RESET_TEXT_SIZE,
    len = MEM_RESET_LIT_SIZE
  vector_base_text_lma :
    org = FLASH_LMA_BASE + MEM_RESET_TEXT_SIZE + MEM_RESET_LIT_SIZE,
    len = MEM_VECBASE_LIT_SIZE
  vector_base_text :
    org = XCHAL_VECBASE_RESET_PADDR_IRAM,
    len = MEM_VECBASE_LIT_SIZE
  vector_int2_lit_lma :
    org = FLASH_LMA_BASE + MEM_RESET_TEXT_SIZE + MEM_RESET_LIT_SIZE + MEM_VECBASE_LIT_SIZE,
    len = MEM_VECT_LIT_SIZE
  vector_int2_lit :
    org = XCHAL_INTLEVEL2_VECTOR_PADDR_IRAM - MEM_VECT_LIT_SIZE,
    len = MEM_VECT_LIT_SIZE
  vector_int2_text_lma :
    org = FLASH_LMA_BASE + MEM_RESET_TEXT_SIZE + MEM_RESET_LIT_SIZE + MEM_VECBASE_LIT_SIZE + MEM_VECT_LIT_SIZE,
    len = MEM_VECT_TEXT_SIZE
  vector_int2_text :
    org =极 XCHAL_INTLEVEL2_VECTOR_PADDR_IRAM,
    len = MEM_VECT_TEXT_SIZE
  vector_int3_lit_lma :
    org = FLASH_LMA_BASE + MEM_RESET_TEXT_SIZE + MEM_RESET_LIT_SIZE + MEM_VECBASE_LIT_SIZE + MEM_VECT_LIT_SIZE + MEM_VECT_TEXT_SIZE,
    len = MEM_VECT极_LIT_SIZE
  vector_int3_lit :
    org = XCHAL_INTLEVEL3_VECTOR_PADDR_IRAM - MEM_VECT_LIT_SIZE,
    len = MEM_VECT_LIT_SIZE
  vector_int3_text_lma :
    org = FLASH_LMA_BASE + MEM_RESET_TEXT_SIZE + MEM_RESET_LIT_SIZE + MEM_VECBASE_LIT_SIZE + MEM_VECT_LIT_SIZE + MEM_VECT_TEXT_SIZE + MEM_VECT_LIT_SIZE,
    len = MEM_VECT_TEXT_SIZE
  vector_int3_text :
    org = XCHAL_INTLEVEL3_VECTOR_PADDR_IRAM,
    len = MEM_VECT_TEXT_SIZE
  vector_int4_lit_lma :
    org = FLASH_LMA_BASE + MEM_RESET_TEXT_SIZE + MEM_RESET_LIT_SIZE + MEM_VECBASE_LIT_SIZE + MEM_VECT_LIT_SIZE + MEM_VECT_TEXT_SIZE + MEM_VECT_LIT_SIZE + MEM_VECT_TEXT_SIZE,
    len = MEM_VECT_LIT_SIZE
  vector_int4_lit :
    org = XCHAL_INTLEVEL4_VECTOR_PADDR_IRAM - MEM_VECT_LIT_SIZE,\极
    len = MEM_VECT_LIT_SIZE
  vector_int4_text_lma :
    org = FLASH_LMA_BASE + MEM_RESET_TEXT_SIZE + MEM_RESET_LIT_SIZE + MEM_VECBASE_LIT_SIZE + MEM_VECT_LIT_SIZE + MEM_VECT_TEXT_SIZE + MEM_VECT_LIT_SIZE + MEM_VECT_TEXT_SIZE + MEM_VECT_LIT_SIZE,
    len = MEM_VECT_TEXT_SIZE
  vector_int4_text :
    org = XCHAL_INTLEVEL4_VECTOR_PADDR_IRAM,
    len = MEM_VECT_TEXT_SIZE
  vector_int5_lit_lma :
    org = FLASH_LMA_BASE + MEM_RESET_TEXT_SIZE + MEM_RESET_LIT_SIZE + MEM_VECBASE_LIT_SIZE + MEM_VECT_LIT_SIZE + MEM_VECT_TEXT_SIZE + MEM_VECT_LIT_SIZE + MEM_VECT_TEXT_SIZE + MEM_VECT_LIT_SIZE + MEM_VECT_TEXT_SIZE,
    len = MEM_VECT_LIT_SIZE
  vector_int5_lit :
    org = XCHAL_INTLEVEL5_VECTOR_PADDR_IRAM - MEM_VECT_LIT_SIZE,
    len = MEM_VECT_LIT_SIZE
  vector_int5_text_lma :
    org = FLASH_LMA_BASE + MEM_RESET_TEXT_SIZE + MEM_RESET_LIT_SIZE + MEM_VECBASE_LIT_SIZE + MEM_VECT_LIT_SIZE + MEM_VECT_TEXT_SIZE + MEM_VECT_LIT_SIZE + MEM_VECT_TEXT_SIZE + MEM_VECT_LIT_SIZE极 + MEM_VECT_TEXT_SIZE + MEM_VECT_LIT_SIZE,
    len = MEM_VECT_TEXT_SIZE
  vector_int5_text :
    org = XCHAL_INTLEVEL5_VECTOR_PADDR_IRAM,
    len = MEM_VECT_TEXT_SIZE
  vector_kernel_lit_lma :
    org = FLASH_LMA_BASE + MEM_RESET_TEXT_SIZE + MEM_RESET_LIT_SIZE + MEM_VECBASE_LIT_SIZE + MEM_VECT_LIT_SIZE + MEM_VECT_TEXT_SIZE + MEM_VECT_LIT_SIZE + MEM_VECT_TEXT_SIZE + MEM_VECT_LIT_SIZE + MEM_VECT_TEXT_SIZE + MEM_VECT_LIT_SIZE + MEM_VECT_TEXT_SIZE,
    len = MEM_VECT_LIT_SIZE
  vector_kernel_lit :
    org = XCHAL_KERNEL_VECTOR_PADDR_IRAM - MEM_VECT_LIT_SIZE,
    len = MEM_VECT_LIT_SIZE
  vector_kernel_text_lma :
      org = FLASH_LMA_BASE + MEM_RESET_TEXT_SIZE + MEM_RESET_LIT_SIZE + MEM_VECBASE_LIT_SIZE + MEM_VECT_LIT_SIZE + MEM_VECT_TEXT_SIZE + MEM_VECT_LIT_SIZE + MEM_VECT_TEXT_SIZE + MEM_VECT_LIT_SIZE + MEM_VECT_TEXT_SIZE + MEM_VECT_LIT_SIZE + MEM_VECT_TEXT_SIZE + MEM_VECT_LIT_SIZE,
      len = MEM_VECT_TEXT_SIZE
  vector_kernel_text :
      org = XCHAL_KERNEL_VECTOR_PADDR_IRAM,
      len = MEM_VECT_TEXT_SIZE
  vector_user_lit_lma :
    org = FLASH_LMA_BASE + MEM_RESET_TEXT_SIZE + MEM_RESET_LIT_SIZE + MEM_VECBASE_LIT_SIZE + MEM_VECT_LIT_SIZE + MEM_VECT_TEXT_SIZE + MEM_VECT_LIT_SIZE + MEM_VECT_TEXT_SIZE + MEM_VECT_LIT_SIZE + MEM_VECT_TEXT_SIZE + MEM_VECT_LIT_SIZE + MEM_VECT_TEXT_SIZE + MEM_VECT_LIT_SIZE + MEM_VECT_TEXT_SIZE,
    len = MEM_VECT_LIT_SIZE
  vector_user_lit :
    org = XCHAL_USER_VECTOR_PADDR_IRAM - MEM_VECT_LIT_SIZE,
    len = MEM_VECT_LIT_SIZE
  vector_user_text_lma :
    org = FLASH_LMA_BASE + MEM_RESET_TEXT_SIZE + MEM_RESET_LIT_SIZE + MEM_VECBASE_LIT_SIZE + MEM_VECT_LIT_SIZE + MEM_VECT_TEXT_SIZE + MEM_VECT_LIT_SIZE + MEM_VECT_TEXT_SIZE + MEM_VECT_LIT_SIZE + MEM_VECT_TEXT_SIZE + MEM_VECT_LIT_SIZE + MEM_VECT_TEXT_SIZE + MEM_VECT_LIT_SIZE + MEM_VECT_TEXT_SIZE + MEM_VECT_LIT_SIZE,
    len = MEM_VECT_TEXT_SIZE
  vector_user_text :
    org = XCHAL_USER_VECTOR_PADDR_IRAM,
    len = MEM_VECT_TEXT_SIZE
  vector_double_lit_lma :
    org = FLASH_LMA_BASE + MEM_RESET_TEXT_SIZE + MEM_RESET_LIT_SIZE + MEM_VECBASE_LIT_SIZE + MEM_VECT_LIT_SIZE + MEM_VECT_TEXT_SIZE + MEM_VECT_LIT_SIZE + MEM_VECT_TEXT_SIZE + MEM_VECT_LIT_SIZE + MEM_VECT_TEXT_SIZE + MEM_VECT_LIT_SIZE + MEM_VECT_TEXT_SIZE + MEM_VECT_LIT_SIZE + MEM_VECT_TEXT_SIZE + MEM_VECT_LIT_SIZE + MEM_VECT_TEXT_SIZE,
    len = MEM_VECT_LIT_SIZE
  vector_double_lit :
    org = XCHAL_DOUBLEEXC_VECTOR_PADDR_IRAM - MEM_VECT_LIT_SIZE,
    len = MEM_VECT_LIT_SIZE
  vector_double_text_lma :
    org = FLASH_LMA_BASE + MEM_RESET_TEXT_SIZE + MEM_RESET_LIT_SIZE + MEM_VECBASE_LIT_SIZE + MEM_VECT_LIT_SIZE + MEM_VECT_TEXT_SIZE + MEM_VECT_LIT_SIZE + MEM_VECT_TEXT_SIZE + MEM_VECT_LIT_SIZE + MEM_VECT_TEXT_SIZE + MEM_VECT_LIT_SIZE + MEM_VECT_TEXT_SIZE + MEM_VECT_LIT_SIZE + MEM_VECT_TEXT_SIZE + MEM_VECT_LIT_SIZE + MEM_VECT_TEXT_SIZE + MEM_VECT_LIT_SIZE,
    len = MEM_VECT_TEXT_SIZE
  vector_double_text :
    org = XCHAL_DOUBLEEXC_VECTOR_PADDR_IRAM,
    len = MEM_VECT_TEXT_SIZE
  vector_int2_lit :
    org = XCHAL_INTLEVEL2_VECTOR_PADDR_IRAM - MEM_VECT_LIT_SIZE,
    len = MEM_VECT_LIT_SIZE
  vector_int2_text :
    org = XCHAL_INTLEVEL2_VECTOR_PADDR_IRAM,
    len = MEM_VECT_TEXT_SIZE
  vector_int3_lit :
    org = XCHAL_INTLEVEL3_VECTOR_PADDR_IRAM - MEM_VECT_LIT_SIZE,
    len = MEM_VECT_LIT_SIZE
  vector_int3_text :
    org = XCHAL_INTLEVEL3_VECTOR_PADDR_IRAM,
    len = MEM_VECT_TEXT_SIZE
  vector_int4_lit :
    org = XCHAL_INTLEVEL4_VECTOR_PADDR_IRAM - MEM_VECT_LIT_SIZE,
    len = MEM_VECT_LIT_SIZE
  vector_int4_text :
    org = XCHAL_INTLEVEL4_VECTOR_PADDR_IRAM,
    len = MEM_VECT_TEXT_SIZE
  vector_int5_lit :
    org = XCHAL_INTLEVEL5_VECTOR_PADDR_IRAM - MEM_VECT_LIT_SIZE,
    len = MEM_VECT_LIT_SIZE
  vector_int5_text :
    org = XCHAL_INTLEVEL5_VECTOR_PADDR_IRAM,
    len = MEM_VECT_TEXT_SIZE
  vector_kernel_lit :
    org = XCHAL_KERNEL_VECTOR_PADDR_IRAM - MEM_VECT_LIT_SIZE,
    len = MEM_VECT_LIT_SIZE
  vector_kernel_text :
    org = XCHAL_KERNEL_VECTOR_PADDR_IRAM,
    len = MEM_VECT_TEXT_SIZE
  vector_user_lit :
    org = XCHAL_USER_VECTOR_PADDR_IRAM - MEM_VECT_LIT_SIZE,
    len = MEM_VECT_LIT_SIZE
  vector_user_text :
    org = XCHAL_USER_VECTOR_PADDR_IRAM,
    len = MEM_VECT_TEXT_SIZE
  vector_double_lit :
    org = XCHAL_DOUBLEEXC_VECTOR_PADDR_IRAM - MEM_VECT_LIT_SIZE,
    len = MEM_VECT_LIT_SIZE
  vector_double_text :
    org = XCHAL_DOUBLEEXC_VECTOR_PADDR_IRAM,
    len = MEM_VECT_TEXT_SIZE

  /* Relocatable text section for boot loader loading */
  adsp_text :
    org = RELOC_TEXT_BASE,
    len = TEXT_SIZE

  /* Relocatable data section for boot loader loading */
  adsp_data :
    org = RELOC_DATA_BASE,
    len = DATA_SIZE

#ifdef CONFIG_GEN_ISR_TABLES
  IDT_LIST :
    org = IDT_BASE,
    len = IDT_SIZE
#endif
}

PHDRS
{
  vector_reset_text_phdr PT_LOAD;
  vector_reset_lit_phdr PT_LOAD;
  vector_base_text_phdr PT_LOAD;
  vector_base_lit_phdr PT_LOAD;
  vector_int2_text_phdr PT_LOAD;
  vector_int2_lit_phdr PT_LOAD;
  vector_int3_text_phdr PT_LOAD;
  vector_int3_lit_phdr PT_LOAD;
  vector_int4_text_phdr PT_LOAD;
  vector_int4_lit_phdr PT_LOAD;
  vector_int5_text_phdr PT_LOAD;
  vector_int5_lit_phdr PT_LOAD;
  vector_kernel_text_phdr PT_LOAD;
  vector_kernel_lit_phdr PT_LOAD;
  vector_user_text_phdr PT_LOAD;
  vector_user_lit_phdr PT_LOAD;
  vector_double_text_phdr PT_LOAD;
  vector_double_lit_phdr PT_LOAD;
  adsp_text_phdr PT_LOAD;
  adsp_data_phdr PT_LOAD;
}

PROVIDE(_memmap_reset_vector = XCHAL_RESET_VECTOR0_PADDR_IRAM);
PROVIDE(_memmap_vecbase_reset = XCHAL_VECBASE_RESET_PADDR_IRAM);

/* Provide relocation base addresses for boot loader */
PROVIDE(__reloc_text_base = RELOC_TEXT_BASE);
PROVIDE(__reloc_data_base = RELOC_DATA_BASE);

ENTRY(CONFIG_KERNEL_ENTRY)

/* Various memory-map dependent cache attribute settings: */
_memmap_cacheattr_wb_base = 0x00000012;
_memmap_cacheattr_wt_base = 0x00000012;
_memmap_cacheattr_bp_base = 0x00000022;
_memmap_cacheattr_unused_mask = 0xFFFFFF00;
_memmap_cacheattr_wb_trapnull = 0x22222212;
_memmap_cacheattr_wba_trapnull = 0x22222212;
_memmap_cacheattr_wbna_trapnull = 0x22222212;
_memmap_cacheattr_wt_trapnull = 0x22222212;
_memmap_cacheattr_bp_trapnull = 0x22222222;
_memmap_cacheattr_wb_strict = 0xFFFFFF12;
_memmap_cacheattr_wt_strict = 0xFFFFFF12;
_memmap_cacheattr_bp_strict = 0xFFFFFF22;
_memmap_cacheattr_wb_allvalid = 0x22222212;
_memmap_cacheattr_wt_allvalid = 0x22222212;
_memmap_cacheattr_bp_allvalid = 0x22222222;
_memmap_region_map = 0x00000003;
PROVIDE(_memmap_cacheattr_reset = _memmap_cacheattr_wb_trapnull);

SECTIONS
{

#include <zephyr/linker/rel-sections.ld>

  /* Vector sections - must remain at fixed addresses */
  .ResetVector.text : ALIGN(4)
  {
    _ResetVector_text_start = ABSOLUTE(.);
    KEEP (*(.ResetVector.text))
    _ResetVector_text_end = ABSOLUTE(.);
  } >vector_reset_text AT>vector_reset_text_lma :vector_reset_text_phdr
    _ResetVector_text_lma_start = LOADADDR(.ResetVector.text);
    _ResetVector_text_lma_end = _ResetVector_text_lma_start + SIZEOF(.ResetVector.text);

  .ResetVector.literal : ALIGN(4)
  {
    _ResetVector_literal_start = ABSOLUTE(.);
    *(.ResetVector.literal)
    _ResetVector_literal_end = ABSOLUTE(.);
  } >vector_reset_lit AT>vector_reset_lit_lma :vector_reset_lit_phdr
    _ResetVector_literal_lma_start = LOADADDR(.ResetVector.literal);
    _ResetVector_literal_lma_end = _ResetVector_literal_lma_start + SIZEOF(.ResetVector.literal);

  .WindowVectors.text : ALIGN(4)
  {
    _WindowVectors_text_start = ABSOLUTE(.);
    KEEP (*(.WindowVectors.text))
    _WindowVectors_text_end = ABSOLUTE(.);
  } >vector_base_text AT>vector_base_text_lma :vector_base_text_phdr
    _WindowVectors_text_lma_start = LOADADDR(.WindowVectors.text);
    _WindowVectors_text_lma_end = _WindowVectors_text_lma_start + SIZEOF(.WindowVectors.text);

  .Level2InterruptVector.literal : ALIGN(4)
  {
    _Level2InterruptVector_literal_start = ABSOLUTE(.);
    *(.Level2InterruptVector.literal)
    _Level2InterruptVector_literal_end = ABSOLUTE(.);
  } >vector_int2_lit AT>vector_int2_lit_lma :vector_int2_lit_phdr
    _Level2InterruptVector_literal_lma_start = LOADADDR(.Level2InterruptVector.literal);
    _Level2InterruptVector_literal_lma_end = _Level2InterruptVector_literal_lma_start + SIZEOF(.Level2InterruptVector.literal);

  .Level2InterruptVector.text : ALIGN(4)
  {
    _Level2InterruptVector_text_start = ABSOLUTE(.);
    KEEP (*(.Level2InterruptVector.text))
    _Level2InterruptVector_text_end = ABSOLUTE(.);
  } >vector_int2_text AT>vector_int2_text_lma :vector_int2_text_phdr
    _Level2InterruptVector_text_lma_start = LOADADDR(.Level2InterruptVector.text);
    _Level2InterruptVector_text_lma_end = _Level2InterruptVector_text_lma_start + SIZEOF(.Level2InterruptVector.text);

  .Level3InterruptVector.literal : ALIGN(4)
  {
    _Level3InterruptVector_literal_start = ABSOLUTE(.);
    *(.Level3InterruptVector.literal)
    _Level3InterruptVector_literal_end = ABSOLUTE(.);
  } >vector_int3_lit AT>vector_int3_lit_lma :vector_int3_lit_phdr
    _Level3InterruptVector_literal_lma_start = LOADADDR(.Level3InterruptVector.literal);
    _Level3InterruptVector_literal_lma_end = _Level3InterruptVector_literal_lma_start + SIZEOF(.Level3InterruptVector.literal);

  .Level3InterruptVector.text : ALIGN(4)
  {
    _Level3InterruptVector_text_start = ABSOLUTE(.);
    KEEP (*(.Level3InterruptVector.text))
    _Level3InterruptVector_text_end = ABSOLUTE(.);
  } >vector_int3_text AT>vector_int3_text_lma :vector_int3_text_phdr
    _Level3InterruptVector_text_lma_start = LOADADDR(.Level3InterruptVector.text);
    _Level3InterruptVector_text_lma_end = _Level3InterruptVector_text_lma_start + SIZEOF(.Level3InterruptVector.text);

  .DebugExceptionVector.literal : ALIGN(4)
  {
    _DebugExceptionVector_literal_start = ABSOLUTE(.);
    *(.DebugExceptionVector.literal)
    _DebugExceptionVector_literal_end = ABSOLUTE(.);
  } >vector_int4_lit AT>vector_int4_lit_lma :vector_int4_lit_phdr
    _DebugExceptionVector_literal_lma_start = LOADADDR(.DebugExceptionVector.literal);
   _DebugExceptionVector_literal_lma_end = _DebugExceptionVector_literal_lma_start + SIZEOF(.DebugExceptionVector.literal);

  .DebugExceptionVector.text : ALIGN(4)
  {
    _DebugExceptionVector_text_start = ABSOLUTE(.);
    KEEP (*(.DebugExceptionVector.text))
    _DebugExceptionVector_text_end = ABSOLUTE(.);
  } >vector_int4_text AT>vector_int4_text_lma :vector_int4_text_phdr
    _DebugExceptionVector_text_lma_start = LOADADDR(.DebugExceptionVector.text);
    _DebugExceptionVector_text_lma_end = _DebugExceptionVector_text_lma_start + SIZEOF(.DebugExceptionVector.text);

  .NMIExceptionVector.literal : ALIGN(4)
  {
    _NMIExceptionVector_literal_start = ABSOLUTE(.);
    *(.NMIExceptionVector.literal)
    _NMIExceptionVector_literal_end = ABSOLUTE(.);
  } >vector_int5_lit AT>vector_int5_lit_lma :vector_int5_lit_phdr
    _NMIExceptionVector_literal_lma_start = LOADADDR(.NMIExceptionVector.literal);
    _NMIExceptionVector_literal_lma_end = _NMIExceptionVector_literal_lma_start + SIZEOF(.NMIExceptionVector.literal);

  .NMIExceptionVector.text : ALIGN(4)
  {
    _NMIExceptionVector_text_start = ABSOLUTE(.);
    KEEP (*(.NMIExceptionVector.text))
    _NMIExceptionVector_text_end = ABSOLUTE(.);
  } >vector_int5_text AT>vector_int5_text_lma :vector_int5_text_phdr
    _NMIExceptionVector_text_lma_start = LOADADDR(.NMIExceptionVector.text);
    _NMIExceptionVector_text_lma_end = _NMIExceptionVector_text_lma_start + SIZEOF(.NMIExceptionVector.text);

  .KernelExceptionVector.literal : ALIGN(4)
  {
    _KernelExceptionVector_literal_start = ABSOLUTE(.);
    *(.KernelExceptionVector.literal)
    _KernelExceptionVector_literal_end = ABSOLUTE(.);
  } >vector_kernel_lit AT>vector_kernel_lit_lma :vector_kernel_lit_phdr
    _KernelExceptionVector_literal_lma_start = LOADADDR(.KernelExceptionVector.literal);
    _KernelExceptionVector_literal_lma_end = _KernelExceptionVector_literal_lma_start + SIZEOF(.KernelExceptionVector.literal);

  .KernelExceptionVector.text : ALIGN(4)
  {
    _KernelExceptionVector_text_start = ABSOLUTE(.);
    KEEP (*(.KernelExceptionVector.text))
    _KernelExceptionVector_text_end = ABSOLUTE(.);
  } >vector_kernel_text AT>vector_kernel_text_lma :vector_kernel_text_phdr
    _KernelExceptionVector_text_lma_start = LOADADDR(.KernelExceptionVector.text);
    _KernelExceptionVector_text_lma_end = _KernelExceptionVector_text_lma_start + SIZEOF(.KernelExceptionVector.text);

  .UserExceptionVector.literal : ALIGN(4)
  {
    _UserExceptionVector_literal_start = ABSOLUTE(.);
    *(.UserExceptionVector.literal)
    _UserExceptionVector_literal_end = ABSOLUTE(.);
  } >vector_user_lit AT>vector_user_lit_lma :vector_user_lit_phdr
    _UserExceptionVector_literal_lma_start = LOADADDR(.UserExceptionVector.literal);
    _UserExceptionVector_literal_lma_end = _UserExceptionVector_literal_lma_start + SIZEOF(.UserExceptionVector.literal);

  .UserExceptionVector.text : ALIGN(4)
  {
    _UserExceptionVector_text_start = ABSOLUTE(.);
    KEEP (*(.UserExceptionVector.text))
    _UserExceptionVector_text_end = ABSOLUTE(.);
  } >vector_user_text AT>vector_user_text_lma :vector_user_text_phdr
    _UserExceptionVector_text_lma_start = LOADADDR(.UserExceptionVector.text);
    _UserExceptionVector_text_lma_end = _UserExceptionVector_text_lma_start + SIZEOF(.UserExceptionVector.text);

  .DoubleExceptionVector.literal : ALIGN(4)
  {
    _DoubleExceptionVector_literal_start = ABSOLUTE(.);
    *(.DoubleExceptionVector.literal)
    _DoubleExceptionVector_literal_end = ABSOLUTE(.);
  } >vector_double_lit AT>vector_double_lit_lma :vector_double_lit_phdr
    _DoubleExceptionVector_literal_lma_start = LOADADDR(.DoubleExceptionVector.literal);
    _DoubleExceptionVector_literal_lma_end = _DoubleExceptionVector_literal_lma_start + SIZEOF(.DoubleExceptionVector.literal);

  .DoubleExceptionVector.text : ALIGN(4)
  {
    _DoubleExceptionVector_text_start = ABSOLUTE(.);
    KEEP (*(.DoubleExceptionVector.text))
    _DoubleExceptionVector_text_end = ABSOLUTE(.);
  } >vector_double_text AT>vector_double_text_lma :vector_double_text_phdr
    _DoubleExceptionVector_text_lma_start = LOADADDR(.DoubleExceptionVector.text);
    _DoubleExceptionVector_text_lma_end = _DoubleExceptionVector_text_lma_start + SIZEOF(.DoubleExceptionVector.text);

  /* Relocatable text sections */
  .iram.text : ALIGN(4)
  {
    _stext = .;
    _iram_text_start = ABSOLUTE(.);
    *(.iram0.literal .iram.literal .iram.text.literal .iram0.text .iram.text)
    _iram_text_end = ABSOLUTE(.);
  } >adsp_text AT>adsp_text_lma :adsp_text_phdr
    _iram_text_lma_start = LOADADDR(.iram.text);
    _iram_text_lma_end = _iram_text_lma_start + SIZEOF(.iram.text);

  .rodata : ALIGN(4)
  {
    __rodata_region_start = ABSOLUTE(.);
    *(.rodata)
    *(.rodata.*)
    *(.gnu.linkonce.r.*)
    *(.rodata1)
    __XT_EXCEPTION_TABLE__ = ABSOLUTE(.);
    KEEP (*(.xt_except_table))
    KEEP (*(.gcc_except_table .gcc_except_table.*))
    *(.gnu.linkonce.e.*)
    *(.gnu.version_r)
    KEEP (*(.eh_frame))
    KEEP (*crtbegin.o(.ctors))
    KEEP (*(EXCLUDE_FILE (*crtend.o) .ctors))
    KEEP (*(SORT(.ctors.*)))
    KEEP (*(.ctors))
    KEEP (*crtbegin.o(.dtors))
    KEEP (*(EXCLUDE_FILE (*crtend.o) .dtors))
    KEEP (*(SORT(.dtors.*)))
    KEEP (*(.dtors))
    __XT_EXCEPTION_DESCS__ = ABSOLUTE(.);
    *(.xt_except_desc)
    *(.gnu.linkonce.h.*)
    __XT_EXCEPTION_DESCS_END__ = ABSOLUTE(.);
    *(.xt_except_desc_end)
    *(.dynamic)
    *(.gnu.version_d)
    . = ALIGN(4);
    _bss_table_start = ABSOLUTE(.);
    LONG(_bss_start)
    LONG(_bss_end)
    _bss_table_end = ABSOLUTE(.);
    __rodata_region_end = ABSOLUTE(.);
  } >adsp_data AT>adsp_data_lma :adsp_data_phdr
    __rodata_lma_start = LOADADDR(.rodata);
    __rodata_lma_end = __rodata_lma_start + SIZEOF(.rodata);

  .module_init : ALIGN(4)
  {
    _module_init_start = ABSOLUTE(.);
    *(*.initcall)
    _module_init_end = ABSOLUTE(.);
  } >adsp_data AT>adsp_data_lma :adsp_data_phdr
    _module_init_lma_start = LOADADDR(.module_init);
    _module_init_lma_end = _module_init_lma_start + SIZEOF(.module_init);

  .text : ALIGN(4)
  {
    _stext = .;
    _text_start = ABSOLUTE(.);
    KEEP (*(.ResetVector.text))
    *(.ResetVector.literal)
    *(.entry.text)
    *(.init.literal)
    KEEP(*(.init))
    *(.literal .text .literal.* .text.* .stub .gnu.warning .gnu.linkonce.literal.* .gnu.linkonce.t.*.literal .gnu.linkonce.t.*)
    *(.fini.literal)
    KEEP(*(.fini))
    *(.gnu.version)
    _text_end = ABSOLUTE(.);
    _etext = .;
  } >adsp_text AT>adsp_text_lma :adsp_text_phdr
    _text_lma_start = LOADADDR(.text);
    _text_lma_end = _text_lma_start + SIZEOF(.text);

#include <zephyr/linker/common-rom.ld>

  .fw_ready : ALIGN(4)
  {
    KEEP(*(".fw_ready"));
    KEEP (*(.fw_ready_metadata))
  } >adsp_data AT>adsp_data_lma :adsp_data_phdr
    _fw_ready_lma_start = LOADADDR(.fw_ready);
    _fw_ready_lma_end = _fw_ready_lma_start + SIZEOF(.fw_ready);

  .noinit : ALIGN(4)
   {
    *(.noinit)
    *(.noinit.*)
  } >adsp_data AT>adsp_data_lma :adsp_data_phdr
    _noinit_lma_start = LOADADDR(.noinit);
    _noinit_lma_end = _noinit_lma_start + SIZEOF(.noinit);

  .data : ALIGN(4)
  {
    __data_start = ABSOLUTE(.);
    *(.data)
    *(.data.*)
    *(.gnu.linkonce.d.*)
    KEEP(*(.gnu.linkonce.d.*personality*))
    *(.data1)
    *(.sdata)
    *(.sdata.*)
    *(.gnu.linkonce.s.*)
    *(.sdata2)
    *(.sdata2.*)
    *(.gnu.linkonce.s2.*)
    KEEP(*(.jcr))
    _trace_ctx_start = ABSOLUTE(.);
    *(.trace_ctx)
    _trace_ctx_end = ABSOLUTE(.);
    . = ALIGN(4);
    *(.gna_model)
    *(NonCacheable)
    __data_end = ABSOLUTE(.);
    . = ALIGN(4096);
  } >adsp_data AT>adsp_data_lma :adsp_data_phdr
    __data_lma_start = LOADADDR(.data);
    __data_lma_end = __data_lma_start + SIZEOF(.data);

  .lit4 : ALIGN(4)
  {
    _lit4_start = ABSOLUTE(.);
    *(*.lit4)
    *(.lit4.*)
    *(.gnu.linkonce.lit4.*)
    _lit4_end = ABSOLUTE(.);
  } >adsp_data AT>adsp_data_lma :adsp_data_phdr
    _lit4_lma_start = LOADADDR(.lit4);
    _lit4_lma_end = _lit4_lma_start + SIZEOF(.lit4);

#include <zephyr/linker/common-ram.ld>

  .bss (NOLOAD) : ALIGN(8)
  {
    . = ALIGN (8);
    _bss_start = ABSOLUTE(.);
    *(.dynsbss)
    *(.sbss)
    *(.sbss.*)
    *(.gnu.linkonce.sb.*)
    *(.scommon)
    *(.sbss2)
    *(.sbss2.*)
    *(.gnu.linkonce.sb2.*)
    *(.dynbss)
    *(.bss)
    *(.bss.*)
    *(.gnu.linkonce.b.*)
    *(COMMON)
    . = ALIGN (8);
    _bss_end = ABSOLUTE(.);
  } >adsp_data :adsp_data_phdr

  .heap_mem (NOLOAD) : ALIGN(8)
  {
    . = ALIGN (8);
    _heap_mem_start = ABSOLUTE(.);
    *(*.heap_mem)
    _heap_mem_end = ABSOLUTE(.);
  } >adsp_data :adsp_data_phdr

  /* stack */
  _end = ALIGN (8);

  /DISCARD/ : { *(.note.GNU-stack) }
  _heap_sentry = RELOC_DATA_BASE + DATA_SIZE;
  __stack = RELOC_DATA_BASE + DATA_SIZE;
  .comment  0 :  { *(.comment) }
  .debug  0 :  { *(.debug) }
  .line  0 :  { *(.line) }
  .debug_srcinfo  0 :  { *(.debug_srcinfo) }
  .debug_sfnames  0 :  { *(.debug_sfnames) }
  .debug_aranges  0 :  { *(.debug_aranges) }
  .debug_pubnames  0 :  { *(.debug_pubnames) }
  .debug_info  0 :  { *(.debug_info) }

  .debug_abbrev  0 :  { *(.debug_abbrev) }
  .debug_line  0 :  { *(.debug_line) }
  .debug_frame  0 :  { *(.debug_frame) }
  .debug_str  0 :  { *(.debug_str) }
  .debug_loc  0 :  { *(.debug_loc) }
  .debug_macinfo  0 :  { *(.debug_macinfo) }
  .debug_weaknames  0 :  { *(.debug_weaknames) }
  .debug_funcnames  0 :  { *(.debug_funcnames) }
  .debug_typenames  0 :  { *(.debug_typenames) }
  .debug_varnames  0 :  { *(.debug_varnames) }
  .debug_ranges  0 :  { *(.debug_ranges) }
  .xtensa.info  0 :  { *(.xtensa.info) }
  .xt.insn 0 :
  {
    KEEP (*(.xt.insn))
    KEEP (*(.gnu.linkonce.x.*))
  }
  .xt.prop 0 :
  {
    KEEP (*(.xt.prop))
    KEEP (*(.xt.prop.*))
    KEEP (*(.gnu.linkonce.prop.*))
  }
  .xt.lit 0 :
  {
    KEEP (*(.xt.lit))
    KEEP (*(.xt.lit.*))
    KEEP (*(.gnu.linkonce.p.*))
  }
  .xt.profile_range 0 :
  {
    KEEP (*(.xt.profile_range))
    KEEP (*(.gnu.linkonce.profile_range.*))
  }
  .xt.profile_ranges 0 :
  {
    KEEP (*(.xt.profile_ranges))
    KEEP (*(.gnu.linkonce.xt.profile_ranges.*))
  }
  .xt.profile_files 0 :
  {
    KEEP (*(.xt.profile_files))
    KEEP (*(.gnu.linkonce.xt.profile_files.*))
  }
#ifdef CONFIG_GEN_ISR_TABLES
#include <zephyr/linker/intlist.ld>
#endif
}