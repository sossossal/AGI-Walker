/**
 * @file linker.ld
 * @brief IMC-22 链接脚本
 */

MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 2M
    SRAM  (rwx) : ORIGIN = 0x20000000, LENGTH = 512K
}

ENTRY(_start)

SECTIONS
{
    /* 代码段 */
    .text :
    {
        KEEP(*(.vectors))       /* 中断向量表 */
        *(.text)
        *(.text*)
        *(.rodata)
        *(.rodata*)
        . = ALIGN(4);
    } > FLASH

    /* 数据段 (已初始化) */
    .data :
    {
        _sdata = .;
        *(.data)
        *(.data*)
        _edata = .;
    } > SRAM AT> FLASH
    
    _sidata = LOADADDR(.data);

    /* BSS 段 (未初始化) */
    .bss :
    {
        _sbss = .;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } > SRAM

    /* 栈 */
    . = ALIGN(8);
    _estack = ORIGIN(SRAM) + LENGTH(SRAM);
}
