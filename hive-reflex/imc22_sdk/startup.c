/**
 * @file startup.c
 * @brief IMC-22 启动代码
 */

#include "imc22.h"

/* 外部符号 (来自链接脚本) */
extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _estack;

/* 函数声明 */
extern int main(void);
void Reset_Handler(void);
void Default_Handler(void);

/* 中断向量表 */
__attribute__((section(".vectors"))) void (*const vector_table[])(void) = {
    (void (*)(void))&_estack, // 初始栈指针
    Reset_Handler,            // 复位向量
    Default_Handler,          // NMI
    Default_Handler,          // Hard Fault
    0,
    0,
    0,
    0,
    0,
    0,
    0,               // 保留
    Default_Handler, // SVC
    0,
    0,               // 保留
    Default_Handler, // PendSV
    Default_Handler, // SysTick

    /* 外设中断 */
    Default_Handler, // IRQ 0: Timer
    Default_Handler, // IRQ 1: UART
    Default_Handler, // IRQ 2: SPI0
    Default_Handler, // IRQ 3: SPI1
    Default_Handler, // IRQ 4: I2C
    Default_Handler, // IRQ 5: CAN
    Default_Handler, // IRQ 6: ADC
    Default_Handler, // IRQ 7: PWM
    Default_Handler, // IRQ 8: DMA
    Default_Handler, // IRQ 9: GPIO
    Default_Handler, // IRQ 10: NPU
};

/**
 * @brief 复位处理函数
 */
void Reset_Handler(void) {
  uint32_t *src, *dest;

  /* 1. 拷贝 .data 段从 Flash 到 SRAM */
  src = &_sidata;
  dest = &_sdata;
  while (dest < &_edata) {
    *dest++ = *src++;
  }

  /* 2. 清零 .bss 段 */
  dest = &_sbss;
  while (dest < &_ebss) {
    *dest++ = 0;
  }

  /* 3. 调用 main */
  main();

  /* 4. 如果 main 返回，死循环 */
  while (1)
    ;
}

/**
 * @brief 默认中断处理
 */
void Default_Handler(void) {
  while (1)
    ;
}

/* 弱符号定义 (允许用户覆盖) */
void TIMER_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void UART_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void CAN_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void NPU_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
