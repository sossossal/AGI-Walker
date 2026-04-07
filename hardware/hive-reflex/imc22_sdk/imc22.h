/**
 * @file imc22.h
 * @brief IMC-22 芯片主头文件
 * @version 1.0
 *
 * IMC-22: RISC-V RV32IMAC + 神经加速器
 * 核心频率: 200MHz
 * SRAM: 512KB
 * Flash: 2MB
 */

#ifndef IMC22_H
#define IMC22_H

#include <stdbool.h>
#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif

/* ========== 基础类型定义 ========== */
typedef volatile uint32_t vuint32_t;
typedef volatile uint16_t vuint16_t;
typedef volatile uint8_t vuint8_t;

/* ========== 芯片配置 ========== */
#define IMC22_SYSCLK_HZ 200000000UL        // 200 MHz
#define IMC22_SRAM_SIZE (512 * 1024)       // 512 KB
#define IMC22_FLASH_SIZE (2 * 1024 * 1024) // 2 MB

/* ========== 内存映射 ========== */
#define IMC22_FLASH_BASE 0x08000000UL
#define IMC22_SRAM_BASE 0x20000000UL
#define IMC22_PERIPH_BASE 0x40000000UL
#define IMC22_NPU_BASE 0x50000000UL // 神经加速器

/* ========== 外设基地址 ========== */
#define TIMER_BASE (IMC22_PERIPH_BASE + 0x00000)
#define GPIO_BASE (IMC22_PERIPH_BASE + 0x10000)
#define UART_BASE (IMC22_PERIPH_BASE + 0x20000)
#define SPI0_BASE (IMC22_PERIPH_BASE + 0x30000)
#define SPI1_BASE (IMC22_PERIPH_BASE + 0x31000)
#define I2C_BASE (IMC22_PERIPH_BASE + 0x40000)
#define CAN_BASE (IMC22_PERIPH_BASE + 0x50000)
#define ADC_BASE (IMC22_PERIPH_BASE + 0x60000)
#define PWM_BASE (IMC22_PERIPH_BASE + 0x70000)
#define DMA_BASE (IMC22_PERIPH_BASE + 0x80000)
#define NVIC_BASE (IMC22_PERIPH_BASE + 0xE0000)

/* ========== 中断向量表 ========== */
typedef enum {
  IRQ_TIMER = 0,
  IRQ_UART = 1,
  IRQ_SPI0 = 2,
  IRQ_SPI1 = 3,
  IRQ_I2C = 4,
  IRQ_CAN = 5,
  IRQ_ADC = 6,
  IRQ_PWM = 7,
  IRQ_DMA = 8,
  IRQ_GPIO = 9,
  IRQ_NPU = 10, // 神经加速器中断
  IRQ_MAX = 32
} IRQn_Type;

/* ========== 定时器寄存器 ========== */
typedef struct {
  vuint32_t CTRL;   // 控制寄存器
  vuint32_t LOAD;   // 重载值
  vuint32_t COUNT;  // 当前计数值
  vuint32_t STATUS; // 状态寄存器
} TIMER_TypeDef;

#define TIMER ((TIMER_TypeDef *)TIMER_BASE)

/* 定时器控制位 */
#define TIMER_CTRL_EN (1 << 0)   // 使能
#define TIMER_CTRL_IE (1 << 1)   // 中断使能
#define TIMER_CTRL_MODE (1 << 2) // 0=单次, 1=周期

/* ========== GPIO 寄存器 ========== */
typedef struct {
  vuint32_t DIR;    // 方向 (0=输入, 1=输出)
  vuint32_t DATA;   // 数据寄存器
  vuint32_t SET;    // 置位寄存器
  vuint32_t CLR;    // 清零寄存器
  vuint32_t TOGGLE; // 翻转寄存器
  vuint32_t IE;     // 中断使能
  vuint32_t IF;     // 中断标志
} GPIO_TypeDef;

#define GPIO ((GPIO_TypeDef *)GPIO_BASE)

/* ========== UART 寄存器 ========== */
typedef struct {
  vuint32_t DATA;   // 数据寄存器
  vuint32_t STATUS; // 状态寄存器
  vuint32_t CTRL;   // 控制寄存器
  vuint32_t BAUD;   // 波特率分频
} UART_TypeDef;

#define UART ((UART_TypeDef *)UART_BASE)

#define UART_STATUS_RXNE (1 << 0) // 接收非空
#define UART_STATUS_TXE (1 << 1)  // 发送空

/* ========== 中断控制器 ========== */
typedef struct {
  vuint32_t ISER;   // 中断使能设置
  vuint32_t ICER;   // 中断使能清除
  vuint32_t ISPR;   // 中断挂起设置
  vuint32_t ICPR;   // 中断挂起清除
  vuint32_t IPR[8]; // 中断优先级 (每个 IRQ 4位)
} NVIC_TypeDef;

#define NVIC ((NVIC_TypeDef *)NVIC_BASE)

/* ========== 内联工具函数 ========== */

/**
 * @brief 使能中断
 */
static inline void NVIC_EnableIRQ(IRQn_Type IRQn) { NVIC->ISER = (1U << IRQn); }

/**
 * @brief 禁用中断
 */
static inline void NVIC_DisableIRQ(IRQn_Type IRQn) {
  NVIC->ICER = (1U << IRQn);
}

/**
 * @brief 设置中断优先级 (0-15)
 */
static inline void NVIC_SetPriority(IRQn_Type IRQn, uint8_t priority) {
  uint32_t reg_idx = IRQn / 8;
  uint32_t bit_pos = (IRQn % 8) * 4;
  NVIC->IPR[reg_idx] =
      (NVIC->IPR[reg_idx] & ~(0xF << bit_pos)) | ((priority & 0xF) << bit_pos);
}

/**
 * @brief 获取系统时钟周期数 (用于计时)
 */
static inline uint32_t GetCycleCount(void) {
  uint32_t count;
  asm volatile("rdcycle %0" : "=r"(count));
  return count;
}

/**
 * @brief 微秒级延迟
 */
static inline void DelayUs(uint32_t us) {
  uint32_t cycles = (IMC22_SYSCLK_HZ / 1000000) * us;
  uint32_t start = GetCycleCount();
  while ((GetCycleCount() - start) < cycles)
    ;
}

/**
 * @brief 毫秒级延迟
 */
static inline void DelayMs(uint32_t ms) { DelayUs(ms * 1000); }

/* ========== 导出驱动接口 ========== */
#include "imc22_adc.h"
#include "imc22_can.h"
#include "imc22_npu.h"
#include "imc22_pwm.h"
#include "imc22_spi.h"


#ifdef __cplusplus
}
#endif

#endif /* IMC22_H */
