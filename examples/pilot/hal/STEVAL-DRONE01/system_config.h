// System configuration for STM32F401 (STEVAL-DRONE01)
//
// Clock configuration for 84MHz operation.
// Peripheral initialization stubs.

#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

// ----------------------------------------------------------------------------
// Clock Configuration
// ----------------------------------------------------------------------------

// External crystal frequency (HSE)
// STEVAL-DRONE01 uses 16MHz crystal
#define HSE_VALUE           16000000U

// Target system clock frequency
// NOTE: Currently running on HSI (no PLL) at 16MHz for testing
#define SYSCLK_FREQ         16000000U   // 16 MHz (HSI mode)

// AHB clock (HCLK) - feeds Cortex-M4 core, memory, DMA
#define HCLK_FREQ           16000000U   // 16 MHz (HSI mode)

// APB1 clock (PCLK1) - low-speed peripherals (I2C, UART2, TIM2-5)
#define PCLK1_FREQ          16000000U   // 16 MHz (HSI mode)

// APB2 clock (PCLK2) - high-speed peripherals (SPI1, UART1, TIM1)
#define PCLK2_FREQ          16000000U   // 16 MHz (HSI mode)

// PLL configuration for 84MHz from 16MHz HSE:
//   SYSCLK = HSE * PLLN / PLLM / PLLP
//   84MHz = 16MHz * 336 / 16 / 4 = 84MHz
//
//   USB requires 48MHz:
//   USB_CLK = HSE * PLLN / PLLM / PLLQ
//   48MHz = 16MHz * 336 / 16 / 7 = 48MHz
#define PLL_M               16
#define PLL_N               336
#define PLL_P               4       // PLLP = 4 (register value = 1)
#define PLL_Q               7       // For USB 48MHz

// ----------------------------------------------------------------------------
// Timing
// ----------------------------------------------------------------------------

// SysTick configuration (1ms tick)
#define SYSTICK_FREQ        1000U       // 1kHz (1ms period)

// Get current tick count (milliseconds since boot)
uint32_t system_get_tick(void);

// Delay for specified milliseconds
void system_delay_ms(uint32_t ms);

// Delay for specified microseconds (busy-wait)
void system_delay_us(uint32_t us);

// Get microsecond timestamp (from DWT cycle counter)
uint32_t system_get_us(void);

// ----------------------------------------------------------------------------
// System Initialization
// ----------------------------------------------------------------------------

// Initialize system clocks (HSE + PLL for 84MHz)
// Returns true on success, false if HSE fails to start
bool system_clock_init(void);

// Initialize SysTick for 1ms interrupt
void system_tick_init(void);

// Initialize DWT cycle counter for microsecond timing
void system_dwt_init(void);

// Full system initialization (clocks + tick + DWT)
bool system_init(void);

// ----------------------------------------------------------------------------
// Peripheral Clock Control
// ----------------------------------------------------------------------------

// Enable/disable peripheral clocks
void system_enable_gpio(char port);     // 'A' to 'H'
void system_enable_spi1(void);
void system_enable_spi2(void);
void system_enable_i2c1(void);
void system_enable_i2c2(void);
void system_enable_tim2(void);
void system_enable_tim4(void);
void system_enable_usart1(void);
void system_enable_usart2(void);
void system_enable_dma1(void);
void system_enable_dma2(void);

// ----------------------------------------------------------------------------
// Error Handling
// ----------------------------------------------------------------------------

// System error handler (called on clock failure, etc.)
// Default implementation loops forever; override as needed.
void system_error_handler(void);

#endif // SYSTEM_CONFIG_H
