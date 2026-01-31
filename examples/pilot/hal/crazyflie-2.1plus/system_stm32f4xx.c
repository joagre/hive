// STM32F4xx System Initialization
//
// Provides SystemInit() called by startup code before main().
// Configures system clocks to 168 MHz using external 8 MHz crystal.
//
// Clock configuration:
//   SYSCLK = 168 MHz (HSE 8MHz * 336 / 2)
//   AHB    = 168 MHz (SYSCLK / 1)
//   APB1   = 42 MHz  (AHB / 4) - max 42 MHz per datasheet
//   APB2   = 84 MHz  (AHB / 2) - max 84 MHz per datasheet

#include "stm32f4xx.h"

// CMSIS requires this variable
uint32_t SystemCoreClock = 16000000; // Default to HSI, updated after PLL config

void SystemInit(void) {
    // Configure flash latency for 168 MHz (5 wait states)
    // Enable prefetch, instruction cache, and data cache
    FLASH->ACR = FLASH_ACR_LATENCY_5WS | FLASH_ACR_PRFTEN | FLASH_ACR_ICEN |
                 FLASH_ACR_DCEN;

    // Enable HSE (external 8 MHz crystal)
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY))
        ;

    // Configure PLL: HSE (8MHz) * 336 / 2 = 168 MHz
    // PLLM[5:0]=0, PLLN[14:6]=6, PLLP[17:16]=16, PLLQ[27:24]=24
    RCC->PLLCFGR = (8U << 0) |   // PLLM = 8 (VCO input = 1 MHz)
                   (336U << 6) | // PLLN = 336 (VCO = 336 MHz)
                   (0U << 16) |  // PLLP = 2 (0 = /2, SYSCLK = 168 MHz)
                   RCC_PLLCFGR_PLLSRC_HSE | // HSE as PLL source
                   (7U << 24);              // PLLQ = 7 (USB = 48 MHz)

    // Enable PLL
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY))
        ;

    // Configure bus prescalers before switching to faster clock
    // AHB = SYSCLK/1, APB1 = AHB/4, APB2 = AHB/2
    RCC->CFGR = RCC_CFGR_HPRE_DIV1 |  // AHB  = 168 MHz
                RCC_CFGR_PPRE1_DIV4 | // APB1 = 42 MHz
                RCC_CFGR_PPRE2_DIV2;  // APB2 = 84 MHz

    // Switch system clock to PLL
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
        ;

    // Update SystemCoreClock for other code that depends on it
    SystemCoreClock = 168000000;

    // Configure NVIC priority grouping (must be done before any NVIC operations)
    // NVIC_PriorityGroup_4: 4 bits preemption, 0 bits subpriority (matches Bitcraze)
    // This is required for NVIC_SetPriority() and NVIC_EnableIRQ() to work correctly
    SCB->AIRCR = (0x5FAUL << 16) | // VECTKEY - required to write AIRCR
                 (3UL << 8);       // PRIGROUP = 3 (4 bits preemption priority)
}
