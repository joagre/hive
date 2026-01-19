// STM32F4xx Device Header (Minimal for Crazyflie HAL)
//
// This is a minimal device header with only the peripherals used by the HAL.
// For full functionality, use the official ST CMSIS headers.

#ifndef STM32F4XX_H
#define STM32F4XX_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// Include core definitions
#include "core_cm4.h"

// ----------------------------------------------------------------------------
// Peripheral Memory Map
// ----------------------------------------------------------------------------

#define PERIPH_BASE       0x40000000UL
#define APB1PERIPH_BASE   PERIPH_BASE
#define APB2PERIPH_BASE   (PERIPH_BASE + 0x00010000UL)
#define AHB1PERIPH_BASE   (PERIPH_BASE + 0x00020000UL)

// APB1 Peripherals
#define TIM2_BASE         (APB1PERIPH_BASE + 0x0000UL)
#define TIM3_BASE         (APB1PERIPH_BASE + 0x0400UL)
#define TIM4_BASE         (APB1PERIPH_BASE + 0x0800UL)
#define USART2_BASE       (APB1PERIPH_BASE + 0x4400UL)
#define SPI2_BASE         (APB1PERIPH_BASE + 0x3800UL)
#define SPI3_BASE         (APB1PERIPH_BASE + 0x3C00UL)
#define I2C1_BASE         (APB1PERIPH_BASE + 0x5400UL)
#define I2C2_BASE         (APB1PERIPH_BASE + 0x5800UL)
#define I2C3_BASE         (APB1PERIPH_BASE + 0x5C00UL)

// APB2 Peripherals
#define SPI1_BASE         (APB2PERIPH_BASE + 0x3000UL)
#define USART1_BASE       (APB2PERIPH_BASE + 0x1000UL)

// AHB1 Peripherals
#define GPIOA_BASE        (AHB1PERIPH_BASE + 0x0000UL)
#define GPIOB_BASE        (AHB1PERIPH_BASE + 0x0400UL)
#define GPIOC_BASE        (AHB1PERIPH_BASE + 0x0800UL)
#define GPIOD_BASE        (AHB1PERIPH_BASE + 0x0C00UL)
#define RCC_BASE          (AHB1PERIPH_BASE + 0x3800UL)
#define FLASH_R_BASE      (AHB1PERIPH_BASE + 0x3C00UL)

// ----------------------------------------------------------------------------
// GPIO Registers
// ----------------------------------------------------------------------------

typedef struct {
    volatile uint32_t MODER;
    volatile uint32_t OTYPER;
    volatile uint32_t OSPEEDR;
    volatile uint32_t PUPDR;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t LCKR;
    volatile uint32_t AFR[2];
} GPIO_TypeDef;

#define GPIOA  ((GPIO_TypeDef *)GPIOA_BASE)
#define GPIOB  ((GPIO_TypeDef *)GPIOB_BASE)
#define GPIOC  ((GPIO_TypeDef *)GPIOC_BASE)
#define GPIOD  ((GPIO_TypeDef *)GPIOD_BASE)

// GPIO bit definitions
#define GPIO_MODER_MODER0     (0x3UL << 0)
#define GPIO_MODER_MODER0_0   (0x1UL << 0)
#define GPIO_MODER_MODER0_1   (0x2UL << 0)
#define GPIO_MODER_MODER1     (0x3UL << 2)
#define GPIO_MODER_MODER1_0   (0x1UL << 2)
#define GPIO_MODER_MODER1_1   (0x2UL << 2)
#define GPIO_MODER_MODER2     (0x3UL << 4)
#define GPIO_MODER_MODER2_0   (0x1UL << 4)
#define GPIO_MODER_MODER2_1   (0x2UL << 4)
#define GPIO_MODER_MODER3     (0x3UL << 6)
#define GPIO_MODER_MODER3_0   (0x1UL << 6)
#define GPIO_MODER_MODER3_1   (0x2UL << 6)
#define GPIO_MODER_MODER4     (0x3UL << 8)
#define GPIO_MODER_MODER4_0   (0x1UL << 8)
#define GPIO_MODER_MODER4_1   (0x2UL << 8)
#define GPIO_MODER_MODER5     (0x3UL << 10)
#define GPIO_MODER_MODER5_0   (0x1UL << 10)
#define GPIO_MODER_MODER5_1   (0x2UL << 10)
#define GPIO_MODER_MODER6     (0x3UL << 12)
#define GPIO_MODER_MODER6_0   (0x1UL << 12)
#define GPIO_MODER_MODER6_1   (0x2UL << 12)
#define GPIO_MODER_MODER7     (0x3UL << 14)
#define GPIO_MODER_MODER7_0   (0x1UL << 14)
#define GPIO_MODER_MODER7_1   (0x2UL << 14)
#define GPIO_MODER_MODER8     (0x3UL << 16)
#define GPIO_MODER_MODER8_0   (0x1UL << 16)
#define GPIO_MODER_MODER8_1   (0x2UL << 16)
#define GPIO_MODER_MODER9     (0x3UL << 18)
#define GPIO_MODER_MODER9_0   (0x1UL << 18)
#define GPIO_MODER_MODER9_1   (0x2UL << 18)

#define GPIO_OTYPER_OT8       (1UL << 8)
#define GPIO_OTYPER_OT9       (1UL << 9)

#define GPIO_OSPEEDER_OSPEEDR0 (0x3UL << 0)
#define GPIO_OSPEEDER_OSPEEDR1 (0x3UL << 2)
#define GPIO_OSPEEDER_OSPEEDR2 (0x3UL << 4)
#define GPIO_OSPEEDER_OSPEEDR3 (0x3UL << 6)
#define GPIO_OSPEEDER_OSPEEDR4 (0x3UL << 8)
#define GPIO_OSPEEDER_OSPEEDR5 (0x3UL << 10)
#define GPIO_OSPEEDER_OSPEEDR6 (0x3UL << 12)
#define GPIO_OSPEEDER_OSPEEDR7 (0x3UL << 14)

#define GPIO_PUPDR_PUPDR0     (0x3UL << 0)
#define GPIO_PUPDR_PUPDR1     (0x3UL << 2)
#define GPIO_PUPDR_PUPDR2     (0x3UL << 4)
#define GPIO_PUPDR_PUPDR3     (0x3UL << 6)
#define GPIO_PUPDR_PUPDR4     (0x3UL << 8)
#define GPIO_PUPDR_PUPDR4_0   (0x1UL << 8)
#define GPIO_PUPDR_PUPDR4_1   (0x2UL << 8)
#define GPIO_PUPDR_PUPDR5     (0x3UL << 10)
#define GPIO_PUPDR_PUPDR6     (0x3UL << 12)
#define GPIO_PUPDR_PUPDR7     (0x3UL << 14)
#define GPIO_PUPDR_PUPDR8     (0x3UL << 16)
#define GPIO_PUPDR_PUPDR8_0   (0x1UL << 16)
#define GPIO_PUPDR_PUPDR9     (0x3UL << 18)
#define GPIO_PUPDR_PUPDR9_0   (0x1UL << 18)

#define GPIO_ODR_OD4          (1UL << 4)

// GPIO alternate function definitions
#define GPIO_AFRL_AFRL0       (0xFUL << 0)
#define GPIO_AFRL_AFRL1       (0xFUL << 4)
#define GPIO_AFRL_AFRL2       (0xFUL << 8)
#define GPIO_AFRL_AFRL3       (0xFUL << 12)
#define GPIO_AFRL_AFRL4       (0xFUL << 16)
#define GPIO_AFRL_AFRL5       (0xFUL << 20)
#define GPIO_AFRL_AFRL6       (0xFUL << 24)
#define GPIO_AFRL_AFRL7       (0xFUL << 28)
#define GPIO_AFRH_AFRH8       (0xFUL << 0)
#define GPIO_AFRH_AFRH9       (0xFUL << 4)
#define GPIO_AFRH_AFRH10      (0xFUL << 8)
#define GPIO_AFRH_AFRH11      (0xFUL << 12)
#define GPIO_AFRH_AFRH12      (0xFUL << 16)
#define GPIO_AFRH_AFRH13      (0xFUL << 20)
#define GPIO_AFRH_AFRH14      (0xFUL << 24)
#define GPIO_AFRH_AFRH15      (0xFUL << 28)

// ----------------------------------------------------------------------------
// RCC Registers
// ----------------------------------------------------------------------------

typedef struct {
    volatile uint32_t CR;
    volatile uint32_t PLLCFGR;
    volatile uint32_t CFGR;
    volatile uint32_t CIR;
    volatile uint32_t AHB1RSTR;
    volatile uint32_t AHB2RSTR;
    volatile uint32_t AHB3RSTR;
    uint32_t RESERVED0;
    volatile uint32_t APB1RSTR;
    volatile uint32_t APB2RSTR;
    uint32_t RESERVED1[2];
    volatile uint32_t AHB1ENR;
    volatile uint32_t AHB2ENR;
    volatile uint32_t AHB3ENR;
    uint32_t RESERVED2;
    volatile uint32_t APB1ENR;
    volatile uint32_t APB2ENR;
} RCC_TypeDef;

#define RCC  ((RCC_TypeDef *)RCC_BASE)

// RCC bit definitions
#define RCC_CR_HSEON          (1UL << 16)
#define RCC_CR_HSERDY         (1UL << 17)
#define RCC_CR_PLLON          (1UL << 24)
#define RCC_CR_PLLRDY         (1UL << 25)

#define RCC_PLLCFGR_PLLM_Pos  0
#define RCC_PLLCFGR_PLLN_Pos  6
#define RCC_PLLCFGR_PLLP_Pos  16
#define RCC_PLLCFGR_PLLSRC_HSE (1UL << 22)
#define RCC_PLLCFGR_PLLQ_Pos  24

#define RCC_CFGR_SW_PLL       (2UL << 0)
#define RCC_CFGR_SWS          (3UL << 2)
#define RCC_CFGR_SWS_PLL      (2UL << 2)
#define RCC_CFGR_HPRE_DIV1    (0UL << 4)
#define RCC_CFGR_PPRE1_DIV4   (5UL << 10)
#define RCC_CFGR_PPRE2_DIV2   (4UL << 13)

#define RCC_AHB1ENR_GPIOAEN   (1UL << 0)
#define RCC_AHB1ENR_GPIOBEN   (1UL << 1)
#define RCC_AHB1ENR_GPIOCEN   (1UL << 2)

#define RCC_APB1ENR_TIM2EN    (1UL << 0)
#define RCC_APB1ENR_USART2EN  (1UL << 17)
#define RCC_APB1ENR_I2C3EN    (1UL << 23)

#define RCC_APB2ENR_SPI1EN    (1UL << 12)

// ----------------------------------------------------------------------------
// FLASH Registers
// ----------------------------------------------------------------------------

typedef struct {
    volatile uint32_t ACR;
    volatile uint32_t KEYR;
    volatile uint32_t OPTKEYR;
    volatile uint32_t SR;
    volatile uint32_t CR;
    volatile uint32_t OPTCR;
} FLASH_TypeDef;

#define FLASH  ((FLASH_TypeDef *)FLASH_R_BASE)

#define FLASH_ACR_LATENCY_5WS (5UL << 0)
#define FLASH_ACR_PRFTEN      (1UL << 8)
#define FLASH_ACR_ICEN        (1UL << 9)
#define FLASH_ACR_DCEN        (1UL << 10)

// ----------------------------------------------------------------------------
// TIM Registers
// ----------------------------------------------------------------------------

typedef struct {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SMCR;
    volatile uint32_t DIER;
    volatile uint32_t SR;
    volatile uint32_t EGR;
    volatile uint32_t CCMR1;
    volatile uint32_t CCMR2;
    volatile uint32_t CCER;
    volatile uint32_t CNT;
    volatile uint32_t PSC;
    volatile uint32_t ARR;
    uint32_t RESERVED1;
    volatile uint32_t CCR1;
    volatile uint32_t CCR2;
    volatile uint32_t CCR3;
    volatile uint32_t CCR4;
} TIM_TypeDef;

#define TIM2  ((TIM_TypeDef *)TIM2_BASE)
#define TIM3  ((TIM_TypeDef *)TIM3_BASE)
#define TIM4  ((TIM_TypeDef *)TIM4_BASE)

#define TIM_CR1_CEN           (1UL << 0)
#define TIM_CR1_ARPE          (1UL << 7)

#define TIM_EGR_UG            (1UL << 0)

#define TIM_CCMR1_OC1M_Pos    4
#define TIM_CCMR1_OC1PE       (1UL << 3)
#define TIM_CCMR1_OC2M_Pos    12
#define TIM_CCMR1_OC2PE       (1UL << 11)
#define TIM_CCMR2_OC3M_Pos    4
#define TIM_CCMR2_OC3PE       (1UL << 3)
#define TIM_CCMR2_OC4M_Pos    12
#define TIM_CCMR2_OC4PE       (1UL << 11)

#define TIM_CCER_CC1E         (1UL << 0)
#define TIM_CCER_CC2E         (1UL << 4)
#define TIM_CCER_CC3E         (1UL << 8)
#define TIM_CCER_CC4E         (1UL << 12)

// ----------------------------------------------------------------------------
// SPI Registers
// ----------------------------------------------------------------------------

typedef struct {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SR;
    volatile uint32_t DR;
    volatile uint32_t CRCPR;
    volatile uint32_t RXCRCR;
    volatile uint32_t TXCRCR;
    volatile uint32_t I2SCFGR;
    volatile uint32_t I2SPR;
} SPI_TypeDef;

#define SPI1  ((SPI_TypeDef *)SPI1_BASE)
#define SPI2  ((SPI_TypeDef *)SPI2_BASE)
#define SPI3  ((SPI_TypeDef *)SPI3_BASE)

#define SPI_CR1_MSTR          (1UL << 2)
#define SPI_CR1_BR_0          (1UL << 3)
#define SPI_CR1_BR_1          (1UL << 4)
#define SPI_CR1_SPE           (1UL << 6)
#define SPI_CR1_SSI           (1UL << 8)
#define SPI_CR1_SSM           (1UL << 9)

#define SPI_SR_RXNE           (1UL << 0)
#define SPI_SR_TXE            (1UL << 1)

// ----------------------------------------------------------------------------
// I2C Registers
// ----------------------------------------------------------------------------

typedef struct {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t OAR1;
    volatile uint32_t OAR2;
    volatile uint32_t DR;
    volatile uint32_t SR1;
    volatile uint32_t SR2;
    volatile uint32_t CCR;
    volatile uint32_t TRISE;
    volatile uint32_t FLTR;
} I2C_TypeDef;

#define I2C1  ((I2C_TypeDef *)I2C1_BASE)
#define I2C2  ((I2C_TypeDef *)I2C2_BASE)
#define I2C3  ((I2C_TypeDef *)I2C3_BASE)

#define I2C_CR1_PE            (1UL << 0)
#define I2C_CR1_START         (1UL << 8)
#define I2C_CR1_STOP          (1UL << 9)
#define I2C_CR1_ACK           (1UL << 10)

#define I2C_SR1_SB            (1UL << 0)
#define I2C_SR1_ADDR          (1UL << 1)
#define I2C_SR1_BTF           (1UL << 2)
#define I2C_SR1_RXNE          (1UL << 6)
#define I2C_SR1_TXE           (1UL << 7)

// ----------------------------------------------------------------------------
// USART Registers
// ----------------------------------------------------------------------------

typedef struct {
    volatile uint32_t SR;
    volatile uint32_t DR;
    volatile uint32_t BRR;
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t CR3;
    volatile uint32_t GTPR;
} USART_TypeDef;

#define USART1  ((USART_TypeDef *)USART1_BASE)
#define USART2  ((USART_TypeDef *)USART2_BASE)

// USART CR1 bit definitions
#define USART_CR1_UE          (1UL << 13)
#define USART_CR1_TE          (1UL << 3)
#define USART_CR1_RE          (1UL << 2)

// USART SR bit definitions
#define USART_SR_TXE          (1UL << 7)
#define USART_SR_RXNE         (1UL << 5)

// ----------------------------------------------------------------------------
// System Variables
// ----------------------------------------------------------------------------

extern uint32_t SystemCoreClock;

#ifdef __cplusplus
}
#endif

#endif // STM32F4XX_H
