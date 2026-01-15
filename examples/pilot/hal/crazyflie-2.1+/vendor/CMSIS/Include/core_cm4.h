// CMSIS Cortex-M4 Core Peripheral Access Layer Header File (Minimal)
//
// This is a minimal header with only the definitions used by the HAL.
// For full functionality, use the official ARM CMSIS headers.

#ifndef __CORE_CM4_H_GENERIC
#define __CORE_CM4_H_GENERIC

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ----------------------------------------------------------------------------
// CMSIS compiler specific defines
// ----------------------------------------------------------------------------

#ifndef __STATIC_INLINE
#define __STATIC_INLINE  static inline
#endif

#ifndef __STATIC_FORCEINLINE
#define __STATIC_FORCEINLINE __attribute__((always_inline)) static inline
#endif

#ifndef __NO_RETURN
#define __NO_RETURN  __attribute__((__noreturn__))
#endif

#ifndef __USED
#define __USED  __attribute__((used))
#endif

#ifndef __WEAK
#define __WEAK  __attribute__((weak))
#endif

#ifndef __PACKED
#define __PACKED  __attribute__((packed, aligned(1)))
#endif

#ifndef __PACKED_STRUCT
#define __PACKED_STRUCT  struct __attribute__((packed, aligned(1)))
#endif

#ifndef __ALIGNED
#define __ALIGNED(x)  __attribute__((aligned(x)))
#endif

// ----------------------------------------------------------------------------
// Register manipulation
// ----------------------------------------------------------------------------

#define __IM   volatile const   // Read only
#define __OM   volatile         // Write only
#define __IOM  volatile         // Read/Write

// ----------------------------------------------------------------------------
// NVIC Functions
// ----------------------------------------------------------------------------

#define NVIC_PRIO_BITS  4  // STM32F4 uses 4 bits for priority

// NVIC registers
#define NVIC_BASE       (0xE000E100UL)
#define SCB_BASE        (0xE000ED00UL)

typedef struct {
    __IOM uint32_t ISER[8U];              // Interrupt Set Enable
    uint32_t RESERVED0[24U];
    __IOM uint32_t ICER[8U];              // Interrupt Clear Enable
    uint32_t RESERVED1[24U];
    __IOM uint32_t ISPR[8U];              // Interrupt Set Pending
    uint32_t RESERVED2[24U];
    __IOM uint32_t ICPR[8U];              // Interrupt Clear Pending
    uint32_t RESERVED3[24U];
    __IOM uint32_t IABR[8U];              // Interrupt Active Bit
    uint32_t RESERVED4[56U];
    __IOM uint8_t  IP[240U];              // Interrupt Priority
    uint32_t RESERVED5[644U];
    __OM  uint32_t STIR;                  // Software Trigger Interrupt
} NVIC_Type;

#define NVIC  ((NVIC_Type *)NVIC_BASE)

// SCB (System Control Block) registers
typedef struct {
    __IM  uint32_t CPUID;                 // CPUID Base Register
    __IOM uint32_t ICSR;                  // Interrupt Control State
    __IOM uint32_t VTOR;                  // Vector Table Offset
    __IOM uint32_t AIRCR;                 // Application Interrupt and Reset Control
    __IOM uint32_t SCR;                   // System Control
    __IOM uint32_t CCR;                   // Configuration and Control
    __IOM uint8_t  SHP[12U];              // System Handler Priority
    __IOM uint32_t SHCSR;                 // System Handler Control and State
    __IOM uint32_t CFSR;                  // Configurable Fault Status
    __IOM uint32_t HFSR;                  // HardFault Status
    __IOM uint32_t DFSR;                  // Debug Fault Status
    __IOM uint32_t MMFAR;                 // MemManage Fault Address
    __IOM uint32_t BFAR;                  // BusFault Address
    __IOM uint32_t AFSR;                  // Auxiliary Fault Status
    __IM  uint32_t PFR[2U];               // Processor Feature
    __IM  uint32_t DFR;                   // Debug Feature
    __IM  uint32_t ADR;                   // Auxiliary Feature
    __IM  uint32_t MMFR[4U];              // Memory Model Feature
    __IM  uint32_t ISAR[5U];              // Instruction Set Attributes
    uint32_t RESERVED0[5U];
    __IOM uint32_t CPACR;                 // Coprocessor Access Control
} SCB_Type;

#define SCB  ((SCB_Type *)SCB_BASE)

// SCB CPACR bits for FPU enable
#define SCB_CPACR_CP10_Pos  20U
#define SCB_CPACR_CP10_Msk  (3UL << SCB_CPACR_CP10_Pos)
#define SCB_CPACR_CP11_Pos  22U
#define SCB_CPACR_CP11_Msk  (3UL << SCB_CPACR_CP11_Pos)

// SCB AIRCR bits
#define SCB_AIRCR_VECTKEY_Pos   16U
#define SCB_AIRCR_VECTKEY_Msk   (0xFFFFUL << SCB_AIRCR_VECTKEY_Pos)
#define SCB_AIRCR_SYSRESETREQ_Pos 2U
#define SCB_AIRCR_SYSRESETREQ_Msk (1UL << SCB_AIRCR_SYSRESETREQ_Pos)

// ----------------------------------------------------------------------------
// SysTick
// ----------------------------------------------------------------------------

#define SysTick_BASE  (0xE000E010UL)

typedef struct {
    __IOM uint32_t CTRL;                  // Control and Status
    __IOM uint32_t LOAD;                  // Reload Value
    __IOM uint32_t VAL;                   // Current Value
    __IM  uint32_t CALIB;                 // Calibration
} SysTick_Type;

#define SysTick  ((SysTick_Type *)SysTick_BASE)

#define SysTick_CTRL_COUNTFLAG_Pos  16U
#define SysTick_CTRL_COUNTFLAG_Msk  (1UL << SysTick_CTRL_COUNTFLAG_Pos)
#define SysTick_CTRL_CLKSOURCE_Pos  2U
#define SysTick_CTRL_CLKSOURCE_Msk  (1UL << SysTick_CTRL_CLKSOURCE_Pos)
#define SysTick_CTRL_TICKINT_Pos    1U
#define SysTick_CTRL_TICKINT_Msk    (1UL << SysTick_CTRL_TICKINT_Pos)
#define SysTick_CTRL_ENABLE_Pos     0U
#define SysTick_CTRL_ENABLE_Msk     (1UL << SysTick_CTRL_ENABLE_Pos)

#define SysTick_LOAD_RELOAD_Msk     (0xFFFFFFUL)

// ----------------------------------------------------------------------------
// FPU (Floating Point Unit) - Cortex-M4F
// ----------------------------------------------------------------------------

#if defined(__ARM_FP) && (__ARM_FP >= 1)

#define FPU_BASE  (0xE000EF30UL)

typedef struct {
    uint32_t RESERVED0[1U];
    __IOM uint32_t FPCCR;                 // FP Context Control
    __IOM uint32_t FPCAR;                 // FP Context Address
    __IOM uint32_t FPDSCR;                // FP Default Status Control
    __IM  uint32_t MVFR0;                 // Media and FP Feature 0
    __IM  uint32_t MVFR1;                 // Media and FP Feature 1
    __IM  uint32_t MVFR2;                 // Media and FP Feature 2
} FPU_Type;

#define FPU  ((FPU_Type *)FPU_BASE)

#define FPU_FPCCR_ASPEN_Pos   31U
#define FPU_FPCCR_ASPEN_Msk   (1UL << FPU_FPCCR_ASPEN_Pos)
#define FPU_FPCCR_LSPEN_Pos   30U
#define FPU_FPCCR_LSPEN_Msk   (1UL << FPU_FPCCR_LSPEN_Pos)

#endif // __ARM_FP

// ----------------------------------------------------------------------------
// Core Functions
// ----------------------------------------------------------------------------

// Enable IRQ
__STATIC_FORCEINLINE void __enable_irq(void) {
    __asm volatile ("cpsie i" : : : "memory");
}

// Disable IRQ
__STATIC_FORCEINLINE void __disable_irq(void) {
    __asm volatile ("cpsid i" : : : "memory");
}

// Wait for interrupt
__STATIC_FORCEINLINE void __WFI(void) {
    __asm volatile ("wfi");
}

// Wait for event
__STATIC_FORCEINLINE void __WFE(void) {
    __asm volatile ("wfe");
}

// Send event
__STATIC_FORCEINLINE void __SEV(void) {
    __asm volatile ("sev");
}

// Instruction Synchronization Barrier
__STATIC_FORCEINLINE void __ISB(void) {
    __asm volatile ("isb 0xF" ::: "memory");
}

// Data Synchronization Barrier
__STATIC_FORCEINLINE void __DSB(void) {
    __asm volatile ("dsb 0xF" ::: "memory");
}

// Data Memory Barrier
__STATIC_FORCEINLINE void __DMB(void) {
    __asm volatile ("dmb 0xF" ::: "memory");
}

// No operation
__STATIC_FORCEINLINE void __NOP(void) {
    __asm volatile ("nop");
}

// Get Main Stack Pointer
__STATIC_FORCEINLINE uint32_t __get_MSP(void) {
    uint32_t result;
    __asm volatile ("MRS %0, msp" : "=r" (result));
    return result;
}

// Set Main Stack Pointer
__STATIC_FORCEINLINE void __set_MSP(uint32_t topOfMainStack) {
    __asm volatile ("MSR msp, %0" : : "r" (topOfMainStack) : );
}

// Get Process Stack Pointer
__STATIC_FORCEINLINE uint32_t __get_PSP(void) {
    uint32_t result;
    __asm volatile ("MRS %0, psp" : "=r" (result));
    return result;
}

// Set Process Stack Pointer
__STATIC_FORCEINLINE void __set_PSP(uint32_t topOfProcStack) {
    __asm volatile ("MSR psp, %0" : : "r" (topOfProcStack) : );
}

// Get PRIMASK
__STATIC_FORCEINLINE uint32_t __get_PRIMASK(void) {
    uint32_t result;
    __asm volatile ("MRS %0, primask" : "=r" (result));
    return result;
}

// Set PRIMASK
__STATIC_FORCEINLINE void __set_PRIMASK(uint32_t priMask) {
    __asm volatile ("MSR primask, %0" : : "r" (priMask) : "memory");
}

// Get BASEPRI
__STATIC_FORCEINLINE uint32_t __get_BASEPRI(void) {
    uint32_t result;
    __asm volatile ("MRS %0, basepri" : "=r" (result));
    return result;
}

// Set BASEPRI
__STATIC_FORCEINLINE void __set_BASEPRI(uint32_t basePri) {
    __asm volatile ("MSR basepri, %0" : : "r" (basePri) : "memory");
}

// Get CONTROL
__STATIC_FORCEINLINE uint32_t __get_CONTROL(void) {
    uint32_t result;
    __asm volatile ("MRS %0, control" : "=r" (result));
    return result;
}

// Set CONTROL
__STATIC_FORCEINLINE void __set_CONTROL(uint32_t control) {
    __asm volatile ("MSR control, %0" : : "r" (control) : "memory");
    __ISB();
}

// ----------------------------------------------------------------------------
// NVIC Functions
// ----------------------------------------------------------------------------

__STATIC_INLINE void NVIC_EnableIRQ(int IRQn) {
    if (IRQn >= 0) {
        NVIC->ISER[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
    }
}

__STATIC_INLINE void NVIC_DisableIRQ(int IRQn) {
    if (IRQn >= 0) {
        NVIC->ICER[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
    }
}

__STATIC_INLINE void NVIC_SetPriority(int IRQn, uint32_t priority) {
    if (IRQn >= 0) {
        NVIC->IP[(uint32_t)IRQn] = (uint8_t)((priority << (8U - NVIC_PRIO_BITS)) & 0xFFUL);
    } else {
        SCB->SHP[(((uint32_t)IRQn) & 0xFUL) - 4UL] = (uint8_t)((priority << (8U - NVIC_PRIO_BITS)) & 0xFFUL);
    }
}

__STATIC_INLINE uint32_t NVIC_GetPriority(int IRQn) {
    if (IRQn >= 0) {
        return ((uint32_t)NVIC->IP[(uint32_t)IRQn] >> (8U - NVIC_PRIO_BITS));
    } else {
        return ((uint32_t)SCB->SHP[(((uint32_t)IRQn) & 0xFUL) - 4UL] >> (8U - NVIC_PRIO_BITS));
    }
}

__STATIC_INLINE void NVIC_SetPendingIRQ(int IRQn) {
    if (IRQn >= 0) {
        NVIC->ISPR[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
    }
}

__STATIC_INLINE void NVIC_ClearPendingIRQ(int IRQn) {
    if (IRQn >= 0) {
        NVIC->ICPR[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
    }
}

__STATIC_INLINE uint32_t NVIC_GetPendingIRQ(int IRQn) {
    if (IRQn >= 0) {
        return ((uint32_t)(((NVIC->ISPR[(((uint32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
    } else {
        return 0U;
    }
}

// System Reset
__NO_RETURN __STATIC_INLINE void NVIC_SystemReset(void) {
    __DSB();
    SCB->AIRCR = ((0x5FAUL << SCB_AIRCR_VECTKEY_Pos) | SCB_AIRCR_SYSRESETREQ_Msk);
    __DSB();
    for(;;) { __NOP(); }
}

// ----------------------------------------------------------------------------
// SysTick Functions
// ----------------------------------------------------------------------------

__STATIC_INLINE uint32_t SysTick_Config(uint32_t ticks) {
    if ((ticks - 1UL) > SysTick_LOAD_RELOAD_Msk) {
        return 1UL;  // Reload value impossible
    }
    SysTick->LOAD = (uint32_t)(ticks - 1UL);
    NVIC_SetPriority(-1, (1UL << NVIC_PRIO_BITS) - 1UL);  // Lowest priority
    SysTick->VAL = 0UL;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
    return 0UL;
}

#ifdef __cplusplus
}
#endif

#endif // __CORE_CM4_H_GENERIC
