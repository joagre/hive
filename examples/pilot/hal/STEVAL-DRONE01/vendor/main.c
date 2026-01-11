/**
 * Sensor test using STM32 HAL + BSP for STEVAL-FCU001V1
 *
 * LED feedback (active low, accent LED accent accent LED on PB5):
 *   1 blink  = Starting
 *   2 blinks = HAL init done
 *   3 blinks = SPI init done
 *   4 blinks = LSM6DSL found (IMU)
 *   5 blinks = LIS2MDL found (Magnetometer)
 *   6 blinks = LPS22HB found (Barometer)
 *   Fast blink = All sensors working
 *   Slow blink = No sensors found
 */

#include "stm32f4xx_hal.h"
#include "steval_fcu001_v1.h"

/* Blink LED n times */
static void blink_n(int n, int on_ms, int off_ms) {
    for (int i = 0; i < n; i++) {
        BSP_LED_On(LED1);
        HAL_Delay(on_ms);
        BSP_LED_Off(LED1);
        HAL_Delay(off_ms);
    }
    HAL_Delay(500);
}

/* Read WHO_AM_I register */
static uint8_t read_who_am_i(SPI_Device_t device, uint8_t reg, uint8_t expected) {
    uint8_t who = 0;
    DrvContextTypeDef ctx = {0};
    ctx.spiDevice = device;
    ctx.ifType = 1;  /* SPI */
    ctx.who_am_i = expected;

    Sensor_IO_SPI_CS_Init(&ctx);
    Sensor_IO_Read(&ctx, reg, &who, 1);

    return who;
}

int main(void) {
    /* Set clock before HAL_Init */
    SystemCoreClock = 16000000;

    /* Initialize HAL */
    HAL_Init();

    /* Initialize LED */
    BSP_LED_Init(LED1);
    BSP_LED_Off(LED1);

    /* 1 blink = Starting */
    blink_n(1, 200, 200);

    /* 2 blinks = HAL init done */
    blink_n(2, 200, 200);

    /* Initialize Sensor SPI */
    Sensor_IO_SPI_Init();
    Sensor_IO_SPI_CS_Init_All();

    /* 3 blinks = SPI init done */
    blink_n(3, 200, 200);

    /* Test LSM6DSL (WHO_AM_I = 0x6A at register 0x0F) */
    uint8_t lsm6dsl_who = read_who_am_i(LSM6DSL, 0x0F, 0x6A);
    if (lsm6dsl_who == 0x6A) {
        blink_n(4, 200, 200);  /* IMU found */
    }

    /* Test LIS2MDL (WHO_AM_I = 0x40 at register 0x4F) */
    uint8_t lis2mdl_who = read_who_am_i(LIS2MDL, 0x4F, 0x40);
    if (lis2mdl_who == 0x40) {
        blink_n(5, 200, 200);  /* Mag found */
    }

    /* Test LPS22HB (WHO_AM_I = 0xB1 at register 0x0F) */
    uint8_t lps22hb_who = read_who_am_i(LPS22HB, 0x0F, 0xB1);
    if (lps22hb_who == 0xB1) {
        blink_n(6, 200, 200);  /* Baro found */
    }

    /* Check if any sensor was found */
    if (lsm6dsl_who != 0x6A && lis2mdl_who != 0x40 && lps22hb_who != 0xB1) {
        /* Slow blink = no sensors */
        while (1) {
            BSP_LED_Toggle(LED1);
            HAL_Delay(1000);
        }
    }

    /* Fast blink = success */
    while (1) {
        BSP_LED_Toggle(LED1);
        HAL_Delay(100);
    }
}

/* Called by HAL_Init */
void HAL_MspInit(void) {
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
}

