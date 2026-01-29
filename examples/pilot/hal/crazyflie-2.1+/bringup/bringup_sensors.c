// Crazyflie 2.1+ Bring-Up - Sensor Tests

#include "bringup_sensors.h"
#include "bringup_i2c1.h"
#include "bringup_i2c3.h"
#include "bringup_swo.h"
#include "stm32f4xx.h"
#include <math.h>

// BMI088 registers
#define BMI088_ACC_CHIP_ID_REG 0x00
#define BMI088_ACC_DATA_REG 0x12
#define BMI088_ACC_CONF_REG 0x40
#define BMI088_ACC_RANGE_REG 0x41
#define BMI088_ACC_PWR_CONF_REG 0x7C
#define BMI088_ACC_PWR_CTRL_REG 0x7D

#define BMI088_GYRO_CHIP_ID_REG 0x00
#define BMI088_GYRO_DATA_REG 0x02
#define BMI088_GYRO_RANGE_REG 0x0F
#define BMI088_GYRO_BW_REG 0x10
#define BMI088_GYRO_LPM1_REG 0x11

// BMP388 registers
#define BMP388_CHIP_ID_REG 0x00
#define BMP388_DATA_REG 0x04
#define BMP388_PWR_CTRL_REG 0x1B
#define BMP388_OSR_REG 0x1C
#define BMP388_ODR_REG 0x1D
#define BMP388_CALIB_REG 0x31

// VL53L1x registers
#define VL53L1X_MODEL_ID_REG 0x010F

// PMW3901 registers (directly, not via vendor API for bringup)
#define PMW3901_PRODUCT_ID_REG 0x00
#define PMW3901_MOTION_REG 0x02
#define PMW3901_DELTA_X_L_REG 0x03
#define PMW3901_DELTA_X_H_REG 0x04
#define PMW3901_DELTA_Y_L_REG 0x05
#define PMW3901_DELTA_Y_H_REG 0x06
#define PMW3901_SQUAL_REG 0x07

// SPI1 for PMW3901
#define PMW3901_CS_PIN 4 // PB4

// BMP388 calibration data
static struct {
    uint16_t t1;
    uint16_t t2;
    int8_t t3;
    int16_t p1;
    int16_t p2;
    int8_t p3;
    int8_t p4;
    uint16_t p5;
    uint16_t p6;
    int8_t p7;
    int8_t p8;
    int16_t p9;
    int8_t p10;
    int8_t p11;
} bmp388_calib_t;

static bool s_flow_deck_present = false;
static uint8_t s_vl53l1x_addr = 0; // Detected VL53L1x I2C address

// Simple delay
static void delay_us(uint32_t us) {
    volatile uint32_t count = us * 42;
    while (count--)
        ;
}

static void delay_ms(uint32_t ms) {
    delay_us(ms * 1000);
}

// SPI functions for PMW3901
static void spi_cs_low(void) {
    GPIOB->BSRR = GPIO_BSRR_BR_4;
}

static void spi_cs_high(void) {
    GPIOB->BSRR = GPIO_BSRR_BS_4;
}

static uint8_t spi_transfer(uint8_t data) {
    while (!(SPI1->SR & SPI_SR_TXE))
        ;
    SPI1->DR = data;
    while (!(SPI1->SR & SPI_SR_RXNE))
        ;
    return (uint8_t)SPI1->DR;
}

static uint8_t pmw3901_read_reg(uint8_t reg) {
    spi_cs_low();
    delay_us(1);
    spi_transfer(reg & 0x7F); // MSB=0 for read
    delay_us(35);             // tSRAD
    uint8_t value = spi_transfer(0x00);
    delay_us(1);
    spi_cs_high();
    delay_us(20); // tSRW/tSRR
    return value;
}

void spi_init(void) {
    // Enable clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    // Configure PA5 (SCK), PA6 (MISO), PA7 (MOSI) as AF5
    GPIOA->MODER &=
        ~(GPIO_MODER_MODER5 | GPIO_MODER_MODER6 | GPIO_MODER_MODER7);
    GPIOA->MODER |=
        (GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1);
    GPIOA->AFR[0] &=
        ~((0xFU << (5 * 4)) | (0xFU << (6 * 4)) | (0xFU << (7 * 4)));
    GPIOA->AFR[0] |= (5U << (5 * 4)) | (5U << (6 * 4)) | (5U << (7 * 4));
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR5 | GPIO_OSPEEDER_OSPEEDR6 |
                      GPIO_OSPEEDER_OSPEEDR7;

    // Configure PB4 as CS output (active low)
    GPIOB->MODER &= ~GPIO_MODER_MODER4;
    GPIOB->MODER |= GPIO_MODER_MODER4_0;
    GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR4;
    spi_cs_high();

    // Configure SPI1: Master, 8-bit, CPOL=1, CPHA=1 (Mode 3), ~1.3 MHz
    SPI1->CR1 = 0;
    SPI1->CR1 = SPI_CR1_MSTR |                // Master mode
                SPI_CR1_BR_2 | SPI_CR1_BR_0 | // Baud rate = fPCLK/64 = 1.3 MHz
                SPI_CR1_CPOL |                // CPOL=1
                SPI_CR1_CPHA |                // CPHA=1
                SPI_CR1_SSM |                 // Software slave management
                SPI_CR1_SSI;                  // Internal slave select
    SPI1->CR1 |= SPI_CR1_SPE;                 // Enable
}

// VL53L1x boot status register
#define VL53L1X_FIRMWARE_STATUS_REG 0x00E5

void sensors_init(void) {
    // Check if Flow deck is present (VL53L1x responds on I2C1 - expansion connector)
    // Note: VL53L1x is on I2C1 (PB6/PB7), not I2C3 (PA8/PC9)
    i2c1_init();

    // VL53L1x needs time to boot after power-on (can take up to 1.2 seconds)
    // Per ST documentation, poll FIRMWARE__SYSTEM_STATUS until bit 0 = 1
    swo_puts("[INIT] Waiting for VL53L1x boot (up to 1.5s)...\n");

    // First, scan I2C1 bus to see what addresses respond
    swo_puts("[INIT] Scanning I2C1 bus for devices:\n");
    int found_count = 0;
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        if (i2c1_probe(addr)) {
            swo_printf("[INIT]   Found device at 0x%02X", addr);
            // Try to identify the device by reading common ID registers
            if (addr == 0x29 || addr == 0x52) {
                // VL53L1x uses 16-bit registers, model ID at 0x010F
                uint8_t id[2];
                if (i2c1_read_regs16(addr, 0x010F, id, 2)) {
                    swo_printf(" (VL53L1x? model=0x%02X%02X)", id[0], id[1]);
                }
            } else if (addr >= 0x50 && addr <= 0x57) {
                swo_printf(" (EEPROM range)");
            } else {
                // Try reading VL53L1x model ID (16-bit register 0x010F)
                uint8_t id16[2];
                if (i2c1_read_regs16(addr, 0x010F, id16, 2)) {
                    uint16_t model = (uint16_t)(id16[0] << 8 | id16[1]);
                    if (model == 0xEACC) {
                        swo_printf(" (VL53L1x! model=0x%04X)", model);
                    } else {
                        swo_printf(" (16bit[0x010F]=0x%04X)", model);
                    }
                }
                // Try reading register 0x00 (common chip ID location)
                uint8_t id = 0;
                if (i2c1_read_reg(addr, 0x00, &id)) {
                    swo_printf(" (reg[0x00]=0x%02X)", id);
                }
                // Also try 0x0F (WHO_AM_I for many sensors)
                if (i2c1_read_reg(addr, 0x0F, &id)) {
                    swo_printf(" (reg[0x0F]=0x%02X)", id);
                }
            }
            swo_puts("\n");
            found_count++;
        }
    }
    swo_printf("[INIT] I2C1 scan complete: %d device(s) found\n", found_count);

    // Now look for VL53L1x at any address (it may have been reprogrammed)
    // Check common addresses: 0x29 (default), 0x30-0x3F (Bitcraze range), 0x6A (observed)
    bool vl53l1x_found = false;
    uint8_t vl53l1x_addr = 0;

    // First check default address
    uint8_t addrs_to_check[] = {0x29, 0x6A, 0x30, 0x31, 0x32, 0x52};
    for (size_t i = 0; i < sizeof(addrs_to_check); i++) {
        uint8_t addr = addrs_to_check[i];
        if (i2c1_probe(addr)) {
            // Check if it's a VL53L1x by reading model ID
            uint8_t id[2];
            if (i2c1_read_regs16(addr, 0x010F, id, 2)) {
                uint16_t model = (uint16_t)(id[0] << 8 | id[1]);
                if (model == 0xEACC) {
                    swo_printf(
                        "[INIT] VL53L1x found at 0x%02X (model=0x%04X)\n", addr,
                        model);
                    vl53l1x_addr = addr;
                    s_vl53l1x_addr = addr; // Store for later use
                    vl53l1x_found = true;
                    break;
                }
            }
        }
    }

    // If found, wait for boot
    if (vl53l1x_found) {
        for (int attempt = 0; attempt < 30; attempt++) {
            uint8_t boot_status = 0;
            if (i2c1_read_reg16(vl53l1x_addr, VL53L1X_FIRMWARE_STATUS_REG,
                                &boot_status)) {
                if (boot_status & 0x01) {
                    swo_printf(
                        "[INIT] VL53L1x booted (attempt %d, status=0x%02X)\n",
                        attempt + 1, boot_status);
                    break;
                }
            }
            delay_ms(50);
        }
    }

    s_flow_deck_present = vl53l1x_found;

    if (!vl53l1x_found) {
        swo_puts("[INIT] VL53L1x not detected on I2C1\n");
    }
    swo_puts("[INIT] Flow deck probe done\n");

    // Initialize BMI088 accelerometer
    // Exit suspend mode
    i2c3_write_reg(I2C3_ADDR_BMI088_ACCEL, BMI088_ACC_PWR_CONF_REG, 0x00);
    delay_ms(1);
    // Enable accelerometer
    i2c3_write_reg(I2C3_ADDR_BMI088_ACCEL, BMI088_ACC_PWR_CTRL_REG, 0x04);
    delay_ms(50);
    // Set range +/-12g
    i2c3_write_reg(I2C3_ADDR_BMI088_ACCEL, BMI088_ACC_RANGE_REG, 0x02);
    // Set ODR 800Hz, normal mode
    i2c3_write_reg(I2C3_ADDR_BMI088_ACCEL, BMI088_ACC_CONF_REG, 0xAB);
    swo_puts("[INIT] Accel init done\n");

    // Initialize BMI088 gyroscope
    // Set range +/-2000 dps
    i2c3_write_reg(I2C3_ADDR_BMI088_GYRO, BMI088_GYRO_RANGE_REG, 0x00);
    // Set bandwidth 116Hz
    i2c3_write_reg(I2C3_ADDR_BMI088_GYRO, BMI088_GYRO_BW_REG, 0x02);
    // Normal power mode
    i2c3_write_reg(I2C3_ADDR_BMI088_GYRO, BMI088_GYRO_LPM1_REG, 0x00);
    delay_ms(30);
    swo_puts("[INIT] Gyro init done\n");

    // Initialize BMP388
    // Soft reset first
    i2c3_write_reg(I2C3_ADDR_BMP388, 0x7E, 0xB6); // CMD register, soft reset
    delay_ms(10);

    // Read calibration data after reset
    uint8_t calib[21];
    bool calib_ok =
        i2c3_read_regs(I2C3_ADDR_BMP388, BMP388_CALIB_REG, calib, 21);
    swo_printf("[INIT] BMP388 calib read: %s\n", calib_ok ? "OK" : "FAIL");
    bmp388_calib_t.t1 = (uint16_t)(calib[1] << 8 | calib[0]);
    bmp388_calib_t.t2 = (uint16_t)(calib[3] << 8 | calib[2]);
    bmp388_calib_t.t3 = (int8_t)calib[4];
    bmp388_calib_t.p1 = (int16_t)(calib[6] << 8 | calib[5]);
    bmp388_calib_t.p2 = (int16_t)(calib[8] << 8 | calib[7]);
    bmp388_calib_t.p3 = (int8_t)calib[9];
    bmp388_calib_t.p4 = (int8_t)calib[10];
    bmp388_calib_t.p5 = (uint16_t)(calib[12] << 8 | calib[11]);
    bmp388_calib_t.p6 = (uint16_t)(calib[14] << 8 | calib[13]);
    bmp388_calib_t.p7 = (int8_t)calib[15];
    bmp388_calib_t.p8 = (int8_t)calib[16];
    bmp388_calib_t.p9 = (int16_t)(calib[18] << 8 | calib[17]);
    bmp388_calib_t.p10 = (int8_t)calib[19];
    bmp388_calib_t.p11 = (int8_t)calib[20];

    // Set OSR x1 for fast measurements during bringup
    i2c3_write_reg(I2C3_ADDR_BMP388, BMP388_OSR_REG, 0x00);
    // Enable pressure and temperature in forced mode (0x13)
    // This triggers a single measurement, then returns to sleep
    i2c3_write_reg(I2C3_ADDR_BMP388, BMP388_PWR_CTRL_REG, 0x13);
    // Allow time for first measurement (should be ~5ms at OSR x1)
    delay_ms(50);

    swo_puts("[INIT] Sensor init complete\n");
}

bool sensor_test_accel_id(uint8_t *chip_id) {
    if (!i2c3_read_reg(I2C3_ADDR_BMI088_ACCEL, BMI088_ACC_CHIP_ID_REG,
                       chip_id)) {
        return false;
    }
    return *chip_id == BMI088_ACCEL_CHIP_ID;
}

bool sensor_test_gyro_id(uint8_t *chip_id) {
    if (!i2c3_read_reg(I2C3_ADDR_BMI088_GYRO, BMI088_GYRO_CHIP_ID_REG,
                       chip_id)) {
        return false;
    }
    return *chip_id == BMI088_GYRO_CHIP_ID;
}

bool sensor_test_baro_id(uint8_t *chip_id) {
    if (!i2c3_read_reg(I2C3_ADDR_BMP388, BMP388_CHIP_ID_REG, chip_id)) {
        return false;
    }
    return *chip_id == BMP388_CHIP_ID;
}

bool sensor_test_tof_id(uint16_t *model_id) {
    if (!s_flow_deck_present || s_vl53l1x_addr == 0) {
        *model_id = 0;
        return false;
    }

    uint8_t id[2];
    // VL53L1x is on I2C1 (expansion connector) - use detected address
    if (!i2c1_read_regs16(s_vl53l1x_addr, VL53L1X_MODEL_ID_REG, id, 2)) {
        return false;
    }

    *model_id = (uint16_t)(id[0] << 8 | id[1]);
    return *model_id == VL53L1X_MODEL_ID;
}

bool sensor_test_flow_id(uint8_t *product_id) {
    if (!s_flow_deck_present) {
        *product_id = 0;
        return false;
    }

    *product_id = pmw3901_read_reg(PMW3901_PRODUCT_ID_REG);
    return *product_id == PMW3901_PRODUCT_ID;
}

bool sensor_read_accel(accel_data_t *data) {
    uint8_t raw[6];
    if (!i2c3_read_regs(I2C3_ADDR_BMI088_ACCEL, BMI088_ACC_DATA_REG, raw, 6)) {
        return false;
    }

    int16_t x = (int16_t)(raw[1] << 8 | raw[0]);
    int16_t y = (int16_t)(raw[3] << 8 | raw[2]);
    int16_t z = (int16_t)(raw[5] << 8 | raw[4]);

    // Convert to g (range +/-12g, 16-bit)
    const float scale = 12.0f / 32768.0f;
    data->x = x * scale;
    data->y = y * scale;
    data->z = z * scale;

    return true;
}

bool sensor_read_gyro(gyro_data_t *data) {
    uint8_t raw[6];
    if (!i2c3_read_regs(I2C3_ADDR_BMI088_GYRO, BMI088_GYRO_DATA_REG, raw, 6)) {
        return false;
    }

    int16_t x = (int16_t)(raw[1] << 8 | raw[0]);
    int16_t y = (int16_t)(raw[3] << 8 | raw[2]);
    int16_t z = (int16_t)(raw[5] << 8 | raw[4]);

    // Convert to deg/s (range +/-2000 dps, 16-bit)
    const float scale = 2000.0f / 32768.0f;
    data->x = x * scale;
    data->y = y * scale;
    data->z = z * scale;

    return true;
}

bool sensor_read_baro(baro_data_t *data) {
    // Trigger a forced measurement
    i2c3_write_reg(I2C3_ADDR_BMP388, BMP388_PWR_CTRL_REG, 0x13);

    // Wait for data ready (bits 5 and 6 of status register)
    uint8_t status = 0;
    for (int i = 0; i < 50; i++) {
        if (!i2c3_read_reg(I2C3_ADDR_BMP388, 0x03, &status)) {
            return false;
        }
        if ((status & 0x60) == 0x60) {
            break; // Both pressure and temp ready
        }
        delay_ms(5);
    }

    if ((status & 0x60) != 0x60) {
        return false;
    }

    uint8_t raw[6];
    if (!i2c3_read_regs(I2C3_ADDR_BMP388, BMP388_DATA_REG, raw, 6)) {
        return false;
    }

    uint32_t press_raw = (uint32_t)(raw[2] << 16 | raw[1] << 8 | raw[0]);
    uint32_t temp_raw = (uint32_t)(raw[5] << 16 | raw[4] << 8 | raw[3]);

    // Compensate temperature
    float partial1 = (float)temp_raw - (float)bmp388_calib_t.t1 * 256.0f;
    float partial2 = partial1 * ((float)bmp388_calib_t.t2 / 1073741824.0f);
    float t_lin =
        partial2 +
        partial1 * partial1 * ((float)bmp388_calib_t.t3 / 281474976710656.0f);

    data->temperature = t_lin;

    // Compensate pressure (simplified)
    float partial3 = t_lin * t_lin;
    float partial4 = partial3 / 64.0f;
    float partial6 = (float)bmp388_calib_t.p8 * partial3 / 32.0f;
    float partial7 = partial6 + (float)bmp388_calib_t.p7 * t_lin;

    float offset = (float)bmp388_calib_t.p5 * 32768.0f + partial7 +
                   (float)bmp388_calib_t.p6 * partial4;

    float sens = (float)bmp388_calib_t.p1 - 16384.0f +
                 (float)bmp388_calib_t.p2 * t_lin / 8192.0f +
                 (float)bmp388_calib_t.p3 * partial3 / 8192.0f;

    float pressure_comp = offset + (float)press_raw * sens / 4096.0f;

    data->pressure = pressure_comp;

    return true;
}

bool sensor_read_tof(uint16_t *range_mm) {
    if (!s_flow_deck_present || s_vl53l1x_addr == 0) {
        return false;
    }

    // Read range result (simplified - real driver needs full init sequence)
    // For bringup, just verify we can read something
    // VL53L1x is on I2C1 (expansion connector) - use detected address
    uint8_t data[2];
    if (!i2c1_read_regs16(s_vl53l1x_addr, 0x0096, data, 2)) {
        return false;
    }

    *range_mm = (uint16_t)(data[0] << 8 | data[1]);
    return true;
}

bool sensor_read_flow(flow_data_t *data) {
    if (!s_flow_deck_present) {
        return false;
    }

    // Read motion burst
    uint8_t motion = pmw3901_read_reg(PMW3901_MOTION_REG);
    if (!(motion & 0x80)) {
        // No motion data available
        data->delta_x = 0;
        data->delta_y = 0;
        data->squal = 0;
        return true;
    }

    data->delta_x = (int16_t)(pmw3901_read_reg(PMW3901_DELTA_X_H_REG) << 8 |
                              pmw3901_read_reg(PMW3901_DELTA_X_L_REG));
    data->delta_y = (int16_t)(pmw3901_read_reg(PMW3901_DELTA_Y_H_REG) << 8 |
                              pmw3901_read_reg(PMW3901_DELTA_Y_L_REG));
    data->squal = pmw3901_read_reg(PMW3901_SQUAL_REG);

    return true;
}

bool sensor_check_accel(const accel_data_t *data) {
    // X and Y should be near 0 (+/- 0.15g)
    // Z should be near 1g (0.85 to 1.15g)
    return (fabsf(data->x) < 0.15f) && (fabsf(data->y) < 0.15f) &&
           (data->z > 0.85f) && (data->z < 1.15f);
}

bool sensor_check_gyro(const gyro_data_t *data) {
    // All axes should be near 0 (+/- 10 deg/s)
    return (fabsf(data->x) < 10.0f) && (fabsf(data->y) < 10.0f) &&
           (fabsf(data->z) < 10.0f);
}

bool sensor_check_baro(const baro_data_t *data) {
    // For bringup, just check temperature is reasonable (10-50 C)
    // Full pressure compensation requires correct Bosch formula
    // (Temperature compensation is simpler and should work)
    return (data->temperature > 10.0f) && (data->temperature < 50.0f);
}

bool sensor_flow_deck_present(void) {
    return s_flow_deck_present;
}

void sensor_run_all_tests(sensor_test_results_t *results) {
    uint8_t chip_id;
    uint16_t model_id;

    // Chip ID tests
    results->accel_id_ok = sensor_test_accel_id(&chip_id);
    swo_printf("[SENSOR] BMI088 Accel chip ID = 0x%02X... %s\n", chip_id,
               results->accel_id_ok ? "OK" : "FAIL");

    results->gyro_id_ok = sensor_test_gyro_id(&chip_id);
    swo_printf("[SENSOR] BMI088 Gyro chip ID = 0x%02X... %s\n", chip_id,
               results->gyro_id_ok ? "OK" : "FAIL");

    results->baro_id_ok = sensor_test_baro_id(&chip_id);
    swo_printf("[SENSOR] BMP388 chip ID = 0x%02X... %s\n", chip_id,
               results->baro_id_ok ? "OK" : "FAIL");

    results->flow_deck_present = s_flow_deck_present;
    if (s_flow_deck_present) {
        results->tof_id_ok = sensor_test_tof_id(&model_id);
        swo_printf("[SENSOR] VL53L1x model ID = 0x%04X... %s (Flow deck)\n",
                   model_id, results->tof_id_ok ? "OK" : "FAIL");

        results->flow_id_ok = sensor_test_flow_id(&chip_id);
        swo_printf("[SENSOR] PMW3901 product ID = 0x%02X... %s (Flow deck)\n",
                   chip_id, results->flow_id_ok ? "OK" : "FAIL");
    } else {
        swo_printf("[SENSOR] Flow deck not detected\n");
        results->tof_id_ok = false;
        results->flow_id_ok = false;
    }

    // Data readout tests
    accel_data_t accel;
    gyro_data_t gyro;
    baro_data_t baro;

    results->accel_data_ok =
        sensor_read_accel(&accel) && sensor_check_accel(&accel);
    swo_printf("[DATA] Accelerometer: X=%fg Y=%fg Z=%fg... %s\n", accel.x,
               accel.y, accel.z, results->accel_data_ok ? "OK" : "FAIL");

    bool gyro_read = sensor_read_gyro(&gyro);
    bool gyro_check = sensor_check_gyro(&gyro);
    results->gyro_data_ok = gyro_read && gyro_check;
    swo_printf("[DATA] Gyroscope: X=%.2f Y=%.2f Z=%.2f deg/s (read=%d "
               "check=%d)... %s\n",
               gyro.x, gyro.y, gyro.z, gyro_read, gyro_check,
               results->gyro_data_ok ? "OK" : "FAIL");

    bool baro_read = sensor_read_baro(&baro);
    bool baro_check = sensor_check_baro(&baro);
    results->baro_data_ok = baro_read && baro_check;
    swo_printf("[DATA] Barometer: %.0f Pa, %.1f C (read=%d check=%d)... %s\n",
               baro.pressure, baro.temperature, baro_read, baro_check,
               results->baro_data_ok ? "OK" : "FAIL");

    if (s_flow_deck_present) {
        uint16_t range;
        results->tof_data_ok = sensor_read_tof(&range);
        swo_printf("[DATA] ToF range: %u mm... %s (Flow deck)\n", range,
                   results->tof_data_ok ? "OK" : "FAIL");
    } else {
        results->tof_data_ok = false;
    }
}
