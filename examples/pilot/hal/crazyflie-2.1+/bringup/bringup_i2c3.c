// Crazyflie 2.1+ Bring-Up - I2C3 Bus Scan and Communication
//
// I2C3: PA8 (SCL), PC9 (SDA) at 400 kHz
// Used for: On-board sensors (BMI088, BMP388)

#include "bringup_i2c3.h"
#include "bringup_swo.h"
#include "stm32f4xx.h"

// I2C3 configuration
// APB1 = 42 MHz, I2C clock = 400 kHz (fast mode)
// CCR = APB1 / (2 * I2C_CLK) = 42000000 / (2 * 400000) = 52.5 -> 53
#define I2C_CCR_VALUE 53
#define I2C_TRISE_VALUE 13 // (42 MHz * 300 ns) + 1 = 13.6 -> 13

// Timeout for I2C operations (in busy-wait iterations)
#define I2C_TIMEOUT 100000

// Simple delay
static void delay_us(uint32_t us) {
    // Approximate delay at 168 MHz
    volatile uint32_t count = us * 42;
    while (count--)
        ;
}

static bool i2c3_wait_flag(volatile uint32_t *reg, uint32_t flag, bool set) {
    uint32_t timeout = I2C_TIMEOUT;
    while (timeout--) {
        bool is_set = (*reg & flag) != 0;
        if (is_set == set) {
            return true;
        }
    }
    return false;
}

static bool i2c3_check_errors(void) {
    uint32_t sr1 = I2C3->SR1;
    if (sr1 & (I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_AF | I2C_SR1_OVR)) {
        // Clear error flags
        I2C3->SR1 = 0;
        // Generate stop
        I2C3->CR1 |= I2C_CR1_STOP;
        return true;
    }
    return false;
}

void i2c3_init(void) {
    // Enable clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C3EN;

    // Bus recovery: toggle SCL 9 times to release any stuck slave
    // Configure PA8 (SCL) as GPIO output first
    GPIOA->MODER &= ~GPIO_MODER_MODER8;
    GPIOA->MODER |= GPIO_MODER_MODER8_0; // Output mode
    GPIOA->OTYPER |= GPIO_OTYPER_OT_8;   // Open-drain
    GPIOA->BSRR = GPIO_BSRR_BS_8;        // Start high
    for (int i = 0; i < 9; i++) {
        GPIOA->BSRR = GPIO_BSRR_BR_8; // SCL low
        for (volatile int d = 0; d < 100; d++)
            ;
        GPIOA->BSRR = GPIO_BSRR_BS_8; // SCL high
        for (volatile int d = 0; d < 100; d++)
            ;
    }

    // Reset I2C3
    RCC->APB1RSTR |= RCC_APB1RSTR_I2C3RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C3RST;

    // Configure PA8 (SCL) as AF4 open-drain
    GPIOA->MODER &= ~GPIO_MODER_MODER8;
    GPIOA->MODER |= GPIO_MODER_MODER8_1;
    GPIOA->OTYPER |= GPIO_OTYPER_OT_8;
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR8;
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR8;
    GPIOA->PUPDR |= GPIO_PUPDR_PUPDR8_0; // Pull-up
    GPIOA->AFR[1] &= ~(0xFU << ((8 - 8) * 4));
    GPIOA->AFR[1] |= (4U << ((8 - 8) * 4)); // AF4 = I2C3

    // Configure PC9 (SDA) as AF4 open-drain
    GPIOC->MODER &= ~GPIO_MODER_MODER9;
    GPIOC->MODER |= GPIO_MODER_MODER9_1;
    GPIOC->OTYPER |= GPIO_OTYPER_OT_9;
    GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR9;
    GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR9;
    GPIOC->PUPDR |= GPIO_PUPDR_PUPDR9_0; // Pull-up
    GPIOC->AFR[1] &= ~(0xFU << ((9 - 8) * 4));
    GPIOC->AFR[1] |= (4U << ((9 - 8) * 4)); // AF4 = I2C3

    // Configure I2C3
    I2C3->CR1 = 0;                          // Disable
    I2C3->CR2 = 42;                         // APB1 frequency in MHz
    I2C3->CCR = I2C_CCR_VALUE | I2C_CCR_FS; // Fast mode
    I2C3->TRISE = I2C_TRISE_VALUE;
    I2C3->CR1 = I2C_CR1_PE; // Enable
}

void i2c3_recover(void) {
    // Disable I2C
    I2C3->CR1 &= ~I2C_CR1_PE;

    // Configure SCL as GPIO output
    GPIOA->MODER &= ~GPIO_MODER_MODER8;
    GPIOA->MODER |= GPIO_MODER_MODER8_0; // Output

    // Toggle SCL 9 times to release stuck slave
    for (int i = 0; i < 9; i++) {
        GPIOA->BSRR = GPIO_BSRR_BR_8; // SCL low
        delay_us(5);
        GPIOA->BSRR = GPIO_BSRR_BS_8; // SCL high
        delay_us(5);
    }

    // Reconfigure as AF
    GPIOA->MODER &= ~GPIO_MODER_MODER8;
    GPIOA->MODER |= GPIO_MODER_MODER8_1;

    // Re-enable I2C
    I2C3->CR1 |= I2C_CR1_PE;
}

bool i2c3_probe(uint8_t addr) {
    // Generate start
    I2C3->CR1 |= I2C_CR1_START;
    if (!i2c3_wait_flag(&I2C3->SR1, I2C_SR1_SB, true)) {
        I2C3->CR1 |= I2C_CR1_STOP;
        return false;
    }

    // Send address (write mode)
    I2C3->DR = (addr << 1) | 0;

    // Wait for address ACK
    uint32_t timeout = I2C_TIMEOUT;
    while (timeout--) {
        uint32_t sr1 = I2C3->SR1;
        if (sr1 & I2C_SR1_AF) {
            // NACK - no device
            I2C3->SR1 = ~I2C_SR1_AF;
            I2C3->CR1 |= I2C_CR1_STOP;
            return false;
        }
        if (sr1 & I2C_SR1_ADDR) {
            // ACK - device present
            (void)I2C3->SR2; // Clear ADDR flag
            I2C3->CR1 |= I2C_CR1_STOP;
            return true;
        }
    }

    I2C3->CR1 |= I2C_CR1_STOP;
    return false;
}

int i2c3_scan(uint8_t *found_addrs, int max_addrs) {
    int count = 0;

    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        if (i2c3_probe(addr)) {
            if (count < max_addrs) {
                found_addrs[count] = addr;
            }
            count++;
        }
        delay_us(100);
    }

    return count;
}

bool i2c3_write(uint8_t addr, const uint8_t *data, size_t len) {
    // Generate start
    I2C3->CR1 |= I2C_CR1_START;
    if (!i2c3_wait_flag(&I2C3->SR1, I2C_SR1_SB, true)) {
        goto error;
    }

    // Send address (write)
    I2C3->DR = (addr << 1) | 0;
    if (!i2c3_wait_flag(&I2C3->SR1, I2C_SR1_ADDR, true)) {
        goto error;
    }
    (void)I2C3->SR2; // Clear ADDR

    // Send data
    for (size_t i = 0; i < len; i++) {
        if (!i2c3_wait_flag(&I2C3->SR1, I2C_SR1_TXE, true)) {
            goto error;
        }
        I2C3->DR = data[i];
    }

    // Wait for transfer complete
    if (!i2c3_wait_flag(&I2C3->SR1, I2C_SR1_BTF, true)) {
        goto error;
    }

    I2C3->CR1 |= I2C_CR1_STOP;
    // Wait for STOP to complete (BUSY flag cleared)
    i2c3_wait_flag(&I2C3->SR2, I2C_SR2_BUSY, false);
    return true;

error:
    if (i2c3_check_errors()) {
        i2c3_recover();
    }
    I2C3->CR1 |= I2C_CR1_STOP;
    i2c3_wait_flag(&I2C3->SR2, I2C_SR2_BUSY, false);
    return false;
}

bool i2c3_read(uint8_t addr, uint8_t *data, size_t len) {
    if (len == 0) {
        return true;
    }

    // Enable ACK
    I2C3->CR1 |= I2C_CR1_ACK;

    // Generate start
    I2C3->CR1 |= I2C_CR1_START;
    if (!i2c3_wait_flag(&I2C3->SR1, I2C_SR1_SB, true)) {
        goto error;
    }

    // Send address (read)
    I2C3->DR = (addr << 1) | 1;
    if (!i2c3_wait_flag(&I2C3->SR1, I2C_SR1_ADDR, true)) {
        goto error;
    }

    if (len == 1) {
        // N=1: Disable ACK before clearing ADDR, then set STOP
        I2C3->CR1 &= ~I2C_CR1_ACK;
        (void)I2C3->SR2; // Clear ADDR
        I2C3->CR1 |= I2C_CR1_STOP;

        if (!i2c3_wait_flag(&I2C3->SR1, I2C_SR1_RXNE, true)) {
            goto error;
        }
        data[0] = (uint8_t)I2C3->DR;
    } else if (len == 2) {
        // N=2: Use POS bit per STM32 reference manual
        // Set POS to shift NACK to second byte
        I2C3->CR1 |= I2C_CR1_POS;
        (void)I2C3->SR2; // Clear ADDR
        I2C3->CR1 &= ~I2C_CR1_ACK;

        // Wait for BTF (both bytes received)
        if (!i2c3_wait_flag(&I2C3->SR1, I2C_SR1_BTF, true)) {
            goto error;
        }

        // Set STOP before reading
        I2C3->CR1 |= I2C_CR1_STOP;
        data[0] = (uint8_t)I2C3->DR;
        data[1] = (uint8_t)I2C3->DR;

        // Clear POS
        I2C3->CR1 &= ~I2C_CR1_POS;
    } else {
        // N>2: Per RM0090 section 27.3.3
        (void)I2C3->SR2; // Clear ADDR

        // Read bytes 0 to N-3 normally
        for (size_t i = 0; i < len - 2; i++) {
            if (!i2c3_wait_flag(&I2C3->SR1, I2C_SR1_RXNE, true)) {
                goto error;
            }
            data[i] = (uint8_t)I2C3->DR;
        }

        // Wait for BTF (byte N-2 in DR, byte N-1 in shift register)
        if (!i2c3_wait_flag(&I2C3->SR1, I2C_SR1_BTF, true)) {
            goto error;
        }
        // Clear ACK, set STOP
        I2C3->CR1 &= ~I2C_CR1_ACK;
        I2C3->CR1 |= I2C_CR1_STOP;
        // Read DataN-2 (shifts DataN-1 to DR)
        data[len - 2] = (uint8_t)I2C3->DR;
        // Wait for DataN-1 to be ready
        if (!i2c3_wait_flag(&I2C3->SR1, I2C_SR1_RXNE, true)) {
            goto error;
        }
        // Read DataN-1
        data[len - 1] = (uint8_t)I2C3->DR;
    }

    // Wait for STOP to complete (BUSY flag cleared)
    i2c3_wait_flag(&I2C3->SR2, I2C_SR2_BUSY, false);
    return true;

error:
    if (i2c3_check_errors()) {
        i2c3_recover();
    }
    I2C3->CR1 &= ~I2C_CR1_POS; // Ensure POS is cleared
    I2C3->CR1 |= I2C_CR1_STOP;
    i2c3_wait_flag(&I2C3->SR2, I2C_SR2_BUSY, false);
    return false;
}

bool i2c3_write_read(uint8_t addr, const uint8_t *write_data, size_t write_len,
                     uint8_t *read_data, size_t read_len) {
    // Generate start
    I2C3->CR1 |= I2C_CR1_START;
    if (!i2c3_wait_flag(&I2C3->SR1, I2C_SR1_SB, true)) {
        goto error;
    }

    // Send address (write)
    I2C3->DR = (addr << 1) | 0;
    if (!i2c3_wait_flag(&I2C3->SR1, I2C_SR1_ADDR, true)) {
        goto error;
    }
    (void)I2C3->SR2;

    // Send write data
    for (size_t i = 0; i < write_len; i++) {
        if (!i2c3_wait_flag(&I2C3->SR1, I2C_SR1_TXE, true)) {
            goto error;
        }
        I2C3->DR = write_data[i];
    }

    if (!i2c3_wait_flag(&I2C3->SR1, I2C_SR1_BTF, true)) {
        goto error;
    }

    // Repeated start
    I2C3->CR1 |= I2C_CR1_ACK;
    I2C3->CR1 |= I2C_CR1_START;
    if (!i2c3_wait_flag(&I2C3->SR1, I2C_SR1_SB, true)) {
        goto error;
    }

    // Send address (read)
    I2C3->DR = (addr << 1) | 1;
    if (!i2c3_wait_flag(&I2C3->SR1, I2C_SR1_ADDR, true)) {
        goto error;
    }

    if (read_len == 1) {
        // N=1: Disable ACK before clearing ADDR, then set STOP
        I2C3->CR1 &= ~I2C_CR1_ACK;
        (void)I2C3->SR2; // Clear ADDR
        I2C3->CR1 |= I2C_CR1_STOP;

        if (!i2c3_wait_flag(&I2C3->SR1, I2C_SR1_RXNE, true)) {
            goto error;
        }
        read_data[0] = (uint8_t)I2C3->DR;
    } else if (read_len == 2) {
        // N=2: Use POS bit per STM32 reference manual
        I2C3->CR1 |= I2C_CR1_POS;
        (void)I2C3->SR2; // Clear ADDR
        I2C3->CR1 &= ~I2C_CR1_ACK;

        // Wait for BTF (both bytes received)
        if (!i2c3_wait_flag(&I2C3->SR1, I2C_SR1_BTF, true)) {
            goto error;
        }

        // Set STOP before reading
        I2C3->CR1 |= I2C_CR1_STOP;
        read_data[0] = (uint8_t)I2C3->DR;
        read_data[1] = (uint8_t)I2C3->DR;

        // Clear POS
        I2C3->CR1 &= ~I2C_CR1_POS;
    } else {
        // N>2: Per RM0090 section 27.3.3
        (void)I2C3->SR2; // Clear ADDR

        // Read bytes 0 to N-3 normally
        for (size_t i = 0; i < read_len - 2; i++) {
            if (!i2c3_wait_flag(&I2C3->SR1, I2C_SR1_RXNE, true)) {
                goto error;
            }
            read_data[i] = (uint8_t)I2C3->DR;
        }

        // Wait for BTF (byte N-2 in DR, byte N-1 in shift register)
        if (!i2c3_wait_flag(&I2C3->SR1, I2C_SR1_BTF, true)) {
            goto error;
        }
        // Clear ACK, set STOP
        I2C3->CR1 &= ~I2C_CR1_ACK;
        I2C3->CR1 |= I2C_CR1_STOP;
        // Read DataN-2 (shifts DataN-1 to DR)
        read_data[read_len - 2] = (uint8_t)I2C3->DR;
        // Wait for DataN-1 to be ready
        if (!i2c3_wait_flag(&I2C3->SR1, I2C_SR1_RXNE, true)) {
            goto error;
        }
        // Read DataN-1
        read_data[read_len - 1] = (uint8_t)I2C3->DR;
    }

    // Wait for STOP bit to be cleared by hardware
    if (!i2c3_wait_flag(&I2C3->CR1, I2C_CR1_STOP, false)) {
        // STOP didn't clear - do full software reset with bus recovery
        I2C3->CR1 = 0; // Disable
        RCC->APB1RSTR |= RCC_APB1RSTR_I2C3RST;
        delay_us(10);
        RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C3RST;

        // Bus recovery: toggle SCL 9 times while I2C is disabled
        GPIOA->MODER &= ~GPIO_MODER_MODER8;
        GPIOA->MODER |= GPIO_MODER_MODER8_0; // Output mode
        GPIOA->BSRR = GPIO_BSRR_BS_8;        // Start high
        for (int i = 0; i < 9; i++) {
            GPIOA->BSRR = GPIO_BSRR_BR_8; // SCL low
            delay_us(5);
            GPIOA->BSRR = GPIO_BSRR_BS_8; // SCL high
            delay_us(5);
        }
        // Reconfigure PA8 as AF
        GPIOA->MODER &= ~GPIO_MODER_MODER8;
        GPIOA->MODER |= GPIO_MODER_MODER8_1;
        // Reconfigure PC9 as AF
        GPIOC->MODER &= ~GPIO_MODER_MODER9;
        GPIOC->MODER |= GPIO_MODER_MODER9_1;

        // Re-configure I2C
        I2C3->CR2 = 42;
        I2C3->CCR = I2C_CCR_VALUE | I2C_CCR_FS;
        I2C3->TRISE = I2C_TRISE_VALUE;
        I2C3->CR1 = I2C_CR1_PE;
    }
    // Wait for BUSY flag cleared
    i2c3_wait_flag(&I2C3->SR2, I2C_SR2_BUSY, false);
    return true;

error:
    if (i2c3_check_errors()) {
        i2c3_recover();
    }
    I2C3->CR1 &= ~I2C_CR1_POS; // Ensure POS is cleared
    I2C3->CR1 |= I2C_CR1_STOP;
    i2c3_wait_flag(&I2C3->CR1, I2C_CR1_STOP, false);
    i2c3_wait_flag(&I2C3->SR2, I2C_SR2_BUSY, false);
    return false;
}

bool i2c3_read_reg(uint8_t addr, uint8_t reg, uint8_t *value) {
    return i2c3_write_read(addr, &reg, 1, value, 1);
}

bool i2c3_read_regs(uint8_t addr, uint8_t reg, uint8_t *data, size_t len) {
    return i2c3_write_read(addr, &reg, 1, data, len);
}

bool i2c3_write_reg(uint8_t addr, uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    return i2c3_write(addr, buf, 2);
}

const char *i2c3_device_name(uint8_t addr) {
    switch (addr) {
    case I2C3_ADDR_BMI088_ACCEL:
        return "BMI088 Accel";
    case I2C3_ADDR_BMI088_GYRO:
        return "BMI088 Gyro";
    case I2C3_ADDR_BMP388:
        return "BMP388 Baro";
    default:
        return "Unknown";
    }
}
