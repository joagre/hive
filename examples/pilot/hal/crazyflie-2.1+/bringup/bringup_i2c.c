// Crazyflie 2.1+ Bring-Up - I2C3 Bus Scan and Communication

#include "bringup_i2c.h"
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

static bool i2c_wait_flag(volatile uint32_t *reg, uint32_t flag, bool set) {
    uint32_t timeout = I2C_TIMEOUT;
    while (timeout--) {
        bool is_set = (*reg & flag) != 0;
        if (is_set == set) {
            return true;
        }
    }
    return false;
}

static bool i2c_check_errors(void) {
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

void i2c_init(void) {
    // Enable clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C3EN;

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

void i2c_recover(void) {
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

bool i2c_probe(uint8_t addr) {
    // Generate start
    I2C3->CR1 |= I2C_CR1_START;
    if (!i2c_wait_flag(&I2C3->SR1, I2C_SR1_SB, true)) {
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

int i2c_scan(uint8_t *found_addrs, int max_addrs) {
    int count = 0;

    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        if (i2c_probe(addr)) {
            if (count < max_addrs) {
                found_addrs[count] = addr;
            }
            count++;
        }
        delay_us(100);
    }

    return count;
}

bool i2c_write(uint8_t addr, const uint8_t *data, size_t len) {
    // Generate start
    I2C3->CR1 |= I2C_CR1_START;
    if (!i2c_wait_flag(&I2C3->SR1, I2C_SR1_SB, true)) {
        goto error;
    }

    // Send address (write)
    I2C3->DR = (addr << 1) | 0;
    if (!i2c_wait_flag(&I2C3->SR1, I2C_SR1_ADDR, true)) {
        goto error;
    }
    (void)I2C3->SR2; // Clear ADDR

    // Send data
    for (size_t i = 0; i < len; i++) {
        if (!i2c_wait_flag(&I2C3->SR1, I2C_SR1_TXE, true)) {
            goto error;
        }
        I2C3->DR = data[i];
    }

    // Wait for transfer complete
    if (!i2c_wait_flag(&I2C3->SR1, I2C_SR1_BTF, true)) {
        goto error;
    }

    I2C3->CR1 |= I2C_CR1_STOP;
    return true;

error:
    if (i2c_check_errors()) {
        i2c_recover();
    }
    I2C3->CR1 |= I2C_CR1_STOP;
    return false;
}

bool i2c_read(uint8_t addr, uint8_t *data, size_t len) {
    if (len == 0) {
        return true;
    }

    // Enable ACK
    I2C3->CR1 |= I2C_CR1_ACK;

    // Generate start
    I2C3->CR1 |= I2C_CR1_START;
    if (!i2c_wait_flag(&I2C3->SR1, I2C_SR1_SB, true)) {
        goto error;
    }

    // Send address (read)
    I2C3->DR = (addr << 1) | 1;
    if (!i2c_wait_flag(&I2C3->SR1, I2C_SR1_ADDR, true)) {
        goto error;
    }

    if (len == 1) {
        // Disable ACK before clearing ADDR
        I2C3->CR1 &= ~I2C_CR1_ACK;
        (void)I2C3->SR2;
        I2C3->CR1 |= I2C_CR1_STOP;
    } else {
        (void)I2C3->SR2;
    }

    // Read data
    for (size_t i = 0; i < len; i++) {
        if (i == len - 2) {
            // Disable ACK before last byte
            I2C3->CR1 &= ~I2C_CR1_ACK;
        }
        if (i == len - 1) {
            I2C3->CR1 |= I2C_CR1_STOP;
        }

        if (!i2c_wait_flag(&I2C3->SR1, I2C_SR1_RXNE, true)) {
            goto error;
        }
        data[i] = (uint8_t)I2C3->DR;
    }

    return true;

error:
    if (i2c_check_errors()) {
        i2c_recover();
    }
    I2C3->CR1 |= I2C_CR1_STOP;
    return false;
}

bool i2c_write_read(uint8_t addr, const uint8_t *write_data, size_t write_len,
                    uint8_t *read_data, size_t read_len) {
    // Generate start
    I2C3->CR1 |= I2C_CR1_START;
    if (!i2c_wait_flag(&I2C3->SR1, I2C_SR1_SB, true)) {
        goto error;
    }

    // Send address (write)
    I2C3->DR = (addr << 1) | 0;
    if (!i2c_wait_flag(&I2C3->SR1, I2C_SR1_ADDR, true)) {
        goto error;
    }
    (void)I2C3->SR2;

    // Send write data
    for (size_t i = 0; i < write_len; i++) {
        if (!i2c_wait_flag(&I2C3->SR1, I2C_SR1_TXE, true)) {
            goto error;
        }
        I2C3->DR = write_data[i];
    }

    if (!i2c_wait_flag(&I2C3->SR1, I2C_SR1_BTF, true)) {
        goto error;
    }

    // Repeated start
    I2C3->CR1 |= I2C_CR1_ACK;
    I2C3->CR1 |= I2C_CR1_START;
    if (!i2c_wait_flag(&I2C3->SR1, I2C_SR1_SB, true)) {
        goto error;
    }

    // Send address (read)
    I2C3->DR = (addr << 1) | 1;
    if (!i2c_wait_flag(&I2C3->SR1, I2C_SR1_ADDR, true)) {
        goto error;
    }

    if (read_len == 1) {
        I2C3->CR1 &= ~I2C_CR1_ACK;
        (void)I2C3->SR2;
        I2C3->CR1 |= I2C_CR1_STOP;
    } else {
        (void)I2C3->SR2;
    }

    // Read data
    for (size_t i = 0; i < read_len; i++) {
        if (i == read_len - 2) {
            I2C3->CR1 &= ~I2C_CR1_ACK;
        }
        if (i == read_len - 1) {
            I2C3->CR1 |= I2C_CR1_STOP;
        }

        if (!i2c_wait_flag(&I2C3->SR1, I2C_SR1_RXNE, true)) {
            goto error;
        }
        read_data[i] = (uint8_t)I2C3->DR;
    }

    return true;

error:
    if (i2c_check_errors()) {
        i2c_recover();
    }
    I2C3->CR1 |= I2C_CR1_STOP;
    return false;
}

bool i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *value) {
    return i2c_write_read(addr, &reg, 1, value, 1);
}

bool i2c_read_regs(uint8_t addr, uint8_t reg, uint8_t *data, size_t len) {
    return i2c_write_read(addr, &reg, 1, data, len);
}

bool i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    return i2c_write(addr, buf, 2);
}

const char *i2c_device_name(uint8_t addr) {
    switch (addr) {
    case I2C_ADDR_BMI088_ACCEL:
        return "BMI088 Accel";
    case I2C_ADDR_BMI088_GYRO:
        return "BMI088 Gyro";
    case I2C_ADDR_BMP388:
        return "BMP388 Baro";
    case I2C_ADDR_VL53L1X:
        return "VL53L1x ToF";
    default:
        return "Unknown";
    }
}
