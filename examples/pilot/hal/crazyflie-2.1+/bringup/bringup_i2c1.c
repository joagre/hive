// Crazyflie 2.1+ Bring-Up - I2C1 Bus (Expansion Connector)
//
// I2C1: PB6 (SCL), PB7 (SDA) at 400 kHz
// Used for: EEPROM (0x50), VL53L1x (0x29)

#include "bringup_i2c1.h"
#include "stm32f4xx.h"

// I2C1 configuration
// APB1 = 42 MHz, I2C clock = 400 kHz (fast mode)
#define I2C_CCR_VALUE 53   // 42 MHz / (2 * 400 kHz)
#define I2C_TRISE_VALUE 13 // (42 MHz * 300 ns) + 1
#define I2C_TIMEOUT 100000

static bool s_i2c1_initialized = false;

static void delay_us(uint32_t us) {
    volatile uint32_t count = us * 42;
    while (count--)
        ;
}

static bool i2c1_wait_flag(volatile uint32_t *reg, uint32_t flag, bool set) {
    uint32_t timeout = I2C_TIMEOUT;
    while (timeout--) {
        bool is_set = (*reg & flag) != 0;
        if (is_set == set) {
            return true;
        }
    }
    return false;
}

void i2c1_init(void) {
    if (s_i2c1_initialized) {
        return;
    }

    // Enable clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    // Reset I2C1
    RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;

    // Configure PB6 (SCL) as AF4 open-drain
    GPIOB->MODER &= ~GPIO_MODER_MODER6;
    GPIOB->MODER |= GPIO_MODER_MODER6_1;      // AF mode
    GPIOB->OTYPER |= GPIO_OTYPER_OT_6;        // Open-drain
    GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6; // High speed
    GPIOB->PUPDR &= ~GPIO_PUPDR_PUPDR6;
    GPIOB->PUPDR |= GPIO_PUPDR_PUPDR6_0; // Pull-up
    GPIOB->AFR[0] &= ~(0xFU << (6 * 4));
    GPIOB->AFR[0] |= (4U << (6 * 4)); // AF4 = I2C1

    // Configure PB7 (SDA) as AF4 open-drain
    GPIOB->MODER &= ~GPIO_MODER_MODER7;
    GPIOB->MODER |= GPIO_MODER_MODER7_1;
    GPIOB->OTYPER |= GPIO_OTYPER_OT_7;
    GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR7;
    GPIOB->PUPDR &= ~GPIO_PUPDR_PUPDR7;
    GPIOB->PUPDR |= GPIO_PUPDR_PUPDR7_0;
    GPIOB->AFR[0] &= ~(0xFU << (7 * 4));
    GPIOB->AFR[0] |= (4U << (7 * 4));

    // Configure I2C1
    I2C1->CR1 = 0;                          // Disable
    I2C1->CR2 = 42;                         // APB1 frequency in MHz
    I2C1->CCR = I2C_CCR_VALUE | I2C_CCR_FS; // Fast mode
    I2C1->TRISE = I2C_TRISE_VALUE;
    I2C1->CR1 = I2C_CR1_PE; // Enable

    s_i2c1_initialized = true;
}

bool i2c1_probe(uint8_t addr) {
    if (!s_i2c1_initialized) {
        i2c1_init();
    }

    // Generate start
    I2C1->CR1 |= I2C_CR1_START;
    if (!i2c1_wait_flag(&I2C1->SR1, I2C_SR1_SB, true)) {
        I2C1->CR1 |= I2C_CR1_STOP;
        return false;
    }

    // Send address (write mode)
    I2C1->DR = (addr << 1) | 0;

    // Wait for address ACK
    uint32_t timeout = I2C_TIMEOUT;
    while (timeout--) {
        uint32_t sr1 = I2C1->SR1;
        if (sr1 & I2C_SR1_AF) {
            I2C1->SR1 = ~I2C_SR1_AF;
            I2C1->CR1 |= I2C_CR1_STOP;
            return false;
        }
        if (sr1 & I2C_SR1_ADDR) {
            (void)I2C1->SR2;
            I2C1->CR1 |= I2C_CR1_STOP;
            return true;
        }
    }

    I2C1->CR1 |= I2C_CR1_STOP;
    return false;
}

bool i2c1_write(uint8_t addr, const uint8_t *data, size_t len) {
    if (!s_i2c1_initialized) {
        i2c1_init();
    }

    // Generate start
    I2C1->CR1 |= I2C_CR1_START;
    if (!i2c1_wait_flag(&I2C1->SR1, I2C_SR1_SB, true)) {
        I2C1->CR1 |= I2C_CR1_STOP;
        return false;
    }

    // Send address (write)
    I2C1->DR = (addr << 1) | 0;
    if (!i2c1_wait_flag(&I2C1->SR1, I2C_SR1_ADDR, true)) {
        I2C1->CR1 |= I2C_CR1_STOP;
        return false;
    }
    (void)I2C1->SR2;

    // Send data
    for (size_t i = 0; i < len; i++) {
        if (!i2c1_wait_flag(&I2C1->SR1, I2C_SR1_TXE, true)) {
            I2C1->CR1 |= I2C_CR1_STOP;
            return false;
        }
        I2C1->DR = data[i];
    }

    // Wait for transfer complete
    if (!i2c1_wait_flag(&I2C1->SR1, I2C_SR1_BTF, true)) {
        I2C1->CR1 |= I2C_CR1_STOP;
        return false;
    }

    I2C1->CR1 |= I2C_CR1_STOP;
    return true;
}

bool i2c1_write_read(uint8_t addr, const uint8_t *wdata, size_t wlen,
                     uint8_t *rdata, size_t rlen) {
    if (!s_i2c1_initialized) {
        i2c1_init();
    }

    // Generate start
    I2C1->CR1 |= I2C_CR1_START;
    if (!i2c1_wait_flag(&I2C1->SR1, I2C_SR1_SB, true)) {
        goto error;
    }

    // Send address (write)
    I2C1->DR = (addr << 1) | 0;
    if (!i2c1_wait_flag(&I2C1->SR1, I2C_SR1_ADDR, true)) {
        goto error;
    }
    (void)I2C1->SR2;

    // Send write data
    for (size_t i = 0; i < wlen; i++) {
        if (!i2c1_wait_flag(&I2C1->SR1, I2C_SR1_TXE, true)) {
            goto error;
        }
        I2C1->DR = wdata[i];
    }

    if (!i2c1_wait_flag(&I2C1->SR1, I2C_SR1_BTF, true)) {
        goto error;
    }

    // Repeated start
    I2C1->CR1 |= I2C_CR1_ACK;
    I2C1->CR1 |= I2C_CR1_START;
    if (!i2c1_wait_flag(&I2C1->SR1, I2C_SR1_SB, true)) {
        goto error;
    }

    // Send address (read)
    I2C1->DR = (addr << 1) | 1;
    if (!i2c1_wait_flag(&I2C1->SR1, I2C_SR1_ADDR, true)) {
        goto error;
    }

    if (rlen == 1) {
        // N=1: Disable ACK before clearing ADDR, then set STOP
        I2C1->CR1 &= ~I2C_CR1_ACK;
        (void)I2C1->SR2; // Clear ADDR
        I2C1->CR1 |= I2C_CR1_STOP;

        if (!i2c1_wait_flag(&I2C1->SR1, I2C_SR1_RXNE, true)) {
            goto error;
        }
        rdata[0] = (uint8_t)I2C1->DR;
    } else if (rlen == 2) {
        // N=2: Use POS bit per STM32 reference manual
        I2C1->CR1 |= I2C_CR1_POS;
        (void)I2C1->SR2; // Clear ADDR
        I2C1->CR1 &= ~I2C_CR1_ACK;

        // Wait for BTF (both bytes received)
        if (!i2c1_wait_flag(&I2C1->SR1, I2C_SR1_BTF, true)) {
            goto error;
        }

        // Set STOP before reading
        I2C1->CR1 |= I2C_CR1_STOP;
        rdata[0] = (uint8_t)I2C1->DR;
        rdata[1] = (uint8_t)I2C1->DR;

        // Clear POS
        I2C1->CR1 &= ~I2C_CR1_POS;
    } else {
        // N>2: Per RM0090 section 27.3.3
        (void)I2C1->SR2; // Clear ADDR

        // Read bytes 0 to N-3 normally
        for (size_t i = 0; i < rlen - 2; i++) {
            if (!i2c1_wait_flag(&I2C1->SR1, I2C_SR1_RXNE, true)) {
                goto error;
            }
            rdata[i] = (uint8_t)I2C1->DR;
        }

        // Wait for BTF (byte N-2 in DR, byte N-1 in shift register)
        if (!i2c1_wait_flag(&I2C1->SR1, I2C_SR1_BTF, true)) {
            goto error;
        }
        // Clear ACK, set STOP
        I2C1->CR1 &= ~I2C_CR1_ACK;
        I2C1->CR1 |= I2C_CR1_STOP;
        // Read DataN-2 (shifts DataN-1 to DR)
        rdata[rlen - 2] = (uint8_t)I2C1->DR;
        // Wait for DataN-1 to be ready
        if (!i2c1_wait_flag(&I2C1->SR1, I2C_SR1_RXNE, true)) {
            goto error;
        }
        // Read DataN-1
        rdata[rlen - 1] = (uint8_t)I2C1->DR;
    }

    // Wait for STOP bit to be cleared by hardware
    if (!i2c1_wait_flag(&I2C1->CR1, I2C_CR1_STOP, false)) {
        // STOP didn't clear - do full software reset with bus recovery
        I2C1->CR1 = 0; // Disable
        RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
        delay_us(10);
        RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;

        // Bus recovery: toggle SCL 9 times while I2C is disabled
        GPIOB->MODER &= ~GPIO_MODER_MODER6;
        GPIOB->MODER |= GPIO_MODER_MODER6_0; // Output mode
        GPIOB->BSRR = GPIO_BSRR_BS_6;        // Start high
        for (int i = 0; i < 9; i++) {
            GPIOB->BSRR = GPIO_BSRR_BR_6; // SCL low
            delay_us(5);
            GPIOB->BSRR = GPIO_BSRR_BS_6; // SCL high
            delay_us(5);
        }
        // Reconfigure PB6 as AF
        GPIOB->MODER &= ~GPIO_MODER_MODER6;
        GPIOB->MODER |= GPIO_MODER_MODER6_1;
        // Reconfigure PB7 as AF
        GPIOB->MODER &= ~GPIO_MODER_MODER7;
        GPIOB->MODER |= GPIO_MODER_MODER7_1;

        // Re-configure I2C
        I2C1->CR2 = 42;
        I2C1->CCR = I2C_CCR_VALUE | I2C_CCR_FS;
        I2C1->TRISE = I2C_TRISE_VALUE;
        I2C1->CR1 = I2C_CR1_PE;
    }
    // Wait for BUSY flag cleared
    i2c1_wait_flag(&I2C1->SR2, I2C_SR2_BUSY, false);
    return true;

error:
    I2C1->CR1 &= ~I2C_CR1_POS; // Ensure POS is cleared
    I2C1->CR1 |= I2C_CR1_STOP;
    i2c1_wait_flag(&I2C1->CR1, I2C_CR1_STOP, false);
    i2c1_wait_flag(&I2C1->SR2, I2C_SR2_BUSY, false);
    return false;
}

bool i2c1_read_reg(uint8_t addr, uint8_t reg, uint8_t *value) {
    return i2c1_write_read(addr, &reg, 1, value, 1);
}

bool i2c1_read_reg16(uint8_t addr, uint16_t reg, uint8_t *value) {
    uint8_t reg_buf[2] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF)};
    return i2c1_write_read(addr, reg_buf, 2, value, 1);
}

bool i2c1_read_regs16(uint8_t addr, uint16_t reg, uint8_t *data, size_t len) {
    uint8_t reg_buf[2] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF)};
    return i2c1_write_read(addr, reg_buf, 2, data, len);
}
