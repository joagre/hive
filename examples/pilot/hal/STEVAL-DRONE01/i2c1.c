// I2C1 driver for STM32F401 (STEVAL-DRONE01)
//
// Direct register access implementation for LIS2MDL and LPS22HD communication.
// Uses polling mode for simplicity.

#include "i2c1.h"
#include "system_config.h"
#include "gpio_config.h"

// ----------------------------------------------------------------------------
// I2C1 Register Definitions
// ----------------------------------------------------------------------------

#define I2C1_BASE           0x40005400U

#define I2C1_CR1            (*(volatile uint32_t *)(I2C1_BASE + 0x00))
#define I2C1_CR2            (*(volatile uint32_t *)(I2C1_BASE + 0x04))
#define I2C1_OAR1           (*(volatile uint32_t *)(I2C1_BASE + 0x08))
#define I2C1_OAR2           (*(volatile uint32_t *)(I2C1_BASE + 0x0C))
#define I2C1_DR             (*(volatile uint32_t *)(I2C1_BASE + 0x10))
#define I2C1_SR1            (*(volatile uint32_t *)(I2C1_BASE + 0x14))
#define I2C1_SR2            (*(volatile uint32_t *)(I2C1_BASE + 0x18))
#define I2C1_CCR            (*(volatile uint32_t *)(I2C1_BASE + 0x1C))
#define I2C1_TRISE          (*(volatile uint32_t *)(I2C1_BASE + 0x20))
#define I2C1_FLTR           (*(volatile uint32_t *)(I2C1_BASE + 0x24))

// I2C_CR1 bits
#define I2C_CR1_PE          (1U << 0)   // Peripheral enable
#define I2C_CR1_SMBUS       (1U << 1)   // SMBus mode
#define I2C_CR1_SMBTYPE     (1U << 3)   // SMBus type
#define I2C_CR1_ENARP       (1U << 4)   // ARP enable
#define I2C_CR1_ENPEC       (1U << 5)   // PEC enable
#define I2C_CR1_ENGC        (1U << 6)   // General call enable
#define I2C_CR1_NOSTRETCH   (1U << 7)   // Clock stretching disable
#define I2C_CR1_START       (1U << 8)   // Start generation
#define I2C_CR1_STOP        (1U << 9)   // Stop generation
#define I2C_CR1_ACK         (1U << 10)  // Acknowledge enable
#define I2C_CR1_POS         (1U << 11)  // Acknowledge/PEC position
#define I2C_CR1_PEC         (1U << 12)  // Packet error checking
#define I2C_CR1_ALERT       (1U << 13)  // SMBus alert
#define I2C_CR1_SWRST       (1U << 15)  // Software reset

// I2C_CR2 bits
#define I2C_CR2_FREQ_MASK   (0x3FU)     // Peripheral clock frequency
#define I2C_CR2_ITERREN     (1U << 8)   // Error interrupt enable
#define I2C_CR2_ITEVTEN     (1U << 9)   // Event interrupt enable
#define I2C_CR2_ITBUFEN     (1U << 10)  // Buffer interrupt enable
#define I2C_CR2_DMAEN       (1U << 11)  // DMA requests enable
#define I2C_CR2_LAST        (1U << 12)  // DMA last transfer

// I2C_SR1 bits
#define I2C_SR1_SB          (1U << 0)   // Start bit generated
#define I2C_SR1_ADDR        (1U << 1)   // Address sent/matched
#define I2C_SR1_BTF         (1U << 2)   // Byte transfer finished
#define I2C_SR1_ADD10       (1U << 3)   // 10-bit header sent
#define I2C_SR1_STOPF       (1U << 4)   // Stop detection (slave)
#define I2C_SR1_RXNE        (1U << 6)   // Data register not empty
#define I2C_SR1_TXE         (1U << 7)   // Data register empty
#define I2C_SR1_BERR        (1U << 8)   // Bus error
#define I2C_SR1_ARLO        (1U << 9)   // Arbitration lost
#define I2C_SR1_AF          (1U << 10)  // Acknowledge failure
#define I2C_SR1_OVR         (1U << 11)  // Overrun/underrun
#define I2C_SR1_PECERR      (1U << 12)  // PEC error
#define I2C_SR1_TIMEOUT     (1U << 14)  // Timeout
#define I2C_SR1_SMBALERT    (1U << 15)  // SMBus alert

// I2C_SR2 bits
#define I2C_SR2_MSL         (1U << 0)   // Master/slave
#define I2C_SR2_BUSY        (1U << 1)   // Bus busy
#define I2C_SR2_TRA         (1U << 2)   // Transmitter/receiver
#define I2C_SR2_GENCALL     (1U << 4)   // General call address
#define I2C_SR2_SMBDEFAULT  (1U << 5)   // SMBus device default
#define I2C_SR2_SMBHOST     (1U << 6)   // SMBus host header
#define I2C_SR2_DUALF       (1U << 7)   // Dual flag

// I2C_CCR bits
#define I2C_CCR_CCR_MASK    (0xFFFU)    // Clock control value
#define I2C_CCR_DUTY        (1U << 14)  // Fast mode duty cycle
#define I2C_CCR_FS          (1U << 15)  // Fast mode selection

// ----------------------------------------------------------------------------
// Static variables
// ----------------------------------------------------------------------------

static i2c1_speed_t s_speed;

// ----------------------------------------------------------------------------
// Private functions
// ----------------------------------------------------------------------------

// Wait for flag with timeout
static i2c1_status_t wait_flag(uint32_t flag, bool set) {
    uint32_t start = system_get_tick();

    while (1) {
        bool current = (I2C1_SR1 & flag) != 0;
        if (current == set) {
            return I2C1_OK;
        }

        // Check for errors
        if (I2C1_SR1 & I2C_SR1_AF) {
            I2C1_SR1 &= ~I2C_SR1_AF;  // Clear flag
            return I2C1_ERR_NACK;
        }
        if (I2C1_SR1 & (I2C_SR1_BERR | I2C_SR1_ARLO)) {
            I2C1_SR1 &= ~(I2C_SR1_BERR | I2C_SR1_ARLO);
            return I2C1_ERR_BUS;
        }

        // Timeout check
        if ((system_get_tick() - start) >= I2C1_TIMEOUT_MS) {
            return I2C1_ERR_TIMEOUT;
        }
    }
}

// Wait for busy flag to clear
static i2c1_status_t wait_not_busy(void) {
    uint32_t start = system_get_tick();

    while (I2C1_SR2 & I2C_SR2_BUSY) {
        if ((system_get_tick() - start) >= I2C1_TIMEOUT_MS) {
            return I2C1_ERR_BUSY;
        }
    }
    return I2C1_OK;
}

// Generate start condition
static i2c1_status_t generate_start(void) {
    I2C1_CR1 |= I2C_CR1_START;
    return wait_flag(I2C_SR1_SB, true);
}

// Generate stop condition
static void generate_stop(void) {
    I2C1_CR1 |= I2C_CR1_STOP;
}

// Send 7-bit address
static i2c1_status_t send_addr(uint8_t addr, bool read) {
    // Send address with R/W bit
    I2C1_DR = (addr << 1) | (read ? 1 : 0);

    // Wait for ADDR flag
    i2c1_status_t status = wait_flag(I2C_SR1_ADDR, true);
    if (status != I2C1_OK) {
        return status;
    }

    // Clear ADDR by reading SR1 and SR2
    (void)I2C1_SR1;
    (void)I2C1_SR2;

    return I2C1_OK;
}

// Configure I2C timing
static void configure_timing(i2c1_speed_t speed) {
    // APB1 clock = 42 MHz
    uint32_t pclk1 = 42000000;
    uint32_t freq_mhz = pclk1 / 1000000;

    // Set peripheral clock frequency
    I2C1_CR2 = (I2C1_CR2 & ~I2C_CR2_FREQ_MASK) | freq_mhz;

    if (speed == I2C1_SPEED_100KHZ) {
        // Standard mode: 100 kHz
        // CCR = Tscl_high / Tpclk1 = (5us) / (1/42MHz) = 210
        uint32_t ccr = pclk1 / (100000 * 2);
        I2C1_CCR = ccr & I2C_CCR_CCR_MASK;

        // Rise time: 1000ns max for standard mode
        // TRISE = (Trise / Tpclk1) + 1 = (1us / 24ns) + 1 = 43
        I2C1_TRISE = freq_mhz + 1;
    } else {
        // Fast mode: 400 kHz
        // With DUTY=0: Tlow/Thigh = 2
        // CCR = Tscl_high / Tpclk1 = (0.833us) / (1/42MHz) = 35
        uint32_t ccr = pclk1 / (400000 * 3);  // Duty=0: Thigh = Tlow/2
        I2C1_CCR = I2C_CCR_FS | (ccr & I2C_CCR_CCR_MASK);

        // Rise time: 300ns max for fast mode
        // TRISE = (Trise / Tpclk1) + 1 = (300ns * 42MHz) + 1 = 13
        I2C1_TRISE = (freq_mhz * 300 / 1000) + 1;
    }
}

// ----------------------------------------------------------------------------
// Public API
// ----------------------------------------------------------------------------

void i2c1_init(i2c1_speed_t speed) {
    s_speed = speed;

    // Enable I2C1 clock
    system_enable_i2c1();

    // Initialize GPIO pins (PB6=SCL, PB7=SDA)
    gpio_init_i2c1();

    // Disable I2C before configuration
    I2C1_CR1 &= ~I2C_CR1_PE;

    // Software reset to clear any stuck state
    I2C1_CR1 |= I2C_CR1_SWRST;
    I2C1_CR1 &= ~I2C_CR1_SWRST;

    // Configure timing for selected speed
    configure_timing(speed);

    // Enable I2C peripheral
    I2C1_CR1 |= I2C_CR1_PE;
}

void i2c1_deinit(void) {
    // Disable I2C
    I2C1_CR1 &= ~I2C_CR1_PE;
}

void i2c1_set_speed(i2c1_speed_t speed) {
    s_speed = speed;

    // Disable I2C
    I2C1_CR1 &= ~I2C_CR1_PE;

    // Reconfigure timing
    configure_timing(speed);

    // Re-enable I2C
    I2C1_CR1 |= I2C_CR1_PE;
}

i2c1_status_t i2c1_write(uint8_t addr, const uint8_t *data, uint16_t len) {
    i2c1_status_t status;

    // Wait for bus to be free
    status = wait_not_busy();
    if (status != I2C1_OK) {
        return status;
    }

    // Enable ACK
    I2C1_CR1 |= I2C_CR1_ACK;

    // Generate start
    status = generate_start();
    if (status != I2C1_OK) {
        generate_stop();
        return status;
    }

    // Send address (write mode)
    status = send_addr(addr, false);
    if (status != I2C1_OK) {
        generate_stop();
        return status;
    }

    // Send data bytes
    for (uint16_t i = 0; i < len; i++) {
        // Wait for TXE
        status = wait_flag(I2C_SR1_TXE, true);
        if (status != I2C1_OK) {
            generate_stop();
            return status;
        }

        // Write data
        I2C1_DR = data[i];
    }

    // Wait for BTF (byte transfer finished)
    status = wait_flag(I2C_SR1_BTF, true);
    if (status != I2C1_OK) {
        generate_stop();
        return status;
    }

    // Generate stop
    generate_stop();

    return I2C1_OK;
}

i2c1_status_t i2c1_read(uint8_t addr, uint8_t *data, uint16_t len) {
    i2c1_status_t status;

    if (len == 0) {
        return I2C1_OK;
    }

    // Wait for bus to be free
    status = wait_not_busy();
    if (status != I2C1_OK) {
        return status;
    }

    // Enable ACK
    I2C1_CR1 |= I2C_CR1_ACK;

    // Generate start
    status = generate_start();
    if (status != I2C1_OK) {
        generate_stop();
        return status;
    }

    // Send address (read mode)
    status = send_addr(addr, true);
    if (status != I2C1_OK) {
        generate_stop();
        return status;
    }

    // Read data bytes
    for (uint16_t i = 0; i < len; i++) {
        // Disable ACK before last byte
        if (i == len - 1) {
            I2C1_CR1 &= ~I2C_CR1_ACK;
            generate_stop();
        }

        // Wait for RXNE
        status = wait_flag(I2C_SR1_RXNE, true);
        if (status != I2C1_OK) {
            generate_stop();
            return status;
        }

        // Read data
        data[i] = (uint8_t)I2C1_DR;
    }

    return I2C1_OK;
}

i2c1_status_t i2c1_write_read(uint8_t addr,
                               const uint8_t *tx_data, uint16_t tx_len,
                               uint8_t *rx_data, uint16_t rx_len) {
    i2c1_status_t status;

    // Wait for bus to be free
    status = wait_not_busy();
    if (status != I2C1_OK) {
        return status;
    }

    // Enable ACK
    I2C1_CR1 |= I2C_CR1_ACK;

    // --- Write phase ---

    // Generate start
    status = generate_start();
    if (status != I2C1_OK) {
        generate_stop();
        return status;
    }

    // Send address (write mode)
    status = send_addr(addr, false);
    if (status != I2C1_OK) {
        generate_stop();
        return status;
    }

    // Send data bytes
    for (uint16_t i = 0; i < tx_len; i++) {
        // Wait for TXE
        status = wait_flag(I2C_SR1_TXE, true);
        if (status != I2C1_OK) {
            generate_stop();
            return status;
        }

        // Write data
        I2C1_DR = tx_data[i];
    }

    // Wait for BTF
    status = wait_flag(I2C_SR1_BTF, true);
    if (status != I2C1_OK) {
        generate_stop();
        return status;
    }

    // --- Read phase (repeated start) ---

    if (rx_len == 0) {
        generate_stop();
        return I2C1_OK;
    }

    // Generate repeated start
    status = generate_start();
    if (status != I2C1_OK) {
        generate_stop();
        return status;
    }

    // Send address (read mode)
    status = send_addr(addr, true);
    if (status != I2C1_OK) {
        generate_stop();
        return status;
    }

    // Read data bytes
    for (uint16_t i = 0; i < rx_len; i++) {
        // Disable ACK before last byte
        if (i == rx_len - 1) {
            I2C1_CR1 &= ~I2C_CR1_ACK;
            generate_stop();
        }

        // Wait for RXNE
        status = wait_flag(I2C_SR1_RXNE, true);
        if (status != I2C1_OK) {
            generate_stop();
            return status;
        }

        // Read data
        rx_data[i] = (uint8_t)I2C1_DR;
    }

    return I2C1_OK;
}

i2c1_status_t i2c1_write_reg(uint8_t addr, uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    return i2c1_write(addr, data, 2);
}

i2c1_status_t i2c1_read_reg(uint8_t addr, uint8_t reg, uint8_t *value) {
    return i2c1_write_read(addr, &reg, 1, value, 1);
}

i2c1_status_t i2c1_read_regs(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len) {
    return i2c1_write_read(addr, &reg, 1, data, len);
}

bool i2c1_probe(uint8_t addr) {
    i2c1_status_t status;

    // Wait for bus to be free
    status = wait_not_busy();
    if (status != I2C1_OK) {
        return false;
    }

    // Generate start
    status = generate_start();
    if (status != I2C1_OK) {
        generate_stop();
        return false;
    }

    // Send address (write mode)
    status = send_addr(addr, false);
    generate_stop();

    return (status == I2C1_OK);
}

bool i2c1_is_busy(void) {
    return (I2C1_SR2 & I2C_SR2_BUSY) != 0;
}

void i2c1_reset(void) {
    // Disable I2C
    I2C1_CR1 &= ~I2C_CR1_PE;

    // Toggle GPIO pins to release stuck SDA
    gpio_i2c1_release_bus();

    // Software reset
    I2C1_CR1 |= I2C_CR1_SWRST;
    I2C1_CR1 &= ~I2C_CR1_SWRST;

    // Reconfigure and enable
    configure_timing(s_speed);
    I2C1_CR1 |= I2C_CR1_PE;
}
