// Crazyflie 2.1+ Bring-Up - Expansion Deck Detection Test
//
// Tests the 1-Wire (OW) interface for expansion deck detection.
// OW pin: PC11 (directly connected to expansion deck EEPROM)
//
// 1-Wire Protocol:
// - Reset pulse (480us low, wait for presence)
// - ROM commands (Read ROM, Search ROM, etc.)
// - Memory commands (Read Memory, etc.)
//
// Deck EEPROM format:
// - Byte 0: VID (Vendor ID, 0xBC = Bitcraze)
// - Byte 1: PID (Product ID)
// - Bytes 2+: Key-value pairs (name, revision, etc.)

#include "bringup_deck.h"
#include "bringup_swo.h"
#include "stm32f4xx.h"
#include <string.h>

// 1-Wire pin: PC11
#define OW_PORT GPIOC
#define OW_PIN 11
#define OW_PIN_MASK (1UL << OW_PIN)

// 1-Wire ROM commands
#define OW_CMD_READ_ROM 0x33
#define OW_CMD_SKIP_ROM 0xCC
#define OW_CMD_SEARCH_ROM 0xF0
#define OW_CMD_MATCH_ROM 0x55

// 1-Wire memory commands
#define OW_CMD_READ_MEMORY 0xF0

// Known Bitcraze deck PIDs
#define DECK_VID_BITCRAZE 0xBC
#define DECK_PID_FLOW 0x09
#define DECK_PID_FLOW_V2 0x0A
#define DECK_PID_ZRANGER 0x0C
#define DECK_PID_ZRANGER_V2 0x0F
#define DECK_PID_LOCO 0x06
#define DECK_PID_LED_RING 0x01
#define DECK_PID_SD_CARD 0x08
#define DECK_PID_AI_DECK 0x12
#define DECK_PID_LIGHTHOUSE 0x10

// Timing (approximate at 168 MHz)
static void delay_us(uint32_t us) {
    volatile uint32_t count = us * 42;
    while (count--)
        ;
}

// ----------------------------------------------------------------------------
// 1-Wire Low-Level
// ----------------------------------------------------------------------------

static void ow_pin_output(void) {
    OW_PORT->MODER &= ~(3UL << (OW_PIN * 2));
    OW_PORT->MODER |= (1UL << (OW_PIN * 2)); // Output mode
}

static void ow_pin_input(void) {
    OW_PORT->MODER &= ~(3UL << (OW_PIN * 2)); // Input mode
}

static void ow_pin_low(void) {
    OW_PORT->ODR &= ~OW_PIN_MASK;
}

static void ow_pin_high(void) {
    OW_PORT->ODR |= OW_PIN_MASK;
}

static bool ow_pin_read(void) {
    return (OW_PORT->IDR & OW_PIN_MASK) != 0;
}

// 1-Wire reset - returns true if device(s) present
static bool ow_reset(void) {
    bool presence = false;

    // Drive low for 480us (reset pulse)
    ow_pin_output();
    ow_pin_low();
    delay_us(480);

    // Release and wait 70us
    ow_pin_input();
    delay_us(70);

    // Check for presence pulse (device pulls low)
    presence = !ow_pin_read();

    // Wait for reset to complete
    delay_us(410);

    return presence;
}

// Write a bit
static void ow_write_bit(bool bit) {
    ow_pin_output();
    ow_pin_low();

    if (bit) {
        // Write 1: release within 15us
        delay_us(6);
        ow_pin_input();
        delay_us(64);
    } else {
        // Write 0: hold low for 60us
        delay_us(60);
        ow_pin_input();
        delay_us(10);
    }
}

// Read a bit
static bool ow_read_bit(void) {
    bool bit;

    ow_pin_output();
    ow_pin_low();
    delay_us(6);

    ow_pin_input();
    delay_us(9);

    bit = ow_pin_read();
    delay_us(55);

    return bit;
}

// Write a byte
static void ow_write_byte(uint8_t byte) {
    for (int i = 0; i < 8; i++) {
        ow_write_bit((byte >> i) & 1);
    }
}

// Read a byte
static uint8_t ow_read_byte(void) {
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        if (ow_read_bit()) {
            byte |= (1 << i);
        }
    }
    return byte;
}

// Calculate CRC8 (1-Wire CRC)
static uint8_t ow_crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        uint8_t byte = data[i];
        for (int j = 0; j < 8; j++) {
            uint8_t mix = (crc ^ byte) & 0x01;
            crc >>= 1;
            if (mix) {
                crc ^= 0x8C;
            }
            byte >>= 1;
        }
    }
    return crc;
}

// ----------------------------------------------------------------------------
// Deck Detection
// ----------------------------------------------------------------------------

void deck_test_init(void) {
    // Enable GPIOC clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    // Configure PC11 as open-drain output with pull-up
    OW_PORT->MODER &= ~(3UL << (OW_PIN * 2));
    OW_PORT->OTYPER |= OW_PIN_MASK;            // Open-drain
    OW_PORT->OSPEEDR |= (3UL << (OW_PIN * 2)); // High speed
    OW_PORT->PUPDR &= ~(3UL << (OW_PIN * 2));
    OW_PORT->PUPDR |= (1UL << (OW_PIN * 2)); // Pull-up

    // Start high
    ow_pin_high();
    ow_pin_input();
}

// Read ROM ID (only works if single device on bus)
static bool deck_read_rom(uint8_t *rom_id) {
    if (!ow_reset()) {
        return false;
    }

    ow_write_byte(OW_CMD_READ_ROM);

    for (int i = 0; i < 8; i++) {
        rom_id[i] = ow_read_byte();
    }

    // Verify CRC
    if (ow_crc8(rom_id, 7) != rom_id[7]) {
        return false;
    }

    return true;
}

// Read deck memory (after ROM command)
static bool deck_read_memory(uint16_t addr, uint8_t *data, size_t len) {
    if (!ow_reset()) {
        return false;
    }

    ow_write_byte(OW_CMD_SKIP_ROM);
    ow_write_byte(OW_CMD_READ_MEMORY);
    ow_write_byte((uint8_t)(addr & 0xFF));
    ow_write_byte((uint8_t)(addr >> 8));

    for (size_t i = 0; i < len; i++) {
        data[i] = ow_read_byte();
    }

    return true;
}

const char *deck_get_name(uint8_t vid, uint8_t pid) {
    if (vid != DECK_VID_BITCRAZE) {
        return "Unknown (non-Bitcraze)";
    }

    switch (pid) {
    case DECK_PID_FLOW:
        return "Flow deck";
    case DECK_PID_FLOW_V2:
        return "Flow deck v2";
    case DECK_PID_ZRANGER:
        return "Z-ranger deck";
    case DECK_PID_ZRANGER_V2:
        return "Z-ranger deck v2";
    case DECK_PID_LOCO:
        return "Loco positioning deck";
    case DECK_PID_LED_RING:
        return "LED ring deck";
    case DECK_PID_SD_CARD:
        return "Micro SD card deck";
    case DECK_PID_AI_DECK:
        return "AI deck";
    case DECK_PID_LIGHTHOUSE:
        return "Lighthouse deck";
    default:
        return "Unknown Bitcraze deck";
    }
}

bool deck_run_test(deck_test_results_t *results) {
    memset(results, 0, sizeof(*results));

    swo_puts("\n=== Expansion Deck Detection ===\n");
    swo_puts("[DECK] 1-Wire interface (PC11)\n");

    // Initialize OW
    swo_puts("[DECK] Initializing 1-Wire... ");
    deck_test_init();
    results->ow_init_ok = true;
    swo_puts("OK\n");

    // Try reset to detect presence
    swo_puts("[DECK] Checking for deck presence... ");
    if (!ow_reset()) {
        swo_puts("No deck detected\n");
        swo_puts("[DECK] (This is normal if no expansion deck is attached)\n");
        return true; // Not a failure - just no deck
    }
    swo_puts("Deck detected!\n");

    // Try to read ROM ID (single device)
    swo_puts("[DECK] Reading deck ROM ID... ");
    deck_info_t *deck = &results->decks[0];

    if (deck_read_rom(deck->rom_id)) {
        swo_printf("OK\n");
        swo_printf("[DECK]   ROM ID: %02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X\n",
                   deck->rom_id[0], deck->rom_id[1], deck->rom_id[2],
                   deck->rom_id[3], deck->rom_id[4], deck->rom_id[5],
                   deck->rom_id[6], deck->rom_id[7]);

        // Read deck memory (VID/PID at address 0)
        swo_puts("[DECK] Reading deck identity... ");
        uint8_t mem[8];
        if (deck_read_memory(0, mem, 8)) {
            deck->vid = mem[0];
            deck->pid = mem[1];
            deck->present = true;
            results->deck_count = 1;

            const char *name = deck_get_name(deck->vid, deck->pid);
            strncpy(deck->name, name, sizeof(deck->name) - 1);

            swo_printf("OK\n");
            swo_printf("[DECK]   VID: 0x%02X (%s)\n", deck->vid,
                       deck->vid == DECK_VID_BITCRAZE ? "Bitcraze" : "Unknown");
            swo_printf("[DECK]   PID: 0x%02X\n", deck->pid);
            swo_printf("[DECK]   Name: %s\n", deck->name);
        } else {
            swo_puts("FAIL (read error)\n");
        }
    } else {
        swo_puts("FAIL (CRC error or multiple devices)\n");
        swo_puts("[DECK] Note: Multiple decks require Search ROM algorithm\n");
    }

    if (results->deck_count > 0) {
        swo_printf("[DECK] Detected %d deck(s)\n", results->deck_count);
    }

    return true;
}
