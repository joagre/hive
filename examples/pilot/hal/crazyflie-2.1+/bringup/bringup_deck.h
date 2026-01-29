// Crazyflie 2.1+ Bring-Up - Expansion Deck Detection Test
//
// The 1-Wire deck EEPROM is on the NRF51, not accessible directly from STM32.
// We use sensor detection as a proxy - detecting deck-specific sensors
// (VL53L1x, PMW3901) confirms the deck is present and working.
//
// Reference: https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/deck_memory_format/

#ifndef BRINGUP_DECK_H
#define BRINGUP_DECK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Maximum number of decks that can be detected
#define MAX_DECKS 4

// Deck information
typedef struct {
    bool present;      // Deck detected
    uint8_t vid;       // Vendor ID (0xBC = Bitcraze)
    uint8_t pid;       // Product ID
    char name[32];     // Deck name
    uint8_t rom_id[8]; // 1-Wire ROM ID (unused in proxy mode)
} deck_info_t;

// Deck test results
typedef struct {
    bool ow_init_ok; // Detection method initialized
    int deck_count;  // Number of decks detected
    deck_info_t decks[MAX_DECKS];
} deck_test_results_t;

// Run deck detection test (uses sensor proxy detection)
// Returns true if detection succeeds (even if no decks found)
bool deck_run_test(deck_test_results_t *results);

// Get known deck name from PID
const char *deck_get_name(uint8_t vid, uint8_t pid);

#endif // BRINGUP_DECK_H
