// Crazyflie 2.1+ Bring-Up - Expansion Deck Detection Test
//
// Tests the 1-Wire (OW) interface for expansion deck detection.
// Each Crazyflie deck contains a 1-Wire EEPROM (DS28E05) with:
// - Vendor ID (VID)
// - Product ID (PID)
// - Deck name
// - Pin configuration
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
    char name[32];     // Deck name (from EEPROM)
    uint8_t rom_id[8]; // 1-Wire ROM ID
} deck_info_t;

// Deck test results
typedef struct {
    bool ow_init_ok; // 1-Wire interface initialized
    int deck_count;  // Number of decks detected
    deck_info_t decks[MAX_DECKS];
} deck_test_results_t;

// Initialize deck detection (configures OW GPIO)
void deck_test_init(void);

// Run deck detection test
// Returns true if OW interface works (even if no decks found)
bool deck_run_test(deck_test_results_t *results);

// Get known deck name from PID
const char *deck_get_name(uint8_t vid, uint8_t pid);

#endif // BRINGUP_DECK_H
