// Crazyflie 2.1+ Bring-Up - Syslink Test
//
// USART6 at 1Mbaud to nRF51822
//   PC6 - TX
//   PC7 - RX
//   PA4 - TXEN (flow control)

#ifndef BRINGUP_SYSLINK_H
#define BRINGUP_SYSLINK_H

#include <stdbool.h>
#include <stdint.h>

// Initialize USART6 for syslink communication
void radio_init(void);

// Poll for incoming packets (call periodically)
void radio_poll(void);

// Get battery voltage (updated by syslink battery packets)
// Returns 0.0 if no battery packet received yet
float radio_get_battery_voltage(void);

// Check if battery packet has been received
bool radio_has_battery_data(void);

// Run radio test (waits for battery packet)
// Returns true if battery packet received within timeout
bool radio_run_test(int timeout_ms);

#endif // BRINGUP_SYSLINK_H
