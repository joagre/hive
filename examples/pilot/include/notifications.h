// Pilot notification definitions
//
// Shared message types for inter-actor communication within the pilot example.
// These are application-level definitions, not part of the hive runtime.

#ifndef PILOT_NOTIFICATIONS_H
#define PILOT_NOTIFICATIONS_H

#include <stdint.h>

// ----------------------------------------------------------------------------
// Flight manager notifications
// ----------------------------------------------------------------------------

typedef enum {
    NOTIFY_FLIGHT_START = 0x01, // Flight manager -> waypoint: begin flight
    NOTIFY_FLIGHT_STOP = 0x02,  // Flight manager -> motor: stop all motors
    NOTIFY_LANDING =
        0x03, // Flight manager -> altitude: initiate controlled landing
    NOTIFY_FLIGHT_LANDED = 0x04, // Altitude -> flight manager: landing complete
    NOTIFY_GO = 0x05, // Comms -> flight manager: ground station sent GO command
    NOTIFY_ABORT = 0x06, // Comms -> flight manager: abort countdown/flight
    NOTIFY_RESET = 0x07, // Flight manager -> all: reset internal state
    NOTIFY_LOW_BATTERY = 0x08, // Battery -> flight manager: voltage critical
} pilot_notification_t;

#endif // PILOT_NOTIFICATIONS_H
