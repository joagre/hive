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
} pilot_notification_t;

// ----------------------------------------------------------------------------
// Request/reply codes for sibling RESET, ARM, DISARM, STATUS
// ----------------------------------------------------------------------------

#define REPLY_OK 0x00
#define REPLY_FAIL 0x01

// Request tags for flight manager to siblings
#define REQUEST_RESET 0x10
#define REQUEST_ARM 0x11
#define REQUEST_DISARM 0x12
#define REQUEST_STATUS 0x13

#endif // PILOT_NOTIFICATIONS_H
