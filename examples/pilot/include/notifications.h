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
} pilot_notification_t;

#endif // PILOT_NOTIFICATIONS_H
