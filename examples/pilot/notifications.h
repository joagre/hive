// Pilot notification definitions
//
// Shared message types for inter-actor communication within the pilot example.
// These are application-level definitions, not part of the hive runtime.

#ifndef PILOT_NOTIFICATIONS_H
#define PILOT_NOTIFICATIONS_H

#include <stdint.h>

// ----------------------------------------------------------------------------
// Supervisor notifications
// ----------------------------------------------------------------------------

typedef enum {
    NOTIFY_FLIGHT_START = 0x01,   // Begin flight sequence
    NOTIFY_FLIGHT_STOP  = 0x02,   // Emergency stop (future use)
} pilot_notification_t;

#endif // PILOT_NOTIFICATIONS_H
