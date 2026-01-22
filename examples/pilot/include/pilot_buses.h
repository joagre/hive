// Shared bus configuration for pilot actors
//
// All actors receive this struct via init_args and extract the buses they need.

#ifndef PILOT_BUSES_H
#define PILOT_BUSES_H

#include "hive_bus.h"

// All buses used in the pilot control pipeline
typedef struct {
    bus_id sensor_bus;
    bus_id state_bus;
    bus_id thrust_bus;
    bus_id position_target_bus;
    bus_id attitude_setpoint_bus;
    bus_id rate_setpoint_bus;
    bus_id torque_bus;
} pilot_buses;

#endif // PILOT_BUSES_H
