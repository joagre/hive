// Shared bus configuration for pilot actors
//
// All actors receive this struct via init_args and extract the buses they need.

#ifndef PILOT_BUSES_H
#define PILOT_BUSES_H

#include "hive_bus.h"

// All buses used in the pilot control pipeline
typedef struct {
    bus_id_t sensor_bus;
    bus_id_t state_bus;
    bus_id_t thrust_bus;
    bus_id_t position_target_bus;
    bus_id_t attitude_setpoint_bus;
    bus_id_t rate_setpoint_bus;
    bus_id_t torque_bus;
} pilot_buses_t;

#endif // PILOT_BUSES_H
