// Crazyflie 2.1+ HAL - Sensors
//
// Read sensor data from IMU, barometer, and flow deck.

#include "../hal.h"
#include "platform.h"

void hal_read_sensors(sensor_data_t *sensors) {
    platform_read_sensors(sensors);
}
