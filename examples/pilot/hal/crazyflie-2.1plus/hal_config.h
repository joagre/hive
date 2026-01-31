// Crazyflie 2.1+ HAL Configuration

#ifndef HAL_CONFIG_H
#define HAL_CONFIG_H

// Motor thrust base value (matching Bitcraze)
#define HAL_BASE_THRUST 36000

// Motor mapping (rotation viewed from above):
// M1 (TIM2_CH1) - Front Right (CCW)
// M2 (TIM2_CH2) - Back Right (CW)
// M3 (TIM2_CH3) - Back Left (CCW)
// M4 (TIM2_CH4) - Front Left (CW)

// Sensor update rate (Hz)
#define HAL_SENSOR_RATE 1000

// Enable radio (syslink to nRF51)
#define HAL_HAS_RADIO 1

#endif // HAL_CONFIG_H
