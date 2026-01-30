// Crazyflie 2.1+ HAL Configuration (v2)

#ifndef HAL_CONFIG_H
#define HAL_CONFIG_H

// Motor thrust base value (matching Bitcraze)
#define HAL_BASE_THRUST 36000

// Motor mapping:
// M1 (TIM2_CH1) - Front Right (CW)
// M2 (TIM2_CH2) - Back Right (CCW)
// M3 (TIM2_CH3) - Back Left (CW)
// M4 (TIM2_CH4) - Front Left (CCW)

// Sensor update rate (Hz)
#define HAL_SENSOR_RATE 1000

// Enable radio (syslink to nRF51)
#define HAL_HAS_RADIO 1

#endif // HAL_CONFIG_H
