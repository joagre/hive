# Third-Party Licenses

This directory contains vendor sensor drivers. Each is licensed under a permissive
open-source license as documented below.

## Summary

| Directory | Type | License | Source |
|-----------|------|---------|--------|
| `CMSIS/Include/` | Unmodified | Apache-2.0 | ARM CMSIS-Core |
| `CMSIS/Device/ST/` | Unmodified | BSD-3-Clause | STMicroelectronics |
| `bosch/bmi08x/` | Unmodified | BSD-3-Clause | Bosch Sensortec GitHub |
| `bosch/bmp3/` | Unmodified | BSD-3-Clause | Bosch Sensortec GitHub |
| `st/vl53l1x/` | **C port** | BSD-3-Clause | ST Arduino library |
| `bitcraze/pmw3901/` | **C port** | MIT | Bitcraze Arduino library |

**C ports** - The VL53L1x and PMW3901 drivers are C ports of the original C++/Arduino
libraries. The original `.cpp` files are included for reference. The C ports implement
the same register-level logic but use plain C with platform I2C/SPI callbacks instead
of Arduino Wire/SPI classes.

---

## Bosch BMI08x (vendor/bosch/bmi08x/)

**License** - BSD-3-Clause
**Source** - https://github.com/boschsensortec/BMI08x_SensorAPI
**Copyright** - (c) 2024 Bosch Sensortec GmbH

The BMI08x sensor API provides accelerometer and gyroscope support for the
BMI085/BMI088 6-axis inertial measurement unit.

```
BSD-3-Clause

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
```

## Bosch BMP3 (vendor/bosch/bmp3/)

**License** - BSD-3-Clause
**Source** - https://github.com/boschsensortec/BMP3_SensorAPI
**Copyright** - (c) 2022 Bosch Sensortec GmbH

The BMP3 sensor API provides barometric pressure and temperature support for
the BMP380/BMP388/BMP390 barometer.

```
BSD-3-Clause

(Same license text as BMI08x above)
```

## ST VL53L1x ULD (vendor/st/vl53l1x/)

**License** - BSD-3-Clause
**Source** - C port derived from https://github.com/VRaktion/mbed-VL53L1X-ULD
**Original** - https://www.st.com/en/imaging-and-photonics-solutions/vl53l1x.html
**Copyright** - (c) 2017 STMicroelectronics

The VL53L1x Ultra Lite Driver provides time-of-flight distance measurement.
This is a C port of the ST/mbed ULD library.

```
BSD-3-Clause "New" or "Revised" License

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from this
   software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
```

## Bitcraze PMW3901 (vendor/bitcraze/pmw3901/)

**License** - MIT
**Source** - C port derived from https://github.com/bitcraze/Bitcraze_PMW3901
**Copyright** - (c) 2017 Bitcraze AB

The PMW3901 driver provides optical flow sensing. This is a C port of the
Bitcraze Arduino library.

```
MIT License

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to
deal in the Software without restriction, including without limitation the
rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

## ARM CMSIS-Core (vendor/CMSIS/Include/)

**License** - Apache-2.0
**Source** - https://github.com/ARM-software/CMSIS_5
**Copyright** - (c) 2009-2023 ARM Limited

ARM Cortex Microcontroller Software Interface Standard (CMSIS) provides
standardized access to processor and peripherals.

Files: `core_cm4.h`, `cmsis_gcc.h`, `cmsis_compiler.h`, `cmsis_version.h`, `mpu_armv7.h`

## STMicroelectronics CMSIS Device (vendor/CMSIS/Device/ST/)

**License** - BSD-3-Clause
**Source** - https://github.com/STMicroelectronics/cmsis_device_f4
**Copyright** - (c) 2017 STMicroelectronics

STM32F4 device-specific CMSIS headers providing register definitions.

Files: `stm32f4xx.h`, `stm32f405xx.h`, `system_stm32f4xx.h`
