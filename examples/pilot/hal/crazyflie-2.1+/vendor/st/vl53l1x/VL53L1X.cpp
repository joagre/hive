/*
 Copyright (c) 2017, STMicroelectronics - All Rights Reserved

 This file : part of VL53L1 Core and : dual licensed,
 either 'STMicroelectronics
 Proprietary license'
 or 'BSD 3-clause "New" or "Revised" License' , at your option.

*******************************************************************************

 'STMicroelectronics Proprietary license'

*******************************************************************************

 License terms: STMicroelectronics Proprietary in accordance with licensing
 terms at www.st.com/sla0081

 STMicroelectronics confidential
 Reproduction and Communication of this document : strictly prohibited unless
 specifically authorized in writing by STMicroelectronics.


*******************************************************************************

 Alternatively, VL53L1 Core may be distributed under the terms of
 'BSD 3-clause "New" or "Revised" License', in which case the following
 provisions apply instead of the ones mentioned above :

*******************************************************************************

 License terms: BSD 3-clause "New" or "Revised" License.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this
 list of conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.

 3. Neither the name of the copyright holder nor the names of its contributors
 may be used to endorse or promote products derived from this software
 without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************
*/

/* Includes */
#include "VL53L1X.h"

//#define DEBUG_MODE

const uint8_t VL51L1X_DEFAULT_CONFIGURATION[] = {
    0x00, /* 0x2d : set bit 2 and 5 to 1 for fast plus mode (1MHz I2C), else
             don't touch */
    0x00, /* 0x2e : bit 0 if I2C pulled up at 1.8V, else set bit 0 to 1 (pull up
             at AVDD) */
    0x00, /* 0x2f : bit 0 if GPIO pulled up at 1.8V, else set bit 0 to 1 (pull
             up at AVDD) */
    0x01, /* 0x30 : set bit 4 to 0 for active high interrupt and 1 for active
             low (bits 3:0 must be 0x1), use SetInterruptPolarity() */
    0x02, /* 0x31 : bit 1 = interrupt depending on the polarity, use
             CheckForDataReady() */
    0x00, /* 0x32 : not user-modifiable */
    0x02, /* 0x33 : not user-modifiable */
    0x08, /* 0x34 : not user-modifiable */
    0x00, /* 0x35 : not user-modifiable */
    0x08, /* 0x36 : not user-modifiable */
    0x10, /* 0x37 : not user-modifiable */
    0x01, /* 0x38 : not user-modifiable */
    0x01, /* 0x39 : not user-modifiable */
    0x00, /* 0x3a : not user-modifiable */
    0x00, /* 0x3b : not user-modifiable */
    0x00, /* 0x3c : not user-modifiable */
    0x00, /* 0x3d : not user-modifiable */
    0xff, /* 0x3e : not user-modifiable */
    0x00, /* 0x3f : not user-modifiable */
    0x0F, /* 0x40 : not user-modifiable */
    0x00, /* 0x41 : not user-modifiable */
    0x00, /* 0x42 : not user-modifiable */
    0x00, /* 0x43 : not user-modifiable */
    0x00, /* 0x44 : not user-modifiable */
    0x00, /* 0x45 : not user-modifiable */
    0x20, /* 0x46 : interrupt configuration 0->level low detection, 1-> level
             high, 2-> Out of window, 3->In window, 0x20-> New sample ready ,
             TBC */
    0x0b, /* 0x47 : not user-modifiable */
    0x00, /* 0x48 : not user-modifiable */
    0x00, /* 0x49 : not user-modifiable */
    0x02, /* 0x4a : not user-modifiable */
    0x0a, /* 0x4b : not user-modifiable */
    0x21, /* 0x4c : not user-modifiable */
    0x00, /* 0x4d : not user-modifiable */
    0x00, /* 0x4e : not user-modifiable */
    0x05, /* 0x4f : not user-modifiable */
    0x00, /* 0x50 : not user-modifiable */
    0x00, /* 0x51 : not user-modifiable */
    0x00, /* 0x52 : not user-modifiable */
    0x00, /* 0x53 : not user-modifiable */
    0xc8, /* 0x54 : not user-modifiable */
    0x00, /* 0x55 : not user-modifiable */
    0x00, /* 0x56 : not user-modifiable */
    0x38, /* 0x57 : not user-modifiable */
    0xff, /* 0x58 : not user-modifiable */
    0x01, /* 0x59 : not user-modifiable */
    0x00, /* 0x5a : not user-modifiable */
    0x08, /* 0x5b : not user-modifiable */
    0x00, /* 0x5c : not user-modifiable */
    0x00, /* 0x5d : not user-modifiable */
    0x01, /* 0x5e : not user-modifiable */
    0xcc, /* 0x5f : not user-modifiable */
    0x0f, /* 0x60 : not user-modifiable */
    0x01, /* 0x61 : not user-modifiable */
    0xf1, /* 0x62 : not user-modifiable */
    0x0d, /* 0x63 : not user-modifiable */
    0x01, /* 0x64 : Sigma threshold MSB (mm in 14.2 format for MSB+LSB), use
             SetSigmaThreshold(), default value 90 mm  */
    0x68, /* 0x65 : Sigma threshold LSB */
    0x00, /* 0x66 : Min count Rate MSB (MCPS in 9.7 format for MSB+LSB), use
             SetSignalThreshold() */
    0x80, /* 0x67 : Min count Rate LSB */
    0x08, /* 0x68 : not user-modifiable */
    0xb8, /* 0x69 : not user-modifiable */
    0x00, /* 0x6a : not user-modifiable */
    0x00, /* 0x6b : not user-modifiable */
    0x00, /* 0x6c : Intermeasurement period MSB, 32 bits register, use
             SetIntermeasurementInMs() */
    0x00, /* 0x6d : Intermeasurement period */
    0x0f, /* 0x6e : Intermeasurement period */
    0x89, /* 0x6f : Intermeasurement period LSB */
    0x00, /* 0x70 : not user-modifiable */
    0x00, /* 0x71 : not user-modifiable */
    0x00, /* 0x72 : distance threshold high MSB (in mm, MSB+LSB), use
             SetD:tanceThreshold() */
    0x00, /* 0x73 : distance threshold high LSB */
    0x00, /* 0x74 : distance threshold low MSB ( in mm, MSB+LSB), use
             SetD:tanceThreshold() */
    0x00, /* 0x75 : distance threshold low LSB */
    0x00, /* 0x76 : not user-modifiable */
    0x01, /* 0x77 : not user-modifiable */
    0x0f, /* 0x78 : not user-modifiable */
    0x0d, /* 0x79 : not user-modifiable */
    0x0e, /* 0x7a : not user-modifiable */
    0x0e, /* 0x7b : not user-modifiable */
    0x00, /* 0x7c : not user-modifiable */
    0x00, /* 0x7d : not user-modifiable */
    0x02, /* 0x7e : not user-modifiable */
    0xc7, /* 0x7f : ROI center, use SetROI() */
    0xff, /* 0x80 : XY ROI (X=Width, Y=Height), use SetROI() */
    0x9B, /* 0x81 : not user-modifiable */
    0x00, /* 0x82 : not user-modifiable */
    0x00, /* 0x83 : not user-modifiable */
    0x00, /* 0x84 : not user-modifiable */
    0x01, /* 0x85 : not user-modifiable */
    0x00, /* 0x86 : clear interrupt, use ClearInterrupt() */
    0x00  /* 0x87 : start ranging, use StartRanging() or StopRanging(), If you
             want an automatic start after VL53L1X_init() call, put 0x40 in
             location 0x87 */
};

static const uint8_t status_rtn[24] = {255, 255, 255, 5,   2,   4,   1,   7,
                                       3,   0,   255, 255, 9,   13,  255, 255,
                                       255, 255, 10,  6,   255, 255, 11,  12};

/** Constructor
 * @param[in] &i2c device I2C to be used for communication
 * @param[in] &pin xshut Mbed InterruptIn PinName to be used as component
 * @param[in] &pin_gpio1 gpio1 Mbed InterruptIn PinName to be used as component
 * GPIO_1 INT
 * @param[in] DevAddr device address, 0x52 by default
 */
VL53L1X::VL53L1X(I2C *i2c, PinName pin_xshut, PinName pin_gpio1)
    : dev_i2c(i2c), xshut(new DigitalOut(pin_xshut)),
      gpio1Int(new InterruptIn(pin_gpio1)), i2cAddress(DEFAULT_DEVICE_ADDRESS) {
}

void VL53L1X::EnableInterrupt() {
  uint8_t interruptPolarity;
  GetInterruptPolarity(&interruptPolarity);
  if (interruptPolarity) {
    gpio1Int->fall(0);
    gpio1Int->rise(callback(this, &VL53L1X::isr));
  } else {
    gpio1Int->rise(0);
    gpio1Int->fall(callback(this, &VL53L1X::isr));
  }
}

void VL53L1X::WakeUp(void) {
    MBED_ASSERT(xshut->is_connected());
    xshut->write(1);
}

void VL53L1X::SetToSleep(void) {
    MBED_ASSERT(xshut->is_connected());
    xshut->write(0);
}

void VL53L1X::SetInterruptCallback(Callback<void(uint16_t, uint8_t)> cbFct) {
  this->cbFct = cbFct;
}

void VL53L1X::isr() {
  GetRangeStatus(&rangeStatus);
  if (rangeStatus <= static_cast<uint8_t>(rangeStatusFilter)) {
    GetDistance(&distanceBuffer);
    if (cbFct != NULL) {
      cbFct.call(distanceBuffer, rangeStatus);
    }
  }
  ClearInterrupt();
}

uint16_t VL53L1X::GetBufferedDistance(void) { return distanceBuffer; }
uint8_t VL53L1X::GetBufferedRangeStatus(void) { return rangeStatus; }

void VL53L1X::SetRangeStatusFilter(VL53L1X::RangeStatus rangingStatus) {
  rangeStatusFilter = rangingStatus;
}

///////////////API1/////////////

int8_t VL53L1X::SensorReset() {

#ifdef DEBUG_MODE
  printf("[VL53L1X] Soft Reset\r\n");
#endif

  int8_t status = 0;
  status = WrByte(SOFT_RESET, 0x00);
  status = WrByte(SOFT_RESET, 0x01);
  return status;
}

//////////////////API2/////////////////////

int8_t VL53L1X::GetSWVersion(VL53L1X_Version_t *pVersion) {
  int8_t Status = 0;

  pVersion->major = IMPLEMENTATION_VER_MAJOR;
  pVersion->minor = IMPLEMENTATION_VER_MINOR;
  pVersion->build = IMPLEMENTATION_VER_SUB;
  pVersion->revision = IMPLEMENTATION_VER_REVISION;
  return Status;
}

int8_t VL53L1X::SetI2CAddress(uint8_t new_address) {
  int8_t status = 0;

  status = WrByte(VL53L1_I2C_SLAVE__DEVICE_ADDRESS, new_address >> 1);
  return status;
}

int8_t VL53L1X::SensorInit() {
  int8_t status = 0;
  uint8_t Addr = 0x00, tmp;

  for (Addr = 0x2D; Addr <= 0x87; Addr++) {
    status = WrByte(Addr, VL51L1X_DEFAULT_CONFIGURATION[Addr - 0x2D]);
  }

  /// VVV POSTINIT VVV///
  status = VL53L1X::StartRanging();
  #ifdef DEBUG_MODE
  printf("[VL53L1X] Start Ranging status %d\r\n", status);
  #endif
  tmp = 0;
  while (tmp == 0) {
    status = VL53L1X::CheckForDataReady(&tmp);
  }
  status = VL53L1X::ClearInterrupt();
  status = VL53L1X::StopRanging();
  status = WrByte(VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND,
                  0x09);    /* two bounds VHV */
  status = WrByte(0x0B, 0); /* start VHV from the previous temperature */
  return status;
}

int8_t VL53L1X::SensorInit0() {
  int8_t status = 0;
  uint8_t Addr = 0x00;

  for (Addr = 0x2D; Addr <= 0x87; Addr++) {
    status = WrByte(Addr, VL51L1X_DEFAULT_CONFIGURATION[Addr - 0x2D]);
  }
  return status;
}

int8_t VL53L1X::SensorInit1() {
  int8_t status = 0;
  uint8_t tmp;

  status = VL53L1X::StartRanging();
  tmp = 0;
  while (tmp == 0) {
    status = VL53L1X::CheckForDataReady(&tmp);
  }
  status = VL53L1X::ClearInterrupt();
  status = VL53L1X::StopRanging();
  status = WrByte(VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND,
                  0x09);    /* two bounds VHV */
  status = WrByte(0x0B, 0); /* start VHV from the previous temperature */
  return status;
}

int8_t VL53L1X::ClearInterrupt() {
  int8_t status = 0;

  status = WrByte(SYSTEM__INTERRUPT_CLEAR, 0x01);
  return status;
}

int8_t VL53L1X::SetInterruptPolarity(uint8_t NewPolarity) {
  uint8_t Temp;
  int8_t status = 0;
  status = RdByte(GPIO_HV_MUX__CTRL, &Temp);
  Temp = Temp & 0xEF;
  status = WrByte(GPIO_HV_MUX__CTRL, Temp | (!(NewPolarity & 1)) << 4);
  return status;
}

int8_t VL53L1X::GetInterruptPolarity(uint8_t *pInterruptPolarity) {
  uint8_t Temp;
  int8_t status = 0;

  status = RdByte(GPIO_HV_MUX__CTRL, &Temp);
  Temp = Temp & 0x10;
  *pInterruptPolarity = !(Temp >> 4);
  return status;
}

int8_t VL53L1X::StartRanging() {
  int8_t status = 0;

  status = WrByte(SYSTEM__MODE_START, 0x40); /* Enable VL53L1X */
  return status;
}

int8_t VL53L1X::StopRanging() {
  int8_t status = 0;

  status = WrByte(SYSTEM__MODE_START, 0x00); /* Disable VL53L1X */
  return status;
}

int8_t VL53L1X::CheckForDataReady(uint8_t *isDataReady) {
  uint8_t Temp;
  uint8_t IntPol;
  int8_t status = 0;

  status = VL53L1X::GetInterruptPolarity(&IntPol);
  status = RdByte(GPIO__TIO_HV_STATUS, &Temp);
  /* Read in the register to check if a new value is available */
  if (status == 0) {
    if ((Temp & 1) == IntPol)
      *isDataReady = 1;
    else
      *isDataReady = 0;
  }
  return status;
}

int8_t VL53L1X::SetTimingBudgetInMs(TimingBudget timingBudget) {
  DistanceModes DM;
  int8_t status = 0;

  status = VL53L1X::GetDistanceMode(&DM);
  if (DM == DistanceModes::Error)
    return 1;
  else if (DM == DistanceModes::Short) { /* Short DistanceMode */
    switch (timingBudget) {
    case TimingBudget::_15ms: /* only available in short distance mode */
      WrWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x01D);
      WrWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0027);
      break;
    case TimingBudget::_20ms:
      WrWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x0051);
      WrWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x006E);
      break;
    case TimingBudget::_33ms:
      WrWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x00D6);
      WrWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x006E);
      break;
    case TimingBudget::_50ms:
      WrWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x1AE);
      WrWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x01E8);
      break;
    case TimingBudget::_100ms:
      WrWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x02E1);
      WrWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0388);
      break;
    case TimingBudget::_200ms:
      WrWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x03E1);
      WrWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0496);
      break;
    case TimingBudget::_500ms:
      WrWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x0591);
      WrWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x05C1);
      break;
    default:
      status = 1;
      break;
    }
  } else {
    switch (timingBudget) {
    case TimingBudget::_20ms:
      WrWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x001E);
      WrWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0022);
      break;
    case TimingBudget::_33ms:
      WrWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x0060);
      WrWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x006E);
      break;
    case TimingBudget::_50ms:
      WrWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x00AD);
      WrWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x00C6);
      break;
    case TimingBudget::_100ms:
      WrWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x01CC);
      WrWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x01EA);
      break;
    case TimingBudget::_200ms:
      WrWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x02D9);
      WrWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x02F8);
      break;
    case TimingBudget::_500ms:
      WrWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x048F);
      WrWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x04A4);
      break;
    default:
      status = 1;
      break;
    }
  }
  return status;
}

int8_t VL53L1X::GetTimingBudgetInMs(TimingBudget *pTimingBudget) {
  uint16_t Temp;
  int8_t status = 0;

  status = RdWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, &Temp);
  switch (Temp) {
  case 0x001D:
    *pTimingBudget = TimingBudget::_15ms;
    break;
  case 0x0051:
  case 0x001E:
    *pTimingBudget = TimingBudget::_20ms;
    break;
  case 0x00D6:
  case 0x0060:
    *pTimingBudget = TimingBudget::_33ms;
    break;
  case 0x1AE:
  case 0x00AD:
    *pTimingBudget = TimingBudget::_50ms;
    break;
  case 0x02E1:
  case 0x01CC:
    *pTimingBudget = TimingBudget::_100ms;
    break;
  case 0x03E1:
  case 0x02D9:
    *pTimingBudget = TimingBudget::_200ms;
    break;
  case 0x0591:
  case 0x048F:
    *pTimingBudget = TimingBudget::_500ms;
    break;
  default:
    status = 1;
    *pTimingBudget = TimingBudget::Error;
  }
  return status;
}

int8_t VL53L1X::SetDistanceMode(DistanceModes DM) {
  TimingBudget TB;
  int8_t status = 0;

  status = VL53L1X::GetTimingBudgetInMs(&TB);
  if (status != 0)
    return 1;
  switch (DM) {
  case DistanceModes::Short:
    status = WrByte(PHASECAL_CONFIG__TIMEOUT_MACROP, 0x14);
    status = WrByte(RANGE_CONFIG__VCSEL_PERIOD_A, 0x07);
    status = WrByte(RANGE_CONFIG__VCSEL_PERIOD_B, 0x05);
    status = WrByte(RANGE_CONFIG__VALID_PHASE_HIGH, 0x38);
    status = WrWord(SD_CONFIG__WOI_SD0, 0x0705);
    status = WrWord(SD_CONFIG__INITIAL_PHASE_SD0, 0x0606);
    break;
  case DistanceModes::Long:
    status = WrByte(PHASECAL_CONFIG__TIMEOUT_MACROP, 0x0A);
    status = WrByte(RANGE_CONFIG__VCSEL_PERIOD_A, 0x0F);
    status = WrByte(RANGE_CONFIG__VCSEL_PERIOD_B, 0x0D);
    status = WrByte(RANGE_CONFIG__VALID_PHASE_HIGH, 0xB8);
    status = WrWord(SD_CONFIG__WOI_SD0, 0x0F0D);
    status = WrWord(SD_CONFIG__INITIAL_PHASE_SD0, 0x0E0E);
    break;
  default:
    status = 1;
    break;
  }

  if (status == 0)
    status = VL53L1X::SetTimingBudgetInMs(TB);
  return status;
}

int8_t VL53L1X::GetDistanceMode(DistanceModes *DM) {
  uint8_t TempDM, status = 0;

  status = RdByte(PHASECAL_CONFIG__TIMEOUT_MACROP, &TempDM);
  if (TempDM == 0x14)
    *DM = DistanceModes::Short;
  if (TempDM == 0x0A)
    *DM = DistanceModes::Long;
  return status;
}

int8_t VL53L1X::SetInterMeasurementInMs(uint32_t InterMeasMs) {
  uint16_t ClockPLL;
  int8_t status = 0;

  status = RdWord(VL53L1_RESULT__OSC_CALIBRATE_VAL, &ClockPLL);
  ClockPLL = ClockPLL & 0x3FF;
  WrDWord(VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD,
          (uint32_t)(ClockPLL * InterMeasMs * 1.075));
  return status;
}

int8_t VL53L1X::GetInterMeasurementInMs(uint16_t *pIM) {
  uint16_t ClockPLL;
  int8_t status = 0;
  uint32_t tmp;

  status = RdDWord(VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD, &tmp);
  *pIM = (uint16_t)tmp;
  status = RdWord(VL53L1_RESULT__OSC_CALIBRATE_VAL, &ClockPLL);
  ClockPLL = ClockPLL & 0x3FF;
  *pIM = (uint16_t)(*pIM / (ClockPLL * 1.065));
  return status;
}

int8_t VL53L1X::BootState(uint8_t *state) {
  int8_t status = 0;
  uint8_t tmp = 0;

  status = RdByte(VL53L1_FIRMWARE__SYSTEM_STATUS, &tmp);
  *state = tmp;
  return status;
}

int8_t VL53L1X::GetSensorId(uint16_t *sensorId) {
  int8_t status = 0;
  uint16_t tmp = 0;

  status = RdWord(VL53L1_IDENTIFICATION__MODEL_ID, &tmp);
  *sensorId = tmp;
  return status;
}

int8_t VL53L1X::GetDistance(uint16_t *distance) {
  int8_t status = 0;
  uint16_t tmp;

  status =
      (RdWord(VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, &tmp));
  *distance = tmp;
  return status;
}

int8_t VL53L1X::GetSignalPerSpad(uint16_t *signalRate) {
  int8_t status = 0;
  uint16_t SpNb = 1, signal;

  status =
      RdWord(VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0,
             &signal);
  status = RdWord(VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0, &SpNb);
  *signalRate = (uint16_t)(2000.0 * signal / SpNb);
  return status;
}

int8_t VL53L1X::GetAmbientPerSpad(uint16_t *ambPerSp) {
  int8_t status = 0;
  uint16_t AmbientRate, SpNb = 1;

  status = RdWord(RESULT__AMBIENT_COUNT_RATE_MCPS_SD, &AmbientRate);
  status = RdWord(VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0, &SpNb);
  *ambPerSp = (uint16_t)(2000.0 * AmbientRate / SpNb);
  return status;
}

int8_t VL53L1X::GetSignalRate(uint16_t *signal) {
  int8_t status = 0;
  uint16_t tmp;

  status = RdWord(
      VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0, &tmp);
  *signal = tmp * 8;
  return status;
}

int8_t VL53L1X::GetSpadNb(uint16_t *spNb) {
  int8_t status = 0;
  uint16_t tmp;

  status = RdWord(VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0, &tmp);
  *spNb = tmp >> 8;
  return status;
}

int8_t VL53L1X::GetAmbientRate(uint16_t *ambRate) {
  int8_t status = 0;
  uint16_t tmp;

  status = RdWord(RESULT__AMBIENT_COUNT_RATE_MCPS_SD, &tmp);
  *ambRate = tmp * 8;
  return status;
}

int8_t VL53L1X::GetRangeStatus(uint8_t *rangeStatus) {
  int8_t status = 0;
  uint8_t RgSt;

  *rangeStatus = 255;
  status = RdByte(VL53L1_RESULT__RANGE_STATUS, &RgSt);
  RgSt = RgSt & 0x1F;
  if (RgSt < 24)
    *rangeStatus = status_rtn[RgSt];
  return status;
}

int8_t VL53L1X::GetResult(VL53L1X_Result_t *pResult) {
  int8_t status = 0;
  uint8_t Temp[17];
  uint8_t RgSt = 255;

  status = ReadMulti(VL53L1_RESULT__RANGE_STATUS, Temp, 17);
  RgSt = Temp[0] & 0x1F;
  if (RgSt < 24)
    RgSt = status_rtn[RgSt];
  pResult->Status = RgSt;
  pResult->Ambient = (Temp[7] << 8 | Temp[8]) * 8;
  pResult->NumSPADs = Temp[3];
  pResult->SigPerSPAD = (Temp[15] << 8 | Temp[16]) * 8;
  pResult->Distance = Temp[13] << 8 | Temp[14];

  return status;
}

int8_t VL53L1X::SetOffset(int16_t OffsetValue) {
  int8_t status = 0;
  int16_t Temp;

  Temp = (OffsetValue * 4);
  WrWord(ALGO__PART_TO_PART_RANGE_OFFSET_MM, (uint16_t)Temp);
  WrWord(MM_CONFIG__INNER_OFFSET_MM, 0x0);
  WrWord(MM_CONFIG__OUTER_OFFSET_MM, 0x0);
  return status;
}

int8_t VL53L1X::GetOffset(int16_t *offset) {
  int8_t status = 0;
  uint16_t Temp;

  status = RdWord(ALGO__PART_TO_PART_RANGE_OFFSET_MM, &Temp);
  Temp = Temp << 3;
  Temp = Temp >> 5;
  *offset = (int16_t)(Temp);
  return status;
}

int8_t VL53L1X::SetXtalk(uint16_t XtalkValue) {
  /* XTalkValue in count per second to avoid float type */
  int8_t status = 0;

  status = WrWord(ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS, 0x0000);
  status = WrWord(ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS, 0x0000);
  status = WrWord(
      ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS,
      (XtalkValue << 9) /
          1000); /* * << 9 (7.9 format) and /1000 to convert cps to kpcs */
  return status;
}

int8_t VL53L1X::GetXtalk(uint16_t *xtalk) {
  int8_t status = 0;
  uint32_t tmp;

  status = RdDWord(ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS, &tmp);
  *xtalk = (uint16_t)(tmp * 1000) >>
           9; /* * 1000 to convert kcps to cps and >> 9 (7.9 format) */
  return status;
}

int8_t VL53L1X::SetDistanceThreshold(uint16_t ThreshLow, uint16_t ThreshHigh,
                                     WindowTypes Window,
                                     uint8_t IntOnNoTarget) {
  int8_t status = 0;
  uint8_t Temp = 0;

  status = RdByte(SYSTEM__INTERRUPT_CONFIG_GPIO, &Temp);
  Temp = Temp & 0x47;
  if (IntOnNoTarget == 0) {
    status = WrByte(SYSTEM__INTERRUPT_CONFIG_GPIO,
                    (Temp | ((uint8_t)Window & 0x07)));
  } else {
    status = WrByte(SYSTEM__INTERRUPT_CONFIG_GPIO,
                    ((Temp | ((uint8_t)Window & 0x07)) | 0x40));
  }
  status = WrWord(SYSTEM__THRESH_HIGH, ThreshHigh);
  status = WrWord(SYSTEM__THRESH_LOW, ThreshLow);
  return status;
}

int8_t VL53L1X::GetDistanceThresholdWindow(WindowTypes *window) {
  int8_t status = 0;
  uint8_t tmp;
  status = RdByte(SYSTEM__INTERRUPT_CONFIG_GPIO, &tmp);
  *window = (WindowTypes)(tmp & 0x7);
  return status;
}

int8_t VL53L1X::GetDistanceThresholdLow(uint16_t *low) {
  int8_t status = 0;
  uint16_t tmp;

  status = RdWord(SYSTEM__THRESH_LOW, &tmp);
  *low = tmp;
  return status;
}

int8_t VL53L1X::GetDistanceThresholdHigh(uint16_t *high) {
  int8_t status = 0;
  uint16_t tmp;

  status = RdWord(SYSTEM__THRESH_HIGH, &tmp);
  *high = tmp;
  return status;
}

int8_t VL53L1X::SetROICenter(uint8_t ROICenter) {
  int8_t status = 0;
  status = WrByte(ROI_CONFIG__USER_ROI_CENTRE_SPAD, ROICenter);
  return status;
}

int8_t VL53L1X::GetROICenter(uint8_t *ROICenter) {
  int8_t status = 0;
  uint8_t tmp;
  status = RdByte(ROI_CONFIG__USER_ROI_CENTRE_SPAD, &tmp);
  *ROICenter = tmp;
  return status;
}

int8_t VL53L1X::SetROI(uint16_t X, uint16_t Y) {
  uint8_t OpticalCenter;
  int8_t status = 0;

  status = RdByte(VL53L1_ROI_CONFIG__MODE_ROI_CENTRE_SPAD, &OpticalCenter);
  if (X > 16)
    X = 16;
  if (Y > 16)
    Y = 16;
  if (X > 10 || Y > 10) {
    OpticalCenter = 199;
  }
  status = WrByte(ROI_CONFIG__USER_ROI_CENTRE_SPAD, OpticalCenter);
  status = WrByte(ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE,
                  (Y - 1) << 4 | (X - 1));
  return status;
}

int8_t VL53L1X::GetROI_XY(uint16_t *ROI_X, uint16_t *ROI_Y) {
  int8_t status = 0;
  uint8_t tmp;

  status = RdByte(ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE, &tmp);
  *ROI_X = ((uint16_t)tmp & 0x0F) + 1;
  *ROI_Y = (((uint16_t)tmp & 0xF0) >> 4) + 1;
  return status;
}

int8_t VL53L1X::SetSignalThreshold(uint16_t Signal) {
  int8_t status = 0;

  WrWord(RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS, Signal >> 3);
  return status;
}

int8_t VL53L1X::GetSignalThreshold(uint16_t *signal) {
  int8_t status = 0;
  uint16_t tmp;

  status = RdWord(RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS, &tmp);
  *signal = tmp << 3;
  return status;
}

int8_t VL53L1X::SetSigmaThreshold(uint16_t Sigma) {
  int8_t status = 0;

  if (Sigma > (0xFFFF >> 2)) {
    return 1;
  }
  /* 16 bits register 14.2 format */
  status = WrWord(RANGE_CONFIG__SIGMA_THRESH, Sigma << 2);
  return status;
}

int8_t VL53L1X::GetSigmaThreshold(uint16_t *sigma) {
  int8_t status = 0;
  uint16_t tmp;

  status = RdWord(RANGE_CONFIG__SIGMA_THRESH, &tmp);
  *sigma = tmp >> 2;
  return status;
}

int8_t VL53L1X::StartTemperatureUpdate() {
  int8_t status = 0;
  uint8_t tmp = 0;

  status =
      WrByte(VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x81); /* full VHV */
  status = WrByte(0x0B, 0x92);
  status = VL53L1X::StartRanging();
  while (tmp == 0) {
    status = VL53L1X::CheckForDataReady(&tmp);
  }
  tmp = 0;
  status = VL53L1X::ClearInterrupt();
  status = VL53L1X::StopRanging();
  status = WrByte(VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND,
                  0x09);    /* two bounds VHV */
  status = WrByte(0x0B, 0); /* start VHV from the previous temperature */
  return status;
}

//////////////////CALIBRATION/////////////////////

#define ALGO__PART_TO_PART_RANGE_OFFSET_MM 0x001E
#define MM_CONFIG__INNER_OFFSET_MM 0x0020
#define MM_CONFIG__OUTER_OFFSET_MM 0x0022

int8_t VL53L1X::CalibrateOffset(uint16_t TargetDistInMm, int16_t *offset) {
  uint8_t i, tmp;
  int16_t AverageDistance = 0;
  uint16_t distance;
  int8_t status = 0;

  status = WrWord(ALGO__PART_TO_PART_RANGE_OFFSET_MM, 0x0);
  status = WrWord(MM_CONFIG__INNER_OFFSET_MM, 0x0);
  status = WrWord(MM_CONFIG__OUTER_OFFSET_MM, 0x0);
  status = StartRanging(); /* Enable VL53L1X sensor */
  for (i = 0; i < 50; i++) {
    tmp = 0;
    while (tmp == 0) {
      status = CheckForDataReady(&tmp);
    }
    status = GetDistance(&distance);
    status = ClearInterrupt();
    AverageDistance = AverageDistance + distance;
  }
  status = StopRanging();
  AverageDistance = AverageDistance / 50;
  *offset = TargetDistInMm - AverageDistance;
  status = WrWord(ALGO__PART_TO_PART_RANGE_OFFSET_MM, *offset * 4);
  return status;
}

int8_t VL53L1X::CalibrateXtalk(uint16_t TargetDistInMm, uint16_t *xtalk) {
  uint8_t i, tmp;
  float AverageSignalRate = 0;
  float AverageDistance = 0;
  float AverageSpadNb = 0;
  uint16_t distance = 0, spadNum;
  uint16_t sr;
  int8_t status = 0;
  uint32_t calXtalk;

  status = WrWord(0x0016, 0);
  status = StartRanging();
  for (i = 0; i < 50; i++) {
    tmp = 0;
    while (tmp == 0) {
      status = CheckForDataReady(&tmp);
    }
    status = GetSignalRate(&sr);
    status = GetDistance(&distance);
    status = ClearInterrupt();
    AverageDistance = AverageDistance + distance;
    status = GetSpadNb(&spadNum);
    AverageSpadNb = AverageSpadNb + spadNum;
    AverageSignalRate = AverageSignalRate + sr;
  }
  status = StopRanging();
  AverageDistance = AverageDistance / 50;
  AverageSpadNb = AverageSpadNb / 50;
  AverageSignalRate = AverageSignalRate / 50;
  /* Calculate Xtalk value */
  calXtalk = (uint16_t)(
      512 * (AverageSignalRate * (1 - (AverageDistance / TargetDistInMm))) /
      AverageSpadNb);
  *xtalk = (uint16_t)(calXtalk * 1000) >> 9;
  status = WrWord(0x0016, calXtalk);
  return status;
}

//////////////////PLATFORM/////////////////////

/* Write and read functions from I2C */

int8_t VL53L1X::WriteMulti(uint16_t index, uint8_t *pdata, uint32_t count) {
  int status;

  status = I2CWrite(i2cAddress, index, pdata, (uint16_t)count);
  return status;
}

int8_t VL53L1X::ReadMulti(uint16_t index, uint8_t *pdata, uint32_t count) {
  int status;

  status = I2CRead(i2cAddress, index, pdata, (uint16_t)count);

  return status;
}

int8_t VL53L1X::WrByte(uint16_t index, uint8_t data) {
  int status;

  status = I2CWrite(i2cAddress, index, &data, 1);
  return status;
}

int8_t VL53L1X::WrWord(uint16_t index, uint16_t data) {
  int status;
  uint8_t buffer[2];

  buffer[0] = data >> 8;
  buffer[1] = data & 0x00FF;
  status = I2CWrite(i2cAddress, index, (uint8_t *)buffer, 2);
  return status;
}

int8_t VL53L1X::WrDWord(uint16_t index, uint32_t data) {
  int status;
  uint8_t buffer[4];

  buffer[0] = (data >> 24) & 0xFF;
  buffer[1] = (data >> 16) & 0xFF;
  buffer[2] = (data >> 8) & 0xFF;
  buffer[3] = (data >> 0) & 0xFF;
  status = I2CWrite(i2cAddress, index, (uint8_t *)buffer, 4);
  return status;
}

int8_t VL53L1X::RdByte(uint16_t index, uint8_t *data) {
  int status;

  status = I2CRead(i2cAddress, index, data, 1);

  return status;
}

int8_t VL53L1X::RdWord(uint16_t index, uint16_t *data) {
  int status;
  uint8_t buffer[2] = {0, 0};

  status = I2CRead(i2cAddress, index, buffer, 2);
  if (!status) {
    *data = (buffer[0] << 8) + buffer[1];
  }
  return status;
}

int8_t VL53L1X::RdDWord(uint16_t index, uint32_t *data) {
  int status;
  uint8_t buffer[4] = {0, 0, 0, 0};

  status = I2CRead(i2cAddress, index, buffer, 4);
  if (!status) {
    *data =
        (buffer[0] << 24) + (buffer[1] << 16) + (buffer[2] << 8) + buffer[3];
  }
  return status;
}

int8_t VL53L1X::UpdateByte(uint16_t index, uint8_t AndData, uint8_t OrData) {
  int status;
  uint8_t buffer = 0;

  /* read data direct onto buffer */
  status = I2CRead(i2cAddress, index, &buffer, 1);
  if (!status) {
    buffer = (buffer & AndData) | OrData;
    status = I2CWrite(i2cAddress, index, &buffer, (uint16_t)1);
  }
  return status;
}

int8_t VL53L1X::I2CWrite(uint8_t DeviceAddr, uint16_t RegisterAddr,
                         uint8_t *pBuffer, uint16_t NumByteToWrite) {
#ifdef DEBUG_MODE
  //printf("[VL53L1X] Beginning transmission to %x\r\n", DeviceAddr);
#endif
#ifdef DEBUG_MODE
  //printf("[VL53L1X] Writing port number %x\r\n", RegisterAddr);
#endif
  uint8_t buffer[NumByteToWrite + 2];
  buffer[0] = RegisterAddr >> 8;
  buffer[1] = RegisterAddr & 0xFF;
  for (int i = 0; i < NumByteToWrite; i++) {
    buffer[2 + i] = pBuffer[i];
  }
  uint8_t result;
  result = dev_i2c->write(DeviceAddr, (char *)buffer, NumByteToWrite + 2);

  if (result) {
#ifdef DEBUG_MODE
    printf("[VL53L1X] I2C WRITE: writing failed %x, %x\r\n", DeviceAddr, RegisterAddr);
#endif
  }

  return result;
}

int8_t VL53L1X::I2CRead(uint8_t DeviceAddr, uint16_t RegisterAddr,
                        uint8_t *pBuffer, uint16_t NumByteToRead) {

// Loop until the port is transmitted correctly
// uint8_t maxAttempts = 5;
// for (uint8_t x = 0; x < maxAttempts; x++)
// {
#ifdef DEBUG_MODE
  //printf("[VL53L1X] Beginning transmission to %x\r\n", DeviceAddr);
#endif
#ifdef DEBUG_MODE
  //printf("[VL53L1X] Writing port number %x\r\n", RegisterAddr);
#endif

  uint8_t buffer[2];
  buffer[0] = RegisterAddr >> 8;
  buffer[1] = RegisterAddr & 0xFF;
  uint8_t result;
  result = dev_i2c->write(DeviceAddr, (char *)buffer, 2, false);

  if (result) {
#ifdef DEBUG_MODE
    printf("[VL53L1X] I2C READ: writing failed %x\r\n", DeviceAddr);
#endif
    return result;
  }

  result = dev_i2c->read(DeviceAddr, (char *)pBuffer, NumByteToRead, true);

  if (result) {
#ifdef DEBUG_MODE
    printf("[VL53L1X] I2C READ: reading failed %x, %x\r\n", DeviceAddr, RegisterAddr);
#endif
  }

  return result;
}

int8_t VL53L1X::WaitUs(int32_t waitUs) {
  wait_us(waitUs);
  return 0;
}

int8_t VL53L1X::WaitMs(int32_t waitMs) {
  wait_us(waitMs * 1000);
  return 0;
}
