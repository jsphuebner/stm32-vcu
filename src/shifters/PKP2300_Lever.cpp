/*
 * This file is part of the Zombieverter project.
 *
 * Copyright (C) 2024 Damien Maguire
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Support for the Blink Marine PKP-2300-SI CANOpen button panel.
 * Spec: https://www.murcal.com/pdf%20folder/15.blink_PKP-2300-SI_CANOpen.pdf
 *
 * CANOpen node ID 1 (default):
 *   TPDO1 (panel → VCU): 0x181 – button states in byte 0, one bit per key
 *   RPDO1 (VCU → panel): 0x201 – LED colors, one byte per LED (K1–K6)
 *
 * LED color codes:
 *   0=off, 1=red, 2=green, 3=blue, 4=cyan, 5=magenta, 6=orange, 7=white
 */

#include "shifters/PKP2300_Lever.h"
#include "params.h"
#include "throttle.h"
#include <cmath>

// CANOpen IDs for node 1
#define PKP_TPDO1 0x195 // button states from panel
#define PKP_RPDO1 0x215 // LED control to panel
#define PKP_MT_TPDO2 0x295 // encoder 1 state from panel
#define PKP_MT_TPDO3 0x395 // encoder 2 state from panel
#define PKP_MT_RPDO2 0x415 // ring LED control to panel
#define PKP_NODE_ID 0x15
#define BUTTON_MSG_TIMEOUT_CYCLES 3 // 300 ms with Task100Ms period

// LED bytes
#define LED_RED 0
#define LED_GREEN 1
#define LED_BLUE 2

// Button (and LED) bit positions in TPDO1 byte 0
#define BTN_REG_DRIVE (1 << 0)
#define BTN_REG_NEUTRAL (1 << 1)
#define BTN_REG_REVERSE (1 << 2)
#define BTN_REG_PARK (1 << 3)
#define BTN_REG_REGEN (1 << 4)
#define BTN_REG_HEAT (1 << 5)

#define BTN_MT_ENC1 (1 << 0)
#define BTN_MT_PARK (1 << 1)
#define BTN_MT_ENC2 (1 << 2)
#define BTN_MT_DRIVE (1 << 3)
#define BTN_MT_NEUTRAL (1 << 4)
#define BTN_MT_REVERSE (1 << 5)

#define MT_ENCODER_CLOCKWISE 0x01
#define MT_ENCODER_COUNTERCLOCKWISE 0x81

static const float SOC_THRESHOLD_REVERSE = 33.3f;
static const float SOC_THRESHOLD_NEUTRAL = 66.6f;
static const float SOC_THRESHOLD_DRIVE = 100.0f;

static int8_t GearToParamDir(Shifter::Sgear gear) {
  switch (gear) {
  case Shifter::PARK:
    return 2;
  case Shifter::REVERSE:
    return -1;
  case Shifter::NEUTRAL:
    return 0;
  case Shifter::DRIVE:
  default:
    return 1;
  }
}

static Shifter::Sgear ParamDirToGear(int dir) {
  if (dir < 0)
    return Shifter::REVERSE;
  if (dir > 1)
    return Shifter::PARK;
  if (dir > 0)
    return Shifter::DRIVE;
  return Shifter::NEUTRAL;
}

static uint8_t GetDriveMask(bool mtModelDetected) {
  return mtModelDetected ? BTN_MT_DRIVE : BTN_REG_DRIVE;
}

static uint8_t GetNeutralMask(bool mtModelDetected) {
  return mtModelDetected ? BTN_MT_NEUTRAL : BTN_REG_NEUTRAL;
}

static uint8_t GetReverseMask(bool mtModelDetected) {
  return mtModelDetected ? BTN_MT_REVERSE : BTN_REG_REVERSE;
}

static uint8_t GetParkMask(bool mtModelDetected) {
  return mtModelDetected ? BTN_MT_PARK : BTN_REG_PARK;
}

static uint8_t GetRegenMask(bool mtModelDetected) {
  return mtModelDetected ? BTN_MT_ENC1 : BTN_REG_REGEN;
}

static uint8_t GetHeatMask(bool mtModelDetected) {
  return mtModelDetected ? BTN_MT_ENC2 : BTN_REG_HEAT;
}

static uint8_t GearToButtonMask(Shifter::Sgear gear, bool mtModelDetected) {
  switch (gear) {
  case Shifter::PARK:
    return GetParkMask(mtModelDetected);
  case Shifter::REVERSE:
    return GetReverseMask(mtModelDetected);
  case Shifter::NEUTRAL:
    return GetNeutralMask(mtModelDetected);
  case Shifter::DRIVE:
  default:
    return GetDriveMask(mtModelDetected);
  }
}

static int ClampStep(int value) {
  if (value < 0)
    return 0;
  if (value > 16)
    return 16;
  return value;
}

static int LevelToStep(float level, float fullScale) {
  if (fullScale <= 0.0f)
    return 0;

  float scaled = (level * 16.0f) / fullScale;
  return ClampStep((int)std::roundf(scaled));
}

static float StepToRegenValue(int step, float maxMagnitude) {
  step = ClampStep(step);
  if (step == 0 || maxMagnitude <= 0.0f)
    return 0.0f;
  return -(maxMagnitude * ((float)step / 16.0f));
}

static int StepToHeatPower(int step, float maxPower) {
  step = ClampStep(step);
  if (maxPower <= 0.0f)
    return 0;
  return (int)std::roundf((maxPower * (float)step) / 16.0f);
}

static uint16_t StepToRingMask(int step) {
  step = ClampStep(step);
  if (step == 0)
    return 0;
  if (step >= 16)
    return 0xFFFF;
  return (uint16_t)((1U << step) - 1U);
}

void PKP2300_Lever::SetCanInterface(CanHardware *c) {
  can = c;
  can->RegisterUserMessage(PKP_TPDO1);
  can->RegisterUserMessage(PKP_MT_TPDO2);
  can->RegisterUserMessage(PKP_MT_TPDO3);
}

void PKP2300_Lever::EnterMtMode() {
  if (mtModelDetected)
    return;

  mtModelDetected = true;
  regenDisabled = false;
  Throttle::noregenreq = false;

  float currentRegen = Param::GetFloat(Param::regenmax);
  if (currentRegen < 0.0f)
    mtLastRegenValue = currentRegen;
}

void PKP2300_Lever::HandleMtEncoderTurn(bool heaterEncoder, uint8_t encoderState) {
  if (encoderState != MT_ENCODER_CLOCKWISE &&
      encoderState != MT_ENCODER_COUNTERCLOCKWISE)
    return;

  if (heaterEncoder) {
    const Param::Attributes *heatAttrs = Param::GetAttrib(Param::HeatPwr);
    float maxHeatPower = heatAttrs != nullptr ? heatAttrs->max : 0.0f;
    int currentStep =
        LevelToStep((float)Param::GetInt(Param::HeatPwr), maxHeatPower);

    if (encoderState == MT_ENCODER_CLOCKWISE) {
      if (!Param::GetBool(Param::HeatReq)) {
        Param::SetInt(Param::HeatReq, 1);
        currentStep = 1;
      } else {
        currentStep++;
      }
    } else {
      currentStep--;
    }

    currentStep = ClampStep(currentStep);
    int heatPowerSetpoint = StepToHeatPower(currentStep, maxHeatPower);
    Param::SetInt(Param::HeatPwr, heatPowerSetpoint);
    return;
  }

  const Param::Attributes *regenAttrs = Param::GetAttrib(Param::regenmax);
  float maxRegenMagnitude =
      regenAttrs != nullptr ? std::fabs(regenAttrs->min) : 0.0f;
  float currentRegenMagnitude = std::fabs(Param::GetFloat(Param::regenmax));
  int currentStep = LevelToStep(currentRegenMagnitude, maxRegenMagnitude);

  if (encoderState == MT_ENCODER_CLOCKWISE)
    currentStep++;
  else
    currentStep--;

  currentStep = ClampStep(currentStep);

  float newRegen = StepToRegenValue(currentStep, maxRegenMagnitude);
  Param::SetFloat(Param::regenmax, newRegen);
  if (newRegen < 0.0f)
    mtLastRegenValue = newRegen;
}

void PKP2300_Lever::DecodeCAN(int id, uint32_t *data) {
  uint8_t *bytes = (uint8_t *)data;

  if (id == PKP_MT_TPDO2 || id == PKP_MT_TPDO3) {
    EnterMtMode();
    HandleMtEncoderTurn(id == PKP_MT_TPDO3, bytes[0]);
    return;
  }

  if (id != PKP_TPDO1)
    return;

  uint8_t buttons = bytes[0];
  uint8_t btnDrive = GetDriveMask(mtModelDetected);
  uint8_t btnNeutral = GetNeutralMask(mtModelDetected);
  uint8_t btnReverse = GetReverseMask(mtModelDetected);
  uint8_t btnPark = GetParkMask(mtModelDetected);
  uint8_t btnRegen = GetRegenMask(mtModelDetected);
  uint8_t btnHeat = GetHeatMask(mtModelDetected);
  buttonMsgTimeout = BUTTON_MSG_TIMEOUT_CYCLES;
  // Only allow changing away from PARK if brake pedal is pressed
  bool allowGearChange = gear != PARK || Param::GetBool(Param::din_brake);

  // Detect rising edges (button just pressed) for toggles and gear commands
  uint8_t pressed = buttons & ~prevButtonState;

  // K3 = Forward/Drive
  if ((pressed & btnDrive) && allowGearChange)
    gear = DRIVE;

  // K6 = Reverse
  if ((pressed & btnReverse) && allowGearChange)
    gear = REVERSE;

  // K2 = Neutral
  if ((pressed & btnNeutral) && allowGearChange)
    gear = NEUTRAL;

  // K5 = Park
  if (pressed & btnPark)
    gear = PARK;

  if (!mtModelDetected) {
    // K1 = toggle regen disable
    if (pressed & btnRegen) {
      regenDisabled = !regenDisabled;
      Throttle::noregenreq = regenDisabled;
    }

    // K4 = toggle heater
    if (pressed & btnHeat) {
      Param::SetInt(Param::HeatReq, Param::GetBool(Param::HeatReq) ? 0 : 1);
    }
  } else {
    // Encoder 1 button toggles regen between off and last set value.
    if (pressed & btnRegen) {
      float currentRegen = Param::GetFloat(Param::regenmax);
      if (currentRegen < 0.0f) {
        mtLastRegenValue = currentRegen;
        Param::SetFloat(Param::regenmax, 0.0f);
      } else {
        float restoreValue = mtLastRegenValue;
        if (restoreValue >= 0.0f) {
          const Param::Attributes *regenAttrs = Param::GetAttrib(Param::regenmax);
          float maxRegenMagnitude =
              regenAttrs != nullptr ? fabsf(regenAttrs->min) : 0.0f;
          restoreValue = StepToRegenValue(1, maxRegenMagnitude);
        }
        Param::SetFloat(Param::regenmax, restoreValue);
        if (restoreValue < 0.0f)
          mtLastRegenValue = restoreValue;
      }
    }

    // Encoder 2 button toggles heater request.
    if (pressed & btnHeat)
      Param::SetInt(Param::HeatReq, Param::GetBool(Param::HeatReq) ? 0 : 1);
  }

  prevButtonState = buttons;
}

void PKP2300_Lever::Task100Ms() {
  int opmode = Param::GetInt(Param::opmode);
  if (opmode == MOD_OFF)
    gear = PARK;

  if (blinkDivider == 0) {
    blinkState = !blinkState;
    // Task100Ms runs at 10 Hz; divider=4 means 500 ms per half-cycle
    // (1 Hz blink).
    blinkDivider = 4;
  } else {
    blinkDivider--;
  }

  if (buttonMsgTimeout > 0) {
    buttonMsgTimeout--;
  } else {
    // Sent continuously while TPDO1 is missing to recover panel state after
    // loss. CANopen NMT Start Remote Node: byte0=0x01 (start), byte1=node ID.
    uint8_t nmtStart[8] = {1, PKP_NODE_ID, 0, 0, 0, 0, 0, 0};
    can->Send(0x000, (uint32_t *)nmtStart, 8);
  }

  int selectedDir = Param::GetInt(Param::dir);
  if (opmode == MOD_RUN && GearToParamDir(gear) != selectedDir) {
    // Show one 100 ms red flash for a rejected user request, then resync to
    // Param::dir for the next SelectDirection cycle.
    Shifter::Sgear requestedGear = gear;
    flashRejectedDirection = true;
    gear = ParamDirToGear(selectedDir);
    rejectedDirection = requestedGear;
  }

  SendLEDs();
  flashRejectedDirection = false;
}

void PKP2300_Lever::SendLEDs() {
  float soc = Param::GetFloat(Param::SOC);
  int opmode = Param::GetInt(Param::opmode);
  bool charging = (opmode == MOD_CHARGE);
  bool heatReq = Param::GetBool(Param::HeatReq);
  bool heaterPowerActive = Param::GetFloat(Param::powerheater) > 0.0f;
  bool regenEnabled = Param::GetFloat(Param::regenmax) < 0.0f;
  uint8_t btnDrive = GetDriveMask(mtModelDetected);
  uint8_t btnNeutral = GetNeutralMask(mtModelDetected);
  uint8_t btnReverse = GetReverseMask(mtModelDetected);
  uint8_t btnPark = GetParkMask(mtModelDetected);
  uint8_t btnRegen = GetRegenMask(mtModelDetected);
  uint8_t btnHeat = GetHeatMask(mtModelDetected);
  uint8_t ledBytes[8] = {0};

  if (soc > SOC_THRESHOLD_REVERSE)
    ledBytes[LED_RED] = ledBytes[LED_GREEN] = btnReverse;
  if (soc > SOC_THRESHOLD_NEUTRAL)
    ledBytes[LED_RED] = ledBytes[LED_GREEN] |= btnNeutral;
  if (soc >= SOC_THRESHOLD_DRIVE)
    ledBytes[LED_RED] = ledBytes[LED_GREEN] |= btnDrive;

  if (charging &&
      blinkState) { // turn on one above current SoC if blinkstate is on
    if (soc < SOC_THRESHOLD_REVERSE)
      ledBytes[LED_RED] = ledBytes[LED_GREEN] = btnReverse;
    else if (soc < SOC_THRESHOLD_NEUTRAL)
      ledBytes[LED_RED] = ledBytes[LED_GREEN] |= btnNeutral;
    else if (soc < SOC_THRESHOLD_DRIVE)
      ledBytes[LED_RED] = ledBytes[LED_GREEN] |= btnDrive;
  }

  // Drive mode always takes precedence over SoC display
  if (opmode == MOD_RUN) {
    switch (gear) {
    case DRIVE:
      ledBytes[LED_BLUE] |= btnDrive;
      ledBytes[LED_RED] &= ~btnDrive;
      ledBytes[LED_GREEN] &= ~btnDrive;
      break;
    case REVERSE:
      ledBytes[LED_BLUE] |= btnReverse;
      ledBytes[LED_RED] &= ~btnReverse;
      ledBytes[LED_GREEN] &= ~btnReverse;
      break;
    case NEUTRAL:
      ledBytes[LED_BLUE] |= btnNeutral;
      ledBytes[LED_RED] &= ~btnNeutral;
      ledBytes[LED_GREEN] &= ~btnNeutral;
      break;
    case PARK:
      ledBytes[LED_BLUE] |= btnPark;
      ledBytes[LED_RED] &= ~btnPark;
      ledBytes[LED_GREEN] &= ~btnPark;
      break;
    }

    if (!mtModelDetected && regenDisabled) {
      ledBytes[LED_BLUE] |= btnRegen;
      ledBytes[LED_RED] &= ~btnRegen;
      ledBytes[LED_GREEN] &= ~btnRegen;
    }
  }

  if (mtModelDetected && regenEnabled) {
    ledBytes[LED_BLUE] |= btnRegen;
    ledBytes[LED_RED] &= ~btnRegen;
    ledBytes[LED_GREEN] &= ~btnRegen;
  }

  if (heatReq && heaterPowerActive) {
    ledBytes[LED_BLUE] |= btnHeat;
    ledBytes[LED_RED] &= ~btnHeat;
    ledBytes[LED_GREEN] &= ~btnHeat;
  }

  if (flashRejectedDirection) {
    uint8_t rejectedMask = GearToButtonMask(rejectedDirection, mtModelDetected);
    ledBytes[LED_RED] |= rejectedMask;
    ledBytes[LED_GREEN] &= ~rejectedMask;
    ledBytes[LED_BLUE] &= ~rejectedMask;
  }

  can->Send(PKP_RPDO1, (uint32_t *)ledBytes, 8);

  if (mtModelDetected) {
    const Param::Attributes *regenAttrs = Param::GetAttrib(Param::regenmax);
    const Param::Attributes *heatAttrs = Param::GetAttrib(Param::HeatPwr);
    float maxRegenMagnitude =
        regenAttrs != nullptr ? std::fabs(regenAttrs->min) : 0.0f;
    float maxHeatPower = heatAttrs != nullptr ? heatAttrs->max : 0.0f;
    float regenMagnitude = std::fabs(Param::GetFloat(Param::regenmax));
    float heaterPower = Param::GetFloat(Param::powerheater);
    int regenStep = LevelToStep(regenMagnitude, maxRegenMagnitude);
    int heaterStep = LevelToStep(heaterPower, maxHeatPower);
    uint16_t regenMask = StepToRingMask(regenStep);
    uint16_t heaterMask = StepToRingMask(heaterStep);
    uint8_t ringLedBytes[8] = {0};

    ringLedBytes[0] = regenMask & 0xFF;
    ringLedBytes[1] = regenMask >> 8;
    ringLedBytes[2] = heaterMask & 0xFF;
    ringLedBytes[3] = heaterMask >> 8;

    can->Send(PKP_MT_RPDO2, (uint32_t *)ringLedBytes, 8);
  }
}

bool PKP2300_Lever::GetGear(Shifter::Sgear &outGear) {
  outGear = gear;
  return true;
}
