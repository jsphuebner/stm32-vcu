/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2024  Johannes Huebner <dev@johanneshuebner.com>
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
 */

/* Control of the Brogen MAS891 On-Board Charger (OBC).
 *
 * Transmit message BMS_2C7 (0x2C7, 100 ms, DLC=8):
 *
 * All multi-byte signals use Motorola byte order (big-endian).
 * "start bit" is the position of the LSB using the DBC convention where
 * bit N = byte (N/8), bit (N%8)  with bit 0 = LSB of byte 0.
 *
 * Signal layout:
 *   Byte 0 : bit 7 = BMS_OnChrgCmd (1 = charge on)
 *   Byte 1 : bits [4:0] = BMS_MaxChgVolt[12:8]
 *   Byte 2 : BMS_MaxChgVolt[7:0]          (factor 0.1 V, range 0–500 V)
 *   Byte 3 : BMS_ChargeVoltage[12:5]
 *   Byte 4 : bits [7:3] = BMS_ChargeVoltage[4:0]
 *            bits [2:0] = BMS_ChgCurrReq[12:10]
 *   Byte 5 : BMS_ChgCurrReq[9:2]          (factor 0.1 A, offset −700 A)
 *   Byte 6 : bits [7:6] = BMS_ChgCurrReq[1:0]
 *            bit  5     = BMS_ObcReadyReq
 *            bit  4     = BMS_OnDischrgCmd
 *            bits [3:0] = BMS_RollingCounter (0–15)
 *   Byte 7 : CRC8 (poly 0x1D, init 0xFF, inverted, over bytes 0–6)
 *
 * Receive messages decoded:
 *   0x2E1 – CP status, e-lock, max DC power
 *   0x2E2 – temperatures
 *   0x2E6 – actual DC voltage and current, max DC voltage and current
 *   0x2EB – OBC status, work mode, AC voltage and current
 *   0x2ED – fault flags (logged, not acted on)
 */

#include "brogenCharger.h"

uint8_t brogenCharger::obcStatus;
uint8_t brogenCharger::obcWorkMode;
float   brogenCharger::dcVoltage;
float   brogenCharger::dcCurrent;
float   brogenCharger::acVoltage;
float   brogenCharger::acCurrent;
float   brogenCharger::tempAir;
float   brogenCharger::tempM1;
float   brogenCharger::tempPFC;
float   brogenCharger::tempLLC;

// ---------------------------------------------------------------------------
// CRC8: poly = 0x1D, init = 0xFF, return bitwise-NOT of result
// ---------------------------------------------------------------------------
uint8_t brogenCharger::crc8(uint8_t *data, uint8_t len)
{
   uint8_t crc = 0xFF;
   for (uint8_t i = 0; i < len; i++) {
      crc ^= data[i];
      for (uint8_t j = 0; j < 8; j++) {
         if (crc & 0x80)
            crc = (crc << 1) ^ 0x1D;
         else
            crc = (crc << 1);
      }
   }
   return ~crc;
}

// ---------------------------------------------------------------------------
// ControlCharge – mirrors the pattern used by other charger modules
// ---------------------------------------------------------------------------
bool brogenCharger::ControlCharge(bool RunCh, bool ACReq)
{
   int chgmode = Param::GetInt(Param::interface);
   switch (chgmode) {
   case Unused:
      if (RunCh && ACReq) {
         clearToStart = true;
         return true;
      }
      clearToStart = false;
      return false;

   case i3LIM:
   case CPC:
   case Foccci:
      if (RunCh && ACReq) {
         clearToStart = true;
         return true;
      }
      clearToStart = false;
      return false;

   case Chademo:
      if (RunCh && ACReq) {
         clearToStart = true;
         return true;
      }
      clearToStart = false;
      return false;
   }
   return false;
}

// ---------------------------------------------------------------------------
// SetCanInterface – register OBC receive messages
// ---------------------------------------------------------------------------
void brogenCharger::SetCanInterface(CanHardware *c)
{
   can = c;
   can->RegisterUserMessage(0x2E1);
   can->RegisterUserMessage(0x2E2);
   can->RegisterUserMessage(0x2E6);
   can->RegisterUserMessage(0x2EB);
   can->RegisterUserMessage(0x2ED);
}

// ---------------------------------------------------------------------------
// Task100Ms – build and transmit BMS_2C7
// ---------------------------------------------------------------------------
void brogenCharger::Task100Ms()
{
   int opmode = Param::GetInt(Param::opmode);
   if (opmode != MOD_CHARGE)
      return;

   uint8_t bytes[8] = {0, 0, 0, 0, 0, 0, 0, 0};

   // Voltage setpoint (0.1 V / bit, 13-bit)
   uint16_t voltRaw = (uint16_t)(Param::GetFloat(Param::Voltspnt) * 10.0f);
   if (voltRaw > 0x1FFF)
      voltRaw = 0x1FFF;

   // Current request derived from power setpoint.
   // physical (A) = raw * 0.1 − 700  →  raw = (700 − I_A) * 10
   // I_A is the positive charging current the VCU wants to draw.
   float voltF = Param::GetFloat(Param::Voltspnt);
   if (voltF < 1.0f)
      voltF = 1.0f;
   float iReq = Param::GetFloat(Param::Pwrspnt) / voltF;
   if (iReq < 0.0f)
      iReq = 0.0f;
   if (iReq > 700.0f)
      iReq = 700.0f;
   uint16_t currRaw = (uint16_t)((700.0f - iReq) * 10.0f);
   if (currRaw > 7000)
      currRaw = 7000;

   uint8_t onChrgCmd = clearToStart ? 1 : 0;

   // Byte 0: OnChrgCmd at bit 7
   bytes[0] = (uint8_t)(onChrgCmd << 7);

   // Bytes 1-2: MaxChgVolt (same as ChargeVoltage for this implementation)
   bytes[1] = (uint8_t)(voltRaw >> 8) & 0x1F;
   bytes[2] = (uint8_t)(voltRaw & 0xFF);

   // Bytes 3-4 upper: ChargeVoltage
   bytes[3] = (uint8_t)(voltRaw >> 5);
   bytes[4] = (uint8_t)((voltRaw & 0x1F) << 3);

   // Bytes 4 lower - 6 upper: ChgCurrReq
   bytes[4] |= (uint8_t)((currRaw >> 10) & 0x07);
   bytes[5]  = (uint8_t)((currRaw >> 2) & 0xFF);
   bytes[6]  = (uint8_t)(((currRaw & 0x03) << 6) |
                         (onChrgCmd << 5) |         // ObcReadyReq = OnChrgCmd
                         (rollingCounter & 0x0F));

   bytes[7] = crc8(bytes, 7);

   can->Send(0x2C7, (uint32_t *)bytes, 8);

   rollingCounter = (rollingCounter + 1) & 0x0F;
}

// ---------------------------------------------------------------------------
// DecodeCAN – dispatch incoming OBC messages
// ---------------------------------------------------------------------------
void brogenCharger::DecodeCAN(int id, uint32_t data[2])
{
   switch (id) {
   case 0x2E1:
      handle2E1(data);
      break;
   case 0x2E2:
      handle2E2(data);
      break;
   case 0x2E6:
      handle2E6(data);
      break;
   case 0x2EB:
      handle2EB(data);
      break;
   case 0x2ED:
      handle2ED(data);
      break;
   }
}

// ---------------------------------------------------------------------------
// handle2E1 – OBC_2E1: CP status, e-lock, max DC power
// ---------------------------------------------------------------------------
void brogenCharger::handle2E1(uint32_t data[2])
{
   // No parameters to update from this message in the current implementation.
   (void)data;
}

// ---------------------------------------------------------------------------
// handle2E2 – OBC_2E2: temperatures (all byte-aligned, offset -40 °C)
//   Byte 0: OBC_AirTemp
//   Byte 1: OBC_M1Temp
//   Byte 2: OBC_PFCTemp
//   Byte 3: OBC_LLCTemp
// ---------------------------------------------------------------------------
void brogenCharger::handle2E2(uint32_t data[2])
{
   uint8_t *bytes = (uint8_t *)data;
   tempAir = (int8_t)bytes[0] - 40;
   tempM1  = (int8_t)bytes[1] - 40;
   tempPFC = (int8_t)bytes[2] - 40;
   tempLLC = (int8_t)bytes[3] - 40;

   float maxTemp = tempM1;
   if (tempPFC > maxTemp) maxTemp = tempPFC;
   if (tempLLC > maxTemp) maxTemp = tempLLC;
   Param::SetFloat(Param::ChgTemp, maxTemp);
}

// ---------------------------------------------------------------------------
// handle2E6 – OBC_2E6: DC voltage and current feedback
//
// Signal layout (Motorola byte order, bit N = byte N/8, bit N%8):
//   Byte 0      : OBC_Ecy         (efficiency, 0.5 %/bit)
//   Byte 1      : OBC_HV_DC_MAXCurr (0.2 A/bit)
//   Byte 2      : OBC_HV_DC_MAXVolt (2 V/bit)
//   Byte 3 + B4[7:6]: OBC_HV_DC_Curr (10 bits, 0.1 A/bit, offset −50 A)
//   B4[5:0] + Byte 5 + B6[7]: OBC_HV_DC_Volt (15 bits, 0.02 V/bit)
//   Byte 6[3:0] : OBC_RollingCounter_2E6
//   Byte 7      : OBC_Checksum_2E6
// ---------------------------------------------------------------------------
void brogenCharger::handle2E6(uint32_t data[2])
{
   uint8_t *bytes = (uint8_t *)data;

   // DC current: start=38 (byte4, bit6), len=10
   uint16_t currRaw = (uint16_t)(((uint16_t)bytes[3] << 2) | (bytes[4] >> 6)) & 0x03FF;
   dcCurrent = (float)currRaw * 0.1f - 50.0f;

   // DC voltage: start=55 (byte6, bit7), len=15
   uint32_t voltRaw = ((uint32_t)(bytes[4] & 0x3F) << 9) |
                      ((uint32_t)bytes[5] << 1) |
                      (bytes[6] >> 7);
   dcVoltage = (float)voltRaw * 0.02f;

   Param::SetFloat(Param::udc, dcVoltage);
}

// ---------------------------------------------------------------------------
// handle2EB – OBC_2EB: OBC status, work mode, AC voltage and current
//
//   Byte 0[7:4] : OBC_Status
//   Byte 0[3:2] : OBC_WorkMode
//   Byte 1      : OBC_AC_Current (0.2 A/bit)
//   Byte 2      : OBC_AC_Voltage (2 V/bit)
// ---------------------------------------------------------------------------
void brogenCharger::handle2EB(uint32_t data[2])
{
   uint8_t *bytes = (uint8_t *)data;
   obcStatus   = bytes[0] >> 4;
   obcWorkMode = (bytes[0] >> 2) & 0x03;
   acCurrent   = (float)bytes[1] * 0.2f;
   acVoltage   = (float)bytes[2] * 2.0f;

   Param::SetFloat(Param::AC_Amps,  acCurrent);
   Param::SetFloat(Param::AC_Volts, acVoltage);
}

// ---------------------------------------------------------------------------
// handle2ED – OBC_2ED: fault/error flags (reserved for future use)
// ---------------------------------------------------------------------------
void brogenCharger::handle2ED(uint32_t data[2])
{
   (void)data;
}
