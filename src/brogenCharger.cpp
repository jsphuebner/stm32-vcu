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
 * All multi-byte signals use Motorola LSB byte order: start bit = LSB
 * position using DBC bit numbering (bit N = byte N/8, bit N%8).
 * Within each byte bits increase normally; at a byte boundary the signal
 * continues in the next-lower byte (big-endian across bytes).
 *
 * BMS_2C7 signal layout:
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
 *   0x2B2 – temperatures (DCDC_2B2: TempM1/Water/PFC/LLC)
 *   0x2E6 – actual DC voltage and current, max DC voltage and current
 *   0x2EB – OBC status, work mode, AC voltage and current
 *   0x2ED – fault flags (logged, not acted on)
 */

#include "brogenCharger.h"

#define CURR_OFFSET_A   700.0f   // BMS_ChgCurrReq physical offset (A)
#define CURR_FACTOR     0.1f     // BMS_ChgCurrReq scale factor (A/raw LSB)
#define CURR_RAW_MAX    7000u    // raw value for 0 A demand
#define TEMP_OFFSET_C   40       // temperature signal offset (°C)

uint8_t brogenCharger::obcStatus;
uint8_t brogenCharger::obcWorkMode;
float   brogenCharger::dcVoltage;
float   brogenCharger::dcCurrent;
float   brogenCharger::acVoltage;
float   brogenCharger::acCurrent;
float   brogenCharger::tempWater;
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
   can->RegisterUserMessage(0x2B2);
   can->RegisterUserMessage(0x2E6);
   can->RegisterUserMessage(0x2EB);
   can->RegisterUserMessage(0x2ED);
}

// ---------------------------------------------------------------------------
// Task10Ms – transmit 10 ms messages (BMS_09D, VCU_2A3) and 20 ms messages
//            (VCU_11F, BCM_17D every second call)
// ---------------------------------------------------------------------------
void brogenCharger::Task10Ms()
{
   sendBMS09D();
   sendVCU2A3();

   if (++counter20ms >= 2) {
      counter20ms = 0;
      sendVCU11F();
      sendBCM17D();
   }
}

// ---------------------------------------------------------------------------
// Task100Ms – transmit BMS_2C7 (charge mode only) plus all 100 ms and
//             500 ms keepalive messages
// ---------------------------------------------------------------------------
void brogenCharger::Task100Ms()
{
   int opmode = Param::GetInt(Param::opmode);

   // BMS_2C7: charge command – only when actively charging
   if (opmode == MOD_CHARGE) {
      uint8_t bytes[8] = {0, 0, 0, 0, 0, 0, 0, 0};

      // Voltage setpoint (0.1 V / bit, 13-bit)
      float voltF = Param::GetFloat(Param::Voltspnt);
      uint16_t voltRaw = (uint16_t)(voltF * 10.0f);
      if (voltRaw > 0x1FFF)
         voltRaw = 0x1FFF;

      // Current request derived from power setpoint.
      // physical (A) = raw * CURR_FACTOR − CURR_OFFSET_A
      //  → raw = (CURR_OFFSET_A − I_A) / CURR_FACTOR
      // I_A is the positive charging current the VCU wants to draw.
      if (voltF < 1.0f)
         voltF = 1.0f;
      float iReq = Param::GetFloat(Param::Pwrspnt) / voltF;
      if (iReq < 0.0f)
         iReq = 0.0f;
      if (iReq > CURR_OFFSET_A)
         iReq = CURR_OFFSET_A;
      uint16_t currRaw = (uint16_t)((CURR_OFFSET_A - iReq) / CURR_FACTOR);
      if (currRaw > CURR_RAW_MAX)
         currRaw = CURR_RAW_MAX;

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

   // 100 ms keepalive messages (sent regardless of opmode)
   sendBMS2CA();
   sendBMS2CE();
   sendVCU2A5();
   sendVCU2A7();
   sendGW2AD();
   sendGW2DF();

   // 500 ms messages (every 5th 100 ms call)
   if (++counter500ms >= 5) {
      counter500ms = 0;
      sendIVI195();
      sendIVI37F();
      sendTBOX28D();
   }
}

// ---------------------------------------------------------------------------
// Task200Ms – transmit 1000 ms messages (GW_3AA, NM frames every 5th call)
// ---------------------------------------------------------------------------
void brogenCharger::Task200Ms()
{
   if (++counter1000ms >= 5) {
      counter1000ms = 0;
      sendGW3AA();
      sendNMMessages();
   }
}

// ---------------------------------------------------------------------------
// sendBMS09D – BMS_09D: HV connection status + pack voltage (10 ms)
//
// Signal layout (Motorola LSB):
//   Byte 0[7:6]: BMS_HVConnectStt (0=Disconnect, 1=Connecting)
//   Byte 2[7:0]: BMS_PackVoltage[12:5]  (factor=0.1 V)
//   Byte 3[7:3]: BMS_PackVoltage[4:0]
// ---------------------------------------------------------------------------
void brogenCharger::sendBMS09D()
{
   uint8_t bytes[8] = {0, 0, 0, 0, 0, 0, 0, 0};

   int opmode = Param::GetInt(Param::opmode);
   uint8_t hvConnStt = (opmode == MOD_CHARGE) ? 1 : 0;
   bytes[0] = (uint8_t)((hvConnStt & 0x03) << 6);

   float udc = Param::GetFloat(Param::udc);
   uint16_t voltRaw = (uint16_t)(udc * 10.0f);
   if (voltRaw > 0x1FFF) voltRaw = 0x1FFF;
   bytes[2] = (uint8_t)(voltRaw >> 5);
   bytes[3] = (uint8_t)((voltRaw & 0x1F) << 3);

   can->Send(0x09D, (uint32_t *)bytes, 8);
}

// ---------------------------------------------------------------------------
// sendVCU2A3 – VCU_2A3: charge load power, thermal permit,
//              OBC discharge permit (10 ms)
//
// Signal layout (Motorola LSB):
//   Byte 0[7:0]: VCU_ChargeLoadPower     (factor=0.1, sent as 0)
//   Byte 1[2:1]: VCU_BMSPreThemalPermit  (0=Default)
//   Byte 5[7:0]: VCU_OBCDischgPermitPower (factor=0.1, sent as 0)
// ---------------------------------------------------------------------------
void brogenCharger::sendVCU2A3()
{
   uint8_t bytes[8] = {0, 0, 0, 0, 0, 0, 0, 0};
   can->Send(0x2A3, (uint32_t *)bytes, 8);
}

// ---------------------------------------------------------------------------
// sendVCU11F – VCU_11F: DCDC control (20 ms, rolling counter + CRC8)
//
// Signal layout (Motorola LSB):
//   Byte 1[6:0]: VCU_DC_IdcHvMaxSetP[10:4] (factor=0.0625, offset=−64 A)
//   Byte 2[7:4]: VCU_DC_IdcHvMaxSetP[3:0]
//   Byte 2[3:0]: VCU_DC_IdcLvMaxSetP[10:7] (factor=1,      offset=−1023 A)
//   Byte 3[7:1]: VCU_DC_IdcLvMaxSetP[6:0]
//   Byte 3[0]:   VCU_DC_UdcLvSetP[8]        (factor=0.125 V)
//   Byte 4[7:0]: VCU_DC_UdcLvSetP[7:0]
//   Byte 5[7:5]: VCU_DC_stModereq           (0=Standby)
//   Byte 6[3:0]: VCU_RollingCounter_11F
//   Byte 7[7:0]: VCU_Checksum_11F           (CRC8 over bytes 0–6)
// ---------------------------------------------------------------------------
void brogenCharger::sendVCU11F()
{
   uint8_t bytes[8] = {0, 0, 0, 0, 0, 0, 0, 0};

   // IdcHvMaxSetP: raw=2047 → 63.9375 A (no HV current limit imposed)
   const uint16_t idcHvRaw = 2047;
   bytes[1]  = (uint8_t)((idcHvRaw >> 4) & 0x7F);   // raw[10:4] → byte1[6:0]
   bytes[2]  = (uint8_t)((idcHvRaw & 0x0F) << 4);   // raw[3:0]  → byte2[7:4]

   // IdcLvMaxSetP: raw=1023 → 0 A (LV current = 0, Standby)
   const uint16_t idcLvRaw = 1023;
   bytes[2] |= (uint8_t)((idcLvRaw >> 7) & 0x0F);   // raw[10:7] → byte2[3:0]
   bytes[3]  = (uint8_t)((idcLvRaw & 0x7F) << 1);   // raw[6:0]  → byte3[7:1]

   // UdcLvSetP: factor=0.125 V/bit, 9-bit value → raw = DCSetPnt / 0.125
   uint16_t udcLvRaw = (uint16_t)(Param::GetFloat(Param::DCSetPnt) / 0.125f);
   if (udcLvRaw > 0x1FF) udcLvRaw = 0x1FF;
   bytes[3] |= (uint8_t)((udcLvRaw >> 8) & 0x01);   // raw[8]    → byte3[0]
   bytes[4]  = (uint8_t)(udcLvRaw & 0xFF);           // raw[7:0]  → byte4

   // stModereq: 0 = Standby (bytes[5] bits[7:5] already 0)

   bytes[6] = rollingCounter11F & 0x0F;
   bytes[7] = crc8(bytes, 7);

   can->Send(0x11F, (uint32_t *)bytes, 8);
   rollingCounter11F = (rollingCounter11F + 1) & 0x0F;
}

// ---------------------------------------------------------------------------
// sendBCM17D – BCM_17D: power distribution status, e-lock request (20 ms)
//
// Signal layout (Motorola LSB):
//   Byte 0[2:0]: BCM_PwrDistributionSts (2=ACC)
//   Byte 2[2]:   BCM_Elock_open_Req     (0=no request)
// ---------------------------------------------------------------------------
void brogenCharger::sendBCM17D()
{
   uint8_t bytes[8] = {0, 0, 0, 0, 0, 0, 0, 0};
   bytes[0] = 2;  // ACC
   can->Send(0x17D, (uint32_t *)bytes, 8);
}

// ---------------------------------------------------------------------------
// sendBMS2CA – BMS_2CA: e-lock request (100 ms)
//
//   Byte 6[7:6]: BMS_ElockRequest (0=no request)
// ---------------------------------------------------------------------------
void brogenCharger::sendBMS2CA()
{
   uint8_t bytes[8] = {0, 0, 0, 0, 0, 0, 0, 0};
   can->Send(0x2CA, (uint32_t *)bytes, 8);
}

// ---------------------------------------------------------------------------
// sendBMS2CE – BMS_2CE: battery-full flag, DC charge connection (100 ms)
//
//   Byte 3[2]:   BMS_Batt_Full      (0=not full)
//   Byte 7[1:0]: BMS_DCChgConnectSt (0=disconnect)
// ---------------------------------------------------------------------------
void brogenCharger::sendBMS2CE()
{
   uint8_t bytes[8] = {0, 0, 0, 0, 0, 0, 0, 0};
   can->Send(0x2CE, (uint32_t *)bytes, 8);
}

// ---------------------------------------------------------------------------
// sendVCU2A5 – VCU_2A5: electric mode flag (100 ms)
//
//   Byte 0[4:2]: VCU_ElecModFlg (3=Vehicle ready while charging, 1=LV on)
// ---------------------------------------------------------------------------
void brogenCharger::sendVCU2A5()
{
   uint8_t bytes[8] = {0, 0, 0, 0, 0, 0, 0, 0};
   int opmode = Param::GetInt(Param::opmode);
   uint8_t modFlg = (opmode == MOD_CHARGE) ? 3 : 1;
   bytes[0] = (uint8_t)((modFlg & 0x07) << 2);
   can->Send(0x2A5, (uint32_t *)bytes, 8);
}

// ---------------------------------------------------------------------------
// sendVCU2A7 – VCU_2A7: fault level, AGS position, drive pump duty (100 ms)
//
//   Byte 0[7:5]: VCU_FaultLvl       (0=no fault)
//   Byte 4[7:0]: VCU_AGSActPosition (0 %)
//   Byte 6[7:4]: VCU_DrvPumpDC      (0 %)
// ---------------------------------------------------------------------------
void brogenCharger::sendVCU2A7()
{
   uint8_t bytes[8] = {0, 0, 0, 0, 0, 0, 0, 0};
   can->Send(0x2A7, (uint32_t *)bytes, 8);
}

// ---------------------------------------------------------------------------
// sendGW2AD – GW_2AD: vehicle speed (100 ms)
//
//   Byte 0[7:0]: ESP_VehicleSpeed[12:5]  (0 km/h)
//   Byte 1[7:3]: ESP_VehicleSpeed[4:0]   (0 km/h)
//   Byte 1[1]:   ESP_VehicleSpeedValid   (1=valid)
// ---------------------------------------------------------------------------
void brogenCharger::sendGW2AD()
{
   uint8_t bytes[8] = {0, 0, 0, 0, 0, 0, 0, 0};
   bytes[1] = 0x02;  // VehicleSpeedValid=1, speed bits=0
   can->Send(0x2AD, (uint32_t *)bytes, 8);
}

// ---------------------------------------------------------------------------
// sendGW2DF – GW_2DF: odometer (100 ms)
//
//   Bytes 0–2: IP_TotalOdometer (0 km, 24-bit big-endian)
//   Byte 5[7]: IP_OdometerSts   (1=valid)
// ---------------------------------------------------------------------------
void brogenCharger::sendGW2DF()
{
   uint8_t bytes[8] = {0, 0, 0, 0, 0, 0, 0, 0};
   bytes[5] = 0x80;  // OdometerSts=1
   can->Send(0x2DF, (uint32_t *)bytes, 8);
}

// ---------------------------------------------------------------------------
// sendIVI195 – IVI_195: infotainment e-lock request (500 ms)
//
//   Byte 1[5:4]: IVI_Elock_open_Req (0=no request)
// ---------------------------------------------------------------------------
void brogenCharger::sendIVI195()
{
   uint8_t bytes[8] = {0, 0, 0, 0, 0, 0, 0, 0};
   can->Send(0x195, (uint32_t *)bytes, 8);
}

// ---------------------------------------------------------------------------
// sendIVI37F – IVI_37F: date/time (500 ms)
//
//   Byte 0[5:0]: IVI_YearSet    (offset 2018; 6 = year 2024)
//   Byte 1[3:0]: IVI_MonthSet   (1 = January)
//   Byte 2[4:0]: IVI_DaySet     (1 = 1st)
//   Byte 3[4:0]: IVI_HourSet    (0)
//   Byte 4[5:0]: IVI_MinuteSet  (0)
//   Byte 5[5:0]: IVI_SecondSet  (0)
// ---------------------------------------------------------------------------
void brogenCharger::sendIVI37F()
{
   uint8_t bytes[8] = {0, 0, 0, 0, 0, 0, 0, 0};
   bytes[0] = 6;  // 2024 − 2018 = 6
   bytes[1] = 1;  // January
   bytes[2] = 1;  // 1st
   can->Send(0x37F, (uint32_t *)bytes, 8);
}

// ---------------------------------------------------------------------------
// sendTBOX28D – TBOX_28D: telematics OTA mode request (500 ms, all zeros)
// ---------------------------------------------------------------------------
void brogenCharger::sendTBOX28D()
{
   uint8_t bytes[8] = {0, 0, 0, 0, 0, 0, 0, 0};
   can->Send(0x28D, (uint32_t *)bytes, 8);
}

// ---------------------------------------------------------------------------
// sendGW3AA – GW_3AA: VIN (1000 ms)
//
//   Bytes 0–6: GW_VIN_Value_1–7 (sent as 0x00)
//   Byte 7:    GW_VINFrameNumber (1)
// ---------------------------------------------------------------------------
void brogenCharger::sendGW3AA()
{
   uint8_t bytes[8] = {0, 0, 0, 0, 0, 0, 0, 1};
   can->Send(0x3AA, (uint32_t *)bytes, 8);
}

// ---------------------------------------------------------------------------
// sendNMMessages – Network Management frames (1000 ms)
//
// Minimal AUTOSAR-NM payload:
//   Byte 0:    SourceID
//   Byte 2[5]: NormalFromReady = 1
//   Byte 3[4:0]: WakeSource
// ---------------------------------------------------------------------------
void brogenCharger::sendNMMessages()
{
   // BMS_402
   uint8_t bms402[8] = {0x01, 0, 0x20, 0x01, 0, 0, 0, 0};
   can->Send(0x402, (uint32_t *)bms402, 8);

   // VCU_405 (Charge wakeup = 0x04)
   uint8_t vcu405[8] = {0x05, 0, 0x20, 0x04, 0, 0, 0, 0};
   can->Send(0x405, (uint32_t *)vcu405, 8);

   // GW_400 (WakeSource_CAN = 0x02 = PT CAN, byte 7[2:0])
   uint8_t gw400[8] = {0x00, 0, 0x20, 0x01, 0, 0, 0, 0x02};
   can->Send(0x400, (uint32_t *)gw400, 8);

   // MCU_401
   uint8_t mcu401[8] = {0x02, 0, 0x20, 0x01, 0, 0, 0, 0};
   can->Send(0x401, (uint32_t *)mcu401, 8);

   // EGS_404
   uint8_t egs404[8] = {0x04, 0, 0x20, 0x01, 0, 0, 0, 0};
   can->Send(0x404, (uint32_t *)egs404, 8);
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
   case 0x2B2:
      handle2B2(data);
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
// handle2B2 – DCDC_2B2: temperatures (all byte-aligned, offset -40 °C)
//   Byte 0: DCDC_TempM1   (start=0)
//   Byte 1: (unused gap)
//   Byte 2: DCDC_TempWater (start=16)
//   Byte 3: DCDC_TempPFC   (start=24)
//   Byte 4: DCDC_TempLLC   (start=32)
// ---------------------------------------------------------------------------
void brogenCharger::handle2B2(uint32_t data[2])
{
   uint8_t *bytes = (uint8_t *)data;
   tempWater = (int8_t)bytes[2] - TEMP_OFFSET_C;
   tempM1    = (int8_t)bytes[0] - TEMP_OFFSET_C;
   tempPFC   = (int8_t)bytes[3] - TEMP_OFFSET_C;
   tempLLC   = (int8_t)bytes[4] - TEMP_OFFSET_C;

   float maxTemp = tempM1;
   if (tempWater > maxTemp) maxTemp = tempWater;
   if (tempPFC   > maxTemp) maxTemp = tempPFC;
   if (tempLLC   > maxTemp) maxTemp = tempLLC;
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
