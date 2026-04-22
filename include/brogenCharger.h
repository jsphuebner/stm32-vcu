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

#ifndef BROGENCHARGER_H
#define BROGENCHARGER_H

/* Control of the Brogen MAS891 On-Board Charger (OBC).
 *
 * All multi-byte signals use Motorola LSB byte order: start bit = LSB
 * position using DBC bit numbering (bit N = byte N/8, bit N%8).
 * Within each byte bits increase normally; at a byte boundary the signal
 * continues in the next-lower byte (big-endian across bytes).
 *
 * CAN messages (all standard 11-bit IDs, DLC=8):
 *
 * VCU sends – 10 ms:
 *   0x09D  BMS_09D   – HV connection status, pack voltage
 *   0x2A3  VCU_2A3   – charge load power, thermal permit, discharge permit
 *
 * VCU sends – 20 ms:
 *   0x11F  VCU_11F   – DCDC control (mode, current/voltage setpoints,
 *                      rolling counter, CRC8)
 *   0x17D  BCM_17D   – power distribution status, e-lock request
 *
 * VCU sends – 100 ms:
 *   0x2C7  BMS_2C7   – charge request (voltage/current setpoints,
 *                      rolling counter, CRC8) – only in charge mode
 *   0x2CA  BMS_2CA   – e-lock request
 *   0x2CE  BMS_2CE   – battery-full flag, DC charge connection status
 *   0x2A5  VCU_2A5   – electric mode flag
 *   0x2A7  VCU_2A7   – fault level, AGS position, drive pump duty
 *   0x2AD  GW_2AD    – vehicle speed (0 km/h while charging)
 *   0x2DF  GW_2DF    – odometer
 *
 * VCU sends – 500 ms:
 *   0x195  IVI_195   – infotainment e-lock request
 *   0x37F  IVI_37F   – date/time
 *   0x28D  TBOX_28D  – telematics OTA mode request
 *
 * VCU sends – 1000 ms:
 *   0x3AA  GW_3AA    – VIN
 *   0x402  BMS_402   – BMS network-management
 *   0x405  VCU_405   – VCU network-management
 *   0x400  GW_400    – gateway network-management
 *   0x401  MCU_401   – motor-controller network-management
 *   0x404  EGS_404   – gearbox-controller network-management
 *
 * ICS sends (100 ms, decoded):
 *   0x2E1  OBC_2E1   – CP status, e-lock state, max DC power
 *   0x2B2  DCDC_2B2  – temperatures (TempM1/Water/PFC/LLC)
 *   0x2E6  OBC_2E6   – actual / maximum DC voltage and current
 *   0x2EB  OBC_2EB   – OBC status, work mode, AC voltage and current
 *   0x2ED  OBC_2ED   – fault / error flags
 */

#include "chargerhw.h"
#include "my_fp.h"
#include "my_math.h"
#include "params.h"
#include <stdint.h>

class brogenCharger : public Chargerhw
{
public:
   void DecodeCAN(int id, uint32_t data[2]);
   void Task10Ms();
   void Task100Ms();
   void Task200Ms();
   bool ControlCharge(bool RunCh, bool ACReq);
   void SetCanInterface(CanHardware *c);

private:
   bool clearToStart = false;
   uint8_t rollingCounter = 0;
   uint8_t rollingCounter11F = 0;
   uint8_t counter20ms = 0;
   uint8_t counter500ms = 0;
   uint8_t counter1000ms = 0;

   static uint8_t  obcStatus;
   static uint8_t  obcWorkMode;
   static float    dcVoltage;
   static float    dcCurrent;
   static float    acVoltage;
   static float    acCurrent;
   static float    tempWater;
   static float    tempM1;
   static float    tempPFC;
   static float    tempLLC;

   static uint8_t  crc8(uint8_t *data, uint8_t len);
   void            sendBMS09D();
   void            sendVCU2A3();
   void            sendVCU11F();
   void            sendBCM17D();
   void            sendBMS2CA();
   void            sendBMS2CE();
   void            sendVCU2A5();
   void            sendVCU2A7();
   void            sendGW2AD();
   void            sendGW2DF();
   void            sendIVI195();
   void            sendIVI37F();
   void            sendTBOX28D();
   void            sendGW3AA();
   void            sendNMMessages();
   static void     handle2E1(uint32_t data[2]);
   static void     handle2B2(uint32_t data[2]);
   static void     handle2E6(uint32_t data[2]);
   static void     handle2EB(uint32_t data[2]);
   static void     handle2ED(uint32_t data[2]);
};

#endif // BROGENCHARGER_H
