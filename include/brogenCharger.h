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
 * CAN messages (all standard 11-bit IDs, DLC=8):
 *
 * VCU sends (100 ms):
 *   0x2C7  BMS_2C7  – charge request with voltage/current setpoints,
 *                     rolling counter and CRC8 checksum.
 *
 * OBC sends (100 ms):
 *   0x2E1  OBC_2E1  – CP status, e-lock state, max DC power
 *   0x2E2  OBC_2E2  – temperatures (air, M1, PFC, LLC, water)
 *   0x2E6  OBC_2E6  – actual / maximum DC voltage and current
 *   0x2EB  OBC_2EB  – OBC status, work mode, AC voltage and current
 *   0x2ED  OBC_2ED  – fault / error flags
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
   void Task100Ms();
   bool ControlCharge(bool RunCh, bool ACReq);
   void SetCanInterface(CanHardware *c);

private:
   bool clearToStart = false;
   uint8_t rollingCounter = 0;

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
   static void     handle2E1(uint32_t data[2]);
   static void     handle2B2(uint32_t data[2]);
   static void     handle2E6(uint32_t data[2]);
   static void     handle2EB(uint32_t data[2]);
   static void     handle2ED(uint32_t data[2]);
};

#endif // BROGENCHARGER_H
