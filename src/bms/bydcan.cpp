#include "bms/bydcan.h"
#include "params.h"
#include <math.h>

namespace {
static uCAN_MSG txMessage;

static uint16_t ClampU16(int value) {
  if (value < 0)
    return 0;
  if (value > 0xFFFF)
    return 0xFFFF;
  return (uint16_t)value;
}

static int16_t ClampS16(int value) {
  if (value < -32768)
    return -32768;
  if (value > 32767)
    return 32767;
  return (int16_t)value;
}
} // namespace

bool BydCan::Enabled() const { return Param::GetBool(Param::BydCanEnable); }

void BydCan::ResetState() {
  inverterStartedUp = false;
  initialDataSent = false;
  initialSequencePending = false;
  cyclic2sPending = false;
  cyclic10sPending = false;
  cyclic60sPending = false;
  initialSequenceIndex = 0;
  cyclic10sIndex = 0;
  tick2s = 0;
  tick10s = 0;
  tick60s = 0;
}

void BydCan::SendFrame(uint16_t id, const uint8_t data[8]) {
  txMessage.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
  txMessage.frame.id = id;
  txMessage.frame.dlc = 8;
  txMessage.frame.data0 = data[0];
  txMessage.frame.data1 = data[1];
  txMessage.frame.data2 = data[2];
  txMessage.frame.data3 = data[3];
  txMessage.frame.data4 = data[4];
  txMessage.frame.data5 = data[5];
  txMessage.frame.data6 = data[6];
  txMessage.frame.data7 = data[7];
  CANSPI_Transmit(&txMessage);
}

bool BydCan::SendInitialSequenceFrame() {
  if (!initialSequencePending)
    return false;

  uint16_t id = 0;
  uint8_t data[8] = {0};

  switch (initialSequenceIndex) {
  case 0: {
    id = 0x250;
    const uint16_t totalCapacity =
        ClampU16((int)lroundf(Param::GetFloat(Param::BattCap) * 10.0f));
    data[0] = 0x03;
    data[1] = 0x29;
    data[2] = 0x00;
    data[3] = 0x66;
    data[4] = totalCapacity >> 8;
    data[5] = totalCapacity & 0xFF;
    data[6] = 0x02;
    data[7] = 0x09;
    break;
  }
  case 1:
    id = 0x290;
    data[0] = 0x06;
    data[1] = 0x37;
    data[2] = 0x10;
    data[3] = 0xD9;
    break;
  case 2:
    id = 0x2D0;
    data[1] = 0x42;
    data[2] = 0x59;
    data[3] = 0x44;
    break;
  case 3:
    id = 0x3D0;
    data[1] = 0x42;
    data[2] = 0x61;
    data[3] = 0x74;
    data[4] = 0x74;
    data[5] = 0x65;
    data[6] = 0x72;
    data[7] = 0x79;
    break;
  case 4:
    id = 0x3D0;
    data[0] = 0x01;
    data[1] = 0x2D;
    data[2] = 0x42;
    data[3] = 0x6F;
    data[4] = 0x78;
    data[5] = 0x20;
    data[6] = 0x50;
    data[7] = 0x72;
    break;
  case 5:
    id = 0x3D0;
    data[0] = 0x02;
    data[1] = 0x65;
    data[2] = 0x6D;
    data[3] = 0x69;
    data[4] = 0x75;
    data[5] = 0x6D;
    data[6] = 0x20;
    data[7] = 0x48;
    break;
  default:
    id = 0x3D0;
    data[0] = 0x03;
    data[1] = 0x56;
    data[2] = 0x53;
    break;
  }

  SendFrame(id, data);
  initialSequenceIndex++;
  if (initialSequenceIndex >= 7) {
    initialSequencePending = false;
    initialSequenceIndex = 0;
    initialDataSent = true;
  }
  return true;
}

bool BydCan::SendCyclicFrame() {
  float packVoltage = Param::GetFloat(Param::udc2);
  if (packVoltage < 10.0f)
    packVoltage = Param::GetFloat(Param::udc);
  if (packVoltage < 10.0f)
    packVoltage = 400.0f;

  const int chargeVoltage =
      ClampU16((int)lroundf(Param::GetFloat(Param::udclim) * 10.0f));
  const int dischargeVoltage =
      ClampU16((int)lroundf(Param::GetFloat(Param::udcmin) * 10.0f));
  const int maxDischargeCurrent =
      ClampU16((int)lroundf(fabsf(Param::GetFloat(Param::idcmin)) * 10.0f));
  const int maxChargeCurrent =
      ClampU16((int)lroundf(Param::GetFloat(Param::BMS_ChargeLim) * 10.0f));

  uint8_t msg110[8] = {0};
  msg110[0] = (chargeVoltage >> 8) & 0xFF;
  msg110[1] = chargeVoltage & 0xFF;
  msg110[2] = (dischargeVoltage >> 8) & 0xFF;
  msg110[3] = dischargeVoltage & 0xFF;
  msg110[4] = (maxDischargeCurrent >> 8) & 0xFF;
  msg110[5] = maxDischargeCurrent & 0xFF;
  msg110[6] = (maxChargeCurrent >> 8) & 0xFF;
  msg110[7] = maxChargeCurrent & 0xFF;

  float socPercent = Param::GetFloat(Param::SOC);
  if (socPercent < 0.0f)
    socPercent = 0.0f;
  if (socPercent > 100.0f)
    socPercent = 100.0f;
  const uint16_t soc = ClampU16((int)lroundf(socPercent * 100.0f));

  const float packCapacityWh = Param::GetFloat(Param::BattCap) * 1000.0f;
  const float fullCapacityAh = packCapacityWh / packVoltage;
  const uint16_t fullCapacityAh10 =
      ClampU16((int)lroundf(fullCapacityAh * 10.0f));
  const uint16_t remCapacityAh10 =
      ClampU16((int)lroundf(fullCapacityAh * (socPercent / 10.0f)));

  uint8_t msg150[8] = {0};
  msg150[0] = soc >> 8;
  msg150[1] = soc & 0xFF;
  msg150[2] = 10000 >> 8;
  msg150[3] = 10000 & 0xFF;
  msg150[4] = remCapacityAh10 >> 8;
  msg150[5] = remCapacityAh10 & 0xFF;
  msg150[6] = fullCapacityAh10 >> 8;
  msg150[7] = fullCapacityAh10 & 0xFF;

  const int voltageDV =
      ClampU16((int)lroundf(Param::GetFloat(Param::udc) * 10.0f));
  const int currentDA =
      ClampS16((int)lroundf(Param::GetFloat(Param::idc) * 10.0f));
  const float bmsTMin = Param::GetFloat(Param::BMS_Tmin);
  const float bmsTMax = Param::GetFloat(Param::BMS_Tmax);
  const int tempAvg = ClampS16((int)lroundf((bmsTMin + bmsTMax) * 5.0f));

  uint8_t msg1D0[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x08};
  msg1D0[0] = (voltageDV >> 8) & 0xFF;
  msg1D0[1] = voltageDV & 0xFF;
  msg1D0[2] = (currentDA >> 8) & 0xFF;
  msg1D0[3] = currentDA & 0xFF;
  msg1D0[4] = (tempAvg >> 8) & 0xFF;
  msg1D0[5] = tempAvg & 0xFF;

  const int tempMax = ClampS16((int)lroundf(bmsTMax * 10.0f));
  const int tempMin = ClampS16((int)lroundf(bmsTMin * 10.0f));
  uint8_t msg210[8] = {0};
  msg210[0] = (tempMax >> 8) & 0xFF;
  msg210[1] = tempMax & 0xFF;
  msg210[2] = (tempMin >> 8) & 0xFF;
  msg210[3] = tempMin & 0xFF;

  const uint8_t msg190[8] = {0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00};

  if (cyclic60sPending) {
    cyclic60sPending = false;
    SendFrame(0x190, msg190);
    return true;
  }

  if (cyclic10sPending) {
    switch (cyclic10sIndex) {
    case 0:
      SendFrame(0x150, msg150);
      break;
    case 1:
      SendFrame(0x1D0, msg1D0);
      break;
    default:
      SendFrame(0x210, msg210);
      break;
    }
    cyclic10sIndex++;
    if (cyclic10sIndex >= 3) {
      cyclic10sPending = false;
      cyclic10sIndex = 0;
    }
    return true;
  }

  if (cyclic2sPending) {
    cyclic2sPending = false;
    SendFrame(0x110, msg110);
    return true;
  }

  return false;
}

void BydCan::Task100Ms() {
  if (!Enabled()) {
    ResetState();
    return;
  }

  if (!inverterStartedUp)
    return;

  tick2s++;
  tick10s++;
  tick60s++;

  if (!initialDataSent)
    initialSequencePending = true;

  if (tick2s >= 20) {
    tick2s = 0;
    cyclic2sPending = true;
  }
  if (tick10s >= 100) {
    tick10s = 0;
    cyclic10sPending = true;
    cyclic10sIndex = 0;
  }
  if (tick60s >= 600) {
    tick60s = 0;
    cyclic60sPending = true;
  }

  if (SendInitialSequenceFrame())
    return;
  SendCyclicFrame();
}

void BydCan::DecodeCAN3(const uCAN_MSG &rxMessage) {
  if (!Enabled())
    return;

  switch (rxMessage.frame.id) {
  case 0x151:
    inverterStartedUp = true;
    if (rxMessage.frame.data0 & 0x01) {
      initialDataSent = false;
      initialSequencePending = true;
      initialSequenceIndex = 0;
    }
    break;
  case 0x091:
  case 0x0D1:
  case 0x111:
  case 0x191:
    inverterStartedUp = true;
    break;
  default:
    break;
  }
}
