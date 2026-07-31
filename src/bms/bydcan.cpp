#include "bms/bydcan.h"
#include "params.h"
#include <math.h>

namespace {
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

void BydCan::SendFrame(uint16_t id, const uint8_t data[8]) {
  uCAN_MSG txMessage;
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

void BydCan::SendInitialData() {
  uint8_t msg250[8] = {0x03, 0x29, 0x00, 0x66, 0x00, 0x00, 0x02, 0x09};
  uint16_t totalCapacity =
      ClampU16((int)lroundf(Param::GetFloat(Param::BattCap) * 10.0f));
  msg250[4] = totalCapacity >> 8;
  msg250[5] = totalCapacity & 0xFF;

  const uint8_t msg290[8] = {0x06, 0x37, 0x10, 0xD9, 0x00, 0x00, 0x00, 0x00};
  const uint8_t msg2D0[8] = {0x00, 0x42, 0x59, 0x44, 0x00, 0x00, 0x00, 0x00};
  const uint8_t msg3D0_0[8] = {0x00, 0x42, 0x61, 0x74, 0x74, 0x65, 0x72, 0x79};
  const uint8_t msg3D0_1[8] = {0x01, 0x2D, 0x42, 0x6F, 0x78, 0x20, 0x50, 0x72};
  const uint8_t msg3D0_2[8] = {0x02, 0x65, 0x6D, 0x69, 0x75, 0x6D, 0x20, 0x48};
  const uint8_t msg3D0_3[8] = {0x03, 0x56, 0x53, 0x00, 0x00, 0x00, 0x00, 0x00};

  SendFrame(0x250, msg250);
  SendFrame(0x290, msg290);
  SendFrame(0x2D0, msg2D0);
  SendFrame(0x3D0, msg3D0_0);
  SendFrame(0x3D0, msg3D0_1);
  SendFrame(0x3D0, msg3D0_2);
  SendFrame(0x3D0, msg3D0_3);
}

void BydCan::UpdateFrames() {
  float packVoltage = Param::GetFloat(Param::udc2);
  if (packVoltage < 10.0f)
    packVoltage = Param::GetFloat(Param::udc);
  if (packVoltage < 10.0f)
    packVoltage = 400.0f;

  const int chargeVoltage = ClampU16((int)lroundf(Param::GetFloat(Param::udclim) * 10.0f));
  const int dischargeVoltage = ClampU16((int)lroundf(Param::GetFloat(Param::udcmin) * 10.0f));
  const int maxDischargeCurrent = ClampU16((int)lroundf(fabsf(Param::GetFloat(Param::idcmin)) * 10.0f));
  const int maxChargeCurrent = ClampU16((int)lroundf(Param::GetFloat(Param::BMS_ChargeLim) * 10.0f));

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
  uint16_t soc = ClampU16((int)lroundf(socPercent * 100.0f));

  const float packCapacityWh = Param::GetFloat(Param::BattCap) * 1000.0f;
  const float fullCapacityAh = packCapacityWh / packVoltage;
  const uint16_t fullCapacityAh10 = ClampU16((int)lroundf(fullCapacityAh * 10.0f));
  const uint16_t remCapacityAh10 = ClampU16((int)lroundf(fullCapacityAh * (socPercent / 10.0f)));

  uint8_t msg150[8] = {0};
  msg150[0] = soc >> 8;
  msg150[1] = soc & 0xFF;
  msg150[2] = 10000 >> 8;
  msg150[3] = 10000 & 0xFF;
  msg150[4] = remCapacityAh10 >> 8;
  msg150[5] = remCapacityAh10 & 0xFF;
  msg150[6] = fullCapacityAh10 >> 8;
  msg150[7] = fullCapacityAh10 & 0xFF;

  const int voltageDV = ClampU16((int)lroundf(Param::GetFloat(Param::udc) * 10.0f));
  const int currentDA = ClampS16((int)lroundf(Param::GetFloat(Param::idc) * 10.0f));
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

  if (tick2s >= 20) {
    tick2s = 0;
    SendFrame(0x110, msg110);
  }
  if (tick10s >= 100) {
    tick10s = 0;
    SendFrame(0x150, msg150);
    SendFrame(0x1D0, msg1D0);
    SendFrame(0x210, msg210);
  }
  if (tick60s >= 600) {
    tick60s = 0;
    SendFrame(0x190, msg190);
  }
}

void BydCan::Task100Ms() {
  if (!Enabled()) {
    inverterStartedUp = false;
    initialDataSent = false;
    tick2s = 0;
    tick10s = 0;
    tick60s = 0;
    return;
  }

  if (!inverterStartedUp)
    return;

  if (!initialDataSent) {
    SendInitialData();
    initialDataSent = true;
  }

  tick2s++;
  tick10s++;
  tick60s++;
  UpdateFrames();
}

void BydCan::DecodeCAN3(const uCAN_MSG &rxMessage) {
  if (!Enabled())
    return;

  switch (rxMessage.frame.id) {
  case 0x151:
    inverterStartedUp = true;
    if (rxMessage.frame.data0 & 0x01)
      SendInitialData();
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
