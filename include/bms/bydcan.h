#ifndef BYDCAN_H
#define BYDCAN_H

#include "CANSPI.h"
#include <stdint.h>

class BydCan {
public:
  void Task100Ms();
  void DecodeCAN3(const uCAN_MSG &rxMessage);

private:
  void UpdateFrames();
  void SendInitialData();
  void SendFrame(uint16_t id, const uint8_t data[8]);
  bool Enabled() const;

  bool initialDataSent = false;
  bool inverterStartedUp = false;
  uint16_t tick2s = 0;
  uint16_t tick10s = 0;
  uint16_t tick60s = 0;
};

#endif // BYDCAN_H
