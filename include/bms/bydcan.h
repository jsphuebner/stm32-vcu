#ifndef BYDCAN_H
#define BYDCAN_H

#include "CANSPI.h"
#include <stdint.h>

class BydCan {
public:
  void Task100Ms();
  void DecodeCAN3(const uCAN_MSG &rxMessage);

private:
  bool SendInitialSequenceFrame();
  bool SendCyclicFrame();
  void SendFrame(uint16_t id, const uint8_t data[8]);
  bool Enabled() const;
  void ResetState();

  bool initialDataSent = false;
  bool inverterStartedUp = false;
  bool initialSequencePending = false;
  bool cyclic2sPending = false;
  bool cyclic10sPending = false;
  bool cyclic60sPending = false;
  uint8_t initialSequenceIndex = 0;
  uint8_t cyclic10sIndex = 0;
  uint16_t tick2s = 0;
  uint16_t tick10s = 0;
  uint16_t tick60s = 0;
};

#endif // BYDCAN_H
