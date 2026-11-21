#pragma once

#include <stdint.h>

struct HarnessReadings {
  bool beltMoving;
  float speedKph;
  bool speedValid;
  float inclinePercent;
  bool inclineValid;
};

class HarnessTelemetry {
 public:
  void begin();
  void sample();

  const HarnessReadings& readings() const;

 private:
  uint32_t lastSpeedSampleMs_ = 0;
  HarnessReadings readings_ = {};
};
