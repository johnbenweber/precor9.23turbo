#include "free_run_control.h"

#include <math.h>

namespace {

constexpr float kMinimumControlDistanceIn = 4.0f;
constexpr float kMaximumControlDistanceIn = 10.0f;
constexpr float kNeutralCenterIn = 7.0f;
constexpr float kNeutralHalfWidthIn = 0.5f;
constexpr float kZoneWidthIn = 0.5f;
constexpr float kMaximumSpeedKph = 20.0f;
constexpr float kSpeedStepsKph[] = {0.16f, 0.32f, 0.64f, 1.6f, 3.2f};
constexpr unsigned int kZoneCount =
    sizeof(kSpeedStepsKph) / sizeof(kSpeedStepsKph[0]);

}  // namespace

void FreeRunControl::update(bool readingValid,
                            bool hasDistance,
                            float distanceIn) {
  if (!readingValid || !hasDistance ||
      distanceIn < kMinimumControlDistanceIn ||
      distanceIn > kMaximumControlDistanceIn) {
    zone_ = Zone::kOutside;
    return;
  }

  const float offsetIn = fabsf(distanceIn - kNeutralCenterIn);
  if (offsetIn <= kNeutralHalfWidthIn) {
    zone_ = Zone::kHold;
    return;
  }

  unsigned int speedZone =
      static_cast<unsigned int>((offsetIn - kNeutralHalfWidthIn) /
                                kZoneWidthIn);
  if (speedZone >= kZoneCount) {
    speedZone = kZoneCount - 1;
  }

  if (distanceIn < kNeutralCenterIn) {
    speedKph_ = fminf(speedKph_ + kSpeedStepsKph[speedZone],
                      kMaximumSpeedKph);
    zone_ = Zone::kAccel;
  } else {
    speedKph_ = fmaxf(speedKph_ - kSpeedStepsKph[speedZone], 0.0f);
    zone_ = Zone::kDecel;
  }
}

float FreeRunControl::speedKph() const {
  return speedKph_;
}

const char* FreeRunControl::zoneName() const {
  switch (zone_) {
    case Zone::kAccel:
      return "accel";
    case Zone::kHold:
      return "hold";
    case Zone::kDecel:
      return "decel";
    case Zone::kOutside:
      return "outside";
  }
  return "outside";
}
