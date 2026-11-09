#pragma once

class FreeRunControl {
 public:
  void update(bool readingValid, bool hasDistance, float distanceIn);

  float speedKph() const;
  const char* zoneName() const;

 private:
  enum class Zone {
    kAccel,
    kHold,
    kDecel,
    kOutside,
  };

  float speedKph_ = 0.0f;
  Zone zone_ = Zone::kOutside;
};
