#pragma once

class SonarSensor {
 public:
  void begin();
  void sample();

  bool hasValidReading() const;
  bool hasDistance() const;
  float averageDistanceIn() const;

 private:
  static constexpr unsigned long kEchoTimeoutUs = 25000;
  static constexpr unsigned int kSmoothingSamples = 5;

  float samples_[kSmoothingSamples] = {};
  unsigned int sampleCount_ = 0;
  unsigned int nextSample_ = 0;
  float averageDistanceIn_ = 0.0f;
  bool hasDistance_ = false;
  bool latestReadingValid_ = false;
};
