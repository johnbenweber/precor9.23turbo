#include "sonar_sensor.h"

#include <Arduino.h>

/*
  HC-SR04 measurement approach adapted from Rui Santos:
  https://randomnerdtutorials.com/esp32-hc-sr04-ultrasonic-arduino/

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

namespace {

constexpr uint8_t kTriggerPin = 5;
constexpr uint8_t kEchoPin = 18;

}  // namespace

void SonarSensor::begin() {
  pinMode(kTriggerPin, OUTPUT);
  pinMode(kEchoPin, INPUT);
  digitalWrite(kTriggerPin, LOW);
}

void SonarSensor::sample() {
  digitalWrite(kTriggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(kTriggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(kTriggerPin, LOW);

  const unsigned long durationUs =
      pulseIn(kEchoPin, HIGH, kEchoTimeoutUs);
  latestReadingValid_ = durationUs != 0;
  if (!latestReadingValid_) {
    return;
  }

  const float distanceCm = durationUs * 0.0343f / 2.0f;
  const float distanceIn = distanceCm / 2.54f;
  samples_[nextSample_] = distanceIn;
  nextSample_ = (nextSample_ + 1) % kSmoothingSamples;
  if (sampleCount_ < kSmoothingSamples) {
    ++sampleCount_;
  }

  float sum = 0.0f;
  for (unsigned int i = 0; i < sampleCount_; ++i) {
    sum += samples_[i];
  }
  averageDistanceIn_ = sum / sampleCount_;
  hasDistance_ = true;
}

bool SonarSensor::hasValidReading() const {
  return latestReadingValid_;
}

bool SonarSensor::hasDistance() const {
  return hasDistance_;
}

float SonarSensor::averageDistanceIn() const {
  return averageDistanceIn_;
}
