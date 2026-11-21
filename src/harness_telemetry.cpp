#include "harness_telemetry.h"

#include <Arduino.h>

namespace {

constexpr uint8_t kBeltMovingPin = 26;
constexpr uint8_t kSpeedPulsePin = 27;
constexpr uint8_t kInclineAdcPin = 34;

constexpr float kRollerDiameterMm = 64.59f;
constexpr float kPulsesPerRotation = 2.0f;
constexpr float kMetersPerKilometer = 1000.0f;
constexpr float kSecondsPerHour = 3600.0f;
constexpr float kPi = 3.14159265f;

// Assumes external conditioning that outputs half the harness voltage.
constexpr float kInclineAdcToHarnessVoltage = 2.0f;
constexpr float kMinimumInclineVoltage = 0.914f;
constexpr float kMaximumInclineVoltage = 4.14f;

volatile uint32_t gSpeedPulseCount = 0;
portMUX_TYPE gSpeedPulseMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR captureSpeedPulse() {
  portENTER_CRITICAL_ISR(&gSpeedPulseMux);
  ++gSpeedPulseCount;
  portEXIT_CRITICAL_ISR(&gSpeedPulseMux);
}

uint32_t takeSpeedPulseCount() {
  portENTER_CRITICAL(&gSpeedPulseMux);
  const uint32_t pulseCount = gSpeedPulseCount;
  gSpeedPulseCount = 0;
  portEXIT_CRITICAL(&gSpeedPulseMux);
  return pulseCount;
}

}  // namespace

void HarnessTelemetry::begin() {
  pinMode(kBeltMovingPin, INPUT);
  pinMode(kSpeedPulsePin, INPUT);
  pinMode(kInclineAdcPin, INPUT);
  // An inverting optocoupler turns pin 8's low-going pulse into a rising edge.
  attachInterrupt(kSpeedPulsePin, captureSpeedPulse, RISING);
  lastSpeedSampleMs_ = millis();
}

void HarnessTelemetry::sample() {
  const uint32_t now = millis();
  readings_.beltMoving = digitalRead(kBeltMovingPin) == LOW;

  const float adcVoltage =
      analogReadMilliVolts(kInclineAdcPin) / 1000.0f;
  const float harnessVoltage = adcVoltage * kInclineAdcToHarnessVoltage;
  readings_.inclineValid =
      harnessVoltage >= kMinimumInclineVoltage &&
      harnessVoltage <= kMaximumInclineVoltage;
  if (readings_.inclineValid) {
    readings_.inclinePercent =
        (kMaximumInclineVoltage - harnessVoltage) * 15.0f /
        (kMaximumInclineVoltage - kMinimumInclineVoltage);
  }

  constexpr uint32_t kSpeedSampleIntervalMs = 1000;
  const uint32_t elapsedMs = now - lastSpeedSampleMs_;
  if (elapsedMs < kSpeedSampleIntervalMs) {
    return;
  }

  const uint32_t pulseCount = takeSpeedPulseCount();
  lastSpeedSampleMs_ = now;
  readings_.speedValid = !readings_.beltMoving || pulseCount > 0;
  const float rollerCircumferenceMeters =
      kPi * kRollerDiameterMm / kMetersPerKilometer;
  readings_.speedKph =
      pulseCount * rollerCircumferenceMeters * kSecondsPerHour /
      (kPulsesPerRotation * elapsedMs);
}

const HarnessReadings& HarnessTelemetry::readings() const {
  return readings_;
}
