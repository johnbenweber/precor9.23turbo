#include <Arduino.h>

#include "ftms_server.h"
#include "free_run_control.h"
#include "harness_telemetry.h"
#include "sonar_sensor.h"

namespace {

constexpr uint32_t kSensorIntervalMs = 100;
constexpr uint32_t kTelemetryNotifyIntervalMs = 1000;
constexpr uint32_t kStatusIntervalMs = 250;
constexpr bool kFreeRunModeEnabled = false;

HarnessTelemetry gHarnessTelemetry;
SonarSensor gSonar;
FreeRunControl gFreeRunControl;

void printStatus() {
  if (kFreeRunModeEnabled) {
    Serial.printf("%lu ms | speed %.2f km/h | zone %s | incline %.1f%%\n",
                  static_cast<unsigned long>(millis()),
                  gFreeRunControl.speedKph(),
                  gFreeRunControl.zoneName(),
                  ftmsInclinePercent());
    return;
  }

  const HarnessReadings& readings = gHarnessTelemetry.readings();
  Serial.printf("%lu ms | belt %s | ",
                static_cast<unsigned long>(millis()),
                readings.beltMoving ? "moving" : "stopped");
  if (readings.speedValid) {
    Serial.printf("speed %.2f km/h | ", readings.speedKph);
  } else {
    Serial.print("speed unavailable | ");
  }
  if (readings.inclineValid) {
    Serial.printf("incline %.1f%%\n", readings.inclinePercent);
  } else {
    Serial.println("incline unavailable");
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  if (kFreeRunModeEnabled) {
    gSonar.begin();
  } else {
    gHarnessTelemetry.begin();
  }
  ftmsBegin(kFreeRunModeEnabled);
}

void loop() {
  static uint32_t lastSensorMs = 0;
  static uint32_t lastTelemetryNotifyMs = 0;
  static uint32_t lastStatusMs = 0;
  const uint32_t now = millis();

  if (now - lastSensorMs >= kSensorIntervalMs) {
    lastSensorMs = now;
    if (kFreeRunModeEnabled) {
      gSonar.sample();
    } else {
      gHarnessTelemetry.sample();
    }
  }

  if (now - lastTelemetryNotifyMs >= kTelemetryNotifyIntervalMs) {
    lastTelemetryNotifyMs = now;
    if (kFreeRunModeEnabled) {
      gFreeRunControl.update(gSonar.hasValidReading(),
                             gSonar.hasDistance(),
                             gSonar.averageDistanceIn());
      ftmsUpdateTreadmillData(gFreeRunControl.speedKph(),
                              true,
                              ftmsInclinePercent(),
                              true);
    } else {
      const HarnessReadings& readings = gHarnessTelemetry.readings();
      ftmsUpdateTreadmillData(readings.speedKph,
                              readings.speedValid,
                              readings.inclinePercent,
                              readings.inclineValid);
    }
  }

  if (now - lastStatusMs >= kStatusIntervalMs) {
    lastStatusMs = now;
    printStatus();
  }
}
