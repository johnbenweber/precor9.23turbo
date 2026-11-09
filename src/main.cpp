#include <Arduino.h>

#include "ftms_server.h"
#include "free_run_control.h"
#include "sonar_sensor.h"

namespace {

constexpr uint32_t kSensorIntervalMs = 100;
constexpr uint32_t kControlIntervalMs = 1000;
constexpr uint32_t kStatusIntervalMs = 250;

SonarSensor gSonar;
FreeRunControl gFreeRunControl;

void printStatus() {
  Serial.printf("%lu ms | speed %.2f km/h | zone %s | incline %.1f%%\n",
                static_cast<unsigned long>(millis()),
                gFreeRunControl.speedKph(),
                gFreeRunControl.zoneName(),
                ftmsInclinePercent());
}

}  // namespace

void setup() {
  Serial.begin(115200);
  gSonar.begin();
  ftmsBegin();
}

void loop() {
  static uint32_t lastSensorMs = 0;
  static uint32_t lastControlMs = 0;
  static uint32_t lastStatusMs = 0;
  const uint32_t now = millis();

  if (now - lastSensorMs >= kSensorIntervalMs) {
    lastSensorMs = now;
    gSonar.sample();
  }

  if (now - lastControlMs >= kControlIntervalMs) {
    lastControlMs = now;
    gFreeRunControl.update(gSonar.hasValidReading(),
                           gSonar.hasDistance(),
                           gSonar.averageDistanceIn());
    ftmsUpdateTreadmillData(gFreeRunControl.speedKph());
  }

  if (now - lastStatusMs >= kStatusIntervalMs) {
    lastStatusMs = now;
    printStatus();
  }
}
