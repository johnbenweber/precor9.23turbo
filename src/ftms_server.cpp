#include "ftms_server.h"

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <atomic>
#include <math.h>

namespace {

constexpr int16_t kMinimumInclineTenthsPercent = 0;
constexpr int16_t kMaximumInclineTenthsPercent = 150;
constexpr uint16_t kInclineStepTenthsPercent = 1;
constexpr char kDeviceName[] = "Precor 9.23 Turbo";

const NimBLEUUID kFtmsServiceUuid((uint16_t)0x1826);
const NimBLEUUID kWahooRunServiceUuid(
    "A026EE0E-0A7D-4AB3-97FA-F1500F9FEB8B");
const NimBLEUUID kTreadmillDataUuid((uint16_t)0x2ACD);
const NimBLEUUID kFitnessMachineFeatureUuid((uint16_t)0x2ACC);
const NimBLEUUID kSupportedInclinationRangeUuid((uint16_t)0x2AD5);
const NimBLEUUID kFtmsControlPointUuid((uint16_t)0x2AD9);
const NimBLEUUID kWahooRunNotifyUuid(
    "A026E03D-0A7D-4AB3-97FA-F1500F9FEB8B");
const NimBLEUUID kWahooRunWriteUuid(
    "A026E03E-0A7D-4AB3-97FA-F1500F9FEB8B");

NimBLECharacteristic* gTreadmillData = nullptr;
NimBLECharacteristic* gControlPoint = nullptr;
std::atomic<int16_t> gInclineTenthsPercent{0};
bool gControlGranted = false;

void setInclineTenthsPercent(int16_t incline) {
  gInclineTenthsPercent.store(static_cast<int16_t>(
      constrain(incline,
                kMinimumInclineTenthsPercent,
                kMaximumInclineTenthsPercent)));
}

int16_t decodeSignedLittleEndian(uint8_t lowByte, uint8_t highByte) {
  const uint16_t raw =
      static_cast<uint16_t>(lowByte) |
      (static_cast<uint16_t>(highByte) << 8);
  return static_cast<int16_t>(raw);
}

int16_t hundredthsToTenths(int16_t value) {
  return static_cast<int16_t>(lroundf(value / 10.0f));
}

void sendControlPointResponse(uint8_t requestOpcode, uint8_t resultCode) {
  const uint8_t response[] = {0x80, requestOpcode, resultCode};
  gControlPoint->setValue(response, sizeof(response));
  gControlPoint->indicate();
}

class WahooRunCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* characteristic,
               NimBLEConnInfo&) override {
    if (characteristic->getUUID() != kWahooRunWriteUuid) {
      return;
    }

    const NimBLEAttValue value = characteristic->getValue();
    if (value.size() < 3 || value[0] != 0x11) {
      return;
    }

    const int16_t slopeHundredthsPercent =
        decodeSignedLittleEndian(value[1], value[2]);
    setInclineTenthsPercent(
        hundredthsToTenths(slopeHundredthsPercent));
  }
};

class FtmsControlPointCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* characteristic,
               NimBLEConnInfo&) override {
    const NimBLEAttValue request = characteristic->getValue();
    if (request.size() == 0) {
      return;
    }

    const uint8_t opcode = request[0];
    if (opcode == 0x00) {
      if (request.size() != 1) {
        sendControlPointResponse(opcode, 0x03);
        return;
      }
      gControlGranted = true;
      sendControlPointResponse(opcode, 0x01);
      return;
    }

    if (opcode == 0x03) {
      if (request.size() != 3) {
        sendControlPointResponse(opcode, 0x03);
        return;
      }
      if (!gControlGranted) {
        sendControlPointResponse(opcode, 0x05);
        return;
      }

      const int16_t incline =
          decodeSignedLittleEndian(request[1], request[2]);
      if (incline < kMinimumInclineTenthsPercent ||
          incline > kMaximumInclineTenthsPercent) {
        sendControlPointResponse(opcode, 0x03);
        return;
      }

      setInclineTenthsPercent(incline);
      sendControlPointResponse(opcode, 0x01);
      return;
    }

    if (opcode == 0x11) {
      if (request.size() != 7) {
        sendControlPointResponse(opcode, 0x03);
        return;
      }
      if (!gControlGranted) {
        sendControlPointResponse(opcode, 0x05);
        return;
      }

      const int16_t gradeHundredthsPercent =
          decodeSignedLittleEndian(request[3], request[4]);
      setInclineTenthsPercent(
          hundredthsToTenths(gradeHundredthsPercent));
      sendControlPointResponse(opcode, 0x01);
      return;
    }

    sendControlPointResponse(opcode, 0x02);
  }
};

class FtmsServerCallbacks : public NimBLEServerCallbacks {
  void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int) override {
    gControlGranted = false;
  }
};

WahooRunCallbacks gWahooRunCallbacks;
FtmsControlPointCallbacks gFtmsControlPointCallbacks;
FtmsServerCallbacks gFtmsServerCallbacks;

}  // namespace

void ftmsBegin() {
  NimBLEDevice::init(kDeviceName);

  NimBLEServer* server = NimBLEDevice::createServer();
  server->setCallbacks(&gFtmsServerCallbacks);
  server->advertiseOnDisconnect(true);

  NimBLEService* ftmsService = server->createService(kFtmsServiceUuid);
  NimBLEService* wahooService =
      server->createService(kWahooRunServiceUuid);

  NimBLECharacteristic* wahooNotify = wahooService->createCharacteristic(
      kWahooRunNotifyUuid,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE_NR |
          NIMBLE_PROPERTY::NOTIFY);
  NimBLECharacteristic* wahooWrite = wahooService->createCharacteristic(
      kWahooRunWriteUuid,
      NIMBLE_PROPERTY::WRITE_NR | NIMBLE_PROPERTY::INDICATE);
  wahooNotify->setCallbacks(&gWahooRunCallbacks);
  wahooWrite->setCallbacks(&gWahooRunCallbacks);

  gTreadmillData = ftmsService->createCharacteristic(
      kTreadmillDataUuid, NIMBLE_PROPERTY::NOTIFY);
  NimBLECharacteristic* machineFeature = ftmsService->createCharacteristic(
      kFitnessMachineFeatureUuid, NIMBLE_PROPERTY::READ);
  NimBLECharacteristic* supportedInclinationRange =
      ftmsService->createCharacteristic(kSupportedInclinationRangeUuid,
                                        NIMBLE_PROPERTY::READ);
  gControlPoint = ftmsService->createCharacteristic(
      kFtmsControlPointUuid,
      NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::INDICATE);
  gControlPoint->setCallbacks(&gFtmsControlPointCallbacks);

  const uint8_t features[] = {0x08, 0x00, 0x00, 0x00,
                              0x02, 0x20, 0x00, 0x00};
  machineFeature->setValue(features, sizeof(features));

  const uint8_t inclinationRange[] = {
      static_cast<uint8_t>(kMinimumInclineTenthsPercent & 0xFF),
      static_cast<uint8_t>(kMinimumInclineTenthsPercent >> 8),
      static_cast<uint8_t>(kMaximumInclineTenthsPercent & 0xFF),
      static_cast<uint8_t>(kMaximumInclineTenthsPercent >> 8),
      static_cast<uint8_t>(kInclineStepTenthsPercent & 0xFF),
      static_cast<uint8_t>(kInclineStepTenthsPercent >> 8)};
  supportedInclinationRange->setValue(inclinationRange,
                                      sizeof(inclinationRange));
  server->start();

  NimBLEAdvertising* advertising = NimBLEDevice::getAdvertising();
  advertising->enableScanResponse(true);
  if (!advertising->setName(kDeviceName)) {
    Serial.println("Warning: could not add BLE device name to scan response.");
  }
  advertising->addServiceUUID(kFtmsServiceUuid);
  advertising->addServiceUUID(kWahooRunServiceUuid);
  if (!advertising->start()) {
    Serial.println("Error: BLE advertising failed to start.");
  }
}

void ftmsUpdateTreadmillData(float speedKph) {
  if (gTreadmillData == nullptr) {
    return;
  }

  const uint16_t speedHundredthsKph =
      static_cast<uint16_t>(speedKph * 100.0f);
  const uint16_t inclineTenthsPercent =
      static_cast<uint16_t>(gInclineTenthsPercent.load());
  const uint8_t treadmillData[] = {
      0x08, 0x00,
      static_cast<uint8_t>(speedHundredthsKph & 0xFF),
      static_cast<uint8_t>(speedHundredthsKph >> 8),
      static_cast<uint8_t>(inclineTenthsPercent & 0xFF),
      static_cast<uint8_t>(inclineTenthsPercent >> 8),
      0x00, 0x00};

  gTreadmillData->setValue(treadmillData, sizeof(treadmillData));
  gTreadmillData->notify();
}

float ftmsInclinePercent() {
  return gInclineTenthsPercent.load() / 10.0f;
}
