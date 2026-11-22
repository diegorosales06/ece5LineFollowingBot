// ************************************************************************************************* //
// BleTelemetry.h - Wireless, print-only debug output over Bluetooth Low Energy (BLE)
//
// Exposes a "Nordic UART Service" (NUS). Any BLE UART app can read it:
//   - ble_viewer.html in this folder (open in Chrome on a laptop)
//   - nRF Connect or Adafruit Bluefruit Connect (phone)
//
// Uses the ESP32 core's built-in BLE library (NimBLE backend on the ESP32-S3), so nothing extra
// needs to be installed. BLE runs on core 0 while loop() runs on core 1; bleSend() only copies
// bytes into the BLE stack's queue, so it barely touches the PID loop.
//
// BLE does NOT use ADC2, so the potentiometers on GPIO 11-13 keep working (Wi-Fi would break them).
// ************************************************************************************************* //
#pragma once

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

// Nordic UART Service UUIDs (the standard ones every BLE UART app recognizes)
#define NUS_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_UUID      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  // phone -> robot (unused, print only)
#define NUS_TX_UUID      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  // robot -> phone (notify)

static BLEServer         *bleServer  = nullptr;
static BLECharacteristic *bleTxChar  = nullptr;
static volatile bool      bleClientConnected = false;
static volatile bool      bleJustConnected   = false;  // set in callback, consumed in loop()

class TelemetryServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *server) override {
    bleClientConnected = true;
    bleJustConnected   = true;
  }
  void onDisconnect(BLEServer *server) override {
    bleClientConnected = false;
    server->startAdvertising();  // let you reconnect mid-run
  }
};

// Call once in setup()
void bleTelemetryBegin(const char *deviceName) {
  BLEDevice::init(deviceName);
  BLEDevice::setMTU(185);  // ask for bigger packets so a whole line fits in one notification

  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new TelemetryServerCallbacks());

  BLEService *service = bleServer->createService(NUS_SERVICE_UUID);
  bleTxChar = service->createCharacteristic(NUS_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  service->createCharacteristic(NUS_RX_UUID, BLECharacteristic::PROPERTY_WRITE_NR);  // apps expect it
  service->start();

  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(NUS_SERVICE_UUID);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
}

bool bleConnected() {
  return bleClientConnected;
}

// Returns true exactly once after each new connection (use it to re-send calibration info)
bool bleConsumeNewConnection() {
  if (!bleJustConnected) return false;
  bleJustConnected = false;
  return true;
}

// Send raw bytes; splits into chunks that fit the negotiated MTU. Never blocks.
void bleSend(const char *data, size_t len) {
  if (!bleClientConnected || bleTxChar == nullptr) return;

  size_t chunk = 20;  // safe default (MTU 23 - 3 bytes header)
  uint16_t mtu = bleServer->getPeerMTU(bleServer->getConnId());
  if (mtu > 23) chunk = mtu - 3;

  while (len > 0) {
    size_t n = len < chunk ? len : chunk;
    bleTxChar->setValue((uint8_t *)data, n);
    bleTxChar->notify();
    data += n;
    len  -= n;
  }
}

void bleSend(const char *str) {
  bleSend(str, strlen(str));
}
