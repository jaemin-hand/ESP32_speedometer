#pragma once

#include <Arduino.h>
#include "driver/twai.h"

enum CanSignalEndian : uint8_t {
  CAN_SIGNAL_LITTLE_ENDIAN = 0,
  CAN_SIGNAL_BIG_ENDIAN = 1,
};

struct CanSpeedDecoderConfig {
  bool enabled = false;
  const char *name = "";
  uint32_t identifier = 0;
  bool extended = false;
  uint8_t startByte = 0;
  uint8_t lengthBytes = 2;
  CanSignalEndian endian = CAN_SIGNAL_LITTLE_ENDIAN;
  float scale = 1.0f;
  float offset = 0.0f;
  uint32_t timeoutMs = 200;
};

struct CanDecodedSpeedState {
  bool valid = false;
  float speedKmh = 0.0f;
  uint32_t identifier = 0;
  uint32_t lastUpdateMs = 0;
  const char *decoderName = "";
};

class CanManager {
public:
  static constexpr uint8_t kMonitorLineCount = 8;
  static constexpr size_t kMonitorLineLength = 48;
  static constexpr size_t kMonitorTextSize = (kMonitorLineCount * kMonitorLineLength) + 32;

  bool begin(gpio_num_t txPin, gpio_num_t rxPin);
  void poll(uint32_t nowMs);

  bool sendTestFrame();
  bool isLinkAlive(uint32_t nowMs, uint32_t aliveWindowMs) const;
  bool hasDecodedSpeed() const;
  float getDecodedSpeedKmh() const;
  const CanDecodedSpeedState &getDecodedSpeedState() const;
  const char *getMonitorText() const;
  void printStatus(const char *prefix = nullptr) const;

private:
  static uint32_t readUnsignedValue(
      const uint8_t *data,
      uint8_t startByte,
      uint8_t lengthBytes,
      CanSignalEndian endian);
  static void formatMonitorLine(
      const twai_message_t &rxMessage,
      char *lineBuf,
      size_t lineBufSize);
  void appendMonitorLine(const twai_message_t &rxMessage);
  void rebuildMonitorText();
  bool tryDecodeSpeed(const twai_message_t &rxMessage, uint32_t nowMs);

  uint32_t lastRxMs_ = 0;
  uint32_t decodedSpeedTimeoutMs_ = 0;
  CanDecodedSpeedState decodedSpeed_;
  char monitorLines_[kMonitorLineCount][kMonitorLineLength] = {};
  char monitorText_[kMonitorTextSize] = "Waiting for CAN data...";
  uint8_t nextMonitorLineIndex_ = 0;
  bool monitorWrapped_ = false;
};
