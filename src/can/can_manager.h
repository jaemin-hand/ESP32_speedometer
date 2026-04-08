#pragma once

#include <Arduino.h>

#include "can_backend.h"
#include "can_profiles.h"

struct CanDecodedSpeedState {
  bool valid = false;
  float speedKmh = 0.0f;
  uint32_t identifier = 0;
  uint32_t lastUpdateMs = 0;
  const char *decoderName = "";
  uint8_t decoderPriority = 0;
};

struct CanDecoderDiagnosticState {
  CanDecodedSpeedState decodedSpeed = {};
  uint32_t timeoutMs = 0;
};

class CanManager {
public:
  static constexpr uint8_t kMonitorLineCount = 8;
  static constexpr size_t kMonitorLineLength = 48;
  static constexpr size_t kMonitorTextSize = (kMonitorLineCount * kMonitorLineLength) + 32;
  static constexpr size_t kMaxDecoderDiagnostics = 8;

  bool begin(
      gpio_num_t txPin,
      gpio_num_t rxPin,
      CanBackendType backendType = CAN_BACKEND_CLASSIC,
      CanProfileId profileId = CAN_PROFILE_SANTAFE_CLASSIC);
  void poll(uint32_t nowMs);
  bool isInitialized() const;
  CanBackendType getBackendType() const;
  const char *getBackendName() const;
  CanProfileId getProfileId() const;
  const char *getProfileName() const;
  CanBackendCapabilities getBackendCapabilities() const;
  CanBackendRequirements getBackendRequirements() const;
  const char *getBackendDiagnosticText() const;
  const char *getProfileBringupNote() const;

  bool sendTestFrame();
  bool isLinkAlive(uint32_t nowMs, uint32_t aliveWindowMs) const;
  bool hasDecodedSpeed() const;
  float getDecodedSpeedKmh() const;
  const CanDecodedSpeedState &getDecodedSpeedState() const;
  bool getDecoderDiagnostic(
      const char *decoderName,
      CanDecodedSpeedState *outState) const;
  const char *getMonitorText() const;
  void printStatus(const char *prefix = nullptr) const;

private:
  static uint32_t readUnsignedValue(
      const uint8_t *data,
      uint8_t startByte,
      uint8_t lengthBytes,
      CanSignalEndian endian);
  static bool decodeSpeedValue(
      const CanFrame &rxFrame,
      const CanSpeedDecoderConfig &config,
      float *speedKmhOut);
  static void formatMonitorLine(
      const CanFrame &rxFrame,
      char *lineBuf,
      size_t lineBufSize);
  void appendMonitorLine(const CanFrame &rxFrame);
  void rebuildMonitorText();
  bool tryDecodeSpeed(const CanFrame &rxFrame, uint32_t nowMs);
  void updateDecoderDiagnostic(
      const CanSpeedDecoderConfig &config,
      uint32_t identifier,
      float speedKmh,
      uint32_t nowMs);
  void expireDecoderDiagnostics(uint32_t nowMs);

  uint32_t lastRxMs_ = 0;
  uint32_t lastRawPrintMs_ = 0;
  uint32_t decodedSpeedTimeoutMs_ = 0;
  CanDecodedSpeedState decodedSpeed_;
  char monitorLines_[kMonitorLineCount][kMonitorLineLength] = {};
  char monitorText_[kMonitorTextSize] = "Waiting for CAN data...";
  uint8_t nextMonitorLineIndex_ = 0;
  bool monitorWrapped_ = false;
  bool initialized_ = false;
  CanBackendType backendType_ = CAN_BACKEND_CLASSIC;
  const CanProfile *configuredProfile_ = nullptr;
  CanDecoderDiagnosticState decoderDiagnostics_[kMaxDecoderDiagnostics] = {};
  ICanBackend *backend_ = nullptr;
};
