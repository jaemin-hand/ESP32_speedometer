#include "can_manager.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "../app/app_config.h"
#include "can_fd_backend.h"
#include "can_profiles.h"
#include "classic_can_backend.h"

namespace {

ClassicCanBackend gClassicCanBackend;
CanFdBackend gCanFdBackend;

// Keep the current CAN path intentionally close to the last known-good
// legacy sketch:
//   - NORMAL mode
//   - default 500 kbps timing
//   - direct non-blocking receive loop
//
// The only deliberate difference is the filter:
// the legacy sketch used a restrictive 0x100-only filter, but the current UI
// has a raw CAN monitor and we want to see all incoming frames while bringing
// the app back up. Once the receive path is stable, we can tighten the filter
// again if needed.

bool isReasonableSpeed(float speedKmh) {
  return isfinite(speedKmh) && speedKmh >= 0.0f && speedKmh <= 400.0f;
}

float decodeSantaFeWheelSample(uint8_t lowByte, uint8_t highByte) {
  const uint16_t rawValue =
      static_cast<uint16_t>(lowByte) |
      (static_cast<uint16_t>(highByte & 0x3F) << 8U);
  return static_cast<float>(rawValue) / 16.0f;
}

// Keep CAN RX processing cooperative with the rest of the app.
// If we drain the bus indefinitely while replay traffic is active,
// GNSS/time/UI updates appear to "freeze" because appLoop cannot return.
constexpr uint8_t kMaxFramesPerPoll = 8;
constexpr uint32_t kRawCanPrintIntervalMs = 100;

ICanBackend *selectBackend(CanBackendType backendType) {
  switch (backendType) {
    case CAN_BACKEND_CLASSIC:
      return &gClassicCanBackend;
    case CAN_BACKEND_FD:
      return &gCanFdBackend;
    default:
      return nullptr;
  }
}

}  // namespace

bool CanManager::begin(
    gpio_num_t txPin,
    gpio_num_t rxPin,
    CanBackendType backendType,
    CanProfileId profileId,
    bool autoDetectProfiles) {
  if (backend_ != nullptr) {
    backend_->end();
  }

  initialized_ = false;
  profileAutoDetect_ = autoDetectProfiles;
  configuredProfile_ = &getCanProfile(profileId);
  selectedProfile_ = configuredProfile_;
  backendType_ = configuredProfile_->backendType;
  if (backendType != backendType_) {
    Serial.printf(
        "CAN profile '%s' overrides requested backend and uses %s\n",
        configuredProfile_->name,
        (backendType_ == CAN_BACKEND_CLASSIC) ? "CLASSIC_CAN" : "CAN_FD");
  }
  backend_ = selectBackend(backendType_);
  if (backend_ == nullptr) {
    return false;
  }

  // Reset runtime state on every re-init so the monitor and decode state do
  // not carry stale values across backend restarts.
  lastRxMs_ = 0;
  lastRawPrintMs_ = 0;
  decodedSpeedTimeoutMs_ = 0;
  decodedSpeed_ = {};
  nextMonitorLineIndex_ = 0;
  monitorWrapped_ = false;
  memset(monitorLines_, 0, sizeof(monitorLines_));
  snprintf(monitorText_, sizeof(monitorText_), "Waiting for CAN data...");
  memset(decoderDiagnostics_, 0, sizeof(decoderDiagnostics_));
  resetTrackedProfiles();
  initializeTrackedProfiles();

  const CanBackendOptions options = {
      .txPin = txPin,
      .rxPin = rxPin,
      .spiSckPin = AppConfig::kCanFdSpiSckPin,
      .spiMosiPin = AppConfig::kCanFdSpiMosiPin,
      .spiMisoPin = AppConfig::kCanFdSpiMisoPin,
      .spiCsPin = AppConfig::kCanFdSpiCsPin,
      .irqPin = AppConfig::kCanFdIrqPin,
      .resetPin = AppConfig::kCanFdResetPin,
      .standbyPin = AppConfig::kCanFdStandbyPin,
  };
  if (!backend_->begin(options)) {
    backend_ = nullptr;
    return false;
  }

  initialized_ = true;
  return true;
}

bool CanManager::isInitialized() const {
  return initialized_;
}

CanBackendType CanManager::getBackendType() const {
  return backendType_;
}

const char *CanManager::getBackendName() const {
  return (backend_ != nullptr) ? backend_->backendName() : "NONE";
}

CanProfileId CanManager::getProfileId() const {
  const CanProfile *activeProfile =
      (selectedProfile_ != nullptr) ? selectedProfile_ : configuredProfile_;
  return (activeProfile != nullptr) ? activeProfile->id : CAN_PROFILE_SANTAFE_CLASSIC;
}

const char *CanManager::getProfileName() const {
  const CanProfile *activeProfile =
      (selectedProfile_ != nullptr) ? selectedProfile_ : configuredProfile_;
  return (activeProfile != nullptr) ? activeProfile->name : "NONE";
}

bool CanManager::isProfileAutoDetectEnabled() const {
  return profileAutoDetect_;
}

uint16_t CanManager::getDetectedProfileConfidence() const {
  return detectedProfileConfidence_;
}

CanBackendCapabilities CanManager::getBackendCapabilities() const {
  return (backend_ != nullptr) ? backend_->capabilities() : CanBackendCapabilities{};
}

CanBackendRequirements CanManager::getBackendRequirements() const {
  return (backend_ != nullptr) ? backend_->requirements() : CanBackendRequirements{};
}

const char *CanManager::getBackendDiagnosticText() const {
  return (backend_ != nullptr) ? backend_->diagnosticText() : "CAN backend not selected";
}

const char *CanManager::getProfileBringupNote() const {
  const CanProfile *activeProfile =
      (selectedProfile_ != nullptr) ? selectedProfile_ : configuredProfile_;
  return (activeProfile != nullptr) ? activeProfile->bringupNote
                                    : "CAN profile note unavailable";
}

void CanManager::poll(uint32_t nowMs) {
  if (!initialized_ || (backend_ == nullptr)) {
    return;
  }

  // Process only a bounded number of frames per app loop iteration.
  // This keeps GNSS/time/LVGL updates responsive even when a CAN replay tool is
  // blasting traffic continuously.
  CanFrame rxFrame;
  uint8_t framesProcessed = 0;
  while ((framesProcessed < kMaxFramesPerPoll) &&
         backend_->receive(&rxFrame)) {
    char monitorLine[kMonitorLineLength] = {0};
    lastRxMs_ = nowMs;
    formatMonitorLine(rxFrame, monitorLine, sizeof(monitorLine));

    if ((lastRawPrintMs_ == 0U) ||
        ((nowMs - lastRawPrintMs_) >= kRawCanPrintIntervalMs)) {
      // Serial.printf("CAN RX: %s\n", monitorLine);
      lastRawPrintMs_ = nowMs;
    }

    appendMonitorLine(rxFrame);
    for (size_t i = 0; i < trackedProfileCount_; ++i) {
      tryDecodeSpeedForProfile(trackedProfiles_[i], rxFrame, nowMs);
    }
    ++framesProcessed;
  }

  rebuildDetectedProfileState(nowMs);
  expireDecoderDiagnostics(nowMs);
}

bool CanManager::sendTestFrame() {
  if (!initialized_ || (backend_ == nullptr)) {
    return false;
  }

  // Keep the same test frame used in the legacy sketch so PC-side captures and
  // past notes remain directly comparable.
  CanFrame txFrame;
  txFrame.identifier = 0x777;
  txFrame.extended = false;
  txFrame.fdFormat = false;
  txFrame.bitrateSwitch = false;
  txFrame.dataLength = 4;
  txFrame.data[0] = 0x01;
  txFrame.data[1] = 0x02;
  txFrame.data[2] = 0x03;
  txFrame.data[3] = 0x04;

  return backend_->transmit(txFrame, pdMS_TO_TICKS(100));
}

bool CanManager::isLinkAlive(uint32_t nowMs, uint32_t aliveWindowMs) const {
  return (lastRxMs_ != 0U) && ((nowMs - lastRxMs_) <= aliveWindowMs);
}

bool CanManager::hasDecodedSpeed() const {
  return decodedSpeed_.valid;
}

float CanManager::getDecodedSpeedKmh() const {
  return decodedSpeed_.speedKmh;
}

const CanDecodedSpeedState &CanManager::getDecodedSpeedState() const {
  return decodedSpeed_;
}

bool CanManager::getDecoderDiagnostic(
    const char *decoderName,
    CanDecodedSpeedState *outState) const {
  if ((decoderName == nullptr) || (outState == nullptr)) {
    return false;
  }

  for (size_t i = 0; i < kMaxDecoderDiagnostics; ++i) {
    const CanDecoderDiagnosticState &diag = decoderDiagnostics_[i];
    if (!diag.decodedSpeed.valid || (diag.decodedSpeed.decoderName == nullptr)) {
      continue;
    }
    if (strcmp(diag.decodedSpeed.decoderName, decoderName) == 0) {
      *outState = diag.decodedSpeed;
      return true;
    }
  }

  return false;
}

const char *CanManager::getMonitorText() const {
  return monitorText_;
}

void CanManager::printStatus(const char *prefix) const {
  if ((prefix != nullptr) && (prefix[0] != '\0')) {
    Serial.println(prefix);
  }

  if (!initialized_) {
    Serial.println("CAN backend not initialized");
    return;
  }

  twai_status_info_t status = {};
  if ((backend_ == nullptr) || !backend_->getStatus(&status)) {
    Serial.println("CAN backend status read failed");
    return;
  }

  Serial.printf(
      "TWAI state=%d tx_err=%lu rx_err=%lu tx_to_send=%lu rx_to_recv=%lu tx_failed=%lu rx_missed=%lu rx_overrun=%lu bus_err=%lu arb_lost=%lu\n",
      static_cast<int>(status.state),
      static_cast<unsigned long>(status.tx_error_counter),
      static_cast<unsigned long>(status.rx_error_counter),
      static_cast<unsigned long>(status.msgs_to_tx),
      static_cast<unsigned long>(status.msgs_to_rx),
      static_cast<unsigned long>(status.tx_failed_count),
      static_cast<unsigned long>(status.rx_missed_count),
      static_cast<unsigned long>(status.rx_overrun_count),
      static_cast<unsigned long>(status.bus_error_count),
      static_cast<unsigned long>(status.arb_lost_count));
}

uint32_t CanManager::readUnsignedValue(
    const uint8_t *data,
    uint8_t startByte,
    uint8_t lengthBytes,
    CanSignalEndian endian) {
  uint32_t value = 0;

  if (endian == CAN_SIGNAL_LITTLE_ENDIAN) {
    for (uint8_t i = 0; i < lengthBytes; ++i) {
      value |= static_cast<uint32_t>(data[startByte + i]) << (8U * i);
    }
  } else {
    for (uint8_t i = 0; i < lengthBytes; ++i) {
      value <<= 8U;
      value |= data[startByte + i];
    }
  }

  return value;
}

bool CanManager::decodeSpeedValue(
    const CanFrame &rxFrame,
    const CanSpeedDecoderConfig &config,
    float *speedKmhOut) {
  if (speedKmhOut == nullptr) {
    return false;
  }

  switch (config.decoderType) {
    case CAN_SPEED_DECODER_UNSIGNED: {
      if ((config.startByte + config.lengthBytes) > rxFrame.dataLength) {
        return false;
      }

      const uint32_t rawValue = readUnsignedValue(
          rxFrame.data,
          config.startByte,
          config.lengthBytes,
          config.endian);
      *speedKmhOut = (static_cast<float>(rawValue) * config.scale) + config.offset;
      return true;
    }

    case CAN_SPEED_DECODER_SANTAFE_WHEEL_AVG_0X386: {
      if (rxFrame.dataLength < 8U) {
        return false;
      }

      const float fl = decodeSantaFeWheelSample(rxFrame.data[0], rxFrame.data[1]);
      const float fr = decodeSantaFeWheelSample(rxFrame.data[2], rxFrame.data[3]);
      const float rl = decodeSantaFeWheelSample(rxFrame.data[4], rxFrame.data[5]);
      const float rr = decodeSantaFeWheelSample(rxFrame.data[6], rxFrame.data[7]);
      *speedKmhOut = (fl + fr + rl + rr) * 0.25f;
      return true;
    }

    default:
      return false;
  }
}

void CanManager::initializeTrackedProfiles() {
  if (configuredProfile_ == nullptr) {
    return;
  }

  if (!profileAutoDetect_) {
    trackedProfiles_[0].profile = configuredProfile_;
    trackedProfileCount_ = 1;
    selectedProfile_ = configuredProfile_;
    return;
  }

  const size_t profileCount = getCanProfileCount();
  for (size_t i = 0; i < profileCount && trackedProfileCount_ < kMaxTrackedProfiles; ++i) {
    const CanProfile &profile = getCanProfileByIndex(i);
    if (profile.backendType != backendType_) {
      continue;
    }

    trackedProfiles_[trackedProfileCount_].profile = &profile;
    ++trackedProfileCount_;
  }

  if (trackedProfileCount_ == 0U) {
    trackedProfiles_[0].profile = configuredProfile_;
    trackedProfileCount_ = 1;
  }
}

void CanManager::resetTrackedProfiles() {
  memset(trackedProfiles_, 0, sizeof(trackedProfiles_));
  trackedProfileCount_ = 0;
  detectedProfileConfidence_ = 0;
  selectedProfile_ = configuredProfile_;
  decodedSpeed_ = {};
}

void CanManager::rebuildDetectedProfileState(uint32_t nowMs) {
  const CanProfile *bestProfile = nullptr;
  const CanDecodedSpeedState *bestSpeed = nullptr;
  uint16_t bestConfidence = 0;

  for (size_t i = 0; i < trackedProfileCount_; ++i) {
    CanProfileDetectState &detectState = trackedProfiles_[i];
    if (detectState.profile == nullptr) {
      continue;
    }

    if (detectState.decodedSpeed.valid &&
        detectState.decodedSpeedTimeoutMs != 0U &&
        (nowMs - detectState.decodedSpeed.lastUpdateMs) > detectState.decodedSpeedTimeoutMs) {
      detectState.decodedSpeed = {};
      detectState.decodedSpeedTimeoutMs = 0;
      detectState.confidence = 0;
    }

    if (!detectState.decodedSpeed.valid) {
      continue;
    }

    if ((bestProfile == nullptr) ||
        (detectState.confidence > bestConfidence) ||
        ((detectState.confidence == bestConfidence) &&
         (detectState.decodedSpeed.decoderPriority > bestSpeed->decoderPriority))) {
      bestProfile = detectState.profile;
      bestSpeed = &detectState.decodedSpeed;
      bestConfidence = detectState.confidence;
    }
  }

  if ((bestProfile != nullptr) && (bestSpeed != nullptr)) {
    selectedProfile_ = bestProfile;
    decodedSpeed_ = *bestSpeed;
    detectedProfileConfidence_ = bestConfidence;
    decodedSpeedTimeoutMs_ = 0;
    return;
  }

  selectedProfile_ = configuredProfile_;
  decodedSpeed_ = {};
  decodedSpeedTimeoutMs_ = 0;
  detectedProfileConfidence_ = 0;
}

void CanManager::updateDecoderDiagnostic(
    const CanSpeedDecoderConfig &config,
    uint32_t identifier,
    float speedKmh,
    uint32_t nowMs) {
  size_t slotIndex = kMaxDecoderDiagnostics;

  for (size_t i = 0; i < kMaxDecoderDiagnostics; ++i) {
    if (decoderDiagnostics_[i].decodedSpeed.valid &&
        decoderDiagnostics_[i].decodedSpeed.decoderName != nullptr &&
        strcmp(decoderDiagnostics_[i].decodedSpeed.decoderName, config.name) == 0) {
      slotIndex = i;
      break;
    }

    if ((slotIndex == kMaxDecoderDiagnostics) &&
        !decoderDiagnostics_[i].decodedSpeed.valid) {
      slotIndex = i;
    }
  }

  if (slotIndex >= kMaxDecoderDiagnostics) {
    return;
  }

  decoderDiagnostics_[slotIndex].decodedSpeed.valid = true;
  decoderDiagnostics_[slotIndex].decodedSpeed.speedKmh = speedKmh;
  decoderDiagnostics_[slotIndex].decodedSpeed.identifier = identifier;
  decoderDiagnostics_[slotIndex].decodedSpeed.lastUpdateMs = nowMs;
  decoderDiagnostics_[slotIndex].decodedSpeed.decoderName = config.name;
  decoderDiagnostics_[slotIndex].decodedSpeed.decoderPriority = config.priority;
  decoderDiagnostics_[slotIndex].timeoutMs = config.timeoutMs;
}

void CanManager::expireDecoderDiagnostics(uint32_t nowMs) {
  for (size_t i = 0; i < kMaxDecoderDiagnostics; ++i) {
    CanDecoderDiagnosticState &diag = decoderDiagnostics_[i];
    if (!diag.decodedSpeed.valid || (diag.timeoutMs == 0U)) {
      continue;
    }

    if ((nowMs - diag.decodedSpeed.lastUpdateMs) > diag.timeoutMs) {
      diag = {};
    }
  }
}

void CanManager::appendMonitorLine(const CanFrame &rxFrame) {
  char monitorLine[kMonitorLineLength] = {0};
  formatMonitorLine(rxFrame, monitorLine, sizeof(monitorLine));

  snprintf(
      monitorLines_[nextMonitorLineIndex_],
      kMonitorLineLength,
      "%s",
      monitorLine);

  nextMonitorLineIndex_ = static_cast<uint8_t>((nextMonitorLineIndex_ + 1U) % kMonitorLineCount);
  if (nextMonitorLineIndex_ == 0U) {
    monitorWrapped_ = true;
  }

  rebuildMonitorText();
}

void CanManager::formatMonitorLine(
    const CanFrame &rxFrame,
    char *lineBuf,
    size_t lineBufSize) {
  if ((lineBuf == nullptr) || (lineBufSize == 0U)) {
    return;
  }

  char dataText[3 * 8 + 1] = {0};
  size_t offset = 0;
  const uint8_t shownBytes = (rxFrame.dataLength < 8U) ? rxFrame.dataLength : 8U;
  for (uint8_t i = 0; i < shownBytes; ++i) {
    offset += static_cast<size_t>(snprintf(
        dataText + offset,
        sizeof(dataText) - offset,
        (i == 0) ? "%02X" : " %02X",
        rxFrame.data[i]));
    if (offset >= sizeof(dataText)) {
      break;
    }
  }

  snprintf(
      lineBuf,
      lineBufSize,
      "0x%03X [%d%s] %s",
      rxFrame.identifier,
      rxFrame.dataLength,
      rxFrame.fdFormat ? "FD" : "",
      dataText);
}

void CanManager::rebuildMonitorText() {
  monitorText_[0] = '\0';

  if (!monitorWrapped_ && nextMonitorLineIndex_ == 0U) {
    snprintf(monitorText_, kMonitorTextSize, "Waiting for CAN data...");
    return;
  }

  const uint8_t lineCount = monitorWrapped_ ? kMonitorLineCount : nextMonitorLineIndex_;
  const uint8_t startIndex = monitorWrapped_ ? nextMonitorLineIndex_ : 0U;

  size_t offset = 0;
  for (uint8_t i = 0; i < lineCount; ++i) {
    const uint8_t lineIndex = static_cast<uint8_t>((startIndex + i) % kMonitorLineCount);
    if (monitorLines_[lineIndex][0] == '\0') {
      continue;
    }

    offset += static_cast<size_t>(snprintf(
        monitorText_ + offset,
        kMonitorTextSize - offset,
        "%s%s",
        monitorLines_[lineIndex],
        (i + 1U < lineCount) ? "\n" : ""));
    if (offset >= kMonitorTextSize) {
      monitorText_[kMonitorTextSize - 1] = '\0';
      break;
    }
  }
}

bool CanManager::tryDecodeSpeedForProfile(
    CanProfileDetectState &detectState,
    const CanFrame &rxFrame,
    uint32_t nowMs) {
  // Speed decoding is optional and sits on top of the raw receive path.
  // Even when no decoder is enabled, CAN RX logging/monitoring should still
  // work exactly like the legacy receive loop.
  if ((detectState.profile == nullptr) || (detectState.profile->speedDecoders == nullptr)) {
    return false;
  }

  for (size_t i = 0; i < detectState.profile->speedDecoderCount; ++i) {
    const CanSpeedDecoderConfig &config = detectState.profile->speedDecoders[i];
    if (!config.enabled) {
      continue;
    }
    if (config.identifier != rxFrame.identifier) {
      continue;
    }
    if (config.fdFormat != rxFrame.fdFormat) {
      continue;
    }
    if (config.extended != rxFrame.extended) {
      continue;
    }
    if (config.lengthBytes == 0U) {
      continue;
    }
    float speedKmh = 0.0f;
    if (!decodeSpeedValue(rxFrame, config, &speedKmh)) {
      continue;
    }
    if (!isReasonableSpeed(speedKmh)) {
      continue;
    }

    const bool decodedStillFresh =
        detectState.decodedSpeed.valid &&
        ((detectState.decodedSpeedTimeoutMs == 0U) ||
         ((nowMs - detectState.decodedSpeed.lastUpdateMs) <= detectState.decodedSpeedTimeoutMs));
    if (decodedStillFresh && (config.priority < detectState.decodedSpeed.decoderPriority)) {
      continue;
    }

    detectState.decodedSpeed.valid = true;
    detectState.decodedSpeed.speedKmh = speedKmh;
    detectState.decodedSpeed.identifier = rxFrame.identifier;
    detectState.decodedSpeed.lastUpdateMs = nowMs;
    detectState.decodedSpeed.decoderName = config.name;
    detectState.decodedSpeed.decoderPriority = config.priority;
    detectState.decodedSpeedTimeoutMs = config.timeoutMs;
    detectState.lastMatchMs = nowMs;
    updateDecoderDiagnostic(config, rxFrame.identifier, speedKmh, nowMs);
    if (detectState.confidence <= (UINT16_MAX - 32U)) {
      detectState.confidence = static_cast<uint16_t>(detectState.confidence + 32U);
    } else {
      detectState.confidence = UINT16_MAX;
    }
    return true;
  }

  return false;
}
