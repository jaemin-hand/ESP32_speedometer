#include "can_manager.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "can_fd_backend.h"
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

// First enabled decoder uses the SantaFe replay candidate that matched earlier
// offline analysis:
//   - ID 0x450
//   - standard frame
//   - byte0 = speed in km/h
//
// Keep the table-based structure so we can add more vehicle profiles later
// without touching the raw receive path again.
constexpr CanSpeedDecoderConfig kSpeedDecoders[] = {
    {
        true,
        "santafe_replay_speed",
        0x450,
        false,
        0,
        1,
        CAN_SIGNAL_LITTLE_ENDIAN,
        1.0f,
        0.0f,
        250,
    },
};

bool isReasonableSpeed(float speedKmh) {
  return isfinite(speedKmh) && speedKmh >= 0.0f && speedKmh <= 400.0f;
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
    CanBackendType backendType) {
  if (backend_ != nullptr) {
    backend_->end();
  }

  initialized_ = false;
  backendType_ = backendType;
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

  const CanBackendOptions options = {
      .txPin = txPin,
      .rxPin = rxPin,
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

void CanManager::poll(uint32_t nowMs) {
  if (!initialized_ || (backend_ == nullptr)) {
    return;
  }

  // Process only a bounded number of frames per app loop iteration.
  // This keeps GNSS/time/LVGL updates responsive even when a CAN replay tool is
  // blasting traffic continuously.
  twai_message_t rxMessage;
  uint8_t framesProcessed = 0;
  while ((framesProcessed < kMaxFramesPerPoll) &&
         backend_->receive(&rxMessage)) {
    char monitorLine[kMonitorLineLength] = {0};
    lastRxMs_ = nowMs;
    formatMonitorLine(rxMessage, monitorLine, sizeof(monitorLine));

    if ((lastRawPrintMs_ == 0U) ||
        ((nowMs - lastRawPrintMs_) >= kRawCanPrintIntervalMs)) {
      Serial.printf("CAN RX: %s\n", monitorLine);
      lastRawPrintMs_ = nowMs;
    }

    appendMonitorLine(rxMessage);
    tryDecodeSpeed(rxMessage, nowMs);
    ++framesProcessed;
  }

  if (decodedSpeed_.valid &&
      decodedSpeedTimeoutMs_ != 0U &&
      (nowMs - decodedSpeed_.lastUpdateMs) > decodedSpeedTimeoutMs_) {
    decodedSpeed_ = {};
    decodedSpeedTimeoutMs_ = 0;
  }
}

bool CanManager::sendTestFrame() {
  if (!initialized_ || (backend_ == nullptr)) {
    return false;
  }

  // Keep the same test frame used in the legacy sketch so PC-side captures and
  // past notes remain directly comparable.
  twai_message_t txMessage = {};
  txMessage.identifier = 0x777;
  txMessage.extd = 0;
  txMessage.data_length_code = 4;
  txMessage.data[0] = 0x01;
  txMessage.data[1] = 0x02;
  txMessage.data[2] = 0x03;
  txMessage.data[3] = 0x04;

  return backend_->transmit(txMessage, pdMS_TO_TICKS(100));
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

void CanManager::appendMonitorLine(const twai_message_t &rxMessage) {
  char monitorLine[kMonitorLineLength] = {0};
  formatMonitorLine(rxMessage, monitorLine, sizeof(monitorLine));

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
    const twai_message_t &rxMessage,
    char *lineBuf,
    size_t lineBufSize) {
  if ((lineBuf == nullptr) || (lineBufSize == 0U)) {
    return;
  }

  char dataText[3 * 8 + 1] = {0};
  size_t offset = 0;
  for (uint8_t i = 0; i < rxMessage.data_length_code && i < 8; ++i) {
    offset += static_cast<size_t>(snprintf(
        dataText + offset,
        sizeof(dataText) - offset,
        (i == 0) ? "%02X" : " %02X",
        rxMessage.data[i]));
    if (offset >= sizeof(dataText)) {
      break;
    }
  }

  snprintf(
      lineBuf,
      lineBufSize,
      "0x%03X [%d] %s",
      rxMessage.identifier,
      rxMessage.data_length_code,
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

bool CanManager::tryDecodeSpeed(const twai_message_t &rxMessage, uint32_t nowMs) {
  // Speed decoding is optional and sits on top of the raw receive path.
  // Even when no decoder is enabled, CAN RX logging/monitoring should still
  // work exactly like the legacy receive loop.
  for (const CanSpeedDecoderConfig &config : kSpeedDecoders) {
    if (!config.enabled) {
      continue;
    }
    if (config.identifier != rxMessage.identifier) {
      continue;
    }
    if (config.extended != (rxMessage.extd != 0)) {
      continue;
    }
    if (config.lengthBytes == 0U) {
      continue;
    }
    if ((config.startByte + config.lengthBytes) > rxMessage.data_length_code) {
      continue;
    }

    const uint32_t rawValue = readUnsignedValue(
        rxMessage.data,
        config.startByte,
        config.lengthBytes,
        config.endian);
    const float speedKmh = (static_cast<float>(rawValue) * config.scale) + config.offset;
    if (!isReasonableSpeed(speedKmh)) {
      continue;
    }

    decodedSpeed_.valid = true;
    decodedSpeed_.speedKmh = speedKmh;
    decodedSpeed_.identifier = rxMessage.identifier;
    decodedSpeed_.lastUpdateMs = nowMs;
    decodedSpeed_.decoderName = config.name;
    decodedSpeedTimeoutMs_ = config.timeoutMs;
    return true;
  }

  return false;
}
