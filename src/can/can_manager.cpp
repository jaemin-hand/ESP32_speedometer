#include "can_manager.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

namespace {

constexpr bool kCanListenOnlyDiagnosticMode = false;

// Replace this template entry with the actual speed signal decoded from
// canAnalyser. Example: ID 0x123, bytes 0-1 little-endian, scale 0.01 km/h.
constexpr CanSpeedDecoderConfig kSpeedDecoders[] = {
    {
        false,
        "speed_template",
        0x000,
        false,
        0,
        2,
        CAN_SIGNAL_LITTLE_ENDIAN,
        0.01f,
        0.0f,
        200,
    },
};

bool isReasonableSpeed(float speedKmh) {
  return isfinite(speedKmh) && speedKmh >= 0.0f && speedKmh <= 400.0f;
}

}  // namespace

bool CanManager::begin(gpio_num_t txPin, gpio_num_t rxPin) {
  initialized_ = false;
  const twai_mode_t twaiMode =
      kCanListenOnlyDiagnosticMode ? TWAI_MODE_LISTEN_ONLY : TWAI_MODE_NORMAL;
  twai_general_config_t gConfig =
      TWAI_GENERAL_CONFIG_DEFAULT(txPin, rxPin, twaiMode);
  twai_timing_config_t tConfig = TWAI_TIMING_CONFIG_500KBITS();

  // Restore the last known-good init path from commit 57a26fbd.
  twai_filter_config_t fConfig = {};
  fConfig.acceptance_code = (0x100 << 21);
  fConfig.acceptance_mask = ~(0x7FF << 21);
  fConfig.single_filter = true;

  Serial.printf(
      "TWAI mode: %s\n",
      kCanListenOnlyDiagnosticMode ? "LISTEN_ONLY" : "NORMAL");
  Serial.println("TWAI timing: using known-good 500 kbps timing from 57a26fbd");
  Serial.printf(
      "TWAI filter: code=0x%08lX mask=0x%08lX single=%d\n",
      static_cast<unsigned long>(fConfig.acceptance_code),
      static_cast<unsigned long>(fConfig.acceptance_mask),
      static_cast<int>(fConfig.single_filter));

  if (twai_driver_install(&gConfig, &tConfig, &fConfig) != ESP_OK) {
    return false;
  }

  if (twai_start() != ESP_OK) {
    twai_driver_uninstall();
    return false;
  }

  initialized_ = true;
  return true;
}

bool CanManager::isInitialized() const {
  return initialized_;
}

void CanManager::poll(uint32_t nowMs) {
  if (!initialized_) {
    return;
  }

  twai_message_t rxMessage;
  while (twai_receive(&rxMessage, pdMS_TO_TICKS(0)) == ESP_OK) {
    char monitorLine[kMonitorLineLength] = {0};
    lastRxMs_ = nowMs;
    formatMonitorLine(rxMessage, monitorLine, sizeof(monitorLine));
    Serial.printf("CAN RX: %s\n", monitorLine);
    appendMonitorLine(rxMessage);
    tryDecodeSpeed(rxMessage, nowMs);
  }

  if (decodedSpeed_.valid &&
      decodedSpeedTimeoutMs_ != 0U &&
      (nowMs - decodedSpeed_.lastUpdateMs) > decodedSpeedTimeoutMs_) {
    decodedSpeed_ = {};
    decodedSpeedTimeoutMs_ = 0;
  }
}

bool CanManager::sendTestFrame() {
  if (!initialized_) {
    return false;
  }

  twai_message_t txMessage = {};
  txMessage.identifier = 0x777;
  txMessage.extd = 0;
  txMessage.data_length_code = 4;
  txMessage.data[0] = 0x01;
  txMessage.data[1] = 0x02;
  txMessage.data[2] = 0x03;
  txMessage.data[3] = 0x04;

  return twai_transmit(&txMessage, pdMS_TO_TICKS(100)) == ESP_OK;
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
    Serial.println("TWAI not initialized");
    return;
  }

  twai_status_info_t status = {};
  if (twai_get_status_info(&status) != ESP_OK) {
    Serial.println("TWAI status read failed");
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
