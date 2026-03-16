#include "can_manager.h"

#include <math.h>

namespace {

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
  twai_general_config_t gConfig =
      TWAI_GENERAL_CONFIG_DEFAULT(txPin, rxPin, TWAI_MODE_NORMAL);
  twai_timing_config_t tConfig = TWAI_TIMING_CONFIG_500KBITS();

  twai_filter_config_t fConfig;
  fConfig.acceptance_code = 0;
  fConfig.acceptance_mask = 0;
  fConfig.single_filter = true;

  if (twai_driver_install(&gConfig, &tConfig, &fConfig) != ESP_OK) {
    return false;
  }

  return twai_start() == ESP_OK;
}

void CanManager::poll(uint32_t nowMs) {
  twai_message_t rxMessage;
  while (twai_receive(&rxMessage, pdMS_TO_TICKS(0)) == ESP_OK) {
    lastRxMs_ = nowMs;
    Serial.printf("CAN received, ID: 0x%X, len: %d\n", rxMessage.identifier, rxMessage.data_length_code);
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
