#include "classic_can_backend.h"

#include <Arduino.h>

const char *ClassicCanBackend::backendName() const {
  return "CLASSIC_CAN";
}

CanBackendCapabilities ClassicCanBackend::capabilities() const {
  return {
      .supportsClassicCan = true,
      .supportsCanFd = false,
      .backendReady = true,
  };
}

CanBackendRequirements ClassicCanBackend::requirements() const {
  return {
      .requiresExternalController = false,
      .requiresExternalTransceiver = true,
      .maxPayloadBytes = 8,
      .driverFamily = "ESP32 TWAI",
      .expectedHardware = "Explicit classic CAN transceiver on configured TX/RX pins",
      .nextBringupStep = "Classic CAN baseline already ready",
  };
}

const char *ClassicCanBackend::diagnosticText() const {
  return "TWAI classic CAN backend ready (legacy baseline path)";
}

bool ClassicCanBackend::begin(const CanBackendOptions &options) {
  end();

  if (options.txPin == GPIO_NUM_NC || options.rxPin == GPIO_NUM_NC) {
    Serial.println("Classic CAN backend requires explicit TX/RX pins");
    return false;
  }

  twai_general_config_t gConfig =
      TWAI_GENERAL_CONFIG_DEFAULT(options.txPin, options.rxPin, TWAI_MODE_NORMAL);
  twai_timing_config_t tConfig = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t fConfig = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  Serial.println("CAN backend: CLASSIC_CAN (TWAI)");
  Serial.println("TWAI mode: NORMAL (legacy baseline)");
  Serial.println("TWAI timing: default 500 kbps (legacy baseline)");
  Serial.println("TWAI filter: ACCEPT_ALL (monitor all incoming IDs)");

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

void ClassicCanBackend::end() {
  if (!initialized_) {
    return;
  }

  twai_stop();
  twai_driver_uninstall();
  initialized_ = false;
}

bool ClassicCanBackend::receive(CanFrame *rxFrame) {
  if (!initialized_ || (rxFrame == nullptr)) {
    return false;
  }

  twai_message_t rxMessage = {};
  if (twai_receive(&rxMessage, pdMS_TO_TICKS(0)) != ESP_OK) {
    return false;
  }

  rxFrame->clear();
  rxFrame->identifier = rxMessage.identifier;
  rxFrame->extended = (rxMessage.extd != 0);
  rxFrame->fdFormat = false;
  rxFrame->bitrateSwitch = false;
  rxFrame->dataLength = rxMessage.data_length_code;
  memcpy(rxFrame->data, rxMessage.data, rxMessage.data_length_code);
  return true;
}

bool ClassicCanBackend::transmit(const CanFrame &txFrame, TickType_t timeoutTicks) {
  if (!initialized_) {
    return false;
  }

  if (txFrame.fdFormat || (txFrame.dataLength > 8U)) {
    Serial.println("Classic CAN backend cannot transmit CAN-FD frames");
    return false;
  }

  twai_message_t txMessage = {};
  txMessage.identifier = txFrame.identifier;
  txMessage.extd = txFrame.extended ? 1 : 0;
  txMessage.data_length_code = txFrame.dataLength;
  memcpy(txMessage.data, txFrame.data, txFrame.dataLength);
  return twai_transmit(&txMessage, timeoutTicks) == ESP_OK;
}

bool ClassicCanBackend::getStatus(twai_status_info_t *statusInfo) const {
  if (!initialized_ || (statusInfo == nullptr)) {
    return false;
  }

  return twai_get_status_info(statusInfo) == ESP_OK;
}
