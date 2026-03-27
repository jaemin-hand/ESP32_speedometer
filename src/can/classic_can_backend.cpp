#include "classic_can_backend.h"

#include <Arduino.h>

const char *ClassicCanBackend::backendName() const {
  return "CLASSIC_CAN";
}

bool ClassicCanBackend::begin(const CanBackendOptions &options) {
  end();

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

bool ClassicCanBackend::receive(twai_message_t *rxMessage) {
  if (!initialized_ || (rxMessage == nullptr)) {
    return false;
  }

  return twai_receive(rxMessage, pdMS_TO_TICKS(0)) == ESP_OK;
}

bool ClassicCanBackend::transmit(const twai_message_t &txMessage, TickType_t timeoutTicks) {
  if (!initialized_) {
    return false;
  }

  return twai_transmit(&txMessage, timeoutTicks) == ESP_OK;
}

bool ClassicCanBackend::getStatus(twai_status_info_t *statusInfo) const {
  if (!initialized_ || (statusInfo == nullptr)) {
    return false;
  }

  return twai_get_status_info(statusInfo) == ESP_OK;
}
