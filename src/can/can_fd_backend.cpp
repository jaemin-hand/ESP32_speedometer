#include "can_fd_backend.h"

#include <Arduino.h>

const char *CanFdBackend::backendName() const {
  return "CAN_FD";
}

bool CanFdBackend::begin(const CanBackendOptions &options) {
  (void)options;

  Serial.println("CAN backend: CAN_FD");
  Serial.println("CAN-FD backend is not implemented on the current TWAI path yet");
  return false;
}

void CanFdBackend::end() {
}

bool CanFdBackend::receive(twai_message_t *rxMessage) {
  (void)rxMessage;
  return false;
}

bool CanFdBackend::transmit(const twai_message_t &txMessage, TickType_t timeoutTicks) {
  (void)txMessage;
  (void)timeoutTicks;
  return false;
}

bool CanFdBackend::getStatus(twai_status_info_t *statusInfo) const {
  (void)statusInfo;
  return false;
}
