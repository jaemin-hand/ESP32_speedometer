#include "can_fd_backend.h"

#include <Arduino.h>

const char *CanFdBackend::backendName() const {
  return "CAN_FD";
}

CanBackendCapabilities CanFdBackend::capabilities() const {
  return {
      .supportsClassicCan = false,
      .supportsCanFd = false,
      .backendReady = false,
  };
}

CanBackendRequirements CanFdBackend::requirements() const {
  return {
      .requiresExternalController = true,
      .requiresExternalTransceiver = true,
      .maxPayloadBytes = 64,
      .driverFamily = "External SPI CAN-FD controller driver (planned MCP2518FD path)",
      .expectedHardware = "MCP2518FD + CAN-FD transceiver",
      .nextBringupStep = "Wire tentative SPI pins, start with polling-mode bring-up, then map Tucson FD candidates into receive/decode flow",
  };
}

const char *CanFdBackend::diagnosticText() const {
  return "CAN-FD backend placeholder only: planned path is MCP2518FD over SPI plus CAN-FD transceiver; current TWAI path remains classic CAN only";
}

bool CanFdBackend::begin(const CanBackendOptions &options) {
  Serial.println("CAN backend: CAN_FD");
  Serial.println(diagnosticText());
  Serial.printf(
      "CAN-FD planned SPI pins: SCK=%d MOSI=%d MISO=%d CS=%d IRQ=%d RST=%d STBY=%d\n",
      static_cast<int>(options.spiSckPin),
      static_cast<int>(options.spiMosiPin),
      static_cast<int>(options.spiMisoPin),
      static_cast<int>(options.spiCsPin),
      static_cast<int>(options.irqPin),
      static_cast<int>(options.resetPin),
      static_cast<int>(options.standbyPin));
  Serial.println("CAN-FD note: first bring-up is planned in polling mode; IRQ is optional for phase 1");
  return false;
}

void CanFdBackend::end() {
}

bool CanFdBackend::receive(CanFrame *rxFrame) {
  (void)rxFrame;
  return false;
}

bool CanFdBackend::transmit(const CanFrame &txFrame, TickType_t timeoutTicks) {
  (void)txFrame;
  (void)timeoutTicks;
  return false;
}

bool CanFdBackend::getStatus(twai_status_info_t *statusInfo) const {
  (void)statusInfo;
  return false;
}
