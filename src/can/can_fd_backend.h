#pragma once

#include <Arduino.h>

#include "can_backend.h"

class CanFdBackend final : public ICanBackend {
public:
  const char *backendName() const override;
  CanBackendCapabilities capabilities() const override;
  CanBackendRequirements requirements() const override;
  const char *diagnosticText() const override;
  bool begin(const CanBackendOptions &options) override;
  void end() override;
  bool receive(CanFrame *rxFrame) override;
  bool transmit(const CanFrame &txFrame, TickType_t timeoutTicks) override;
  bool getStatus(twai_status_info_t *statusInfo) const override;

private:
  bool spiResetDevice();
  bool spiReadRegister32(uint16_t address, uint32_t *value);
  static uint16_t buildInstruction(uint8_t command, uint16_t address);

  CanBackendOptions options_ = {};
  bool spiInitialized_ = false;
  bool backendReady_ = false;
  uint32_t oscReg_ = 0;
  uint32_t ioconReg_ = 0;
  char diagnosticTextBuf_[192] = {};
};
