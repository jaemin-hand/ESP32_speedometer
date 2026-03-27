#pragma once

#include "can_backend.h"

class ClassicCanBackend final : public ICanBackend {
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
  bool initialized_ = false;
};
