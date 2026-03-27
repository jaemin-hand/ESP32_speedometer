#pragma once

#include "can_backend.h"

class ClassicCanBackend final : public ICanBackend {
public:
  const char *backendName() const override;
  bool begin(const CanBackendOptions &options) override;
  void end() override;
  bool receive(twai_message_t *rxMessage) override;
  bool transmit(const twai_message_t &txMessage, TickType_t timeoutTicks) override;
  bool getStatus(twai_status_info_t *statusInfo) const override;

private:
  bool initialized_ = false;
};
