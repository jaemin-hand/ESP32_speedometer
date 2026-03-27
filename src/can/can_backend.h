#pragma once

#include <stdint.h>

#include "driver/twai.h"

enum CanBackendType : uint8_t {
  CAN_BACKEND_CLASSIC = 0,
  CAN_BACKEND_FD = 1,
};

struct CanBackendOptions {
  gpio_num_t txPin = GPIO_NUM_NC;
  gpio_num_t rxPin = GPIO_NUM_NC;
};

class ICanBackend {
public:
  virtual ~ICanBackend() = default;

  virtual const char *backendName() const = 0;
  virtual bool begin(const CanBackendOptions &options) = 0;
  virtual void end() = 0;
  virtual bool receive(twai_message_t *rxMessage) = 0;
  virtual bool transmit(const twai_message_t &txMessage, TickType_t timeoutTicks) = 0;
  virtual bool getStatus(twai_status_info_t *statusInfo) const = 0;
};
