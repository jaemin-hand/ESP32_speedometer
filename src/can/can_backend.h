#pragma once

#include <stdint.h>

#include <string.h>

#include "driver/twai.h"

enum CanBackendType : uint8_t {
  CAN_BACKEND_CLASSIC = 0,
  CAN_BACKEND_FD = 1,
};

struct CanBackendOptions {
  gpio_num_t txPin = GPIO_NUM_NC;
  gpio_num_t rxPin = GPIO_NUM_NC;
  gpio_num_t spiSckPin = GPIO_NUM_NC;
  gpio_num_t spiMosiPin = GPIO_NUM_NC;
  gpio_num_t spiMisoPin = GPIO_NUM_NC;
  gpio_num_t spiCsPin = GPIO_NUM_NC;
  gpio_num_t irqPin = GPIO_NUM_NC;
  gpio_num_t resetPin = GPIO_NUM_NC;
  gpio_num_t standbyPin = GPIO_NUM_NC;
  uint32_t canClockHz = 40000000UL;
  uint32_t nominalBitRate = 500000UL;
  uint32_t dataBitRate = 2000000UL;
};

struct CanBackendCapabilities {
  bool supportsClassicCan = false;
  bool supportsCanFd = false;
  bool backendReady = false;
};

struct CanBackendRequirements {
  bool requiresExternalController = false;
  bool requiresExternalTransceiver = false;
  uint8_t maxPayloadBytes = 8;
  const char *driverFamily = "";
  const char *expectedHardware = "";
  const char *nextBringupStep = "";
};

struct CanFrame {
  uint32_t identifier = 0;
  bool extended = false;
  bool fdFormat = false;
  bool bitrateSwitch = false;
  uint8_t dataLength = 0;
  uint8_t data[64] = {0};

  void clear() {
    identifier = 0;
    extended = false;
    fdFormat = false;
    bitrateSwitch = false;
    dataLength = 0;
    memset(data, 0, sizeof(data));
  }
};

class ICanBackend {
public:
  virtual ~ICanBackend() = default;

  virtual const char *backendName() const = 0;
  virtual CanBackendCapabilities capabilities() const = 0;
  virtual CanBackendRequirements requirements() const = 0;
  virtual const char *diagnosticText() const = 0;
  virtual bool begin(const CanBackendOptions &options) = 0;
  virtual void end() = 0;
  virtual bool receive(CanFrame *rxFrame) = 0;
  virtual bool transmit(const CanFrame &txFrame, TickType_t timeoutTicks) = 0;
  virtual bool getStatus(twai_status_info_t *statusInfo) const = 0;
};
