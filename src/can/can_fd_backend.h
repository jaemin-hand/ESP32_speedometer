#pragma once

#include <Arduino.h>

#include "driver/spi_master.h"

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
  bool spiTransfer(
      const uint8_t *txData,
      uint8_t *rxData,
      size_t length,
      uint8_t spiMode = 0);
  bool spiReadRegister32(uint16_t address, uint32_t *value, uint8_t spiMode = 0);
  bool spiWriteRegister32(uint16_t address, uint32_t value, uint8_t spiMode = 0);
  bool spiReadBytes(uint16_t address, uint8_t *data, size_t length, uint8_t spiMode = 0);
  static uint16_t buildInstruction(uint8_t command, uint16_t address);
  bool looksLikeReadableRegister(uint32_t value) const;
  void refreshLinkDiagnostics(uint32_t nowMs);
  bool setOperationMode(uint8_t reqop, uint32_t timeoutMs);
  bool initializeControllerForNormalRx();
  bool pollReceiveFifo(CanFrame *rxFrame);
  static uint8_t dlcToLength(uint8_t dlc);

  CanBackendOptions options_ = {};
  bool spiInitialized_ = false;
  bool spiLinkReady_ = false;
  bool controllerConfigured_ = false;
  spi_device_handle_t spiHandle_ = nullptr;
  uint32_t oscReg_ = 0;
  uint32_t ioconReg_ = 0;
  uint32_t lastProbeMs_ = 0;
  char diagnosticTextBuf_[192] = {};
};
