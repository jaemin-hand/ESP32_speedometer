#include "can_fd_backend.h"

#include <Arduino.h>
#include <SPI.h>

namespace {

constexpr uint8_t kSpiCmdReset = 0x0;
constexpr uint8_t kSpiCmdWrite = 0x2;
constexpr uint8_t kSpiCmdRead = 0x3;
constexpr uint32_t kSpiClockHz = 1000000;
constexpr uint16_t kRegOsc = 0x0E00;
constexpr uint16_t kRegIocon = 0x0E04;

SPIClass &fdSpi() {
  return SPI;
}

}  // namespace

const char *CanFdBackend::backendName() const {
  return "CAN_FD";
}

CanBackendCapabilities CanFdBackend::capabilities() const {
  return {
      .supportsClassicCan = false,
      .supportsCanFd = true,
      .backendReady = backendReady_,
  };
}

CanBackendRequirements CanFdBackend::requirements() const {
  return {
      .requiresExternalController = true,
      .requiresExternalTransceiver = false,
      .maxPayloadBytes = 64,
      .driverFamily = "External SPI CAN-FD controller driver (MCP2517FD stage-1 bring-up)",
      .expectedHardware = "MCP2517FD board with integrated transceiver",
      .nextBringupStep = "Confirm SPI register access first, then add polling-mode RX path and map Tucson FD candidates into decode flow",
  };
}

const char *CanFdBackend::diagnosticText() const {
  return diagnosticTextBuf_[0] != '\0'
             ? diagnosticTextBuf_
             : "CAN-FD backend not initialized";
}

bool CanFdBackend::begin(const CanBackendOptions &options) {
  options_ = options;
  backendReady_ = false;
  spiInitialized_ = false;
  oscReg_ = 0;
  ioconReg_ = 0;
  snprintf(
      diagnosticTextBuf_,
      sizeof(diagnosticTextBuf_),
      "CAN-FD backend idle: waiting for SPI bring-up");

  Serial.println("CAN backend: CAN_FD");
  Serial.printf(
      "CAN-FD SPI pins: SCK=%d MOSI=%d MISO=%d CS=%d IRQ=%d RST=%d STBY=%d\n",
      static_cast<int>(options.spiSckPin),
      static_cast<int>(options.spiMosiPin),
      static_cast<int>(options.spiMisoPin),
      static_cast<int>(options.spiCsPin),
      static_cast<int>(options.irqPin),
      static_cast<int>(options.resetPin),
      static_cast<int>(options.standbyPin));
  Serial.println("CAN-FD note: stage 1 checks MCP2517FD SPI link only; RX/TX engine is not implemented yet");

  if (options.spiSckPin == GPIO_NUM_NC || options.spiMosiPin == GPIO_NUM_NC ||
      options.spiMisoPin == GPIO_NUM_NC || options.spiCsPin == GPIO_NUM_NC) {
    snprintf(
        diagnosticTextBuf_,
        sizeof(diagnosticTextBuf_),
        "CAN-FD SPI pin configuration incomplete");
    return false;
  }

  pinMode(static_cast<uint8_t>(options.spiCsPin), OUTPUT);
  digitalWrite(static_cast<uint8_t>(options.spiCsPin), HIGH);

  if (options.resetPin != GPIO_NUM_NC) {
    pinMode(static_cast<uint8_t>(options.resetPin), OUTPUT);
    digitalWrite(static_cast<uint8_t>(options.resetPin), HIGH);
  }
  if (options.standbyPin != GPIO_NUM_NC) {
    pinMode(static_cast<uint8_t>(options.standbyPin), OUTPUT);
    digitalWrite(static_cast<uint8_t>(options.standbyPin), LOW);
  }

  fdSpi().begin(
      static_cast<int>(options.spiSckPin),
      static_cast<int>(options.spiMisoPin),
      static_cast<int>(options.spiMosiPin),
      static_cast<int>(options.spiCsPin));
  spiInitialized_ = true;
  delay(5);

  if (!spiResetDevice()) {
    snprintf(
        diagnosticTextBuf_,
        sizeof(diagnosticTextBuf_),
        "CAN-FD SPI reset command failed");
    return false;
  }
  delay(5);

  if (!spiReadRegister32(kRegOsc, &oscReg_) ||
      !spiReadRegister32(kRegIocon, &ioconReg_)) {
    snprintf(
        diagnosticTextBuf_,
        sizeof(diagnosticTextBuf_),
        "CAN-FD SPI read failed: OSC=0x%08lX IOCON=0x%08lX",
        static_cast<unsigned long>(oscReg_),
        static_cast<unsigned long>(ioconReg_));
    return false;
  }

  const bool looksLikeRealDevice =
      (oscReg_ != 0x00000000UL) && (oscReg_ != 0xFFFFFFFFUL) &&
      ((ioconReg_ & 0x03U) == 0x03U);
  backendReady_ = looksLikeRealDevice;

  snprintf(
      diagnosticTextBuf_,
      sizeof(diagnosticTextBuf_),
      backendReady_
          ? "CAN-FD SPI link OK: OSC=0x%08lX IOCON=0x%08lX (RX/TX engine pending)"
          : "CAN-FD SPI link suspicious: OSC=0x%08lX IOCON=0x%08lX",
      static_cast<unsigned long>(oscReg_),
      static_cast<unsigned long>(ioconReg_));
  Serial.println(diagnosticTextBuf_);
  return backendReady_;
}

void CanFdBackend::end() {
  if (spiInitialized_) {
    fdSpi().end();
  }
  spiInitialized_ = false;
  backendReady_ = false;
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

bool CanFdBackend::spiResetDevice() {
  if (!spiInitialized_) {
    return false;
  }

  SPISettings settings(kSpiClockHz, MSBFIRST, SPI_MODE0);
  fdSpi().beginTransaction(settings);
  digitalWrite(static_cast<uint8_t>(options_.spiCsPin), LOW);
  fdSpi().transfer16(buildInstruction(kSpiCmdReset, 0x000));
  digitalWrite(static_cast<uint8_t>(options_.spiCsPin), HIGH);
  fdSpi().endTransaction();
  return true;
}

bool CanFdBackend::spiReadRegister32(uint16_t address, uint32_t *value) {
  if (!spiInitialized_ || (value == nullptr)) {
    return false;
  }

  SPISettings settings(kSpiClockHz, MSBFIRST, SPI_MODE0);
  uint32_t readValue = 0;

  fdSpi().beginTransaction(settings);
  digitalWrite(static_cast<uint8_t>(options_.spiCsPin), LOW);
  fdSpi().transfer16(buildInstruction(kSpiCmdRead, address));
  for (uint8_t i = 0; i < 4U; ++i) {
    readValue <<= 8U;
    readValue |= fdSpi().transfer(0x00);
  }
  digitalWrite(static_cast<uint8_t>(options_.spiCsPin), HIGH);
  fdSpi().endTransaction();

  *value = readValue;
  return true;
}

uint16_t CanFdBackend::buildInstruction(uint8_t command, uint16_t address) {
  return static_cast<uint16_t>(((command & 0x0FU) << 12U) | (address & 0x0FFFU));
}
