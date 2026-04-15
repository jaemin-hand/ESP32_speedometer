#include "can_fd_backend.h"

#include <Arduino.h>

#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"

namespace {

constexpr uint8_t kSpiCmdReset = 0x0;
constexpr uint8_t kSpiCmdWrite = 0x2;
constexpr uint8_t kSpiCmdRead = 0x3;
constexpr uint32_t kSpiClockHz = 10000;
constexpr uint8_t kSpiMode0 = 0;
constexpr uint32_t kProbeIntervalMs = 1000;
constexpr spi_host_device_t kSpiHost = SPI3_HOST;
constexpr size_t kRegisterTransferBytes = 6U;
constexpr size_t kMaxSpiReadTransferBytes = 66U;

// Register map (MCP2517FD / MCP2518FD family)
constexpr uint16_t kRegOsc = 0x0E00;
constexpr uint16_t kRegIocon = 0x0E04;
constexpr uint16_t kRegC1Con = 0x000;
constexpr uint16_t kRegC1NbtCfg = 0x004;
constexpr uint16_t kRegC1DbtCfg = 0x008;
constexpr uint16_t kRegC1Tdc = 0x00C;
constexpr uint16_t kRegC1Int = 0x01C;
constexpr uint16_t kRegC1RxIf = 0x020;
constexpr uint16_t kRegC1TefCon = 0x040;
constexpr uint16_t kRegC1FifoBa = 0x04C;
constexpr uint16_t kRegC1TxqCon = 0x050;
constexpr uint16_t kRegC1FifoCon1 = 0x05C;
constexpr uint16_t kRegC1FifoSta1 = 0x060;
constexpr uint16_t kRegC1FifoUa1 = 0x064;
constexpr uint16_t kRegC1FltCon0 = 0x1D0;
constexpr uint16_t kRegC1FltObj0 = 0x1F0;
constexpr uint16_t kRegC1Mask0 = 0x1F4;

constexpr uint32_t kC1ConReqopMask = 0x07000000UL;
constexpr uint32_t kC1ConOpmodMask = 0x00E00000UL;
constexpr uint32_t kC1ConTxqEnMask = 0x00100000UL;
constexpr uint32_t kC1ConStefMask = 0x00080000UL;
constexpr uint32_t kC1ConBrsDisMask = 0x00001000UL;

constexpr uint8_t kReqopNormal = 0x0;
constexpr uint8_t kReqopConfig = 0x4;
constexpr uint32_t kFifoConUincMask = 0x00000100UL;
constexpr uint32_t kFifoConFresetMask = 0x00000400UL;
constexpr uint32_t kFifoConFsizeShift = 24U;
constexpr uint32_t kFifoConPlsizeShift = 29U;
constexpr uint32_t kFifoConPlsize64 = 7U;

constexpr uint32_t kFifoStaNotEmptyMask = 0x00000001UL;

constexpr uint32_t kTdcModeAuto = 0x2U;
constexpr uint32_t kMessageRamBase = 0x00000400UL;
constexpr uint8_t kRxHeaderBytes = 8;
constexpr uint32_t kCanClock40MHz = 40000000UL;
constexpr uint32_t kCanClock20MHz = 20000000UL;
constexpr uint32_t kNominal500K = 500000UL;
constexpr uint32_t kData2M = 2000000UL;

struct CanFdTimingRegisters {
  uint32_t nominalBitTiming = 0;
  uint32_t dataBitTiming = 0;
  uint32_t tdcConfig = 0;
  const char *label = "unsupported";
};

constexpr uint32_t makeCiNbtCfg(
    uint8_t sjw,
    uint8_t tseg2,
    uint8_t tseg1,
    uint8_t brp) {
  return static_cast<uint32_t>(sjw) |
         (static_cast<uint32_t>(tseg2) << 8U) |
         (static_cast<uint32_t>(tseg1) << 16U) |
         (static_cast<uint32_t>(brp) << 24U);
}

constexpr uint32_t makeCiDbtCfg(
    uint8_t sjw,
    uint8_t tseg2,
    uint8_t tseg1,
    uint8_t brp) {
  return makeCiNbtCfg(sjw, tseg2, tseg1, brp);
}

constexpr uint32_t makeCiTdc(uint8_t mode, uint8_t tdco) {
  return (static_cast<uint32_t>(mode) << 16U) |
         (static_cast<uint32_t>(tdco) << 8U);
}

constexpr CanFdTimingRegisters kTiming40MHz500K2M = {
    .nominalBitTiming = makeCiNbtCfg(15U, 15U, 62U, 0U),
    .dataBitTiming = makeCiDbtCfg(3U, 3U, 14U, 0U),
    .tdcConfig = makeCiTdc(kTdcModeAuto, 15U),
    .label = "40MHz / 500k / 2M",
};

constexpr CanFdTimingRegisters kTiming20MHz500K2M = {
    .nominalBitTiming = makeCiNbtCfg(7U, 7U, 30U, 0U),
    .dataBitTiming = makeCiDbtCfg(1U, 1U, 6U, 0U),
    .tdcConfig = makeCiTdc(kTdcModeAuto, 7U),
    .label = "20MHz / 500k / 2M",
};

bool resolveTimingRegisters(
    uint32_t canClockHz,
    uint32_t nominalBitRate,
    uint32_t dataBitRate,
    CanFdTimingRegisters *registers) {
  if (registers == nullptr) {
    return false;
  }
  if (nominalBitRate != kNominal500K || dataBitRate != kData2M) {
    return false;
  }

  if (canClockHz == kCanClock40MHz) {
    *registers = kTiming40MHz500K2M;
    return true;
  }
  if (canClockHz == kCanClock20MHz) {
    *registers = kTiming20MHz500K2M;
    return true;
  }

  return false;
}

size_t roundUpToWord(size_t length) {
  return (length + 3U) & ~static_cast<size_t>(0x03U);
}

inline void setChipSelect(gpio_num_t pin, bool active) {
  gpio_set_level(pin, active ? 0 : 1);
}

inline const char *safeEspErrName(esp_err_t err) {
  const char *name = esp_err_to_name(err);
  return name != nullptr ? name : "UNKNOWN";
}

}  // namespace

const char *CanFdBackend::backendName() const {
  return "CAN_FD";
}

CanBackendCapabilities CanFdBackend::capabilities() const {
  return {
      .supportsClassicCan = true,
      .supportsCanFd = true,
      .backendReady = controllerConfigured_,
  };
}

CanBackendRequirements CanFdBackend::requirements() const {
  return {
      .requiresExternalController = true,
      .requiresExternalTransceiver = false,
      .maxPayloadBytes = 64,
      .driverFamily = "External SPI CAN-FD controller driver (MCP2517FD stage-2 polling RX)",
      .expectedHardware = "MCP2517FD board with integrated transceiver",
      .nextBringupStep = "Confirm raw RX first, then tune bitrate/data-rate and map FD candidates into decode flow",
  };
}

const char *CanFdBackend::diagnosticText() const {
  return diagnosticTextBuf_[0] != '\0'
             ? diagnosticTextBuf_
             : "CAN-FD backend not initialized";
}

bool CanFdBackend::begin(const CanBackendOptions &options) {
  options_ = options;
  spiLinkReady_ = false;
  controllerConfigured_ = false;
  spiInitialized_ = false;
  spiHandle_ = nullptr;
  oscReg_ = 0;
  ioconReg_ = 0;
  lastProbeMs_ = 0;
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
  Serial.printf(
      "CAN-FD bus setup: host=%d controller_clk=%lu nominal=%lu data=%lu\n",
      static_cast<int>(kSpiHost),
      static_cast<unsigned long>(options.canClockHz),
      static_cast<unsigned long>(options.nominalBitRate),
      static_cast<unsigned long>(options.dataBitRate));
  Serial.println("CAN-FD note: stage 2 uses ESP-IDF spi_master in mode 0 and enables polling RX in normal mode (ACK enabled)");

  if (options.spiSckPin == GPIO_NUM_NC || options.spiMosiPin == GPIO_NUM_NC ||
      options.spiMisoPin == GPIO_NUM_NC || options.spiCsPin == GPIO_NUM_NC) {
    snprintf(
        diagnosticTextBuf_,
        sizeof(diagnosticTextBuf_),
        "CAN-FD SPI pin configuration incomplete");
    return false;
  }

  if (options.resetPin != GPIO_NUM_NC) {
    pinMode(static_cast<uint8_t>(options.resetPin), OUTPUT);
    digitalWrite(static_cast<uint8_t>(options.resetPin), HIGH);
  }
  if (options.standbyPin != GPIO_NUM_NC) {
    pinMode(static_cast<uint8_t>(options.standbyPin), OUTPUT);
    digitalWrite(static_cast<uint8_t>(options.standbyPin), LOW);
  }

  gpio_reset_pin(options.spiCsPin);
  gpio_set_direction(options.spiCsPin, GPIO_MODE_OUTPUT);
  gpio_set_pull_mode(options.spiCsPin, GPIO_FLOATING);
  gpio_set_level(options.spiCsPin, 1);

  spi_bus_config_t busConfig = {};
  busConfig.mosi_io_num = static_cast<int>(options.spiMosiPin);
  busConfig.miso_io_num = static_cast<int>(options.spiMisoPin);
  busConfig.sclk_io_num = static_cast<int>(options.spiSckPin);
  busConfig.quadwp_io_num = -1;
  busConfig.quadhd_io_num = -1;
  busConfig.data4_io_num = -1;
  busConfig.data5_io_num = -1;
  busConfig.data6_io_num = -1;
  busConfig.data7_io_num = -1;
  busConfig.max_transfer_sz = static_cast<int>(kMaxSpiReadTransferBytes);
  busConfig.flags = SPICOMMON_BUSFLAG_MASTER;
  busConfig.intr_flags = 0;

  esp_err_t err = spi_bus_initialize(kSpiHost, &busConfig, SPI_DMA_DISABLED);
  if (err != ESP_OK) {
    snprintf(
        diagnosticTextBuf_,
        sizeof(diagnosticTextBuf_),
        "CAN-FD SPI bus init failed: %s (%d)",
        safeEspErrName(err),
        static_cast<int>(err));
    Serial.println(diagnosticTextBuf_);
    return false;
  }

  spi_device_interface_config_t devConfig = {};
  devConfig.command_bits = 0;
  devConfig.address_bits = 0;
  devConfig.dummy_bits = 0;
  devConfig.mode = kSpiMode0;
  devConfig.clock_speed_hz = static_cast<int>(kSpiClockHz);
  devConfig.spics_io_num = -1;
  devConfig.queue_size = 1;
  devConfig.flags = 0;

  err = spi_bus_add_device(kSpiHost, &devConfig, &spiHandle_);
  if (err != ESP_OK) {
    spi_bus_free(kSpiHost);
    snprintf(
        diagnosticTextBuf_,
        sizeof(diagnosticTextBuf_),
        "CAN-FD SPI add-device failed: %s (%d)",
        safeEspErrName(err),
        static_cast<int>(err));
    Serial.println(diagnosticTextBuf_);
    return false;
  }

  spiInitialized_ = true;
  delay(5);

  uint32_t oscMode0 = 0;
  uint32_t ioconMode0 = 0;
  uint32_t c1ConMode0 = 0;
  uint32_t oscMode0AfterReset = 0;
  uint32_t ioconMode0AfterReset = 0;
  uint32_t c1ConMode0AfterReset = 0;

  spiReadRegister32(kRegOsc, &oscMode0, kSpiMode0);
  spiReadRegister32(kRegIocon, &ioconMode0, kSpiMode0);
  spiReadRegister32(kRegC1Con, &c1ConMode0, kSpiMode0);

  spiResetDevice();
  delay(5);
  spiReadRegister32(kRegOsc, &oscMode0AfterReset, kSpiMode0);
  spiReadRegister32(kRegIocon, &ioconMode0AfterReset, kSpiMode0);
  spiReadRegister32(kRegC1Con, &c1ConMode0AfterReset, kSpiMode0);

  const bool mode0LooksGood =
      looksLikeReadableRegister(oscMode0) && looksLikeReadableRegister(ioconMode0) &&
      ((ioconMode0 & 0x03U) == 0x03U);
  const bool mode0ResetLooksGood =
      looksLikeReadableRegister(oscMode0AfterReset) &&
      looksLikeReadableRegister(ioconMode0AfterReset) &&
      ((ioconMode0AfterReset & 0x03U) == 0x03U);

  if (mode0ResetLooksGood) {
    oscReg_ = oscMode0AfterReset;
    ioconReg_ = ioconMode0AfterReset;
    spiLinkReady_ = true;
    snprintf(
        diagnosticTextBuf_,
        sizeof(diagnosticTextBuf_),
        "CAN-FD SPI link OK (mode0/reset): OSC=0x%08lX IOCON=0x%08lX C1CON=0x%08lX",
        static_cast<unsigned long>(oscReg_),
        static_cast<unsigned long>(ioconReg_),
        static_cast<unsigned long>(c1ConMode0AfterReset));
  } else if (mode0LooksGood) {
    oscReg_ = oscMode0;
    ioconReg_ = ioconMode0;
    spiLinkReady_ = true;
    snprintf(
        diagnosticTextBuf_,
        sizeof(diagnosticTextBuf_),
        "CAN-FD SPI link OK (mode0): OSC=0x%08lX IOCON=0x%08lX C1CON=0x%08lX",
        static_cast<unsigned long>(oscReg_),
        static_cast<unsigned long>(ioconReg_),
        static_cast<unsigned long>(c1ConMode0));
  } else {
    oscReg_ = oscMode0AfterReset;
    ioconReg_ = ioconMode0AfterReset;
    snprintf(
        diagnosticTextBuf_,
        sizeof(diagnosticTextBuf_),
        "CAN-FD SPI suspicious: m0=0x%08lX/0x%08lX/0x%08lX m0r=0x%08lX/0x%08lX/0x%08lX",
        static_cast<unsigned long>(oscMode0),
        static_cast<unsigned long>(ioconMode0),
        static_cast<unsigned long>(c1ConMode0),
        static_cast<unsigned long>(oscMode0AfterReset),
        static_cast<unsigned long>(ioconMode0AfterReset),
        static_cast<unsigned long>(c1ConMode0AfterReset));
  }

  Serial.println(diagnosticTextBuf_);
  if (!spiLinkReady_) {
    return true;
  }

  controllerConfigured_ = initializeControllerForNormalRx();
  if (controllerConfigured_) {
    snprintf(
        diagnosticTextBuf_,
        sizeof(diagnosticTextBuf_),
        "CAN-FD normal RX ready: OSC=0x%08lX IOCON=0x%08lX FIFO1 polling active",
        static_cast<unsigned long>(oscReg_),
        static_cast<unsigned long>(ioconReg_));
    Serial.println(diagnosticTextBuf_);
  } else {
    snprintf(
        diagnosticTextBuf_,
        sizeof(diagnosticTextBuf_),
        "CAN-FD SPI link OK but controller init failed: OSC=0x%08lX IOCON=0x%08lX",
        static_cast<unsigned long>(oscReg_),
        static_cast<unsigned long>(ioconReg_));
    Serial.println(diagnosticTextBuf_);
  }

  return true;
}

void CanFdBackend::end() {
  if (spiHandle_ != nullptr) {
    spi_bus_remove_device(spiHandle_);
    spiHandle_ = nullptr;
  }
  if (spiInitialized_) {
    spi_bus_free(kSpiHost);
  }
  spiInitialized_ = false;
  spiLinkReady_ = false;
  controllerConfigured_ = false;
}

bool CanFdBackend::receive(CanFrame *rxFrame) {
  if (controllerConfigured_ && pollReceiveFifo(rxFrame)) {
    return true;
  }

  refreshLinkDiagnostics(millis());
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

bool CanFdBackend::spiTransfer(
    const uint8_t *txData,
    uint8_t *rxData,
    size_t length,
    uint8_t spiMode) {
  (void)spiMode;
  if (!spiInitialized_ || spiHandle_ == nullptr || txData == nullptr || length == 0U) {
    return false;
  }

  spi_transaction_t transaction = {};
  transaction.length = static_cast<size_t>(length * 8U);
  transaction.tx_buffer = txData;
  transaction.rx_buffer = rxData;

  setChipSelect(options_.spiCsPin, true);
  const esp_err_t err = spi_device_polling_transmit(spiHandle_, &transaction);
  setChipSelect(options_.spiCsPin, false);

  if (err != ESP_OK) {
    snprintf(
        diagnosticTextBuf_,
        sizeof(diagnosticTextBuf_),
        "CAN-FD SPI transfer failed: %s (%d)",
        safeEspErrName(err),
        static_cast<int>(err));
    return false;
  }

  return true;
}

bool CanFdBackend::spiResetDevice() {
  if (!spiInitialized_) {
    return false;
  }

  const uint16_t instruction = buildInstruction(kSpiCmdReset, 0x000);
  uint8_t txBuffer[2] = {
      static_cast<uint8_t>(instruction >> 8U),
      static_cast<uint8_t>(instruction & 0xFFU),
  };
  return spiTransfer(txBuffer, nullptr, sizeof(txBuffer), kSpiMode0);
}

bool CanFdBackend::spiReadRegister32(uint16_t address, uint32_t *value, uint8_t spiMode) {
  if (!spiInitialized_ || (value == nullptr)) {
    return false;
  }

  const uint16_t instruction = buildInstruction(kSpiCmdRead, address);
  uint8_t txBuffer[kRegisterTransferBytes] = {
      static_cast<uint8_t>(instruction >> 8U),
      static_cast<uint8_t>(instruction & 0xFFU),
      0x00,
      0x00,
      0x00,
      0x00,
  };
  uint8_t rxBuffer[kRegisterTransferBytes] = {};

  if (!spiTransfer(txBuffer, rxBuffer, sizeof(txBuffer), spiMode)) {
    return false;
  }

  *value = static_cast<uint32_t>(rxBuffer[2]) |
           (static_cast<uint32_t>(rxBuffer[3]) << 8U) |
           (static_cast<uint32_t>(rxBuffer[4]) << 16U) |
           (static_cast<uint32_t>(rxBuffer[5]) << 24U);
  return true;
}

bool CanFdBackend::spiWriteRegister32(uint16_t address, uint32_t value, uint8_t spiMode) {
  if (!spiInitialized_) {
    return false;
  }

  const uint16_t instruction = buildInstruction(kSpiCmdWrite, address);
  uint8_t txBuffer[kRegisterTransferBytes] = {
      static_cast<uint8_t>(instruction >> 8U),
      static_cast<uint8_t>(instruction & 0xFFU),
      static_cast<uint8_t>((value >> 0U) & 0xFFU),
      static_cast<uint8_t>((value >> 8U) & 0xFFU),
      static_cast<uint8_t>((value >> 16U) & 0xFFU),
      static_cast<uint8_t>((value >> 24U) & 0xFFU),
  };

  return spiTransfer(txBuffer, nullptr, sizeof(txBuffer), spiMode);
}

bool CanFdBackend::spiReadBytes(uint16_t address, uint8_t *data, size_t length, uint8_t spiMode) {
  if (!spiInitialized_ || (data == nullptr) || (length == 0U)) {
    return false;
  }
  if ((length + 2U) > kMaxSpiReadTransferBytes) {
    return false;
  }

  const uint16_t instruction = buildInstruction(kSpiCmdRead, address);
  uint8_t txBuffer[kMaxSpiReadTransferBytes] = {};
  uint8_t rxBuffer[kMaxSpiReadTransferBytes] = {};
  txBuffer[0] = static_cast<uint8_t>(instruction >> 8U);
  txBuffer[1] = static_cast<uint8_t>(instruction & 0xFFU);

  if (!spiTransfer(txBuffer, rxBuffer, length + 2U, spiMode)) {
    return false;
  }

  memcpy(data, &rxBuffer[2], length);
  return true;
}

uint16_t CanFdBackend::buildInstruction(uint8_t command, uint16_t address) {
  return static_cast<uint16_t>(((command & 0x0FU) << 12U) | (address & 0x0FFFU));
}

bool CanFdBackend::looksLikeReadableRegister(uint32_t value) const {
  return value != 0x00000000UL && value != 0xFFFFFFFFUL;
}

void CanFdBackend::refreshLinkDiagnostics(uint32_t nowMs) {
  if (!spiInitialized_ || ((nowMs - lastProbeMs_) < kProbeIntervalMs)) {
    return;
  }

  lastProbeMs_ = nowMs;

  uint32_t oscMode0 = 0;
  uint32_t ioconMode0 = 0;
  uint32_t c1ConMode0 = 0;
  spiReadRegister32(kRegOsc, &oscMode0, kSpiMode0);
  spiReadRegister32(kRegIocon, &ioconMode0, kSpiMode0);
  spiReadRegister32(kRegC1Con, &c1ConMode0, kSpiMode0);

  oscReg_ = oscMode0;
  ioconReg_ = ioconMode0;

  if (controllerConfigured_) {
    uint32_t fifoSta1 = 0;
    spiReadRegister32(kRegC1FifoSta1, &fifoSta1, kSpiMode0);
    snprintf(
        diagnosticTextBuf_,
        sizeof(diagnosticTextBuf_),
        "CAN-FD RX ready: OSC=0x%08lX IOCON=0x%08lX C1CON=0x%08lX FIFO1STA=0x%08lX",
        static_cast<unsigned long>(oscReg_),
        static_cast<unsigned long>(ioconReg_),
        static_cast<unsigned long>(c1ConMode0),
        static_cast<unsigned long>(fifoSta1));
    return;
  }

  const bool readable =
      looksLikeReadableRegister(oscMode0) && looksLikeReadableRegister(ioconMode0);

  if (spiLinkReady_ && readable) {
    snprintf(
        diagnosticTextBuf_,
        sizeof(diagnosticTextBuf_),
        "CAN-FD SPI link ready, controller not configured: OSC=0x%08lX IOCON=0x%08lX C1CON=0x%08lX",
        static_cast<unsigned long>(oscMode0),
        static_cast<unsigned long>(ioconMode0),
        static_cast<unsigned long>(c1ConMode0));
    return;
  }

  snprintf(
      diagnosticTextBuf_,
      sizeof(diagnosticTextBuf_),
      readable
          ? "CAN-FD SPI probe: OSC=0x%08lX IOCON=0x%08lX C1CON=0x%08lX"
          : "CAN-FD SPI probe suspicious: OSC=0x%08lX IOCON=0x%08lX C1CON=0x%08lX",
      static_cast<unsigned long>(oscMode0),
      static_cast<unsigned long>(ioconMode0),
      static_cast<unsigned long>(c1ConMode0));
}

bool CanFdBackend::setOperationMode(uint8_t reqop, uint32_t timeoutMs) {
  uint32_t c1con = 0;
  if (!spiReadRegister32(kRegC1Con, &c1con, kSpiMode0)) {
    snprintf(
        diagnosticTextBuf_,
        sizeof(diagnosticTextBuf_),
        "CAN-FD mode set failed: cannot read C1CON before reqop=%u",
        static_cast<unsigned>(reqop));
    return false;
  }

  c1con &= ~kC1ConReqopMask;
  c1con |= static_cast<uint32_t>(reqop & 0x07U) << 24U;
  if (!spiWriteRegister32(kRegC1Con, c1con, kSpiMode0)) {
    snprintf(
        diagnosticTextBuf_,
        sizeof(diagnosticTextBuf_),
        "CAN-FD mode set failed: cannot write C1CON reqop=%u value=0x%08lX",
        static_cast<unsigned>(reqop),
        static_cast<unsigned long>(c1con));
    return false;
  }

  const uint32_t deadline = millis() + timeoutMs;
  while (millis() <= deadline) {
    uint32_t statusCon = 0;
    if (!spiReadRegister32(kRegC1Con, &statusCon, kSpiMode0)) {
      snprintf(
          diagnosticTextBuf_,
          sizeof(diagnosticTextBuf_),
          "CAN-FD mode set failed: cannot read C1CON while waiting reqop=%u",
          static_cast<unsigned>(reqop));
      return false;
    }
    const uint8_t opmod = static_cast<uint8_t>((statusCon & kC1ConOpmodMask) >> 21U);
    if (opmod == reqop) {
      return true;
    }
    delay(1);
  }

  uint32_t finalCon = 0;
  spiReadRegister32(kRegC1Con, &finalCon, kSpiMode0);
  snprintf(
      diagnosticTextBuf_,
      sizeof(diagnosticTextBuf_),
      "CAN-FD mode timeout: reqop=%u C1CON=0x%08lX",
      static_cast<unsigned>(reqop),
      static_cast<unsigned long>(finalCon));
  return false;
}

bool CanFdBackend::initializeControllerForNormalRx() {
  CanFdTimingRegisters timing = {};
  if (!resolveTimingRegisters(
          options_.canClockHz,
          options_.nominalBitRate,
          options_.dataBitRate,
          &timing)) {
    snprintf(
        diagnosticTextBuf_,
        sizeof(diagnosticTextBuf_),
        "CAN-FD init failed: unsupported timing clk=%lu nominal=%lu data=%lu",
        static_cast<unsigned long>(options_.canClockHz),
        static_cast<unsigned long>(options_.nominalBitRate),
        static_cast<unsigned long>(options_.dataBitRate));
    return false;
  }

  if (!spiResetDevice()) {
    snprintf(
        diagnosticTextBuf_,
        sizeof(diagnosticTextBuf_),
        "CAN-FD init failed: reset command failed");
    return false;
  }
  delay(10);

  if (!setOperationMode(kReqopConfig, 50)) {
    Serial.println(diagnosticTextBuf_);
    return false;
  }

  uint32_t c1con = 0;
  if (!spiReadRegister32(kRegC1Con, &c1con, kSpiMode0)) {
    snprintf(
        diagnosticTextBuf_,
        sizeof(diagnosticTextBuf_),
        "CAN-FD init failed: cannot read C1CON in config mode");
    return false;
  }
  c1con &= ~(kC1ConTxqEnMask | kC1ConStefMask | kC1ConBrsDisMask);
  if (!spiWriteRegister32(kRegC1Con, c1con, kSpiMode0)) {
    snprintf(
        diagnosticTextBuf_,
        sizeof(diagnosticTextBuf_),
        "CAN-FD init failed: cannot write C1CON config value=0x%08lX",
        static_cast<unsigned long>(c1con));
    return false;
  }

  if (!spiWriteRegister32(kRegC1NbtCfg, timing.nominalBitTiming, kSpiMode0) ||
      !spiWriteRegister32(kRegC1DbtCfg, timing.dataBitTiming, kSpiMode0) ||
      !spiWriteRegister32(kRegC1Tdc, timing.tdcConfig, kSpiMode0)) {
    snprintf(
        diagnosticTextBuf_,
        sizeof(diagnosticTextBuf_),
        "CAN-FD init failed: bit timing write failed (%s)",
        timing.label);
    return false;
  }

  if (!spiWriteRegister32(kRegC1Int, 0x00000000UL, kSpiMode0) ||
      !spiWriteRegister32(kRegC1RxIf, 0x00000000UL, kSpiMode0) ||
      !spiWriteRegister32(kRegC1TefCon, 0x00000000UL, kSpiMode0) ||
      !spiWriteRegister32(kRegC1TxqCon, 0x00000000UL, kSpiMode0) ||
      !spiWriteRegister32(kRegC1FifoBa, kMessageRamBase, kSpiMode0)) {
    snprintf(
        diagnosticTextBuf_,
        sizeof(diagnosticTextBuf_),
        "CAN-FD init failed: baseline register write failed");
    return false;
  }

  const uint32_t fifoCon1 =
      (kFifoConPlsize64 << kFifoConPlsizeShift) |
      (0U << kFifoConFsizeShift) |
      kFifoConFresetMask;
  if (!spiWriteRegister32(kRegC1FifoCon1, fifoCon1, kSpiMode0)) {
    snprintf(
        diagnosticTextBuf_,
        sizeof(diagnosticTextBuf_),
        "CAN-FD init failed: FIFO1 config write failed");
    return false;
  }

  if (!spiWriteRegister32(kRegC1FltObj0, 0x00000000UL, kSpiMode0) ||
      !spiWriteRegister32(kRegC1Mask0, 0x00000000UL, kSpiMode0) ||
      !spiWriteRegister32(kRegC1FltCon0, 0x00000081UL, kSpiMode0)) {
    snprintf(
        diagnosticTextBuf_,
        sizeof(diagnosticTextBuf_),
        "CAN-FD init failed: filter config write failed");
    return false;
  }

  if (!setOperationMode(kReqopNormal, 100)) {
    Serial.println(diagnosticTextBuf_);
    return false;
  }

  return true;
}

bool CanFdBackend::pollReceiveFifo(CanFrame *rxFrame) {
  if (rxFrame == nullptr) {
    return false;
  }

  uint32_t fifoSta1 = 0;
  if (!spiReadRegister32(kRegC1FifoSta1, &fifoSta1, kSpiMode0)) {
    return false;
  }
  if ((fifoSta1 & kFifoStaNotEmptyMask) == 0U) {
    return false;
  }

  uint32_t fifoUa1 = 0;
  if (!spiReadRegister32(kRegC1FifoUa1, &fifoUa1, kSpiMode0)) {
    return false;
  }

  uint8_t header[kRxHeaderBytes] = {0};
  if (!spiReadBytes(static_cast<uint16_t>(fifoUa1 & 0x0FFFU), header, sizeof(header), kSpiMode0)) {
    return false;
  }

  const uint8_t r1 = header[5];
  const uint8_t dlc = static_cast<uint8_t>(r1 & 0x0FU);
  const uint8_t payloadLength = dlcToLength(dlc);
  if (payloadLength > sizeof(rxFrame->data)) {
    return false;
  }

  rxFrame->clear();
  rxFrame->fdFormat = (r1 & 0x80U) != 0U;
  rxFrame->bitrateSwitch = (r1 & 0x40U) != 0U;
  rxFrame->extended = (r1 & 0x10U) != 0U;
  rxFrame->dataLength = payloadLength;

  const uint16_t sid =
      static_cast<uint16_t>(header[0]) |
      (static_cast<uint16_t>(header[1] & 0x07U) << 8U) |
      (static_cast<uint16_t>(header[3] & 0x20U) << 6U);
  if (rxFrame->extended) {
    const uint32_t eid =
        (static_cast<uint32_t>((header[1] >> 3U) & 0x1FU)) |
        (static_cast<uint32_t>(header[2]) << 5U) |
        (static_cast<uint32_t>(header[3] & 0x1FU) << 13U);
    rxFrame->identifier = (static_cast<uint32_t>(sid & 0x7FFU) << 18U) | eid;
  } else {
    rxFrame->identifier = sid & 0x7FFU;
  }

  if (payloadLength > 0U) {
    uint8_t payloadBuffer[sizeof(rxFrame->data)] = {0};
    const size_t paddedLength = roundUpToWord(payloadLength);
    if (!spiReadBytes(
            static_cast<uint16_t>((fifoUa1 + kRxHeaderBytes) & 0x0FFFU),
            payloadBuffer,
            paddedLength,
            kSpiMode0)) {
      return false;
    }
    memcpy(rxFrame->data, payloadBuffer, payloadLength);
  }

  uint32_t fifoCon1 = 0;
  if (!spiReadRegister32(kRegC1FifoCon1, &fifoCon1, kSpiMode0)) {
    return false;
  }
  fifoCon1 |= kFifoConUincMask;
  if (!spiWriteRegister32(kRegC1FifoCon1, fifoCon1, kSpiMode0)) {
    return false;
  }

  return true;
}

uint8_t CanFdBackend::dlcToLength(uint8_t dlc) {
  static constexpr uint8_t kDlcLengthMap[16] = {
      0, 1, 2, 3, 4, 5, 6, 7,
      8, 12, 16, 20, 24, 32, 48, 64,
  };
  return kDlcLengthMap[dlc & 0x0FU];
}


