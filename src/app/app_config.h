#pragma once

#include "../can/can_backend.h"
#include "../can/can_profiles.h"

namespace AppConfig {

// Current bring-up priority is CAN-FD through the external MCP2517FD board.
constexpr CanBackendType kRequestedCanBackend = CAN_BACKEND_FD;
constexpr CanProfileId kActiveCanProfile = CAN_PROFILE_TUCSON_FD_CANDIDATES;

// Local clock display offset from UTC in minutes.
constexpr int32_t kLocalUtcOffsetMinutes = 9 * 60;

// MCP2517FD SPI pin plan for CAN-FD bring-up.
// We intentionally keep Classic CAN on GPIO2/GPIO48 unchanged and reserve a
// separate SPI path for the external CAN-FD controller.
constexpr gpio_num_t kCanFdSpiSckPin = GPIO_NUM_5;
constexpr gpio_num_t kCanFdSpiMosiPin = GPIO_NUM_4;
constexpr gpio_num_t kCanFdSpiMisoPin = GPIO_NUM_45;
constexpr gpio_num_t kCanFdSpiCsPin = GPIO_NUM_32;
constexpr uint32_t kCanFdControllerClockHz = 40000000UL;
constexpr uint32_t kCanFdNominalBitRate = 500000UL;
constexpr uint32_t kCanFdDataBitRate = 2000000UL;

// First bring-up can start in polling mode, so IRQ/RESET/STBY stay optional.
constexpr gpio_num_t kCanFdIrqPin = GPIO_NUM_NC;
constexpr gpio_num_t kCanFdResetPin = GPIO_NUM_NC;
constexpr gpio_num_t kCanFdStandbyPin = GPIO_NUM_NC;

struct PulseInputCalibrationConfig {
  // If true, use the direct meters-per-pulse value below.
  // If false, derive meters-per-pulse from wheel circumference and the
  // effective pulse count per wheel revolution.
  bool useDirectMetersPerPulse = false;
  float directMetersPerPulse = 0.5f;

  // Effective wheel travel per full wheel revolution.
  float wheelCircumferenceMeters = 2.0f;

  // Effective pulses generated for one wheel revolution.
  uint16_t pulsesPerWheelRevolution = 4;
};

constexpr float resolvePulseMetersPerPulse(
    const PulseInputCalibrationConfig &calibration) {
  if (calibration.useDirectMetersPerPulse) {
    return calibration.directMetersPerPulse;
  }

  if (calibration.pulsesPerWheelRevolution == 0U) {
    return 0.0f;
  }

  return calibration.wheelCircumferenceMeters /
         static_cast<float>(calibration.pulsesPerWheelRevolution);
}

struct PulseInputConfig {
  gpio_num_t inputPin = GPIO_NUM_NC;
  bool usePullup = true;

  // Calibration:
  // speed_kmh = pulse_rate_hz * metersPerPulse * 3.6
  PulseInputCalibrationConfig calibration = {};

  // Runtime tuning:
  uint32_t sampleWindowMs = 200;
  uint32_t timeoutMs = 500;
  uint32_t minPulseIntervalUs = 1500;
  float speedFilterAlpha = 0.25f;
};

// Pulse input is disabled for now because a board GPIO is reused as MCP2517FD CS
// during CAN-FD bring-up.
constexpr PulseInputConfig kPulseInputConfig = {
    .inputPin = GPIO_NUM_NC,
    .usePullup = true,
    .calibration =
        {
            .useDirectMetersPerPulse = false,
            .directMetersPerPulse = 0.5f,
            .wheelCircumferenceMeters = 2.0f,
            .pulsesPerWheelRevolution = 4,
        },
    .sampleWindowMs = 200,
    .timeoutMs = 500,
    .minPulseIntervalUs = 1500,
    .speedFilterAlpha = 0.25f,
};

}  // namespace AppConfig
