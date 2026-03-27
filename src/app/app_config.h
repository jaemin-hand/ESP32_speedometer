#pragma once

#include "../can/can_backend.h"
#include "../can/can_profiles.h"

namespace AppConfig {

// Keep the currently working SantaFe classic CAN path as the default baseline.
// When we start real CAN-FD bring-up, switching profile/backend should only
// require changing these values instead of touching app_main wiring.
constexpr CanBackendType kRequestedCanBackend = CAN_BACKEND_CLASSIC;
constexpr CanProfileId kActiveCanProfile = CAN_PROFILE_SANTAFE_CLASSIC;

// Tentative MCP2518FD SPI pin plan for future CAN-FD bring-up.
// We intentionally keep Classic CAN on GPIO2/GPIO48 unchanged and reserve a
// separate SPI path for the external CAN-FD controller.
constexpr gpio_num_t kCanFdSpiSckPin = GPIO_NUM_5;
constexpr gpio_num_t kCanFdSpiMosiPin = GPIO_NUM_4;
constexpr gpio_num_t kCanFdSpiMisoPin = GPIO_NUM_45;
constexpr gpio_num_t kCanFdSpiCsPin = GPIO_NUM_46;

// First bring-up can start in polling mode, so IRQ/RESET/STBY stay optional.
constexpr gpio_num_t kCanFdIrqPin = GPIO_NUM_NC;
constexpr gpio_num_t kCanFdResetPin = GPIO_NUM_NC;
constexpr gpio_num_t kCanFdStandbyPin = GPIO_NUM_NC;

}  // namespace AppConfig
