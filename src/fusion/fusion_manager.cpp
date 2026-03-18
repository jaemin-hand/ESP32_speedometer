#include "fusion_manager.h"

void FusionManager::update(const FusionInputs &inputs) {
  state_.mode = mode_;
  state_.corrActive = false;
  state_.selectedSource = SPEED_SOURCE_NONE;
  state_.selectedSpeedKmh = 0.0f;

  switch (mode_) {
    case SPEED_MODE_AUTO:
      if (inputs.gnssValid) {
        state_.selectedSource = SPEED_SOURCE_GNSS;
        state_.selectedSpeedKmh = inputs.gnssSpeedKmh;
      } else if (inputs.canValid) {
        state_.selectedSource = SPEED_SOURCE_CAN;
        state_.selectedSpeedKmh = inputs.canSpeedKmh;
      } else if (inputs.extValid) {
        state_.selectedSource = SPEED_SOURCE_EXT;
        state_.selectedSpeedKmh = inputs.extSpeedKmh;
      }
      break;

    case SPEED_MODE_GNSS:
      if (inputs.gnssValid) {
        state_.selectedSource = SPEED_SOURCE_GNSS;
        state_.selectedSpeedKmh = inputs.gnssSpeedKmh;
      }
      break;

    case SPEED_MODE_CAN:
      if (inputs.canValid) {
        state_.selectedSource = SPEED_SOURCE_CAN;
        state_.selectedSpeedKmh = inputs.canSpeedKmh;
      }
      break;

    case SPEED_MODE_EXT:
      if (inputs.extValid) {
        state_.selectedSource = SPEED_SOURCE_EXT;
        state_.selectedSpeedKmh = inputs.extSpeedKmh;
      }
      break;
  }
}

void FusionManager::cycleMode() {
  switch (mode_) {
    case SPEED_MODE_AUTO:
      mode_ = SPEED_MODE_GNSS;
      break;
    case SPEED_MODE_GNSS:
      mode_ = SPEED_MODE_CAN;
      break;
    case SPEED_MODE_CAN:
      mode_ = SPEED_MODE_EXT;
      break;
    case SPEED_MODE_EXT:
      mode_ = SPEED_MODE_AUTO;
      break;
  }

  state_.mode = mode_;
}

const FusionState &FusionManager::getState() const {
  return state_;
}

const char *FusionManager::modeToText(SpeedSourceMode mode) {
  switch (mode) {
    case SPEED_MODE_AUTO: return "AUTO";
    case SPEED_MODE_GNSS: return "GNSS";
    case SPEED_MODE_CAN: return "CAN";
    case SPEED_MODE_EXT: return "EXT";
    default: return "UNKNOWN";
  }
}

const char *FusionManager::sourceToText(SpeedSource source) {
  switch (source) {
    case SPEED_SOURCE_GNSS: return "GNSS";
    case SPEED_SOURCE_CAN: return "CAN";
    case SPEED_SOURCE_EXT: return "EXT";
    default: return "NONE";
  }
}
