#include "fusion_manager.h"

namespace {

constexpr uint32_t kGnssStableHoldMs = 1000;
constexpr uint32_t kCanStableHoldMs = 200;
constexpr uint32_t kExtStableHoldMs = 200;
constexpr int kGnssStableMinSats = 4;
constexpr int kGnssStableMinMode = 1;

bool isGnssStableCandidate(const FusionInputs &inputs) {
  return inputs.gnssValid &&
         (inputs.gnssMode >= kGnssStableMinMode) &&
         (inputs.gnssSatellites >= kGnssStableMinSats);
}

}  // namespace

bool FusionManager::updateStableFlag(
    bool condition,
    uint32_t nowMs,
    uint32_t holdMs,
    uint32_t &sinceMs) {
  if (!condition) {
    sinceMs = 0;
    return false;
  }

  if (sinceMs == 0U) {
    sinceMs = nowMs;
  }

  return (nowMs - sinceMs) >= holdMs;
}

void FusionManager::update(const FusionInputs &inputs) {
  const bool gnssStable = updateStableFlag(
      isGnssStableCandidate(inputs),
      inputs.nowMs,
      kGnssStableHoldMs,
      gnssCandidateSinceMs_);
  const bool canStable = updateStableFlag(
      inputs.canValid,
      inputs.nowMs,
      kCanStableHoldMs,
      canCandidateSinceMs_);
  const bool extStable = updateStableFlag(
      inputs.extValid,
      inputs.nowMs,
      kExtStableHoldMs,
      extCandidateSinceMs_);

  state_.mode = mode_;
  state_.autoState = autoState_;
  state_.corrActive = false;
  state_.selectedSource = SPEED_SOURCE_NONE;
  state_.selectedSpeedKmh = 0.0f;
  state_.gnssStable = gnssStable;
  state_.canStable = canStable;
  state_.extStable = extStable;

  switch (mode_) {
    case SPEED_MODE_AUTO:
      switch (autoState_) {
        case AUTO_STATE_SEARCH:
          if (gnssStable) {
            autoState_ = AUTO_STATE_GNSS_ACTIVE;
          } else if (canStable) {
            autoState_ = AUTO_STATE_CAN_FALLBACK;
          } else if (extStable) {
            autoState_ = AUTO_STATE_EXT_FALLBACK;
          }
          break;

        case AUTO_STATE_GNSS_ACTIVE:
          if (!isGnssStableCandidate(inputs)) {
            if (canStable) {
              autoState_ = AUTO_STATE_CAN_FALLBACK;
            } else if (extStable) {
              autoState_ = AUTO_STATE_EXT_FALLBACK;
            } else {
              autoState_ = AUTO_STATE_SEARCH;
            }
          }
          break;

        case AUTO_STATE_CAN_FALLBACK:
          if (gnssStable) {
            autoState_ = AUTO_STATE_GNSS_ACTIVE;
          } else if (!inputs.canValid) {
            if (extStable) {
              autoState_ = AUTO_STATE_EXT_FALLBACK;
            } else {
              autoState_ = AUTO_STATE_SEARCH;
            }
          }
          break;

        case AUTO_STATE_EXT_FALLBACK:
          if (gnssStable) {
            autoState_ = AUTO_STATE_GNSS_ACTIVE;
          } else if (canStable) {
            autoState_ = AUTO_STATE_CAN_FALLBACK;
          } else if (!inputs.extValid) {
            autoState_ = AUTO_STATE_SEARCH;
          }
          break;
      }

      state_.autoState = autoState_;
      switch (autoState_) {
        case AUTO_STATE_GNSS_ACTIVE:
          if (inputs.gnssValid) {
            state_.selectedSource = SPEED_SOURCE_GNSS;
            state_.selectedSpeedKmh = inputs.gnssSpeedKmh;
          }
          break;

        case AUTO_STATE_CAN_FALLBACK:
          if (inputs.canValid) {
            state_.selectedSource = SPEED_SOURCE_CAN;
            state_.selectedSpeedKmh = inputs.canSpeedKmh;
          }
          break;

        case AUTO_STATE_EXT_FALLBACK:
          if (inputs.extValid) {
            state_.selectedSource = SPEED_SOURCE_EXT;
            state_.selectedSpeedKmh = inputs.extSpeedKmh;
          }
          break;

        case AUTO_STATE_SEARCH:
        default:
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
      }
      break;

    case SPEED_MODE_GNSS:
      autoState_ = AUTO_STATE_GNSS_ACTIVE;
      state_.autoState = autoState_;
      if (inputs.gnssValid) {
        state_.selectedSource = SPEED_SOURCE_GNSS;
        state_.selectedSpeedKmh = inputs.gnssSpeedKmh;
      }
      break;

    case SPEED_MODE_CAN:
      autoState_ = AUTO_STATE_CAN_FALLBACK;
      state_.autoState = autoState_;
      if (inputs.canValid) {
        state_.selectedSource = SPEED_SOURCE_CAN;
        state_.selectedSpeedKmh = inputs.canSpeedKmh;
      }
      break;

    case SPEED_MODE_EXT:
      autoState_ = AUTO_STATE_EXT_FALLBACK;
      state_.autoState = autoState_;
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
      autoState_ = AUTO_STATE_GNSS_ACTIVE;
      break;
    case SPEED_MODE_GNSS:
      mode_ = SPEED_MODE_CAN;
      autoState_ = AUTO_STATE_CAN_FALLBACK;
      break;
    case SPEED_MODE_CAN:
      mode_ = SPEED_MODE_EXT;
      autoState_ = AUTO_STATE_EXT_FALLBACK;
      break;
    case SPEED_MODE_EXT:
      mode_ = SPEED_MODE_AUTO;
      autoState_ = AUTO_STATE_SEARCH;
      break;
  }

  state_.mode = mode_;
  state_.autoState = autoState_;
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

const char *FusionManager::autoStateToText(AutoState autoState) {
  switch (autoState) {
    case AUTO_STATE_SEARCH: return "SEARCH";
    case AUTO_STATE_GNSS_ACTIVE: return "GNSS_ACTIVE";
    case AUTO_STATE_CAN_FALLBACK: return "CAN_FALLBACK";
    case AUTO_STATE_EXT_FALLBACK: return "EXT_FALLBACK";
    default: return "UNKNOWN";
  }
}
