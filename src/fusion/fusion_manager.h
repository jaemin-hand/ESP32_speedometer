#pragma once

#include "../app/app_types.h"

class FusionManager {
public:
  void update(const FusionInputs &inputs);
  void cycleMode();
  void resetCanContext();

  const FusionState &getState() const;

  static const char *modeToText(SpeedSourceMode mode);
  static const char *sourceToText(SpeedSource source);
  static const char *autoStateToText(AutoState autoState);

private:
  static bool updateStableFlag(bool condition, uint32_t nowMs, uint32_t holdMs, uint32_t &sinceMs);
  void updateCanCorrelationLearning(const FusionInputs &inputs, bool gnssStable, bool canStable);
  void updateExtCorrelationLearning(const FusionInputs &inputs, bool gnssStable, bool extStable);

  SpeedSourceMode mode_ = SPEED_MODE_AUTO;
  AutoState autoState_ = AUTO_STATE_SEARCH;
  FusionState state_;
  uint32_t gnssCandidateSinceMs_ = 0;
  uint32_t canCandidateSinceMs_ = 0;
  uint32_t extCandidateSinceMs_ = 0;
  uint32_t lastCanCorrSampleMs_ = 0;
  float canCorrFactor_ = 1.0f;
  uint16_t canCorrSampleCount_ = 0;
  uint32_t lastExtCorrSampleMs_ = 0;
  float extCorrFactor_ = 1.0f;
  uint16_t extCorrSampleCount_ = 0;
};
