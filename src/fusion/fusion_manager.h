#pragma once

#include "../app/app_types.h"

class FusionManager {
public:
  void update(const FusionInputs &inputs);
  void cycleMode();

  const FusionState &getState() const;

  static const char *modeToText(SpeedSourceMode mode);
  static const char *sourceToText(SpeedSource source);

private:
  SpeedSourceMode mode_ = SPEED_MODE_AUTO;
  FusionState state_;
};
