#pragma once

#include <stdint.h>

class DistanceManager {
public:
  void update(uint32_t nowMs, float currentSpeedKmh);
  void reset(uint32_t nowMs);

  float getSelectedSpeedKmh() const;
  double getDistanceMeters() const;
  uint32_t getTripElapsedMs() const;

private:
  float selectedSpeedKmh_ = 0.0f;
  float previousIntegratedSpeedKmh_ = 0.0f;
  double distanceMeters_ = 0.0;
  uint32_t lastDistanceUpdateMs_ = 0;
  uint32_t tripStartMs_ = 0;
  uint32_t tripElapsedMs_ = 0;
};
