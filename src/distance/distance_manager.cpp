#include "distance_manager.h"

void DistanceManager::update(uint32_t nowMs, float currentSpeedKmh) {
  if (lastDistanceUpdateMs_ == 0U) {
    lastDistanceUpdateMs_ = nowMs;
    tripStartMs_ = nowMs;
    tripElapsedMs_ = 0U;
    selectedSpeedKmh_ = currentSpeedKmh;
    previousIntegratedSpeedKmh_ = currentSpeedKmh;
    return;
  }

  float deltaSeconds = static_cast<float>(nowMs - lastDistanceUpdateMs_) * 0.001f;
  if (deltaSeconds < 0.0f) {
    deltaSeconds = 0.0f;
  }
  if (deltaSeconds > 0.1f) {
    deltaSeconds = 0.1f;
  }

  const float averageSpeedMps = ((previousIntegratedSpeedKmh_ + currentSpeedKmh) * 0.5f) / 3.6f;
  distanceMeters_ += static_cast<double>(averageSpeedMps * deltaSeconds);

  selectedSpeedKmh_ = currentSpeedKmh;
  previousIntegratedSpeedKmh_ = currentSpeedKmh;
  lastDistanceUpdateMs_ = nowMs;
  tripElapsedMs_ = nowMs - tripStartMs_;
}

void DistanceManager::reset(uint32_t nowMs) {
  distanceMeters_ = 0.0;
  previousIntegratedSpeedKmh_ = selectedSpeedKmh_;
  lastDistanceUpdateMs_ = nowMs;
  tripStartMs_ = nowMs;
  tripElapsedMs_ = 0U;
}

float DistanceManager::getSelectedSpeedKmh() const {
  return selectedSpeedKmh_;
}

double DistanceManager::getDistanceMeters() const {
  return distanceMeters_;
}

uint32_t DistanceManager::getTripElapsedMs() const {
  return tripElapsedMs_;
}
