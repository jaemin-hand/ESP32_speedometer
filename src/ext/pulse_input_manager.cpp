#include "pulse_input_manager.h"

#include <math.h>

namespace {

constexpr float MPS_TO_KMH = 3.6f;

}  // namespace

PulseInputManager *PulseInputManager::instance_ = nullptr;

bool PulseInputManager::begin(
    gpio_num_t inputPin,
    bool usePullup,
    float metersPerPulse,
    uint32_t sampleWindowMs,
    uint32_t timeoutMs,
    uint32_t minPulseIntervalUs,
    float speedFilterAlpha) {
  configured_ = false;
  inputPin_ = inputPin;
  metersPerPulse_ = metersPerPulse;
  sampleWindowMs_ = sampleWindowMs;
  timeoutMs_ = timeoutMs;
  minPulseIntervalUs_ = minPulseIntervalUs;
  speedFilterAlpha_ = speedFilterAlpha;
  pulseCountIsr_ = 0;
  lastPulseMsIsr_ = 0;
  lastPulseUsIsr_ = 0;
  lastSampleMs_ = millis();
  lastPulseCountSample_ = 0;
  state_ = {};

  if (inputPin_ == GPIO_NUM_NC || metersPerPulse_ <= 0.0f) {
    return false;
  }
  if (speedFilterAlpha_ < 0.0f) {
    speedFilterAlpha_ = 0.0f;
  } else if (speedFilterAlpha_ > 1.0f) {
    speedFilterAlpha_ = 1.0f;
  }

  pinMode(static_cast<uint8_t>(inputPin_), usePullup ? INPUT_PULLUP : INPUT);
  instance_ = this;
  attachInterrupt(digitalPinToInterrupt(static_cast<uint8_t>(inputPin_)), onPulseEdge, RISING);

  configured_ = true;
  state_.configured = true;
  return true;
}

void PulseInputManager::update(uint32_t nowMs) {
  if (!configured_) {
    state_.configured = false;
    state_.valid = false;
    state_.stale = false;
    state_.speedKmh = 0.0f;
    return;
  }

  uint32_t pulseCountSnapshot = 0;
  uint32_t lastPulseMsSnapshot = 0;
  noInterrupts();
  pulseCountSnapshot = pulseCountIsr_;
  lastPulseMsSnapshot = lastPulseMsIsr_;
  interrupts();

  state_.configured = true;
  state_.totalPulseCount = pulseCountSnapshot;
  state_.lastPulseMs = lastPulseMsSnapshot;

  if ((nowMs - lastSampleMs_) >= sampleWindowMs_) {
    const uint32_t elapsedMs = nowMs - lastSampleMs_;
    const uint32_t pulseDelta = pulseCountSnapshot - lastPulseCountSample_;
    const float elapsedSeconds = static_cast<float>(elapsedMs) / 1000.0f;

    if ((pulseDelta > 0U) && (elapsedSeconds > 0.0f)) {
      const float distanceMeters = static_cast<float>(pulseDelta) * metersPerPulse_;
      const float speedMps = distanceMeters / elapsedSeconds;
      const float instantSpeedKmh = speedMps * MPS_TO_KMH;

      if (!state_.valid || state_.filteredSpeedKmh <= 0.0f) {
        state_.filteredSpeedKmh = instantSpeedKmh;
      } else {
        state_.filteredSpeedKmh =
            ((1.0f - speedFilterAlpha_) * state_.filteredSpeedKmh) +
            (speedFilterAlpha_ * instantSpeedKmh);
      }
      state_.speedKmh = state_.filteredSpeedKmh;
    }

    lastSampleMs_ = nowMs;
    lastPulseCountSample_ = pulseCountSnapshot;
  }

  if (pulseCountSnapshot == 0U) {
    state_.valid = false;
    state_.stale = false;
    state_.speedKmh = 0.0f;
    return;
  }

  if ((lastPulseMsSnapshot != 0U) && ((nowMs - lastPulseMsSnapshot) > timeoutMs_)) {
    state_.valid = false;
    state_.stale = true;
    state_.speedKmh = 0.0f;
    state_.filteredSpeedKmh = 0.0f;
    return;
  }

  state_.valid = true;
  state_.stale = false;
}

bool PulseInputManager::isConfigured() const {
  return configured_;
}

bool PulseInputManager::isValid() const {
  return state_.valid;
}

float PulseInputManager::getSpeedKmh() const {
  return state_.speedKmh;
}

const PulseInputState &PulseInputManager::getState() const {
  return state_;
}

void IRAM_ATTR PulseInputManager::onPulseEdge() {
  if (instance_ != nullptr) {
    instance_->handlePulseEdgeFromIsr();
  }
}

void IRAM_ATTR PulseInputManager::handlePulseEdgeFromIsr() {
  const uint32_t nowUs = micros();
  if (lastPulseUsIsr_ != 0U && (nowUs - lastPulseUsIsr_) < minPulseIntervalUs_) {
    return;
  }

  lastPulseUsIsr_ = nowUs;
  ++pulseCountIsr_;
  lastPulseMsIsr_ = millis();
}
