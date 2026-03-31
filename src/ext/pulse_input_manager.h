#pragma once

#include <Arduino.h>

struct PulseInputState {
  bool configured = false;
  bool valid = false;
  bool stale = false;
  float speedKmh = 0.0f;
  uint32_t totalPulseCount = 0;
  uint32_t lastPulseMs = 0;
};

class PulseInputManager {
public:
  bool begin(
      gpio_num_t inputPin,
      bool usePullup,
      float metersPerPulse,
      uint32_t sampleWindowMs,
      uint32_t timeoutMs);
  void update(uint32_t nowMs);

  bool isConfigured() const;
  bool isValid() const;
  float getSpeedKmh() const;
  const PulseInputState &getState() const;

private:
  static void IRAM_ATTR onPulseEdge();
  void handlePulseEdgeFromIsr();

  static PulseInputManager *instance_;

  gpio_num_t inputPin_ = GPIO_NUM_NC;
  bool configured_ = false;
  float metersPerPulse_ = 0.0f;
  uint32_t sampleWindowMs_ = 200;
  uint32_t timeoutMs_ = 500;
  volatile uint32_t pulseCountIsr_ = 0;
  volatile uint32_t lastPulseMsIsr_ = 0;
  uint32_t lastSampleMs_ = 0;
  uint32_t lastPulseCountSample_ = 0;
  PulseInputState state_;
};
