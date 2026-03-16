#pragma once

#include <stdint.h>

enum DisplayUnit : uint8_t {
  DISPLAY_UNIT_KMH = 0,
  DISPLAY_UNIT_MPH = 1,
};

struct GpsData {
  bool pvtValid = false;
  int pvtMode = 0;
  int errorCode = 0;
  int satellites = 0;
  float altitudeM = 0.0f;
  float speedKnots = 0.0f;
  float speedKmh = 0.0f;
  double latitude = 0.0;
  double longitude = 0.0;
  char dateStr[11] = {0};
  char timeStr[9] = {0};
};

enum SpeedSourceMode : uint8_t {
  SPEED_MODE_AUTO = 0,
  SPEED_MODE_GNSS = 1,
  SPEED_MODE_CAN = 2,
  SPEED_MODE_EXT = 3,
};

enum SpeedSource : uint8_t {
  SPEED_SOURCE_NONE = 0,
  SPEED_SOURCE_GNSS = 1,
  SPEED_SOURCE_CAN = 2,
  SPEED_SOURCE_EXT = 3,
};

struct FusionInputs {
  bool gnssValid = false;
  float gnssSpeedKmh = 0.0f;
  bool canValid = false;
  float canSpeedKmh = 0.0f;
  bool extValid = false;
  float extSpeedKmh = 0.0f;
};

struct FusionState {
  SpeedSourceMode mode = SPEED_MODE_AUTO;
  SpeedSource selectedSource = SPEED_SOURCE_NONE;
  float selectedSpeedKmh = 0.0f;
  bool corrActive = false;
};

enum UiAction : uint8_t {
  UI_ACTION_NONE = 0,
  UI_ACTION_SEND_TEST_CAN = 1 << 0,
  UI_ACTION_SOURCE_CAL = 1 << 1,
  UI_ACTION_RESET_DISTANCE = 1 << 2,
};

struct UiSnapshot {
  bool extValid = false;
  float extSpeedKmh = 0.0f;
  bool gpsValid = false;
  float gpsSpeedKmh = 0.0f;
  bool canValid = false;
  float canSpeedKmh = 0.0f;
  double distanceMeters = 0.0;
  uint32_t tripElapsedMs = 0;
  SpeedSourceMode sourceMode = SPEED_MODE_AUTO;
  SpeedSource selectedSource = SPEED_SOURCE_NONE;
  bool corrActive = false;
  GpsData gps;
};
