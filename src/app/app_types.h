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
  bool sbfStreamActive = false;
  bool receiverTimeValid = false;
  uint32_t sbfAgeMs = UINT32_MAX;
  uint32_t pvtAgeMs = UINT32_MAX;
  uint32_t receiverTimeAgeMs = UINT32_MAX;
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

enum AutoState : uint8_t {
  AUTO_STATE_SEARCH = 0,
  AUTO_STATE_GNSS_ACTIVE = 1,
  AUTO_STATE_CAN_FALLBACK = 2,
  AUTO_STATE_EXT_FALLBACK = 3,
};

enum GnssSpeedQuality : uint8_t {
  GNSS_SPEED_QUALITY_LOST = 0,
  GNSS_SPEED_QUALITY_LOW = 1,
  GNSS_SPEED_QUALITY_MID = 2,
  GNSS_SPEED_QUALITY_HIGH = 3,
};

enum GnssLinkQuality : uint8_t {
  GNSS_LINK_QUALITY_LOST = 0,
  GNSS_LINK_QUALITY_HOLD = 1,
  GNSS_LINK_QUALITY_LIVE = 2,
};

struct FusionInputs {
  uint32_t nowMs = 0;
  bool gnssValid = false;
  int gnssMode = 0;
  int gnssSatellites = 0;
  float gnssSpeedKmh = 0.0f;
  bool canValid = false;
  float canSpeedKmh = 0.0f;
  bool extValid = false;
  float extSpeedKmh = 0.0f;
};

struct FusionState {
  SpeedSourceMode mode = SPEED_MODE_AUTO;
  AutoState autoState = AUTO_STATE_SEARCH;
  SpeedSource selectedSource = SPEED_SOURCE_NONE;
  float selectedSpeedKmh = 0.0f;
  bool gnssStable = false;
  bool canStable = false;
  bool extStable = false;
  bool corrActive = false;
  bool corrLearned = false;
  float corrFactor = 1.0f;
  float correctedCanSpeedKmh = 0.0f;
  uint16_t corrSampleCount = 0;
};

enum UiAction : uint8_t {
  UI_ACTION_NONE = 0,
  UI_ACTION_SEND_TEST_CAN = 1 << 0,
  UI_ACTION_SOURCE_CAL = 1 << 1,
  UI_ACTION_RESET_DISTANCE = 1 << 2,
};

struct UiSnapshot {
  static constexpr uint16_t CAN_MONITOR_TEXT_SIZE = 512;

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
  GnssSpeedQuality gnssSpeedQuality = GNSS_SPEED_QUALITY_LOST;
  GnssLinkQuality gnssLinkQuality = GNSS_LINK_QUALITY_LOST;
  char canMonitorText[CAN_MONITOR_TEXT_SIZE] = {0};
  GpsData gps;
};
