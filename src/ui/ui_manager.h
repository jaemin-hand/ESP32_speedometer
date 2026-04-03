#pragma once

#include <stdint.h>

#include "lvgl.h"

#include "../app/app_types.h"

class UiManager {
public:
  void begin();
  void update(const UiSnapshot &snapshot);
  uint8_t consumeActions();

private:
  enum TimeDisplayMode : uint8_t {
    TIME_DISPLAY_TRIP = 0,
    TIME_DISPLAY_LOCAL = 1,
  };

  enum DistanceDisplayUnit : uint8_t {
    DISTANCE_UNIT_METER = 0,
    DISTANCE_UNIT_MILE = 1,
  };

  static void onExtPanel(lv_event_t *e);
  static void onGpsPanel(lv_event_t *e);
  static void onCanPanel(lv_event_t *e);
  static void onDistancePanel(lv_event_t *e);
  static void onTimePanel(lv_event_t *e);
  static void onCanMonitorButton(lv_event_t *e);
  static void onCanMonitorSendButton(lv_event_t *e);
  static void onCanMonitorBackButton(lv_event_t *e);
  static void onResetButton(lv_event_t *e);

  static UiManager *instance_;

  static lv_obj_t *createCell(
      lv_obj_t *parent,
      lv_coord_t x,
      lv_coord_t y,
      lv_coord_t w,
      lv_coord_t h,
      lv_event_cb_t cb);
  static lv_obj_t *createTitle(lv_obj_t *parent, const char *text);

  void queueAction(uint8_t action);
  void toggleDisplayUnit(DisplayUnit &unit);
  void toggleDistanceUnit();
  void toggleTimeMode();
  void showCanMonitor(bool visible);
  void setCellHighlight(lv_obj_t *cell, bool active);
  void updateUtcAnchor(const char *utcText);
  bool tryFormatAnchoredLocal(char *timeBuf, size_t timeBufSize) const;
  void updateSpeedDisplay(
      lv_obj_t *valueLabel,
      lv_obj_t *unitLabel,
      float incomingSpeedKmh,
      bool valid,
      DisplayUnit unit,
      float &lastValidSpeedKmh,
      bool &hasLastValidSpeed);
  void formatSpeed(char *valueBuf, size_t valueBufSize, float speedKmh, bool valid, DisplayUnit unit) const;
  void formatDistance(char *valueBuf, size_t valueBufSize, char *unitBuf, size_t unitBufSize, double distanceMeters) const;
  void formatTime(char *timeBuf, size_t timeBufSize, const UiSnapshot &snapshot) const;
  void renderTimeText(const char *timeText);
  SpeedSource getHighlightedSource(const UiSnapshot &snapshot) const;

  static constexpr uint8_t kTimeCharSlots = 10;

  DisplayUnit extUnit_ = DISPLAY_UNIT_KMH;
  DisplayUnit gpsUnit_ = DISPLAY_UNIT_KMH;
  DisplayUnit canUnit_ = DISPLAY_UNIT_KMH;
  DistanceDisplayUnit distanceUnit_ = DISTANCE_UNIT_METER;
  TimeDisplayMode timeMode_ = TIME_DISPLAY_TRIP;
  uint8_t pendingActions_ = UI_ACTION_NONE;

  lv_obj_t *cellExt_ = nullptr;
  lv_obj_t *cellGps_ = nullptr;
  lv_obj_t *cellCan_ = nullptr;
  lv_obj_t *cellDistance_ = nullptr;
  lv_obj_t *cellTime_ = nullptr;
  lv_obj_t *cellSats_ = nullptr;
  lv_obj_t *buttonCanMonitor_ = nullptr;
  lv_obj_t *buttonReset_ = nullptr;
  lv_obj_t *overlayCanMonitor_ = nullptr;
  lv_obj_t *labelCanMonitorText_ = nullptr;
  lv_obj_t *buttonCanMonitorSend_ = nullptr;
  lv_obj_t *buttonCanMonitorBack_ = nullptr;

  lv_obj_t *labelExtValue_ = nullptr;
  lv_obj_t *labelExtUnit_ = nullptr;
  lv_obj_t *labelGpsValue_ = nullptr;
  lv_obj_t *labelGpsUnit_ = nullptr;
  lv_obj_t *labelCanValue_ = nullptr;
  lv_obj_t *labelCanUnit_ = nullptr;
  lv_obj_t *labelDistanceValue_ = nullptr;
  lv_obj_t *labelDistanceUnit_ = nullptr;
  lv_obj_t *labelTimeTitle_ = nullptr;
  lv_obj_t *timeValueBox_ = nullptr;
  lv_obj_t *labelTimeChars_[kTimeCharSlots] = {};
  lv_obj_t *labelSatsValue_ = nullptr;
  lv_obj_t *labelModeStatus_ = nullptr;
  lv_obj_t *labelUsingStatus_ = nullptr;
  lv_obj_t *labelGnssQuality_ = nullptr;
  lv_obj_t *labelGnssLink_ = nullptr;
  lv_obj_t *labelGnssCn0_ = nullptr;

  float lastExtSpeedKmh_ = 0.0f;
  float lastGpsSpeedKmh_ = 0.0f;
  float lastCanSpeedKmh_ = 0.0f;
  int lastValidSats_ = 0;
  bool hasLastExtSpeed_ = false;
  bool hasLastGpsSpeed_ = false;
  bool hasLastCanSpeed_ = false;
  bool hasLastSats_ = false;
  bool hasUtcAnchor_ = false;
  int32_t localUtcOffsetMinutes_ = 0;
  uint32_t utcAnchorMs_ = 0;
  uint32_t utcAnchorSecondsOfDay_ = 0;
  char lastUtcSource_[9] = {0};
};


