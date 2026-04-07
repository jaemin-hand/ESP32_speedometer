#include "ui_manager.h"

#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

namespace {

constexpr float KMH_TO_MPH = 0.621371f;
constexpr double METERS_TO_MILES = 0.000621371192;
constexpr lv_coord_t TIME_CHAR_WIDTH = 31;
constexpr lv_coord_t TIME_CHAR_HEIGHT = 92;
constexpr uint32_t LIVE_TEXT_COLOR = 0xFFFFFF;
constexpr uint32_t STALE_TEXT_COLOR = 0x5A5A5A;
constexpr uint32_t SATS_LIVE_COLOR = 0xD8F7FF;
constexpr uint8_t SPEED_SEGMENT_SLOT_COUNT = 5;
constexpr uint32_t SEGMENT_OFF_COLOR_LIVE = 0x151515;
constexpr uint32_t SEGMENT_OFF_COLOR_STALE = 0x0E0E0E;
constexpr float SPEED_GAUGE_STEP_KMH = 10.0f;
constexpr float SPEED_GAUGE_BLUE_RANGE_KMH = 100.0f;
constexpr float SPEED_GAUGE_MAX_KMH = 200.0f;
constexpr lv_coord_t SPEED_GAUGE_SEGMENT_HEIGHT = 21;
constexpr lv_coord_t SPEED_GAUGE_SEGMENT_GAP = 8;
constexpr lv_coord_t SPEED_GAUGE_WIDTH = 319;
constexpr lv_opa_t SPEED_GAUGE_LIVE_OPACITY = 156;
constexpr lv_opa_t SPEED_GAUGE_STALE_OPACITY = 88;
constexpr lv_opa_t SPEED_GAUGE_OFF_OPACITY = 22;
constexpr uint32_t SPEED_GAUGE_COLOR_HEX = 0x2ea8ff;
constexpr uint32_t SPEED_GAUGE_OVER_COLOR_HEX = 0xffaa4a;

uint32_t segmentOffColorFor(uint32_t onColorHex) {
  return (onColorHex == STALE_TEXT_COLOR) ? SEGMENT_OFF_COLOR_STALE : SEGMENT_OFF_COLOR_LIVE;
}

uint32_t applyLocalUtcOffset(uint32_t secondsOfDay, int32_t offsetMinutes) {
  int32_t shiftedSeconds =
      static_cast<int32_t>(secondsOfDay) + (offsetMinutes * 60);

  shiftedSeconds %= 86400;
  if (shiftedSeconds < 0) {
    shiftedSeconds += 86400;
  }

  return static_cast<uint32_t>(shiftedSeconds);
}

const char *modeToText(SpeedSourceMode mode) {
  switch (mode) {
    case SPEED_MODE_AUTO: return "AUTO";
    case SPEED_MODE_GNSS: return "GNSS";
    case SPEED_MODE_CAN: return "CAN";
    case SPEED_MODE_EXT: return "EXT";
    default: return "UNKNOWN";
  }
}

const char *sourceToText(SpeedSource source) {
  switch (source) {
    case SPEED_SOURCE_GNSS: return "GNSS";
    case SPEED_SOURCE_CAN: return "CAN";
    case SPEED_SOURCE_EXT: return "EXT";
    default: return "NONE";
  }
}

const char *gnssLinkQualityToText(GnssLinkQuality quality) {
  switch (quality) {
    case GNSS_LINK_QUALITY_LIVE: return "LINK LIVE";
    case GNSS_LINK_QUALITY_HOLD: return "LINK HOLD";
    case GNSS_LINK_QUALITY_LOST:
    default: return "LINK LOST";
  }
}

lv_color_t gnssLinkQualityColor(GnssLinkQuality quality) {
  switch (quality) {
    case GNSS_LINK_QUALITY_LIVE: return lv_color_hex(0x6be8ff);
    case GNSS_LINK_QUALITY_HOLD: return lv_color_hex(0xffd24a);
    case GNSS_LINK_QUALITY_LOST:
    default: return lv_color_hex(STALE_TEXT_COLOR);
  }
}

lv_color_t gnssCn0Color(const GpsData &gps) {
  if (!gps.cn0Valid) {
    return lv_color_hex(STALE_TEXT_COLOR);
  }
  if (gps.cn0AvgDbHz >= 40.0f && gps.cn0SignalCount >= 8U) {
    return lv_color_hex(0x82ff3f);
  }
  if (gps.cn0AvgDbHz >= 34.0f && gps.cn0SignalCount >= 4U) {
    return lv_color_hex(0xffd24a);
  }
  return lv_color_hex(0xff8f66);
}

#if LV_FONT_MONTSERRAT_48
  #define FONT_NUMBER (&lv_font_montserrat_48)
#elif LV_FONT_MONTSERRAT_40
  #define FONT_NUMBER (&lv_font_montserrat_40)
#elif LV_FONT_MONTSERRAT_32
  #define FONT_NUMBER (&lv_font_montserrat_32)
#else
  #define FONT_NUMBER LV_FONT_DEFAULT
#endif

#if LV_FONT_MONTSERRAT_48
  #define FONT_TIME (&lv_font_montserrat_48)
#elif LV_FONT_MONTSERRAT_40
  #define FONT_TIME (&lv_font_montserrat_40)
#elif LV_FONT_MONTSERRAT_32
  #define FONT_TIME (&lv_font_montserrat_32)
#else
  #define FONT_TIME LV_FONT_DEFAULT
#endif

#if LV_FONT_MONTSERRAT_40
  #define FONT_TITLE (&lv_font_montserrat_40)
#elif LV_FONT_MONTSERRAT_32
  #define FONT_TITLE (&lv_font_montserrat_32)
#else
  #define FONT_TITLE LV_FONT_DEFAULT
#endif

#if LV_FONT_MONTSERRAT_32
  #define FONT_UNIT (&lv_font_montserrat_32)
#elif LV_FONT_MONTSERRAT_24
  #define FONT_UNIT (&lv_font_montserrat_24)
#elif LV_FONT_MONTSERRAT_20
  #define FONT_UNIT (&lv_font_montserrat_20)
#elif LV_FONT_MONTSERRAT_16
  #define FONT_UNIT (&lv_font_montserrat_16)
#else
  #define FONT_UNIT LV_FONT_DEFAULT
#endif

#if LV_FONT_MONTSERRAT_32
  #define FONT_BUTTON (&lv_font_montserrat_32)
#elif LV_FONT_MONTSERRAT_24
  #define FONT_BUTTON (&lv_font_montserrat_24)
#else
  #define FONT_BUTTON LV_FONT_DEFAULT
#endif

}  // namespace

UiManager *UiManager::instance_ = nullptr;

void UiManager::begin() {
  instance_ = this;

  lv_obj_t *screen = lv_scr_act();
  lv_obj_set_style_bg_color(screen, lv_color_black(), 0);
  lv_obj_set_style_bg_grad_dir(screen, LV_GRAD_DIR_NONE, 0);
  lv_obj_set_style_pad_all(screen, 0, 0);

  const lv_coord_t topH = 300;
  const lv_coord_t leftW = 341;
  const lv_coord_t midW = 341;
  const lv_coord_t rightW = 342;
  const lv_coord_t bottomY = topH;

  cellExt_ = createCell(screen, 0, 0, leftW, topH, onExtPanel);
  cellGps_ = createCell(screen, leftW, 0, midW, topH, onGpsPanel);
  cellCan_ = createCell(screen, leftW + midW, 0, rightW, topH, onCanPanel);
  cellDistance_ = createCell(screen, 0, bottomY, leftW, 300, onDistancePanel);
  cellTime_ = createCell(screen, leftW, bottomY, midW, 300, onTimePanel);
  cellSats_ = createCell(screen, leftW + midW, bottomY, rightW, 300, nullptr);

  createTitle(cellExt_, "EXT");
  createTitle(cellGps_, "GPS");
  createTitle(cellCan_, "CAN");
  createTitle(cellDistance_, "Distance");
  labelTimeTitle_ = createTitle(cellTime_, "Time");

  createSpeedGauge(cellExt_, extGaugeSegments_, 10, 6, SPEED_GAUGE_WIDTH);
  createSpeedGauge(cellGps_, gpsGaugeSegments_, 10, 6, SPEED_GAUGE_WIDTH);
  createSpeedGauge(cellCan_, canGaugeSegments_, 10, 6, SPEED_GAUGE_WIDTH);

  labelExtValue_ = createSegmentDisplay(cellExt_, 18, 92, 286, 110);

  labelExtUnit_ = lv_label_create(cellExt_);
  lv_label_set_text(labelExtUnit_, "km/h");
  lv_obj_set_style_text_color(labelExtUnit_, lv_color_white(), 0);
  lv_obj_set_style_text_font(labelExtUnit_, FONT_UNIT, 0);
  lv_obj_set_pos(labelExtUnit_, 214, 204);

  labelGpsValue_ = createSegmentDisplay(cellGps_, 22, 92, 286, 110);

  labelGpsUnit_ = lv_label_create(cellGps_);
  lv_label_set_text(labelGpsUnit_, "km/h");
  lv_obj_set_style_text_color(labelGpsUnit_, lv_color_white(), 0);
  lv_obj_set_style_text_font(labelGpsUnit_, FONT_UNIT, 0);
  lv_obj_set_pos(labelGpsUnit_, 214, 204);

  labelCanValue_ = createSegmentDisplay(cellCan_, 22, 92, 286, 110);

  labelCanUnit_ = lv_label_create(cellCan_);
  lv_label_set_text(labelCanUnit_, "km/h");
  lv_obj_set_style_text_color(labelCanUnit_, lv_color_white(), 0);
  lv_obj_set_style_text_font(labelCanUnit_, FONT_UNIT, 0);
  lv_obj_set_pos(labelCanUnit_, 214, 204);

  labelDistanceValue_ = lv_label_create(cellDistance_);
  lv_label_set_text(labelDistanceValue_, "0.0");
  lv_obj_set_style_text_color(labelDistanceValue_, lv_color_white(), 0);
  lv_obj_set_style_text_font(labelDistanceValue_, FONT_NUMBER, 0);
  lv_obj_set_pos(labelDistanceValue_, 44, 128);

  labelDistanceUnit_ = lv_label_create(cellDistance_);
  lv_label_set_text(labelDistanceUnit_, "m");
  lv_obj_set_style_text_color(labelDistanceUnit_, lv_color_white(), 0);
  lv_obj_set_style_text_font(labelDistanceUnit_, FONT_UNIT, 0);
  lv_obj_set_pos(labelDistanceUnit_, 216, 188);

  timeValueBox_ = lv_obj_create(cellTime_);
  lv_obj_set_size(timeValueBox_, TIME_CHAR_WIDTH * kTimeCharSlots, TIME_CHAR_HEIGHT);
  lv_obj_align(timeValueBox_, LV_ALIGN_CENTER, 0, 30);
  lv_obj_set_style_bg_opa(timeValueBox_, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(timeValueBox_, 0, 0);
  lv_obj_set_style_pad_all(timeValueBox_, 0, 0);
  lv_obj_clear_flag(timeValueBox_, LV_OBJ_FLAG_SCROLLABLE);

  for (uint8_t i = 0; i < kTimeCharSlots; ++i) {
    labelTimeChars_[i] = lv_label_create(timeValueBox_);
    lv_label_set_text(labelTimeChars_[i], "");
    lv_obj_set_style_text_color(labelTimeChars_[i], lv_color_white(), 0);
    lv_obj_set_style_text_font(labelTimeChars_[i], FONT_TIME, 0);
    lv_obj_set_width(labelTimeChars_[i], TIME_CHAR_WIDTH);
    lv_obj_set_height(labelTimeChars_[i], TIME_CHAR_HEIGHT);
    lv_obj_set_style_text_align(labelTimeChars_[i], LV_TEXT_ALIGN_LEFT, 0);
    lv_obj_set_pos(labelTimeChars_[i], static_cast<lv_coord_t>(i * TIME_CHAR_WIDTH), 0);
  }
  renderTimeText("00:00.0");

  lv_obj_t *labelSatsTitle = lv_label_create(cellSats_);
  lv_label_set_text(labelSatsTitle, "SATs");
  lv_obj_set_style_text_color(labelSatsTitle, lv_color_white(), 0);
  lv_obj_set_style_text_font(labelSatsTitle, FONT_TITLE, 0);
  lv_obj_set_pos(labelSatsTitle, 232, 106);

  labelModeStatus_ = lv_label_create(cellSats_);
  lv_label_set_text(labelModeStatus_, "MODE AUTO");
  lv_obj_set_style_text_color(labelModeStatus_, lv_color_hex(0x82ff3f), 0);
  lv_obj_set_style_text_font(labelModeStatus_, FONT_UNIT, 0);
  lv_obj_set_pos(labelModeStatus_, 24, 18);

  labelUsingStatus_ = lv_label_create(cellSats_);
  lv_label_set_text(labelUsingStatus_, "USING NONE");
  lv_obj_set_style_text_color(labelUsingStatus_, lv_color_white(), 0);
  lv_obj_set_style_text_font(labelUsingStatus_, FONT_UNIT, 0);
  lv_obj_set_pos(labelUsingStatus_, 24, 56);

  labelGnssLink_ = lv_label_create(cellSats_);
  lv_label_set_text(labelGnssLink_, "LINK LOST");
  lv_obj_set_style_text_color(labelGnssLink_, lv_color_hex(STALE_TEXT_COLOR), 0);
  lv_obj_set_style_text_font(labelGnssLink_, FONT_UNIT, 0);
  lv_obj_set_pos(labelGnssLink_, 24, 96);

  labelGnssCn0_ = lv_label_create(cellSats_);
  lv_label_set_text(labelGnssCn0_, "C/N0 --.-");
  lv_obj_set_style_text_color(labelGnssCn0_, lv_color_hex(STALE_TEXT_COLOR), 0);
  lv_obj_set_style_text_font(labelGnssCn0_, FONT_UNIT, 0);
  lv_obj_set_pos(labelGnssCn0_, 24, 132);

  labelSatsValue_ = lv_label_create(cellSats_);
  lv_label_set_text(labelSatsValue_, "0");
  lv_obj_set_style_text_color(labelSatsValue_, lv_color_hex(SATS_LIVE_COLOR), 0);
  lv_obj_set_style_text_font(labelSatsValue_, FONT_NUMBER, 0);
  lv_obj_set_pos(labelSatsValue_, 72, 120);

  buttonCanMonitor_ = lv_btn_create(cellSats_);
  lv_obj_set_size(buttonCanMonitor_, 118, 108);
  lv_obj_align(buttonCanMonitor_, LV_ALIGN_BOTTOM_RIGHT, -118, 0);
  lv_obj_set_style_radius(buttonCanMonitor_, 0, 0);
  lv_obj_set_style_bg_color(buttonCanMonitor_, lv_color_black(), 0);
  lv_obj_set_style_border_width(buttonCanMonitor_, 2, 0);
  lv_obj_set_style_border_color(buttonCanMonitor_, lv_color_white(), 0);
  lv_obj_set_style_shadow_width(buttonCanMonitor_, 0, 0);
  lv_obj_add_event_cb(buttonCanMonitor_, onCanMonitorButton, LV_EVENT_CLICKED, nullptr);

  lv_obj_t *labelCanMonitor = lv_label_create(buttonCanMonitor_);
  lv_label_set_text(labelCanMonitor, "CAN");
  lv_obj_set_style_text_color(labelCanMonitor, lv_color_white(), 0);
  lv_obj_set_style_text_font(labelCanMonitor, FONT_BUTTON, 0);
  lv_obj_center(labelCanMonitor);

  buttonReset_ = lv_btn_create(cellSats_);
  lv_obj_set_size(buttonReset_, 118, 108);
  lv_obj_align(buttonReset_, LV_ALIGN_BOTTOM_RIGHT, 0, 0);
  lv_obj_set_style_radius(buttonReset_, 0, 0);
  lv_obj_set_style_bg_color(buttonReset_, lv_color_black(), 0);
  lv_obj_set_style_border_width(buttonReset_, 2, 0);
  lv_obj_set_style_border_color(buttonReset_, lv_color_white(), 0);
  lv_obj_set_style_shadow_width(buttonReset_, 0, 0);
  lv_obj_add_event_cb(buttonReset_, onResetButton, LV_EVENT_CLICKED, nullptr);

  lv_obj_t *labelReset = lv_label_create(buttonReset_);
  lv_label_set_text(labelReset, "RESET");
  lv_obj_set_style_text_color(labelReset, lv_color_white(), 0);
  lv_obj_set_style_text_font(labelReset, FONT_BUTTON, 0);
  lv_obj_center(labelReset);

  overlayCanMonitor_ = lv_obj_create(screen);
  lv_obj_set_size(overlayCanMonitor_, 1024, 600);
  lv_obj_set_pos(overlayCanMonitor_, 0, 0);
  lv_obj_set_style_radius(overlayCanMonitor_, 0, 0);
  lv_obj_set_style_bg_color(overlayCanMonitor_, lv_color_black(), 0);
  lv_obj_set_style_border_width(overlayCanMonitor_, 2, 0);
  lv_obj_set_style_border_color(overlayCanMonitor_, lv_color_white(), 0);
  lv_obj_set_style_pad_all(overlayCanMonitor_, 20, 0);
  lv_obj_set_style_shadow_width(overlayCanMonitor_, 0, 0);
  lv_obj_add_flag(overlayCanMonitor_, LV_OBJ_FLAG_HIDDEN);

  lv_obj_t *labelCanMonitorTitle = lv_label_create(overlayCanMonitor_);
  lv_label_set_text(labelCanMonitorTitle, "CAN Monitor");
  lv_obj_set_style_text_color(labelCanMonitorTitle, lv_color_white(), 0);
  lv_obj_set_style_text_font(labelCanMonitorTitle, FONT_TITLE, 0);
  lv_obj_align(labelCanMonitorTitle, LV_ALIGN_TOP_LEFT, 8, 4);

  labelCanMonitorText_ = lv_label_create(overlayCanMonitor_);
  lv_label_set_text(labelCanMonitorText_, "Waiting for CAN data...");
  lv_label_set_long_mode(labelCanMonitorText_, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(labelCanMonitorText_, 940);
  lv_obj_set_style_text_color(labelCanMonitorText_, lv_color_white(), 0);
  lv_obj_set_style_text_font(labelCanMonitorText_, FONT_UNIT, 0);
  lv_obj_align(labelCanMonitorText_, LV_ALIGN_TOP_LEFT, 8, 68);

  buttonCanMonitorSend_ = lv_btn_create(overlayCanMonitor_);
  lv_obj_set_size(buttonCanMonitorSend_, 190, 80);
  lv_obj_align(buttonCanMonitorSend_, LV_ALIGN_BOTTOM_LEFT, 0, 0);
  lv_obj_set_style_radius(buttonCanMonitorSend_, 0, 0);
  lv_obj_set_style_bg_color(buttonCanMonitorSend_, lv_color_black(), 0);
  lv_obj_set_style_border_width(buttonCanMonitorSend_, 2, 0);
  lv_obj_set_style_border_color(buttonCanMonitorSend_, lv_color_white(), 0);
  lv_obj_set_style_shadow_width(buttonCanMonitorSend_, 0, 0);
  lv_obj_add_event_cb(buttonCanMonitorSend_, onCanMonitorSendButton, LV_EVENT_CLICKED, nullptr);

  lv_obj_t *labelCanMonitorSend = lv_label_create(buttonCanMonitorSend_);
  lv_label_set_text(labelCanMonitorSend, "Send CAN");
  lv_obj_set_style_text_color(labelCanMonitorSend, lv_color_white(), 0);
  lv_obj_set_style_text_font(labelCanMonitorSend, FONT_BUTTON, 0);
  lv_obj_center(labelCanMonitorSend);

  buttonCanMonitorBack_ = lv_btn_create(overlayCanMonitor_);
  lv_obj_set_size(buttonCanMonitorBack_, 150, 80);
  lv_obj_align(buttonCanMonitorBack_, LV_ALIGN_BOTTOM_RIGHT, 0, 0);
  lv_obj_set_style_radius(buttonCanMonitorBack_, 0, 0);
  lv_obj_set_style_bg_color(buttonCanMonitorBack_, lv_color_black(), 0);
  lv_obj_set_style_border_width(buttonCanMonitorBack_, 2, 0);
  lv_obj_set_style_border_color(buttonCanMonitorBack_, lv_color_white(), 0);
  lv_obj_set_style_shadow_width(buttonCanMonitorBack_, 0, 0);
  lv_obj_add_event_cb(buttonCanMonitorBack_, onCanMonitorBackButton, LV_EVENT_CLICKED, nullptr);

  lv_obj_t *labelCanMonitorBack = lv_label_create(buttonCanMonitorBack_);
  lv_label_set_text(labelCanMonitorBack, "Back");
  lv_obj_set_style_text_color(labelCanMonitorBack, lv_color_white(), 0);
  lv_obj_set_style_text_font(labelCanMonitorBack, FONT_BUTTON, 0);
  lv_obj_center(labelCanMonitorBack);
}

void UiManager::update(const UiSnapshot &snapshot) {
  char valueBuf[32];
  char unitBuf[16];

  localUtcOffsetMinutes_ = snapshot.localUtcOffsetMinutes;
  updateUtcAnchor(snapshot.gps.timeStr);

  updateSpeedDisplay(
      labelExtValue_,
      labelExtUnit_,
      extGaugeSegments_,
      snapshot.extSpeedKmh,
      snapshot.extValid,
      extUnit_,
      lastExtSpeedKmh_,
      hasLastExtSpeed_);
  updateSpeedDisplay(
      labelGpsValue_,
      labelGpsUnit_,
      gpsGaugeSegments_,
      snapshot.gpsSpeedKmh,
      snapshot.gpsValid,
      gpsUnit_,
      lastGpsSpeedKmh_,
      hasLastGpsSpeed_);
  updateSpeedDisplay(
      labelCanValue_,
      labelCanUnit_,
      canGaugeSegments_,
      snapshot.canSpeedKmh,
      snapshot.canValid,
      canUnit_,
      lastCanSpeedKmh_,
      hasLastCanSpeed_);

  formatDistance(valueBuf, sizeof(valueBuf), unitBuf, sizeof(unitBuf), snapshot.distanceMeters);
  const bool distanceValueChanged = strcmp(lv_label_get_text(labelDistanceValue_), valueBuf) != 0;
  const bool distanceUnitChanged = strcmp(lv_label_get_text(labelDistanceUnit_), unitBuf) != 0;
  if (distanceValueChanged || distanceUnitChanged) {
    lv_label_set_text(labelDistanceValue_, valueBuf);
    lv_label_set_text(labelDistanceUnit_, unitBuf);
    lv_obj_invalidate(labelDistanceValue_);
    lv_obj_invalidate(labelDistanceUnit_);
    lv_obj_invalidate(cellDistance_);
  }

  formatTime(valueBuf, sizeof(valueBuf), snapshot);
  renderTimeText(valueBuf);
  lv_label_set_text(labelTimeTitle_, timeMode_ == TIME_DISPLAY_TRIP ? "Time" : "Local");

  bool satsStale = false;
  int displaySats = snapshot.gps.satellites;
  if (snapshot.gpsValid) {
    lastValidSats_ = snapshot.gps.satellites;
    hasLastSats_ = true;
  } else if (hasLastSats_) {
    displaySats = lastValidSats_;
    satsStale = true;
  }

  if (!snapshot.gpsValid && !hasLastSats_) {
    snprintf(valueBuf, sizeof(valueBuf), "--");
  } else {
    snprintf(valueBuf, sizeof(valueBuf), "%d", displaySats > 99 ? 99 : displaySats);
  }
  lv_label_set_text(labelSatsValue_, valueBuf);
  lv_obj_set_style_text_color(
      labelSatsValue_,
      lv_color_hex(satsStale ? STALE_TEXT_COLOR : SATS_LIVE_COLOR),
      0);

  if (labelGnssLink_ != nullptr) {
    lv_label_set_text(labelGnssLink_, gnssLinkQualityToText(snapshot.gnssLinkQuality));
    lv_obj_set_style_text_color(
        labelGnssLink_,
        gnssLinkQualityColor(snapshot.gnssLinkQuality),
        0);
  }

  if (labelGnssCn0_ != nullptr) {
    bool cn0Stale = false;
    float displayCn0DbHz = snapshot.gps.cn0AvgDbHz;
    if (snapshot.gps.cn0Valid) {
      lastValidGnssCn0DbHz_ = snapshot.gps.cn0AvgDbHz;
      hasLastGnssCn0_ = true;
    } else if (hasLastGnssCn0_) {
      displayCn0DbHz = lastValidGnssCn0DbHz_;
      cn0Stale = true;
    }

    if (snapshot.gps.cn0Valid || hasLastGnssCn0_) {
      snprintf(
          valueBuf,
          sizeof(valueBuf),
          "C/N0 %.1f",
          static_cast<double>(displayCn0DbHz));
    } else {
      snprintf(valueBuf, sizeof(valueBuf), "C/N0 --.-");
    }
    lv_label_set_text(labelGnssCn0_, valueBuf);
    lv_obj_set_style_text_color(
        labelGnssCn0_,
        cn0Stale ? lv_color_hex(STALE_TEXT_COLOR) : gnssCn0Color(snapshot.gps),
        0);
  }

  lv_label_set_text(labelCanMonitorText_, snapshot.canMonitorText[0] ? snapshot.canMonitorText : "Waiting for CAN data...");

  if (labelModeStatus_ != nullptr) {
    snprintf(valueBuf, sizeof(valueBuf), "MODE %s", modeToText(snapshot.sourceMode));
    lv_label_set_text(labelModeStatus_, valueBuf);
  }
  if (labelUsingStatus_ != nullptr) {
    snprintf(valueBuf, sizeof(valueBuf), "USING %s", sourceToText(snapshot.selectedSource));
    lv_label_set_text(labelUsingStatus_, valueBuf);
  }

  const SpeedSource highlightedSource = getHighlightedSource(snapshot);
  setCellHighlight(cellExt_, highlightedSource == SPEED_SOURCE_EXT);
  setCellHighlight(cellGps_, highlightedSource == SPEED_SOURCE_GNSS);
  setCellHighlight(cellCan_, highlightedSource == SPEED_SOURCE_CAN);
}

uint8_t UiManager::consumeActions() {
  const uint8_t actions = pendingActions_;
  pendingActions_ = UI_ACTION_NONE;
  return actions;
}

void UiManager::onExtPanel(lv_event_t *e) {
  (void)e;
  if (instance_ != nullptr) {
    instance_->toggleDisplayUnit(instance_->extUnit_);
  }
}

void UiManager::onGpsPanel(lv_event_t *e) {
  (void)e;
  if (instance_ != nullptr) {
    instance_->toggleDisplayUnit(instance_->gpsUnit_);
  }
}

void UiManager::onCanPanel(lv_event_t *e) {
  (void)e;
  if (instance_ != nullptr) {
    instance_->toggleDisplayUnit(instance_->canUnit_);
  }
}

void UiManager::onDistancePanel(lv_event_t *e) {
  (void)e;
  if (instance_ != nullptr) {
    instance_->toggleDistanceUnit();
  }
}

void UiManager::onTimePanel(lv_event_t *e) {
  (void)e;
  if (instance_ != nullptr) {
    instance_->toggleTimeMode();
  }
}

void UiManager::onCanMonitorButton(lv_event_t *e) {
  (void)e;
  if (instance_ != nullptr) {
    instance_->showCanMonitor(true);
  }
}

void UiManager::onCanMonitorSendButton(lv_event_t *e) {
  (void)e;
  if (instance_ != nullptr) {
    instance_->queueAction(UI_ACTION_SEND_TEST_CAN);
  }
}

void UiManager::onCanMonitorBackButton(lv_event_t *e) {
  (void)e;
  if (instance_ != nullptr) {
    instance_->showCanMonitor(false);
  }
}

void UiManager::onResetButton(lv_event_t *e) {
  (void)e;
  if (instance_ != nullptr) {
    instance_->queueAction(UI_ACTION_RESET_DISTANCE);
  }
}

lv_obj_t *UiManager::createCell(
    lv_obj_t *parent,
    lv_coord_t x,
    lv_coord_t y,
    lv_coord_t w,
    lv_coord_t h,
    lv_event_cb_t cb) {
  lv_obj_t *cell = lv_btn_create(parent);
  lv_obj_set_size(cell, w, h);
  lv_obj_set_pos(cell, x, y);
  lv_obj_set_style_radius(cell, 0, 0);
  lv_obj_set_style_bg_color(cell, lv_color_black(), 0);
  lv_obj_set_style_border_width(cell, 2, 0);
  lv_obj_set_style_border_color(cell, lv_color_white(), 0);
  lv_obj_set_style_shadow_width(cell, 0, 0);
  lv_obj_set_style_pad_all(cell, 0, 0);
  if (cb != nullptr) {
    lv_obj_add_event_cb(cell, cb, LV_EVENT_CLICKED, nullptr);
  } else {
    lv_obj_clear_flag(cell, LV_OBJ_FLAG_CLICKABLE);
  }
  return cell;
}

lv_obj_t *UiManager::createTitle(lv_obj_t *parent, const char *text) {
  lv_obj_t *label = lv_label_create(parent);
  lv_label_set_text(label, text);
  lv_obj_set_style_text_color(label, lv_color_white(), 0);
  lv_obj_set_style_text_font(label, FONT_TITLE, 0);
  lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 18);
  return label;
}

lv_obj_t *UiManager::createSegmentDisplay(
    lv_obj_t *parent,
    lv_coord_t x,
    lv_coord_t y,
    lv_coord_t w,
    lv_coord_t h) {
  lv_obj_t *display = lv_obj_create(parent);
  lv_obj_set_size(display, w, h);
  lv_obj_set_pos(display, x, y);
  lv_obj_set_style_bg_opa(display, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(display, 0, 0);
  lv_obj_set_style_radius(display, 0, 0);
  lv_obj_set_style_pad_all(display, 0, 0);
  lv_obj_clear_flag(display, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(display, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(display, onSpeedSegmentDraw, LV_EVENT_DRAW_MAIN, nullptr);
  return display;
}

void UiManager::createSpeedGauge(
    lv_obj_t *parent,
    lv_obj_t **segments,
    lv_coord_t x,
    lv_coord_t y,
    lv_coord_t w) {
  if ((parent == nullptr) || (segments == nullptr)) {
    return;
  }

  const lv_color_t gaugeColor = lv_color_hex(SPEED_GAUGE_COLOR_HEX);
  for (uint8_t index = 0; index < UiManager::kSpeedGaugeSegmentCount; ++index) {
    lv_obj_t *segment = lv_obj_create(parent);
    segments[index] = segment;

    const lv_coord_t segmentY =
        y + static_cast<lv_coord_t>((UiManager::kSpeedGaugeSegmentCount - 1 - index) *
                                    (SPEED_GAUGE_SEGMENT_HEIGHT + SPEED_GAUGE_SEGMENT_GAP));
    lv_obj_set_pos(segment, x, segmentY);
    lv_obj_set_size(segment, w, SPEED_GAUGE_SEGMENT_HEIGHT);
    lv_obj_set_style_bg_color(segment, gaugeColor, 0);
    lv_obj_set_style_bg_opa(segment, SPEED_GAUGE_OFF_OPACITY, 0);
    lv_obj_set_style_border_width(segment, 0, 0);
    lv_obj_set_style_radius(segment, 6, 0);
    lv_obj_set_style_shadow_width(segment, 0, 0);
    lv_obj_clear_flag(segment, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(segment, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_move_background(segment);
  }
}

void UiManager::onSpeedSegmentDraw(lv_event_t *e) {
  if (instance_ != nullptr) {
    instance_->drawSpeedSegmentDisplay(e);
  }
}

void UiManager::queueAction(uint8_t action) {
  pendingActions_ |= action;
}

void UiManager::toggleDisplayUnit(DisplayUnit &unit) {
  unit = (unit == DISPLAY_UNIT_KMH) ? DISPLAY_UNIT_MPH : DISPLAY_UNIT_KMH;
}

void UiManager::toggleDistanceUnit() {
  distanceUnit_ =
      (distanceUnit_ == DISTANCE_UNIT_METER) ? DISTANCE_UNIT_MILE : DISTANCE_UNIT_METER;
}

void UiManager::toggleTimeMode() {
  timeMode_ =
      (timeMode_ == TIME_DISPLAY_TRIP) ? TIME_DISPLAY_LOCAL : TIME_DISPLAY_TRIP;
}

void UiManager::showCanMonitor(bool visible) {
  if (overlayCanMonitor_ == nullptr) {
    return;
  }

  if (visible) {
    lv_obj_clear_flag(overlayCanMonitor_, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(overlayCanMonitor_);
  } else {
    lv_obj_add_flag(overlayCanMonitor_, LV_OBJ_FLAG_HIDDEN);
  }
}

void UiManager::setCellHighlight(lv_obj_t *cell, bool active) {
  if (cell == nullptr) {
    return;
  }

  lv_obj_set_style_border_color(
      cell,
      active ? lv_color_hex(0x82ff3f) : lv_color_white(),
      0);
  lv_obj_set_style_border_width(cell, active ? 5 : 2, 0);
}

void UiManager::updateUtcAnchor(const char *utcText) {
  if ((utcText == nullptr) || (strlen(utcText) < 8U)) {
    return;
  }

  if (strncmp(utcText, lastUtcSource_, sizeof(lastUtcSource_) - 1U) == 0) {
    return;
  }

  unsigned hour = 0;
  unsigned minute = 0;
  unsigned second = 0;
  if (sscanf(utcText, "%2u:%2u:%2u", &hour, &minute, &second) != 3) {
    return;
  }

  if ((hour > 23U) || (minute > 59U) || (second > 59U)) {
    return;
  }

  utcAnchorSecondsOfDay_ = (hour * 3600U) + (minute * 60U) + second;
  utcAnchorMs_ = millis();
  hasUtcAnchor_ = true;
  snprintf(lastUtcSource_, sizeof(lastUtcSource_), "%s", utcText);
}

bool UiManager::tryFormatAnchoredLocal(char *timeBuf, size_t timeBufSize) const {
  if (!hasUtcAnchor_) {
    return false;
  }

  const uint32_t elapsedSeconds = (millis() - utcAnchorMs_) / 1000U;
  const uint32_t totalSeconds =
      applyLocalUtcOffset((utcAnchorSecondsOfDay_ + elapsedSeconds) % 86400U, localUtcOffsetMinutes_);
  const uint32_t hour = totalSeconds / 3600U;
  const uint32_t minute = (totalSeconds / 60U) % 60U;
  const uint32_t second = totalSeconds % 60U;

  snprintf(
      timeBuf,
      timeBufSize,
      "%02lu:%02lu:%02lu",
      static_cast<unsigned long>(hour),
      static_cast<unsigned long>(minute),
      static_cast<unsigned long>(second));
  return true;
}

void UiManager::updateSpeedDisplay(
    lv_obj_t *valueLabel,
    lv_obj_t *unitLabel,
    lv_obj_t **gaugeSegments,
    float incomingSpeedKmh,
    bool valid,
    DisplayUnit unit,
    float &lastValidSpeedKmh,
    bool &hasLastValidSpeed) {
  if ((valueLabel == nullptr) || (unitLabel == nullptr)) {
    return;
  }

  bool staleDisplay = false;
  float displaySpeedKmh = incomingSpeedKmh;

  if (valid) {
    lastValidSpeedKmh = incomingSpeedKmh;
    hasLastValidSpeed = true;
  } else if (hasLastValidSpeed) {
    displaySpeedKmh = lastValidSpeedKmh;
    staleDisplay = true;
    valid = true;
  }

  char valueBuf[32];
  formatSpeed(valueBuf, sizeof(valueBuf), displaySpeedKmh, valid, unit);
  setSegmentDisplayState(
      valueLabel,
      valueBuf,
      staleDisplay ? STALE_TEXT_COLOR : LIVE_TEXT_COLOR);
  updateSpeedGauge(gaugeSegments, displaySpeedKmh, valid, staleDisplay);
  lv_label_set_text(unitLabel, unit == DISPLAY_UNIT_KMH ? "km/h" : "mph");

  const lv_color_t textColor =
      lv_color_hex(staleDisplay ? STALE_TEXT_COLOR : LIVE_TEXT_COLOR);
  lv_obj_set_style_text_color(unitLabel, textColor, 0);
}

void UiManager::updateSpeedGauge(
    lv_obj_t **segments,
    float displaySpeedKmh,
    bool active,
    bool stale) {
  if (segments == nullptr) {
    return;
  }

  uint8_t blueCount = 0;
  uint8_t orangeCount = 0;
  if (active && (displaySpeedKmh > 0.0f)) {
    const float clampedSpeed =
        (displaySpeedKmh > SPEED_GAUGE_MAX_KMH) ? SPEED_GAUGE_MAX_KMH : displaySpeedKmh;

    const float baseBlueSpeed =
        (clampedSpeed > SPEED_GAUGE_BLUE_RANGE_KMH) ? SPEED_GAUGE_BLUE_RANGE_KMH : clampedSpeed;
    blueCount = static_cast<uint8_t>(baseBlueSpeed / SPEED_GAUGE_STEP_KMH);
    if (blueCount > UiManager::kSpeedGaugeSegmentCount) {
      blueCount = UiManager::kSpeedGaugeSegmentCount;
    }

    if (clampedSpeed > SPEED_GAUGE_BLUE_RANGE_KMH) {
      orangeCount = static_cast<uint8_t>(
          (clampedSpeed - SPEED_GAUGE_BLUE_RANGE_KMH) / SPEED_GAUGE_STEP_KMH);
      if (orangeCount > UiManager::kSpeedGaugeSegmentCount) {
        orangeCount = UiManager::kSpeedGaugeSegmentCount;
      }
    }
  }

  const lv_opa_t onOpacity = stale ? SPEED_GAUGE_STALE_OPACITY : SPEED_GAUGE_LIVE_OPACITY;
  const lv_color_t blueColor = lv_color_hex(SPEED_GAUGE_COLOR_HEX);
  const lv_color_t orangeColor = lv_color_hex(SPEED_GAUGE_OVER_COLOR_HEX);
  for (uint8_t index = 0; index < UiManager::kSpeedGaugeSegmentCount; ++index) {
    if (segments[index] == nullptr) {
      continue;
    }

    lv_color_t color = blueColor;
    lv_opa_t opacity = SPEED_GAUGE_OFF_OPACITY;

    if (index < orangeCount) {
      color = orangeColor;
      opacity = onOpacity;
    } else if (index < blueCount) {
      color = blueColor;
      opacity = onOpacity;
    }

    lv_obj_set_style_bg_color(segments[index], color, 0);
    lv_obj_set_style_bg_opa(
        segments[index],
        opacity,
        0);
  }
}

void UiManager::formatSpeed(
    char *valueBuf,
    size_t valueBufSize,
    float speedKmh,
    bool valid,
    DisplayUnit unit) const {
  if (!valid) {
    snprintf(valueBuf, valueBufSize, "--.-");
    return;
  }

  const float displaySpeed =
      (unit == DISPLAY_UNIT_MPH) ? speedKmh * KMH_TO_MPH : speedKmh;
  snprintf(valueBuf, valueBufSize, "%.1f", displaySpeed);
}

void UiManager::formatDistance(
    char *valueBuf,
    size_t valueBufSize,
    char *unitBuf,
    size_t unitBufSize,
    double distanceMeters) const {
  if (distanceUnit_ == DISTANCE_UNIT_MILE) {
    snprintf(valueBuf, valueBufSize, "%.2f", distanceMeters * METERS_TO_MILES);
    snprintf(unitBuf, unitBufSize, "mile");
  } else {
    snprintf(valueBuf, valueBufSize, "%.1f", distanceMeters);
    snprintf(unitBuf, unitBufSize, "m");
  }
}

void UiManager::formatTime(char *timeBuf, size_t timeBufSize, const UiSnapshot &snapshot) const {
  if (timeMode_ == TIME_DISPLAY_LOCAL) {
    if (tryFormatAnchoredLocal(timeBuf, timeBufSize)) {
      return;
    }

    unsigned hour = 0;
    unsigned minute = 0;
    unsigned second = 0;
    if (sscanf(snapshot.gps.timeStr, "%2u:%2u:%2u", &hour, &minute, &second) == 3) {
      const uint32_t localSeconds = applyLocalUtcOffset(
          (hour * 3600U) + (minute * 60U) + second,
          localUtcOffsetMinutes_);
      snprintf(
          timeBuf,
          timeBufSize,
          "%02lu:%02lu:%02lu",
          static_cast<unsigned long>(localSeconds / 3600U),
          static_cast<unsigned long>((localSeconds / 60U) % 60U),
          static_cast<unsigned long>(localSeconds % 60U));
    } else {
      snprintf(timeBuf, timeBufSize, "%s", "--:--:--");
    }
    return;
  }

  const uint32_t totalMs = snapshot.tripElapsedMs;
  const uint32_t totalSeconds = totalMs / 1000U;
  const uint32_t hours = totalSeconds / 3600U;
  const uint32_t minutes = (totalSeconds / 60U) % 60U;
  const uint32_t seconds = totalSeconds % 60U;
  const uint32_t tenths = (totalMs % 1000U) / 100U;

  if (hours > 0U) {
    snprintf(timeBuf, timeBufSize, "%lu:%02lu:%02lu.%lu",
             static_cast<unsigned long>(hours),
             static_cast<unsigned long>(minutes),
             static_cast<unsigned long>(seconds),
             static_cast<unsigned long>(tenths));
    return;
  }

  snprintf(timeBuf, timeBufSize, "%02lu:%02lu.%lu",
           static_cast<unsigned long>(minutes),
           static_cast<unsigned long>(seconds),
           static_cast<unsigned long>(tenths));
}

void UiManager::renderTimeText(const char *timeText) {
  if (timeText == nullptr) {
    return;
  }

  const size_t textLength = strlen(timeText);
  const size_t visibleLength = (textLength > kTimeCharSlots) ? kTimeCharSlots : textLength;
  const size_t startSlot = kTimeCharSlots - visibleLength;

  for (uint8_t i = 0; i < kTimeCharSlots; ++i) {
    if (labelTimeChars_[i] == nullptr) {
      continue;
    }

    if ((i >= startSlot) && ((i - startSlot) < visibleLength)) {
      char charBuf[2] = {
          timeText[i - startSlot],
          '\0',
      };
      lv_label_set_text(labelTimeChars_[i], charBuf);
    } else {
      lv_label_set_text(labelTimeChars_[i], "");
    }
  }
}

void UiManager::setSegmentDisplayState(lv_obj_t *display, const char *text, uint32_t onColorHex) {
  if (display == nullptr || text == nullptr) {
    return;
  }

  SegmentDisplayState *state = nullptr;
  if (display == labelExtValue_) {
    state = &extSpeedDisplayState_;
  } else if (display == labelGpsValue_) {
    state = &gpsSpeedDisplayState_;
  } else if (display == labelCanValue_) {
    state = &canSpeedDisplayState_;
  }

  if (state == nullptr) {
    return;
  }

  bool changed = false;
  if (strncmp(state->text, text, sizeof(state->text)) != 0) {
    snprintf(state->text, sizeof(state->text), "%s", text);
    changed = true;
  }
  if (state->onColorHex != onColorHex) {
    state->onColorHex = onColorHex;
    changed = true;
  }

  if (changed) {
    lv_obj_invalidate(display);
  }
}

void UiManager::drawSpeedSegmentDisplay(lv_event_t *e) const {
  lv_obj_t *display = lv_event_get_target(e);
  lv_draw_ctx_t *drawCtx = lv_event_get_draw_ctx(e);
  if (display == nullptr || drawCtx == nullptr) {
    return;
  }

  const SegmentDisplayState *state = nullptr;
  if (display == labelExtValue_) {
    state = &extSpeedDisplayState_;
  } else if (display == labelGpsValue_) {
    state = &gpsSpeedDisplayState_;
  } else if (display == labelCanValue_) {
    state = &canSpeedDisplayState_;
  }

  if (state == nullptr) {
    return;
  }

  static constexpr lv_coord_t kSlotGap = 6;
  static constexpr lv_coord_t kSegThickness = 8;
  static constexpr lv_coord_t kVerticalInset = 9;
  static constexpr lv_coord_t kHorizontalBarInset = 2;
  static constexpr lv_coord_t kSideInset = 1;
  static constexpr lv_coord_t kMidGap = 4;
  static constexpr lv_coord_t kTopBarLift = 4;
  static constexpr lv_coord_t kTopClusterDrop = 2;
  static constexpr lv_coord_t kBarChamfer = 6;
  static constexpr lv_coord_t kVerticalChamfer = 6;
  static constexpr lv_coord_t kBottomBarLift = 6;
  static constexpr uint8_t kSegTop = 0x01;
  static constexpr uint8_t kSegTopLeft = 0x02;
  static constexpr uint8_t kSegTopRight = 0x04;
  static constexpr uint8_t kSegMid = 0x08;
  static constexpr uint8_t kSegBottomLeft = 0x10;
  static constexpr uint8_t kSegBottomRight = 0x20;
  static constexpr uint8_t kSegBottom = 0x40;

  auto segmentMaskFor = [&](char c) -> uint8_t {
    switch (c) {
      case '0': return kSegTop | kSegTopLeft | kSegTopRight | kSegBottomLeft | kSegBottomRight | kSegBottom;
      case '1': return kSegTopRight | kSegBottomRight;
      case '2': return kSegTop | kSegTopRight | kSegMid | kSegBottomLeft | kSegBottom;
      case '3': return kSegTop | kSegTopRight | kSegMid | kSegBottomRight | kSegBottom;
      case '4': return kSegTopLeft | kSegTopRight | kSegMid | kSegBottomRight;
      case '5': return kSegTop | kSegTopLeft | kSegMid | kSegBottomRight | kSegBottom;
      case '6': return kSegTop | kSegTopLeft | kSegMid | kSegBottomLeft | kSegBottomRight | kSegBottom;
      case '7': return kSegTop | kSegTopRight | kSegBottomRight;
      case '8': return kSegTop | kSegTopLeft | kSegTopRight | kSegMid | kSegBottomLeft | kSegBottomRight | kSegBottom;
      case '9': return kSegTop | kSegTopLeft | kSegTopRight | kSegMid | kSegBottomRight | kSegBottom;
      case '-': return kSegMid;
      default: return 0;
    }
  };

  lv_area_t coords;
  lv_obj_get_coords(display, &coords);
  const lv_coord_t width = lv_area_get_width(&coords);
  const lv_coord_t height = lv_area_get_height(&coords);
  const lv_coord_t slotWidth = (width - (kSlotGap * (SPEED_SEGMENT_SLOT_COUNT - 1))) / SPEED_SEGMENT_SLOT_COUNT;
  const lv_coord_t slotHeight = height;
  const lv_coord_t verticalLen = (slotHeight - (2 * kVerticalInset) - (3 * kSegThickness) - kMidGap) / 2;

  char slots[SPEED_SEGMENT_SLOT_COUNT];
  memset(slots, ' ', sizeof(slots));
  const size_t textLen = strnlen(state->text, sizeof(state->text));
  const size_t copyLen = (textLen > SPEED_SEGMENT_SLOT_COUNT) ? SPEED_SEGMENT_SLOT_COUNT : textLen;
  for (size_t i = 0; i < copyLen; ++i) {
    slots[SPEED_SEGMENT_SLOT_COUNT - copyLen + i] = state->text[i];
  }

  lv_draw_rect_dsc_t rectDsc;
  lv_draw_rect_dsc_init(&rectDsc);
  rectDsc.radius = 0;
  rectDsc.bg_opa = LV_OPA_COVER;
  rectDsc.border_width = 0;

  const lv_color_t onColor = lv_color_hex(state->onColorHex);
  const lv_color_t offColor = lv_color_hex(segmentOffColorFor(state->onColorHex));

  auto drawHorizontalSegment = [&](const lv_area_t &area, bool on) {
    rectDsc.bg_color = on ? onColor : offColor;
    lv_point_t points[6] = {
        {static_cast<lv_coord_t>(area.x1 + kBarChamfer), area.y1},
        {static_cast<lv_coord_t>(area.x2 - kBarChamfer), area.y1},
        {area.x2, static_cast<lv_coord_t>((area.y1 + area.y2) / 2)},
        {static_cast<lv_coord_t>(area.x2 - kBarChamfer), area.y2},
        {static_cast<lv_coord_t>(area.x1 + kBarChamfer), area.y2},
        {area.x1, static_cast<lv_coord_t>((area.y1 + area.y2) / 2)},
    };
    lv_draw_polygon(drawCtx, &rectDsc, points, 6);
  };

  auto drawVerticalSegment = [&](const lv_area_t &area, bool on) {
    rectDsc.bg_color = on ? onColor : offColor;
    const lv_coord_t centerX = static_cast<lv_coord_t>((area.x1 + area.x2) / 2);
    lv_point_t points[6] = {
        {centerX, area.y1},
        {area.x2, static_cast<lv_coord_t>(area.y1 + kVerticalChamfer)},
        {area.x2, static_cast<lv_coord_t>(area.y2 - kVerticalChamfer)},
        {centerX, area.y2},
        {area.x1, static_cast<lv_coord_t>(area.y2 - kVerticalChamfer)},
        {area.x1, static_cast<lv_coord_t>(area.y1 + kVerticalChamfer)},
    };
    lv_draw_polygon(drawCtx, &rectDsc, points, 6);
  };

  for (uint8_t slot = 0; slot < SPEED_SEGMENT_SLOT_COUNT; ++slot) {
    const char c = slots[slot];
    const lv_coord_t baseX = coords.x1 + static_cast<lv_coord_t>(slot * (slotWidth + kSlotGap));
    const lv_coord_t baseY = coords.y1;

    if (c == '.') {
      lv_area_t dotArea{
          static_cast<lv_coord_t>(baseX + (slotWidth / 2) - 6),
          static_cast<lv_coord_t>(baseY + slotHeight - 22),
          static_cast<lv_coord_t>(baseX + (slotWidth / 2) + 6),
          static_cast<lv_coord_t>(baseY + slotHeight - 10)};
      rectDsc.bg_color = onColor;
      lv_draw_rect(drawCtx, &rectDsc, &dotArea);
      continue;
    }

    const uint8_t mask = segmentMaskFor(c);

    lv_area_t topArea{
        static_cast<lv_coord_t>(baseX + kHorizontalBarInset + kSegThickness / 2),
        static_cast<lv_coord_t>(baseY + kVerticalInset - kTopBarLift + kTopClusterDrop),
        static_cast<lv_coord_t>(baseX + slotWidth - kHorizontalBarInset - kSegThickness / 2),
        static_cast<lv_coord_t>(baseY + kVerticalInset - kTopBarLift + kTopClusterDrop + kSegThickness)};
    lv_area_t midArea{
        topArea.x1,
        static_cast<lv_coord_t>(baseY + kVerticalInset + verticalLen + kSegThickness + kMidGap / 2),
        topArea.x2,
        static_cast<lv_coord_t>(baseY + kVerticalInset + verticalLen + (2 * kSegThickness) + kMidGap / 2)};
    lv_area_t bottomArea{
        topArea.x1,
        static_cast<lv_coord_t>(baseY + slotHeight - kVerticalInset - kSegThickness - kBottomBarLift),
        topArea.x2,
        static_cast<lv_coord_t>(baseY + slotHeight - kVerticalInset - kBottomBarLift)};
    lv_area_t topLeftArea{
        static_cast<lv_coord_t>(baseX + kSideInset),
        static_cast<lv_coord_t>(baseY + kVerticalInset + kSegThickness / 2 + kTopClusterDrop),
        static_cast<lv_coord_t>(baseX + kSideInset + kSegThickness),
        static_cast<lv_coord_t>(baseY + kVerticalInset + kSegThickness / 2 + kTopClusterDrop + verticalLen)};
    lv_area_t topRightArea{
        static_cast<lv_coord_t>(baseX + slotWidth - kSideInset - kSegThickness),
        topLeftArea.y1,
        static_cast<lv_coord_t>(baseX + slotWidth - kSideInset),
        topLeftArea.y2};
    lv_area_t bottomLeftArea{
        topLeftArea.x1,
        static_cast<lv_coord_t>(midArea.y2 - kSegThickness / 2),
        topLeftArea.x2,
        static_cast<lv_coord_t>(midArea.y2 - kSegThickness / 2 + verticalLen)};
    lv_area_t bottomRightArea{
        topRightArea.x1,
        bottomLeftArea.y1,
        topRightArea.x2,
        bottomLeftArea.y2};

    drawHorizontalSegment(topArea, (mask & kSegTop) != 0U);

    drawVerticalSegment(topLeftArea, (mask & kSegTopLeft) != 0U);

    drawVerticalSegment(topRightArea, (mask & kSegTopRight) != 0U);

    drawHorizontalSegment(midArea, (mask & kSegMid) != 0U);

    drawVerticalSegment(bottomLeftArea, (mask & kSegBottomLeft) != 0U);

    drawVerticalSegment(bottomRightArea, (mask & kSegBottomRight) != 0U);

    drawHorizontalSegment(bottomArea, (mask & kSegBottom) != 0U);
  }
}

SpeedSource UiManager::getHighlightedSource(const UiSnapshot &snapshot) const {
  if (snapshot.sourceMode == SPEED_MODE_AUTO) {
    return snapshot.selectedSource;
  }

  switch (snapshot.sourceMode) {
    case SPEED_MODE_GNSS:
      return SPEED_SOURCE_GNSS;
    case SPEED_MODE_CAN:
      return SPEED_SOURCE_CAN;
    case SPEED_MODE_EXT:
      return SPEED_SOURCE_EXT;
    case SPEED_MODE_AUTO:
    default:
      return snapshot.selectedSource;
  }
}

