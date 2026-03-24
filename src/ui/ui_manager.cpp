#include "ui_manager.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

namespace {

constexpr float KMH_TO_MPH = 0.621371f;
constexpr double METERS_TO_MILES = 0.000621371192;
constexpr lv_coord_t TIME_CHAR_WIDTH = 31;
constexpr lv_coord_t TIME_CHAR_HEIGHT = 92;

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

  labelExtValue_ = lv_label_create(cellExt_);
  lv_label_set_text(labelExtValue_, "--.-");
  lv_obj_set_style_text_color(labelExtValue_, lv_color_white(), 0);
  lv_obj_set_style_text_font(labelExtValue_, FONT_NUMBER, 0);
  lv_obj_set_pos(labelExtValue_, 18, 96);

  labelExtUnit_ = lv_label_create(cellExt_);
  lv_label_set_text(labelExtUnit_, "km/h");
  lv_obj_set_style_text_color(labelExtUnit_, lv_color_white(), 0);
  lv_obj_set_style_text_font(labelExtUnit_, FONT_UNIT, 0);
  lv_obj_set_pos(labelExtUnit_, 200, 186);

  labelGpsValue_ = lv_label_create(cellGps_);
  lv_label_set_text(labelGpsValue_, "--.-");
  lv_obj_set_style_text_color(labelGpsValue_, lv_color_white(), 0);
  lv_obj_set_style_text_font(labelGpsValue_, FONT_NUMBER, 0);
  lv_obj_set_pos(labelGpsValue_, 24, 96);

  labelGpsUnit_ = lv_label_create(cellGps_);
  lv_label_set_text(labelGpsUnit_, "km/h");
  lv_obj_set_style_text_color(labelGpsUnit_, lv_color_white(), 0);
  lv_obj_set_style_text_font(labelGpsUnit_, FONT_UNIT, 0);
  lv_obj_set_pos(labelGpsUnit_, 206, 186);

  labelCanValue_ = lv_label_create(cellCan_);
  lv_label_set_text(labelCanValue_, "--.-");
  lv_obj_set_style_text_color(labelCanValue_, lv_color_white(), 0);
  lv_obj_set_style_text_font(labelCanValue_, FONT_NUMBER, 0);
  lv_obj_set_pos(labelCanValue_, 24, 96);

  labelCanUnit_ = lv_label_create(cellCan_);
  lv_label_set_text(labelCanUnit_, "km/h");
  lv_obj_set_style_text_color(labelCanUnit_, lv_color_white(), 0);
  lv_obj_set_style_text_font(labelCanUnit_, FONT_UNIT, 0);
  lv_obj_set_pos(labelCanUnit_, 206, 186);

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

  labelSatsValue_ = lv_label_create(cellSats_);
  lv_label_set_text(labelSatsValue_, "0");
  lv_obj_set_style_text_color(labelSatsValue_, lv_color_hex(0xd8f7ff), 0);
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

  formatSpeed(valueBuf, sizeof(valueBuf), snapshot.extSpeedKmh, snapshot.extValid, extUnit_);
  lv_label_set_text(labelExtValue_, valueBuf);
  lv_label_set_text(labelExtUnit_, extUnit_ == DISPLAY_UNIT_KMH ? "km/h" : "mph");

  formatSpeed(valueBuf, sizeof(valueBuf), snapshot.gpsSpeedKmh, snapshot.gpsValid, gpsUnit_);
  lv_label_set_text(labelGpsValue_, valueBuf);
  lv_label_set_text(labelGpsUnit_, gpsUnit_ == DISPLAY_UNIT_KMH ? "km/h" : "mph");

  formatSpeed(valueBuf, sizeof(valueBuf), snapshot.canSpeedKmh, snapshot.canValid, canUnit_);
  lv_label_set_text(labelCanValue_, valueBuf);
  lv_label_set_text(labelCanUnit_, canUnit_ == DISPLAY_UNIT_KMH ? "km/h" : "mph");

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
  lv_label_set_text(labelTimeTitle_, timeMode_ == TIME_DISPLAY_TRIP ? "Time" : "UTC");

  snprintf(valueBuf, sizeof(valueBuf), "%d", snapshot.gps.satellites > 99 ? 99 : snapshot.gps.satellites);
  lv_label_set_text(labelSatsValue_, valueBuf);
  lv_label_set_text(labelCanMonitorText_, snapshot.canMonitorText[0] ? snapshot.canMonitorText : "Waiting for CAN data...");

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
      (timeMode_ == TIME_DISPLAY_TRIP) ? TIME_DISPLAY_UTC : TIME_DISPLAY_TRIP;
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
  if (timeMode_ == TIME_DISPLAY_UTC) {
    snprintf(timeBuf, timeBufSize, "%s", strlen(snapshot.gps.timeStr) ? snapshot.gps.timeStr : "--:--:--");
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
