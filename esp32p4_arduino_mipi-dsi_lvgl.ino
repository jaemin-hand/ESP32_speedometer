#pragma GCC push_options
#pragma GCC optimize("O3")

#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "driver/twai.h"
#include "lvgl.h"
#include "pins_config.h"
#include "src/lcd/jd9165_lcd.h"
#include "src/touch/gt911_touch.h"

#define CAN_RX_PIN 2
#define CAN_TX_PIN 48

#define GPS_RX_PIN 3
#define GPS_TX_PIN 47

HardwareSerial GPSSerial(1);

jd9165_lcd lcd = jd9165_lcd(LCD_RST);
gt911_touch touch = gt911_touch(TP_I2C_SDA, TP_I2C_SCL, TP_RST, TP_INT);

static lv_disp_draw_buf_t draw_buf;
static lv_color_t *buf;
static lv_color_t *buf1;

constexpr uint8_t SBF_SYNC_1 = 0x24;
constexpr uint8_t SBF_SYNC_2 = 0x40;
constexpr uint16_t SBF_BLOCK_PVT_GEODETIC = 4007;
constexpr uint16_t SBF_BLOCK_RECEIVER_TIME = 5914;
constexpr size_t SBF_BLOCK_BUFFER_SIZE = 512;
constexpr double SBF_DNU_THRESHOLD = -1.99e10;
constexpr double RAD_TO_DEGREES = 57.29577951308232;
constexpr uint32_t UI_UPDATE_INTERVAL_MS = 10;
constexpr uint32_t DEBUG_PRINT_INTERVAL_MS = 1000;
constexpr uint32_t SIGNAL_ALIVE_MS = 500;
constexpr float KMH_TO_MPH = 0.621371f;
constexpr float MPS_TO_KNOTS = 1.9438445f;

#if LV_FONT_MONTSERRAT_48
  #define FONT_SPEED (&lv_font_montserrat_48)
#elif LV_FONT_MONTSERRAT_40
  #define FONT_SPEED (&lv_font_montserrat_40)
#elif LV_FONT_MONTSERRAT_32
  #define FONT_SPEED (&lv_font_montserrat_32)
#else
  #define FONT_SPEED LV_FONT_DEFAULT
#endif

#if LV_FONT_MONTSERRAT_40
  #define FONT_DISTANCE (&lv_font_montserrat_40)
#elif LV_FONT_MONTSERRAT_32
  #define FONT_DISTANCE (&lv_font_montserrat_32)
#else
  #define FONT_DISTANCE LV_FONT_DEFAULT
#endif

#if LV_FONT_MONTSERRAT_32
  #define FONT_TITLE (&lv_font_montserrat_32)
#else
  #define FONT_TITLE LV_FONT_DEFAULT
#endif

enum DisplayUnit {
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

GpsData gps;
static uint8_t sbfBlockBuffer[SBF_BLOCK_BUFFER_SIZE];
static size_t sbfBlockSize = 0;
static size_t sbfExpectedLength = 0;

static DisplayUnit displayUnit = DISPLAY_UNIT_KMH;
static float selectedSpeedKmh = 0.0f;
static float previousIntegratedSpeedKmh = 0.0f;
static double distanceMeters = 0.0;
static uint32_t lastDistanceUpdateMs = 0;
static uint32_t lastCanRxMs = 0;

static lv_obj_t *meterGauge = nullptr;
static lv_meter_scale_t *meterScale = nullptr;
static lv_meter_indicator_t *meterNeedle = nullptr;
static lv_meter_indicator_t *meterArcCool = nullptr;
static lv_meter_indicator_t *meterArcWarm = nullptr;

static lv_obj_t *labelSpeedValue = nullptr;
static lv_obj_t *labelSpeedUnit = nullptr;
static lv_obj_t *labelDistanceValue = nullptr;
static lv_obj_t *labelClock = nullptr;
static lv_obj_t *labelStatusLine1 = nullptr;
static lv_obj_t *labelStatusLine2 = nullptr;
static lv_obj_t *labelStatusLine3 = nullptr;
static lv_obj_t *labelSatCount = nullptr;

static lv_obj_t *badgeAuto = nullptr;
static lv_obj_t *badgeGnss = nullptr;
static lv_obj_t *badgeCan = nullptr;
static lv_obj_t *badgeExt = nullptr;
static lv_obj_t *badgeCorr = nullptr;

void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
  const int offsetx1 = area->x1;
  const int offsetx2 = area->x2;
  const int offsety1 = area->y1;
  const int offsety2 = area->y2;
  lcd.lcd_draw_bitmap(offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, &color_p->full);
  lv_disp_flush_ready(disp);
}

void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{
  (void)indev_driver;

  bool touched;
  uint16_t touchX;
  uint16_t touchY;

  touched = touch.getTouch(&touchX, &touchY);

  if (!touched) {
    data->state = LV_INDEV_STATE_REL;
  } else {
    data->state = LV_INDEV_STATE_PR;
    data->point.x = touchX;
    data->point.y = touchY;
  }
}

template <typename T>
T readLittleEndian(const uint8_t *data) {
  T value;
  memcpy(&value, data, sizeof(T));
  return value;
}

uint16_t computeSbfCrc(const uint8_t *data, size_t length) {
  uint16_t crc = 0;

  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      if ((crc & 0x8000U) != 0) {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021U);
      } else {
        crc <<= 1;
      }
    }
  }

  return crc;
}

bool isSbfDoubleValid(double value) {
  return isfinite(value) && value > SBF_DNU_THRESHOLD;
}

bool isSbfFloatValid(float value) {
  return isfinite(value) && value > static_cast<float>(SBF_DNU_THRESHOLD);
}

float clampFloat(float value, float minValue, float maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

const char* pvtStatusToText(int pvtMode, int errorCode) {
  if (errorCode != 0 || pvtMode == 0) {
    switch (errorCode) {
      case 1: return "NO MEAS";
      case 2: return "NO EPH";
      case 3: return "DOP>15";
      case 4: return "RESIDUAL";
      case 5: return "NO CONV";
      case 6: return "OUTLIER";
      case 7: return "EXPORT";
      case 8: return "NO CORR";
      case 9: return "NO BASE";
      case 10: return "WAIT FIX";
      default: return "NO PVT";
    }
  }

  switch (pvtMode) {
    case 1: return "SPP";
    case 2: return "DGPS";
    case 3: return "FIXED";
    case 4: return "RTK FIX";
    case 5: return "RTK FLOAT";
    case 6: return "SBAS";
    case 7: return "MB RTK FIX";
    case 8: return "MB RTK FLT";
    case 10: return "PPP";
    default: return "OTHER";
  }
}

void parsePvtGeodeticBlock(const uint8_t *block, uint16_t length) {
  if (length < 75) {
    return;
  }

  gps.pvtMode = block[14] & 0x0F;
  gps.errorCode = block[15];
  gps.pvtValid = (gps.errorCode == 0 && gps.pvtMode != 0);
  gps.satellites = block[74];

  if (!gps.pvtValid) {
    gps.speedKnots = 0.0f;
    gps.speedKmh = 0.0f;
    return;
  }

  const double latitudeRad = readLittleEndian<double>(block + 16);
  const double longitudeRad = readLittleEndian<double>(block + 24);
  const double heightM = readLittleEndian<double>(block + 32);
  const float undulationM = readLittleEndian<float>(block + 40);
  const float velocityNorth = readLittleEndian<float>(block + 44);
  const float velocityEast = readLittleEndian<float>(block + 48);

  if (isSbfDoubleValid(latitudeRad)) {
    gps.latitude = latitudeRad * RAD_TO_DEGREES;
  }

  if (isSbfDoubleValid(longitudeRad)) {
    gps.longitude = longitudeRad * RAD_TO_DEGREES;
  }

  if (isSbfDoubleValid(heightM) && isSbfFloatValid(undulationM)) {
    gps.altitudeM = static_cast<float>(heightM - undulationM);
  } else if (isSbfDoubleValid(heightM)) {
    gps.altitudeM = static_cast<float>(heightM);
  }

  if (isSbfFloatValid(velocityNorth) && isSbfFloatValid(velocityEast)) {
    const float speedMps = sqrtf((velocityNorth * velocityNorth) + (velocityEast * velocityEast));
    gps.speedKmh = speedMps * 3.6f;
    gps.speedKnots = speedMps * MPS_TO_KNOTS;
  }
}

void parseReceiverTimeBlock(const uint8_t *block, uint16_t length) {
  if (length < 22) {
    return;
  }

  const int8_t utcYear = static_cast<int8_t>(block[14]);
  const int8_t utcMonth = static_cast<int8_t>(block[15]);
  const int8_t utcDay = static_cast<int8_t>(block[16]);
  const int8_t utcHour = static_cast<int8_t>(block[17]);
  const int8_t utcMinute = static_cast<int8_t>(block[18]);
  const int8_t utcSecond = static_cast<int8_t>(block[19]);
  const uint8_t syncLevel = block[21];

  const bool utcValid =
      ((syncLevel & 0x07U) == 0x07U) &&
      utcYear >= 0 &&
      utcMonth >= 1 && utcMonth <= 12 &&
      utcDay >= 1 && utcDay <= 31 &&
      utcHour >= 0 && utcHour <= 23 &&
      utcMinute >= 0 && utcMinute <= 59 &&
      utcSecond >= 0 && utcSecond <= 59;

  if (!utcValid) {
    return;
  }

  snprintf(gps.dateStr, sizeof(gps.dateStr), "20%02d-%02d-%02d", utcYear, utcMonth, utcDay);
  snprintf(gps.timeStr, sizeof(gps.timeStr), "%02d:%02d:%02d", utcHour, utcMinute, utcSecond);
}

void handleSbfBlock(const uint8_t *block, uint16_t length) {
  if (length < 8) {
    return;
  }

  const uint16_t blockId = readLittleEndian<uint16_t>(block + 4);
  const uint16_t blockNumber = blockId & 0x1FFFU;

  switch (blockNumber) {
    case SBF_BLOCK_PVT_GEODETIC:
      parsePvtGeodeticBlock(block, length);
      break;
    case SBF_BLOCK_RECEIVER_TIME:
      parseReceiverTimeBlock(block, length);
      break;
    default:
      break;
  }
}

void resetSbfDecoder() {
  sbfBlockSize = 0;
  sbfExpectedLength = 0;
}

void processSbfByte(uint8_t byteValue) {
  if (sbfBlockSize == 0) {
    if (byteValue == SBF_SYNC_1) {
      sbfBlockBuffer[sbfBlockSize++] = byteValue;
    }
    return;
  }

  if (sbfBlockSize == 1) {
    if (byteValue == SBF_SYNC_2) {
      sbfBlockBuffer[sbfBlockSize++] = byteValue;
    } else if (byteValue == SBF_SYNC_1) {
      sbfBlockBuffer[0] = byteValue;
      sbfBlockSize = 1;
    } else {
      resetSbfDecoder();
    }
    return;
  }

  if (sbfBlockSize >= SBF_BLOCK_BUFFER_SIZE) {
    resetSbfDecoder();
    return;
  }

  sbfBlockBuffer[sbfBlockSize++] = byteValue;

  if (sbfBlockSize == 8) {
    sbfExpectedLength = readLittleEndian<uint16_t>(sbfBlockBuffer + 6);
    if (sbfExpectedLength < 8 ||
        (sbfExpectedLength % 4) != 0 ||
        sbfExpectedLength > SBF_BLOCK_BUFFER_SIZE) {
      resetSbfDecoder();
      return;
    }
  }

  if (sbfExpectedLength != 0 && sbfBlockSize == sbfExpectedLength) {
    const uint16_t receivedCrc = readLittleEndian<uint16_t>(sbfBlockBuffer + 2);
    const uint16_t computedCrc = computeSbfCrc(sbfBlockBuffer + 4, sbfExpectedLength - 4);

    if (receivedCrc == computedCrc) {
      handleSbfBlock(sbfBlockBuffer, static_cast<uint16_t>(sbfExpectedLength));
    }

    resetSbfDecoder();
  }
}

bool isCanLinkAlive(uint32_t nowMs) {
  return (lastCanRxMs != 0U) && ((nowMs - lastCanRxMs) <= SIGNAL_ALIVE_MS);
}

float getDisplaySpeed() {
  if (displayUnit == DISPLAY_UNIT_MPH) {
    return selectedSpeedKmh * KMH_TO_MPH;
  }

  return selectedSpeedKmh;
}

void applyDisplayUnit() {
  if (meterGauge == nullptr || meterScale == nullptr) {
    return;
  }

  if (displayUnit == DISPLAY_UNIT_KMH) {
    lv_meter_set_scale_range(meterGauge, meterScale, 0, 160, 250, 145);
    lv_meter_set_indicator_start_value(meterGauge, meterArcCool, 0);
    lv_meter_set_indicator_end_value(meterGauge, meterArcCool, 80);
    lv_meter_set_indicator_start_value(meterGauge, meterArcWarm, 80);
    lv_meter_set_indicator_end_value(meterGauge, meterArcWarm, 160);
  } else {
    lv_meter_set_scale_range(meterGauge, meterScale, 0, 100, 250, 145);
    lv_meter_set_indicator_start_value(meterGauge, meterArcCool, 0);
    lv_meter_set_indicator_end_value(meterGauge, meterArcCool, 50);
    lv_meter_set_indicator_start_value(meterGauge, meterArcWarm, 50);
    lv_meter_set_indicator_end_value(meterGauge, meterArcWarm, 100);
  }
}

void setBadgeState(lv_obj_t *badge, bool active, lv_color_t activeColor) {
  if (badge == nullptr) {
    return;
  }

  if (active) {
    lv_obj_set_style_bg_color(badge, activeColor, 0);
    lv_obj_set_style_border_color(badge, lv_color_white(), 0);
    lv_obj_set_style_text_color(badge, lv_color_black(), 0);
  } else {
    lv_obj_set_style_bg_color(badge, lv_color_hex(0x263240), 0);
    lv_obj_set_style_border_color(badge, lv_color_hex(0x435160), 0);
    lv_obj_set_style_text_color(badge, lv_color_hex(0xaab6c1), 0);
  }
}

lv_obj_t *createBadge(lv_obj_t *parent, const char *text, lv_coord_t x, lv_coord_t y, lv_coord_t w) {
  lv_obj_t *badge = lv_btn_create(parent);
  lv_obj_set_size(badge, w, 32);
  lv_obj_set_pos(badge, x, y);
  lv_obj_clear_flag(badge, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(badge, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_style_radius(badge, 18, 0);
  lv_obj_set_style_border_width(badge, 1, 0);
  lv_obj_set_style_shadow_width(badge, 0, 0);
  lv_obj_set_style_pad_all(badge, 0, 0);

  lv_obj_t *label = lv_label_create(badge);
  lv_label_set_text(label, text);
  lv_obj_center(label);
  return badge;
}

lv_obj_t *createRoundButton(lv_obj_t *parent, lv_coord_t x, lv_coord_t y, const char *text, lv_event_cb_t cb) {
  lv_obj_t *btn = lv_btn_create(parent);
  lv_obj_set_size(btn, 86, 86);
  lv_obj_set_pos(btn, x, y);
  lv_obj_set_style_radius(btn, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_bg_color(btn, lv_color_hex(0xf3f1ed), 0);
  lv_obj_set_style_border_width(btn, 4, 0);
  lv_obj_set_style_border_color(btn, lv_color_hex(0x7d7b78), 0);
  lv_obj_set_style_shadow_width(btn, 14, 0);
  lv_obj_set_style_shadow_color(btn, lv_color_hex(0xbab2a9), 0);
  lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, nullptr);

  lv_obj_t *inner = lv_obj_create(btn);
  lv_obj_set_size(inner, 24, 24);
  lv_obj_center(inner);
  lv_obj_clear_flag(inner, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_radius(inner, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_bg_color(inner, lv_color_hex(0xd8d4cf), 0);
  lv_obj_set_style_border_width(inner, 0, 0);

  lv_obj_t *label = lv_label_create(parent);
  lv_label_set_text(label, text);
  lv_obj_set_width(label, 110);
  lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_RIGHT, 0);
  lv_obj_set_style_text_color(label, lv_color_hex(0x272727), 0);
  lv_obj_align_to(label, btn, LV_ALIGN_OUT_LEFT_MID, -18, 0);

  return btn;
}

void sendTestCanFrame() {
  twai_message_t tx_message = {};
  tx_message.identifier = 0x777;
  tx_message.extd = 0;
  tx_message.data_length_code = 4;
  tx_message.data[0] = 0x01;
  tx_message.data[1] = 0x02;
  tx_message.data[2] = 0x03;
  tx_message.data[3] = 0x04;

  if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK) {
    Serial.println("SEL pressed -> CAN test frame sent");
  } else {
    Serial.println("SEL pressed -> CAN test frame failed");
  }
}

static void onUnitButton(lv_event_t *e) {
  (void)e;
  displayUnit = (displayUnit == DISPLAY_UNIT_KMH) ? DISPLAY_UNIT_MPH : DISPLAY_UNIT_KMH;
  applyDisplayUnit();
}

static void onSelButton(lv_event_t *e) {
  (void)e;
  sendTestCanFrame();
}

static void onSourceButton(lv_event_t *e) {
  (void)e;
  Serial.println("Source/CAL pressed -> source fusion UI placeholder");
}

static void onResetButton(lv_event_t *e) {
  (void)e;
  distanceMeters = 0.0;
  previousIntegratedSpeedKmh = selectedSpeedKmh;
  Serial.println("Distance reset");
}

void printGpsSummary() {
  Serial.println("========== GPS SUMMARY ==========");
  Serial.printf("Valid      : %s\n", gps.pvtValid ? "YES" : "NO");
  Serial.printf("Mode       : %d (%s)\n", gps.pvtMode, pvtStatusToText(gps.pvtMode, gps.errorCode));
  Serial.printf("Sats       : %d\n", gps.satellites);
  Serial.printf("Speed      : %.2f km/h\n", selectedSpeedKmh);
  Serial.print("Lat        : "); Serial.println(gps.latitude, 8);
  Serial.print("Lon        : "); Serial.println(gps.longitude, 8);
  Serial.printf("Alt(MSL)   : %.2f m\n", gps.altitudeM);
  Serial.printf("Dist       : %.2f m\n", distanceMeters);
  Serial.printf("Date       : %s\n", strlen(gps.dateStr) ? gps.dateStr : "---- -- --");
  Serial.printf("Time(UTC)  : %s\n", strlen(gps.timeStr) ? gps.timeStr : "--:--:--");
  Serial.println("=================================");
}

void updateVehicleState(uint32_t nowMs) {
  const float currentSpeedKmh = gps.pvtValid ? gps.speedKmh : 0.0f;

  if (lastDistanceUpdateMs == 0U) {
    lastDistanceUpdateMs = nowMs;
    selectedSpeedKmh = currentSpeedKmh;
    previousIntegratedSpeedKmh = currentSpeedKmh;
    return;
  }

  float deltaSeconds = static_cast<float>(nowMs - lastDistanceUpdateMs) * 0.001f;
  if (deltaSeconds < 0.0f) {
    deltaSeconds = 0.0f;
  }
  if (deltaSeconds > 0.1f) {
    deltaSeconds = 0.1f;
  }

  const float averageSpeedMps = ((previousIntegratedSpeedKmh + currentSpeedKmh) * 0.5f) / 3.6f;
  distanceMeters += static_cast<double>(averageSpeedMps * deltaSeconds);

  selectedSpeedKmh = currentSpeedKmh;
  previousIntegratedSpeedKmh = currentSpeedKmh;
  lastDistanceUpdateMs = nowMs;
}

void updateMainUi() {
  const float displaySpeed = getDisplaySpeed();
  const int32_t maxDisplaySpeed = (displayUnit == DISPLAY_UNIT_KMH) ? 160 : 100;
  const int32_t needleValue = static_cast<int32_t>(lroundf(clampFloat(displaySpeed, 0.0f, static_cast<float>(maxDisplaySpeed))));
  const uint32_t nowMs = millis();

  if (meterGauge != nullptr && meterNeedle != nullptr) {
    lv_meter_set_indicator_value(meterGauge, meterNeedle, needleValue);
  }

  if (labelSpeedValue != nullptr) {
    char speedText[16];
    snprintf(speedText, sizeof(speedText), "%05.1f", displaySpeed);
    lv_label_set_text(labelSpeedValue, speedText);
  }

  if (labelSpeedUnit != nullptr) {
    lv_label_set_text(labelSpeedUnit, (displayUnit == DISPLAY_UNIT_KMH) ? "km/h" : "mph");
  }

  if (labelDistanceValue != nullptr) {
    char distText[24];
    snprintf(distText, sizeof(distText), "%07.1f m", distanceMeters);
    lv_label_set_text(labelDistanceValue, distText);
  }

  if (labelClock != nullptr) {
    char timeText[32];
    snprintf(timeText, sizeof(timeText), "UTC %s", strlen(gps.timeStr) ? gps.timeStr : "--:--:--");
    lv_label_set_text(labelClock, timeText);
  }

  if (labelStatusLine1 != nullptr) {
    char line1[64];
    snprintf(line1, sizeof(line1), "FIX %-10s  SAT %02d", pvtStatusToText(gps.pvtMode, gps.errorCode), gps.satellites);
    lv_label_set_text(labelStatusLine1, line1);
  }

  if (labelStatusLine2 != nullptr) {
    char line2[96];
    snprintf(line2, sizeof(line2), "LAT %.5f   LON %.5f", gps.latitude, gps.longitude);
    lv_label_set_text(labelStatusLine2, line2);
  }

  if (labelStatusLine3 != nullptr) {
    char line3[96];
    snprintf(line3, sizeof(line3), "ALT %.1f m   DATE %s", gps.altitudeM, strlen(gps.dateStr) ? gps.dateStr : "---- -- --");
    lv_label_set_text(labelStatusLine3, line3);
  }

  if (labelSatCount != nullptr) {
    char satText[8];
    snprintf(satText, sizeof(satText), "%02d", gps.satellites > 99 ? 99 : gps.satellites);
    lv_label_set_text(labelSatCount, satText);
  }

  const bool gnssActive = gps.pvtValid;
  const bool canActive = isCanLinkAlive(nowMs);

  setBadgeState(badgeAuto, gnssActive || canActive, lv_color_hex(0x7ec9ff));
  setBadgeState(badgeGnss, gnssActive, lv_color_hex(0x8bb5ff));
  setBadgeState(badgeCan, canActive, lv_color_hex(0x77b3ff));
  setBadgeState(badgeExt, false, lv_color_hex(0xc4ff77));
  setBadgeState(badgeCorr, false, lv_color_hex(0xffc24b));
}

void createMainUi() {
  lv_obj_t *screen = lv_scr_act();
  lv_obj_set_style_bg_color(screen, lv_color_hex(0xf4efe8), 0);
  lv_obj_set_style_bg_grad_dir(screen, LV_GRAD_DIR_NONE, 0);
  lv_obj_set_style_pad_all(screen, 0, 0);

  lv_obj_t *title = lv_label_create(screen);
  lv_label_set_text(title, "VSD02-GPS");
  lv_obj_set_style_text_font(title, FONT_TITLE, 0);
  lv_obj_set_style_text_color(title, lv_color_hex(0x1d1d1d), 0);
  lv_obj_set_pos(title, 28, 20);

  lv_obj_t *subtitle = lv_label_create(screen);
  lv_label_set_text(subtitle, "100Hz GPS Speed Sensor / GPS CAN Vehicle Speed Display");
  lv_obj_set_style_text_color(subtitle, lv_color_hex(0x4b4b4b), 0);
  lv_obj_set_pos(subtitle, 250, 34);

  lv_obj_t *displayPanel = lv_obj_create(screen);
  lv_obj_set_size(displayPanel, 710, 485);
  lv_obj_set_pos(displayPanel, 28, 72);
  lv_obj_clear_flag(displayPanel, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_radius(displayPanel, 26, 0);
  lv_obj_set_style_bg_color(displayPanel, lv_color_hex(0x060914), 0);
  lv_obj_set_style_border_width(displayPanel, 12, 0);
  lv_obj_set_style_border_color(displayPanel, lv_color_hex(0xd8cfc4), 0);
  lv_obj_set_style_shadow_width(displayPanel, 16, 0);
  lv_obj_set_style_shadow_color(displayPanel, lv_color_hex(0xc4bbb0), 0);

  // lv_meter is fundamentally a circular widget. Keep it square and let the
  // lower portion clip outside the panel so ticks, arcs, and needle share the
  // same center point.
  meterGauge = lv_meter_create(displayPanel);
  lv_obj_set_size(meterGauge, 414, 414);
  lv_obj_align(meterGauge, LV_ALIGN_TOP_MID, 0, -10);
  lv_obj_set_style_bg_opa(meterGauge, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(meterGauge, 0, 0);
  lv_obj_set_style_pad_all(meterGauge, 0, 0);
  lv_obj_set_style_text_color(meterGauge, lv_color_white(), LV_PART_TICKS);

  meterScale = lv_meter_add_scale(meterGauge);
  lv_meter_set_scale_ticks(meterGauge, meterScale, 41, 1, 8, lv_color_hex(0x6f89a8));
  lv_meter_set_scale_major_ticks(meterGauge, meterScale, 5, 3, 15, lv_color_white(), 12);
  meterArcCool = lv_meter_add_arc(meterGauge, meterScale, 11, lv_color_hex(0x28ddff), 0);
  meterArcWarm = lv_meter_add_arc(meterGauge, meterScale, 11, lv_color_hex(0xffa726), 0);
  meterNeedle = lv_meter_add_needle_line(meterGauge, meterScale, 3, lv_color_hex(0xffd54f), -16);
  applyDisplayUnit();

  labelSpeedUnit = lv_label_create(displayPanel);
  lv_label_set_text(labelSpeedUnit, "km/h");
  lv_obj_set_style_text_color(labelSpeedUnit, lv_color_hex(0xe8f3ff), 0);
  lv_obj_set_style_text_font(labelSpeedUnit, FONT_TITLE, 0);
  lv_obj_align(labelSpeedUnit, LV_ALIGN_TOP_RIGHT, -26, 22);

  labelSpeedValue = lv_label_create(displayPanel);
  lv_label_set_text(labelSpeedValue, "000.0");
  lv_obj_set_style_text_color(labelSpeedValue, lv_color_hex(0xffb300), 0);
  lv_obj_set_style_text_font(labelSpeedValue, FONT_SPEED, 0);
  lv_obj_align(labelSpeedValue, LV_ALIGN_TOP_MID, 42, 152);

  lv_obj_t *distLabel = lv_label_create(displayPanel);
  lv_label_set_text(distLabel, "Dist.");
  lv_obj_set_style_text_color(distLabel, lv_color_hex(0xe7edf7), 0);
  lv_obj_set_style_text_font(distLabel, FONT_TITLE, 0);
  lv_obj_set_pos(distLabel, 24, 308);

  labelDistanceValue = lv_label_create(displayPanel);
  lv_label_set_text(labelDistanceValue, "0000.0 m");
  lv_obj_set_style_text_color(labelDistanceValue, lv_color_hex(0xeef6ff), 0);
  lv_obj_set_style_text_font(labelDistanceValue, FONT_DISTANCE, 0);
  lv_obj_set_pos(labelDistanceValue, 210, 334);

  labelStatusLine1 = lv_label_create(displayPanel);
  lv_label_set_text(labelStatusLine1, "FIX ----       SAT 00");
  lv_obj_set_style_text_color(labelStatusLine1, lv_color_hex(0xb6cae0), 0);
  lv_obj_set_pos(labelStatusLine1, 24, 392);

  labelStatusLine2 = lv_label_create(displayPanel);
  lv_label_set_text(labelStatusLine2, "LAT 0.00000   LON 0.00000");
  lv_obj_set_style_text_color(labelStatusLine2, lv_color_hex(0x91a9c2), 0);
  lv_obj_set_pos(labelStatusLine2, 24, 417);

  labelStatusLine3 = lv_label_create(displayPanel);
  lv_label_set_text(labelStatusLine3, "ALT 0.0 m     DATE ---- -- --");
  lv_obj_set_style_text_color(labelStatusLine3, lv_color_hex(0x91a9c2), 0);
  lv_obj_set_pos(labelStatusLine3, 24, 442);

  labelClock = lv_label_create(displayPanel);
  lv_label_set_text(labelClock, "UTC --:--:--");
  lv_obj_set_style_text_color(labelClock, lv_color_hex(0xeef6ff), 0);
  lv_obj_align(labelClock, LV_ALIGN_BOTTOM_RIGHT, -20, -12);

  badgeExt = createBadge(displayPanel, "EXT", 18, 448, 74);
  badgeGnss = createBadge(displayPanel, "GNSS", 104, 448, 88);
  badgeCan = createBadge(displayPanel, "CAN", 205, 448, 78);
  badgeAuto = createBadge(displayPanel, "AUTO", 295, 448, 86);
  badgeCorr = createBadge(displayPanel, "CORR", 394, 448, 88);

  lv_obj_t *sideTitle = lv_label_create(screen);
  lv_label_set_text(sideTitle, "STATUS");
  lv_obj_set_style_text_color(sideTitle, lv_color_hex(0x2f2f2f), 0);
  lv_obj_set_pos(sideTitle, 782, 88);

  struct LedRow {
    const char *text;
    lv_coord_t y;
  };

  const LedRow ledRows[] = {
    {"EXT_Sen", 128},
    {"GPS", 176},
    {"CAN", 224},
    {"Auto", 272},
    {"CORR", 320},
  };

  for (const LedRow &row : ledRows) {
    lv_obj_t *text = lv_label_create(screen);
    lv_label_set_text(text, row.text);
    lv_obj_set_style_text_color(text, lv_color_hex(0x252525), 0);
    lv_obj_set_pos(text, 780, row.y);
  }

  lv_obj_t *satLabel = lv_label_create(screen);
  lv_label_set_text(satLabel, "GPS / SAT");
  lv_obj_set_style_text_color(satLabel, lv_color_hex(0x252525), 0);
  lv_obj_set_pos(satLabel, 770, 378);

  lv_obj_t *satBox = lv_obj_create(screen);
  lv_obj_set_size(satBox, 122, 96);
  lv_obj_set_pos(satBox, 790, 410);
  lv_obj_clear_flag(satBox, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_radius(satBox, 12, 0);
  lv_obj_set_style_bg_color(satBox, lv_color_hex(0x831515), 0);
  lv_obj_set_style_border_width(satBox, 4, 0);
  lv_obj_set_style_border_color(satBox, lv_color_hex(0x3a0a0a), 0);
  lv_obj_set_style_shadow_width(satBox, 18, 0);
  lv_obj_set_style_shadow_color(satBox, lv_color_hex(0x6f2929), 0);

  labelSatCount = lv_label_create(satBox);
  lv_label_set_text(labelSatCount, "00");
  lv_obj_set_style_text_color(labelSatCount, lv_color_hex(0xfff2f2), 0);
  lv_obj_set_style_text_font(labelSatCount, FONT_DISTANCE, 0);
  lv_obj_center(labelSatCount);

  createRoundButton(screen, 918, 98, "Unit", onUnitButton);
  createRoundButton(screen, 918, 202, "SEL", onSelButton);
  createRoundButton(screen, 918, 306, "Source\nCAL", onSourceButton);
  createRoundButton(screen, 918, 474, "Reset", onResetButton);
}

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESP32P4 speed display prototype");

  lcd.begin();
  touch.begin();

  lv_init();

  size_t buffer_size = sizeof(lv_color_t) * LCD_H_RES * LCD_V_RES;
  buf = (lv_color_t *)heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM);
  buf1 = (lv_color_t *)heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM);
  assert(buf);
  assert(buf1);

  lv_disp_draw_buf_init(&draw_buf, buf, buf1, LCD_H_RES * LCD_V_RES);

  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = LCD_H_RES;
  disp_drv.ver_res = LCD_V_RES;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  disp_drv.full_refresh = false;
  lv_disp_drv_register(&disp_drv);

  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register(&indev_drv);

  GPSSerial.begin(115200, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("GNSS serial ready for SBF");

  twai_general_config_t g_config =
      TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();

  twai_filter_config_t f_config;
  f_config.acceptance_code = 0;
  f_config.acceptance_mask = 0;
  f_config.single_filter = true;

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    twai_start();
    Serial.println("CAN(TWAI) initialized");
  } else {
    Serial.println("CAN(TWAI) initialization failed");
  }

  createMainUi();
}

void loop()
{
  static unsigned long lastPrintMs = 0;
  static unsigned long lastUiUpdateMs = 0;

  const uint32_t nowMs = millis();

  twai_message_t rx_message;
  if (twai_receive(&rx_message, pdMS_TO_TICKS(0)) == ESP_OK) {
    lastCanRxMs = nowMs;
    Serial.printf("CAN received, ID: 0x%X, len: %d\n", rx_message.identifier, rx_message.data_length_code);
  }

  while (GPSSerial.available()) {
    processSbfByte(static_cast<uint8_t>(GPSSerial.read()));
  }

  updateVehicleState(nowMs);

  if ((nowMs - lastPrintMs) >= DEBUG_PRINT_INTERVAL_MS) {
    lastPrintMs = nowMs;
    printGpsSummary();
  }

  if ((nowMs - lastUiUpdateMs) >= UI_UPDATE_INTERVAL_MS) {
    lastUiUpdateMs = nowMs;
    updateMainUi();
  }

  lv_timer_handler();
  delay(1);
}

#pragma GCC pop_options
