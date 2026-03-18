#pragma GCC push_options
#pragma GCC optimize("O3")

#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "driver/twai.h"
#include "lvgl.h"
#include "pins_config.h"
#include "src/app/app_types.h"
#include "src/can/can_manager.h"
#include "src/distance/distance_manager.h"
#include "src/fusion/fusion_manager.h"
#include "src/gnss/gnss_manager.h"
#include "src/lcd/jd9165_lcd.h"
#include "src/touch/gt911_touch.h"
#include "src/ui/ui_manager.h"

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
static lv_disp_drv_t disp_drv;

constexpr uint32_t UI_UPDATE_INTERVAL_MS = 33;
constexpr uint32_t DEBUG_PRINT_INTERVAL_MS = 1000;
constexpr uint32_t SIGNAL_ALIVE_MS = 500;

GnssManager gnss;
CanManager canManager;
DistanceManager distanceManager;
FusionManager fusionManager;
UiManager uiManager;

bool on_lcd_color_trans_done(esp_lcd_panel_handle_t panel,
                             esp_lcd_dpi_panel_event_data_t *edata,
                             void *user_ctx)
{
  (void)panel;
  (void)edata;

  lv_disp_drv_t *disp = static_cast<lv_disp_drv_t *>(user_ctx);
  if (disp != nullptr) {
    lv_disp_flush_ready(disp);
  }
  return false;
}

void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
  (void)disp;
  const int offsetx1 = area->x1;
  const int offsetx2 = area->x2;
  const int offsety1 = area->y1;
  const int offsety2 = area->y2;
  lcd.lcd_draw_bitmap(offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, &color_p->full);
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

void printGpsSummary() {
  const GpsData &gps = gnss.getData();
  const FusionState &fusionState = fusionManager.getState();
  const CanDecodedSpeedState &canSpeedState = canManager.getDecodedSpeedState();

  Serial.println("========== GPS SUMMARY ==========");
  Serial.printf("Valid      : %s\n", gps.pvtValid ? "YES" : "NO");
  Serial.printf("Mode       : %d (%s)\n", gps.pvtMode, GnssManager::pvtStatusToText(gps.pvtMode, gps.errorCode));
  Serial.printf("SourceMode : %s\n", FusionManager::modeToText(fusionState.mode));
  Serial.printf("Selected   : %s\n", FusionManager::sourceToText(fusionState.selectedSource));
  Serial.printf("CAN Decode : %s\n", canSpeedState.valid ? "YES" : "NO");
  if (canSpeedState.valid) {
    Serial.printf("CAN Speed  : %.2f km/h (%s, 0x%X)\n",
                  canSpeedState.speedKmh,
                  canSpeedState.decoderName,
                  canSpeedState.identifier);
  }
  Serial.printf("Sats       : %d\n", gps.satellites);
  Serial.printf("Speed      : %.2f km/h\n", distanceManager.getSelectedSpeedKmh());
  Serial.print("Lat        : "); Serial.println(gps.latitude, 8);
  Serial.print("Lon        : "); Serial.println(gps.longitude, 8);
  Serial.printf("Alt(MSL)   : %.2f m\n", gps.altitudeM);
  Serial.printf("Dist       : %.2f m\n", distanceManager.getDistanceMeters());
  Serial.printf("Date       : %s\n", strlen(gps.dateStr) ? gps.dateStr : "---- -- --");
  Serial.printf("Time(UTC)  : %s\n", strlen(gps.timeStr) ? gps.timeStr : "--:--:--");
  Serial.println("=================================");
}

UiSnapshot buildUiSnapshot(const GpsData &gps) {
  const FusionState &fusionState = fusionManager.getState();
  UiSnapshot snapshot;
  snapshot.extValid = false;
  snapshot.extSpeedKmh = 0.0f;
  snapshot.gpsValid = gps.pvtValid;
  snapshot.gpsSpeedKmh = gps.speedKmh;
  snapshot.canValid = canManager.hasDecodedSpeed();
  snapshot.canSpeedKmh = canManager.getDecodedSpeedKmh();
  snapshot.distanceMeters = distanceManager.getDistanceMeters();
  snapshot.tripElapsedMs = distanceManager.getTripElapsedMs();
  snapshot.sourceMode = fusionState.mode;
  snapshot.selectedSource = fusionState.selectedSource;
  snapshot.corrActive = fusionState.corrActive;
  snapshot.gps = gps;
  return snapshot;
}

void handleUiActions(uint8_t actions, uint32_t nowMs) {
  if ((actions & UI_ACTION_SEND_TEST_CAN) != 0U) {
    if (canManager.sendTestFrame()) {
      Serial.println("SEL pressed -> CAN test frame sent");
    } else {
      Serial.println("SEL pressed -> CAN test frame failed");
    }
  }

  if ((actions & UI_ACTION_SOURCE_CAL) != 0U) {
    fusionManager.cycleMode();
    Serial.printf("Source/CAL pressed -> source mode = %s\n",
                  FusionManager::modeToText(fusionManager.getState().mode));
  }

  if ((actions & UI_ACTION_RESET_DISTANCE) != 0U) {
    distanceManager.reset(nowMs);
    Serial.println("Trip reset");
  }
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

  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = LCD_H_RES;
  disp_drv.ver_res = LCD_V_RES;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  disp_drv.full_refresh = false;
  lv_disp_drv_register(&disp_drv);

  if (!lcd.registerColorTransferDoneCallback(on_lcd_color_trans_done, &disp_drv)) {
    Serial.println("LCD flush callback registration failed");
  }

  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register(&indev_drv);

  gnss.begin(GPSSerial, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("GNSS serial ready for SBF");

  if (canManager.begin((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN)) {
    Serial.println("CAN(TWAI) initialized");
  } else {
    Serial.println("CAN(TWAI) initialization failed");
  }

  uiManager.begin();
}

void loop()
{
  static unsigned long lastPrintMs = 0;
  static unsigned long lastUiUpdateMs = 0;

  const uint32_t nowMs = millis();

  canManager.poll(nowMs);

  gnss.update();
  const GpsData &gps = gnss.getData();

  FusionInputs fusionInputs;
  fusionInputs.gnssValid = gps.pvtValid;
  fusionInputs.gnssSpeedKmh = gps.speedKmh;
  fusionInputs.canValid = canManager.hasDecodedSpeed();
  fusionInputs.canSpeedKmh = canManager.getDecodedSpeedKmh();
  fusionInputs.extValid = false;
  fusionInputs.extSpeedKmh = 0.0f;

  fusionManager.update(fusionInputs);
  distanceManager.update(nowMs, fusionManager.getState().selectedSpeedKmh);

  if ((nowMs - lastPrintMs) >= DEBUG_PRINT_INTERVAL_MS) {
    lastPrintMs = nowMs;
    printGpsSummary();
  }

  if ((nowMs - lastUiUpdateMs) >= UI_UPDATE_INTERVAL_MS) {
    lastUiUpdateMs = nowMs;
    uiManager.update(buildUiSnapshot(gps));
  }

  lv_timer_handler();
  handleUiActions(uiManager.consumeActions(), nowMs);
  delay(1);
}

#pragma GCC pop_options
