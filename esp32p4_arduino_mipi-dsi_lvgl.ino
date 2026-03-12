#pragma GCC push_options
#pragma GCC optimize("O3")

#include <Arduino.h>
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
#define GPS_TX_PIN 47   // 지금은 연결 X

HardwareSerial GPSSerial(1);

jd9165_lcd lcd = jd9165_lcd(LCD_RST);
gt911_touch touch = gt911_touch(TP_I2C_SDA, TP_I2C_SCL, TP_RST, TP_INT);

static lv_disp_draw_buf_t draw_buf;
static lv_color_t *buf;
static lv_color_t *buf1;

// -------------------- GPS 데이터 구조체 --------------------
struct GpsData {
  bool rmcValid = false;
  int fixQuality = 0;
  int satellites = 0;
  float altitudeM = 0.0f;
  float speedKnots = 0.0f;
  float speedKmh = 0.0f;
  double latitude = 0.0;
  double longitude = 0.0;
  char dateStr[11] = {0};   // YYYY-MM-DD
  char timeStr[9]  = {0};   // HH:MM:SS
};

GpsData gps;

// -------------------- LVGL UI 핸들 --------------------
static lv_obj_t *labelSpeed = nullptr;
static lv_obj_t *labelSats  = nullptr;
static lv_obj_t *labelFix   = nullptr;
static lv_obj_t *labelTime  = nullptr;

// -------------------- LCD Flush --------------------
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
  const int offsetx1 = area->x1;
  const int offsetx2 = area->x2;
  const int offsety1 = area->y1;
  const int offsety2 = area->y2;
  lcd.lcd_draw_bitmap(offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, &color_p->full);
  lv_disp_flush_ready(disp);
}

// -------------------- Touch Read --------------------
void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{
  bool touched;
  uint16_t touchX, touchY;

  touched = touch.getTouch(&touchX, &touchY);

  if (!touched) {
    data->state = LV_INDEV_STATE_REL;
  } else {
    data->state = LV_INDEV_STATE_PR;
    data->point.x = touchX;
    data->point.y = touchY;
  }
}

// -------------------- CAN 버튼 콜백 --------------------
static void btn_send_can_event_cb(lv_event_t * e) {
  lv_event_code_t code = lv_event_get_code(e);

  if (code == LV_EVENT_CLICKED) {
    Serial.println("디스플레이 버튼 터치 감지됨 CAN 데이터 전송 시도...");

    twai_message_t tx_message = {};
    tx_message.identifier = 0x777;
    tx_message.extd = 0;
    tx_message.data_length_code = 4;

    tx_message.data[0] = 0x01;
    tx_message.data[1] = 0x02;
    tx_message.data[2] = 0x03;
    tx_message.data[3] = 0x04;

    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK) {
      Serial.println("-> CAN 전송 성공 (ID: 0x777)");
    } else {
      Serial.println("-> CAN 전송 실패");
    }
  }
}

// -------------------- GPS 유틸 --------------------
double nmeaToDecimalDegrees(const char* raw, char hemisphere) {
  if (raw == nullptr || strlen(raw) == 0) return 0.0;

  double val = atof(raw);
  int degrees = (int)(val / 100);
  double minutes = val - (degrees * 100);
  double decimal = degrees + (minutes / 60.0);

  if (hemisphere == 'S' || hemisphere == 'W') {
    decimal = -decimal;
  }
  return decimal;
}

void parseNmeaTime(const char* raw, char* outBuf, size_t outSize) {
  if (raw == nullptr || strlen(raw) < 6) {
    outBuf[0] = '\0';
    return;
  }

  // hhmmss.sss -> HH:MM:SS
  snprintf(outBuf, outSize, "%.2s:%.2s:%.2s", raw, raw + 2, raw + 4);
}

void parseNmeaDate(const char* raw, char* outBuf, size_t outSize) {
  if (raw == nullptr || strlen(raw) != 6) {
    outBuf[0] = '\0';
    return;
  }

  // ddmmyy -> 20yy-mm-dd
  snprintf(outBuf, outSize, "20%.2s-%.2s-%.2s", raw + 4, raw + 2, raw);
}

const char* fixQualityToText(int fixQuality) {
  switch (fixQuality) {
    case 0: return "NO FIX";
    case 1: return "GPS";
    case 2: return "DGPS";
    case 4: return "RTK FIX";
    case 5: return "RTK FLOAT";
    default: return "OTHER";
  }
}

// -------------------- GPS 파싱 --------------------
void parseGGA(char* line) {
  char* fields[20] = {0};
  int idx = 0;

  char* token = strtok(line, ",");
  while (token != nullptr && idx < 20) {
    fields[idx++] = token;
    token = strtok(nullptr, ",");
  }

  if (idx < 10) return;

  // $GPGGA,time,lat,N/S,lon,E/W,fix,sats,hdop,alt,...

  if (fields[1] && strlen(fields[1]) > 0) {
    parseNmeaTime(fields[1], gps.timeStr, sizeof(gps.timeStr));
  }

  if (fields[2] && fields[3] && strlen(fields[2]) > 0 && strlen(fields[3]) > 0) {
    gps.latitude = nmeaToDecimalDegrees(fields[2], fields[3][0]);
  }

  if (fields[4] && fields[5] && strlen(fields[4]) > 0 && strlen(fields[5]) > 0) {
    gps.longitude = nmeaToDecimalDegrees(fields[4], fields[5][0]);
  }

  if (fields[6] && strlen(fields[6]) > 0) {
    gps.fixQuality = atoi(fields[6]);
  }

  if (fields[7] && strlen(fields[7]) > 0) {
    gps.satellites = atoi(fields[7]);
  }

  if (fields[9] && strlen(fields[9]) > 0) {
    gps.altitudeM = atof(fields[9]);
  }
}

void parseRMC(char* line) {
  char* fields[20] = {0};
  int idx = 0;

  char* token = strtok(line, ",");
  while (token != nullptr && idx < 20) {
    fields[idx++] = token;
    token = strtok(nullptr, ",");
  }

  if (idx < 10) return;

  // $GPRMC,time,status,lat,N/S,lon,E/W,speed,course,date,...

  if (fields[1] && strlen(fields[1]) > 0) {
    parseNmeaTime(fields[1], gps.timeStr, sizeof(gps.timeStr));
  }

  if (fields[2] && strlen(fields[2]) > 0) {
    gps.rmcValid = (fields[2][0] == 'A');
  }

  if (fields[3] && fields[4] && strlen(fields[3]) > 0 && strlen(fields[4]) > 0) {
    gps.latitude = nmeaToDecimalDegrees(fields[3], fields[4][0]);
  }

  if (fields[5] && fields[6] && strlen(fields[5]) > 0 && strlen(fields[6]) > 0) {
    gps.longitude = nmeaToDecimalDegrees(fields[5], fields[6][0]);
  }

  if (fields[7] && strlen(fields[7]) > 0) {
    gps.speedKnots = atof(fields[7]);
    gps.speedKmh = gps.speedKnots * 1.852f;
  }

  if (fields[9] && strlen(fields[9]) > 0) {
    parseNmeaDate(fields[9], gps.dateStr, sizeof(gps.dateStr));
  }
}

void handleNmeaLine(const String& lineIn) {
  if (lineIn.length() == 0) return;

  char line[256];
  lineIn.toCharArray(line, sizeof(line));

  if (strncmp(line, "$GPGGA", 6) == 0 || strncmp(line, "$GNGGA", 6) == 0) {
    parseGGA(line);
  } 
  else if (strncmp(line, "$GPRMC", 6) == 0 || strncmp(line, "$GNRMC", 6) == 0) {
    parseRMC(line);
  }
}

void printGpsSummary() {
  Serial.println("========== GPS SUMMARY ==========");
  Serial.printf("Valid      : %s\n", gps.rmcValid ? "YES" : "NO");
  Serial.printf("Fix        : %d (%s)\n", gps.fixQuality, fixQualityToText(gps.fixQuality));
  Serial.printf("Sats       : %d\n", gps.satellites);
  Serial.printf("Speed      : %.2f knots / %.2f km/h\n", gps.speedKnots, gps.speedKmh);
  Serial.print("Lat        : "); Serial.println(gps.latitude, 8);
  Serial.print("Lon        : "); Serial.println(gps.longitude, 8);
  Serial.printf("Alt        : %.2f m\n", gps.altitudeM);
  Serial.printf("Date       : %s\n", gps.dateStr);
  Serial.printf("Time(UTC)  : %s\n", gps.timeStr);
  Serial.println("=================================");
}

// -------------------- UI 생성 / 업데이트 --------------------
void createMainUi() {
  // 제목
  lv_obj_t *title = lv_label_create(lv_scr_act());
  lv_label_set_text(title, "GPS Monitor");
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 20);

  labelSpeed = lv_label_create(lv_scr_act());
  lv_label_set_text(labelSpeed, "Speed : --.-- km/h");
  lv_obj_align(labelSpeed, LV_ALIGN_TOP_LEFT, 20, 70);

  labelSats = lv_label_create(lv_scr_act());
  lv_label_set_text(labelSats, "Sats  : --");
  lv_obj_align(labelSats, LV_ALIGN_TOP_LEFT, 20, 110);

  labelFix = lv_label_create(lv_scr_act());
  lv_label_set_text(labelFix, "Fix   : --");
  lv_obj_align(labelFix, LV_ALIGN_TOP_LEFT, 20, 150);

  labelTime = lv_label_create(lv_scr_act());
  lv_label_set_text(labelTime, "Time  : --:--:--");
  lv_obj_align(labelTime, LV_ALIGN_TOP_LEFT, 20, 190);

  // CAN 테스트 버튼
  lv_obj_t *btn = lv_btn_create(lv_scr_act());
  lv_obj_set_size(btn, 220, 70);
  lv_obj_align(btn, LV_ALIGN_BOTTOM_MID, 0, -30);
  lv_obj_add_event_cb(btn, btn_send_can_event_cb, LV_EVENT_ALL, NULL);

  lv_obj_t *btnLabel = lv_label_create(btn);
  lv_label_set_text(btnLabel, "Send CAN Data");
  lv_obj_center(btnLabel);
}

void updateGpsLabels() {
  char bufText[64];

  if (labelSpeed) {
    snprintf(bufText, sizeof(bufText), "Speed : %.2f km/h", gps.speedKmh);
    lv_label_set_text(labelSpeed, bufText);
  }

  if (labelSats) {
    snprintf(bufText, sizeof(bufText), "Sats  : %d", gps.satellites);
    lv_label_set_text(labelSats, bufText);
  }

  if (labelFix) {
    snprintf(bufText, sizeof(bufText), "Fix   : %d (%s)", gps.fixQuality, fixQualityToText(gps.fixQuality));
    lv_label_set_text(labelFix, bufText);
  }

  if (labelTime) {
    snprintf(bufText, sizeof(bufText), "Time  : %s UTC", strlen(gps.timeStr) ? gps.timeStr : "--:--:--");
    lv_label_set_text(labelTime, bufText);
  }
}

// -------------------- Setup --------------------
void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESP32P4 MIPI DSI LVGL + GPS");

  lcd.begin();
  touch.begin();

  lv_init();

  size_t buffer_size = sizeof(int32_t) * LCD_H_RES * LCD_V_RES;
  buf  = (lv_color_t *)heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM);
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
  disp_drv.full_refresh = true;
  lv_disp_drv_register(&disp_drv);

  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register(&indev_drv);

  // GPS UART
  GPSSerial.begin(115200, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("GPS Serial(115200) 개통 완료");

  // CAN init
  twai_general_config_t g_config =
      TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();

  twai_filter_config_t f_config;
  f_config.acceptance_code = (0x100 << 21);
  f_config.acceptance_mask = ~(0x7FF << 21);
  f_config.single_filter = true;

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    twai_start();
    Serial.println("CAN(TWAI) 드라이버 초기화 및 시작 완료!");
  } else {
    Serial.println("CAN 드라이버 초기화 실패!");
  }

  createMainUi();
}

// -------------------- Loop --------------------
void loop()
{
  static String gpsLine = "";
  static unsigned long lastPrintMs = 0;
  static unsigned long lastUiUpdateMs = 0;

  // CAN 수신 확인
  twai_message_t rx_message;
  if (twai_receive(&rx_message, pdMS_TO_TICKS(0)) == ESP_OK) {
    Serial.printf("CAN 수신 완료! ID: 0x%X, 길이: %d\n", rx_message.identifier, rx_message.data_length_code);
    Serial.print("데이터: ");
    for (int i = 0; i < rx_message.data_length_code; i++) {
      Serial.printf("%02X ", rx_message.data[i]);
    }
    Serial.println();
  }

  // GPS 라인 수신
  while (GPSSerial.available()) {
    char c = GPSSerial.read();

    if (c == '\n') {
      if (gpsLine.length() > 0) {
        handleNmeaLine(gpsLine);
        gpsLine = "";
      }
    } else if (c != '\r') {
      gpsLine += c;

      if (gpsLine.length() > 200) {
        gpsLine = "";
      }
    }
  }

  // 1초마다 시리얼 요약 출력
  if (millis() - lastPrintMs >= 1000) {
    lastPrintMs = millis();
    printGpsSummary();
  }

  // 200ms마다 LCD 갱신
  if (millis() - lastUiUpdateMs >= 200) {
    lastUiUpdateMs = millis();
    updateGpsLabels();
  }

  lv_timer_handler();
  delay(5);
}

#pragma GCC pop_options