#pragma GCC push_options
#pragma GCC optimize("O3")

#include <Arduino.h>
#include <stdlib.h>
#include <string.h>
#include "driver/twai.h"

#define CAN_RX_PIN 2
#define CAN_TX_PIN 48

static unsigned long lastStatusMs = 0;

static void printCanStatus(const char *prefix) {
  if (prefix != nullptr && prefix[0] != '\0') {
    Serial.println(prefix);
  }

  twai_status_info_t status = {};
  if (twai_get_status_info(&status) != ESP_OK) {
    Serial.println("TWAI status read failed");
    return;
  }

  Serial.printf(
      "TWAI state=%d tx_err=%lu rx_err=%lu tx_to_send=%lu rx_to_recv=%lu tx_failed=%lu rx_missed=%lu rx_overrun=%lu bus_err=%lu arb_lost=%lu\n",
      static_cast<int>(status.state),
      static_cast<unsigned long>(status.tx_error_counter),
      static_cast<unsigned long>(status.rx_error_counter),
      static_cast<unsigned long>(status.msgs_to_tx),
      static_cast<unsigned long>(status.msgs_to_rx),
      static_cast<unsigned long>(status.tx_failed_count),
      static_cast<unsigned long>(status.rx_missed_count),
      static_cast<unsigned long>(status.rx_overrun_count),
      static_cast<unsigned long>(status.bus_error_count),
      static_cast<unsigned long>(status.arb_lost_count));
}

static bool sendTestFrame() {
  twai_message_t txMessage = {};
  txMessage.identifier = 0x777;
  txMessage.extd = 0;
  txMessage.data_length_code = 4;
  txMessage.data[0] = 0x01;
  txMessage.data[1] = 0x02;
  txMessage.data[2] = 0x03;
  txMessage.data[3] = 0x04;

  return twai_transmit(&txMessage, pdMS_TO_TICKS(100)) == ESP_OK;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("LEGACY CAN TEST 57a26fbd");

  twai_general_config_t gConfig =
      TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t tConfig = TWAI_TIMING_CONFIG_500KBITS();

  twai_filter_config_t fConfig = {};
  fConfig.acceptance_code = (0x100 << 21);
  fConfig.acceptance_mask = ~(0x7FF << 21);
  fConfig.single_filter = true;

  Serial.println("TWAI mode: NORMAL");
  Serial.println("TWAI timing: default 500 kbps");
  Serial.printf(
      "TWAI filter: code=0x%08lX mask=0x%08lX single=%d\n",
      static_cast<unsigned long>(fConfig.acceptance_code),
      static_cast<unsigned long>(fConfig.acceptance_mask),
      static_cast<int>(fConfig.single_filter));

  if (twai_driver_install(&gConfig, &tConfig, &fConfig) == ESP_OK) {
    if (twai_start() == ESP_OK) {
      Serial.println("CAN(TWAI) initialized");
    } else {
      Serial.println("CAN(TWAI) start failed");
    }
  } else {
    Serial.println("CAN(TWAI) initialization failed");
  }

  Serial.println("Commands:");
  Serial.println("  s : send 0x777 test frame");
  Serial.println("  a : print TWAI status");
}

void loop() {
  twai_message_t rxMessage = {};
  while (twai_receive(&rxMessage, pdMS_TO_TICKS(0)) == ESP_OK) {
    Serial.printf("CAN RX: 0x%03X [%d] ", rxMessage.identifier, rxMessage.data_length_code);
    for (int i = 0; i < rxMessage.data_length_code; ++i) {
      Serial.printf("%02X ", rxMessage.data[i]);
    }
    Serial.println();
  }

  if (Serial.available() > 0) {
    const char cmd = static_cast<char>(Serial.read());
    if (cmd == 's' || cmd == 'S') {
      if (sendTestFrame()) {
        Serial.println("CAN TX: 0x777 [4] 01 02 03 04");
      } else {
        Serial.println("CAN TX failed");
        printCanStatus("CAN diagnostics");
      }
    } else if (cmd == 'a' || cmd == 'A') {
      printCanStatus("CAN status");
    }
  }

  const unsigned long nowMs = millis();
  if ((nowMs - lastStatusMs) >= 1000) {
    lastStatusMs = nowMs;
    printCanStatus("CAN status");
  }

  delay(1);
}

#pragma GCC pop_options
