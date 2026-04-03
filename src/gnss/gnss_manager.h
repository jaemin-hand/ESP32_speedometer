#pragma once

#include <Arduino.h>

#include "../app/app_types.h"

class GnssManager {
public:
  void begin(HardwareSerial &serial, int rxPin, int txPin, uint32_t baudRate = 115200U);
  void update();

  const GpsData &getData() const;
  void printSbfDiagnostics(const char *label) const;

  static const char *pvtStatusToText(int pvtMode, int errorCode);

private:
  void resetDecoder();
  void processSbfByte(uint8_t byteValue);
  void handleSbfBlock(const uint8_t *block, uint16_t length);
  void parsePvtGeodeticBlock(const uint8_t *block, uint16_t length);
  void parseReceiverTimeBlock(const uint8_t *block, uint16_t length);
  void parseMeasEpochBlock(const uint8_t *block, uint16_t length);

  HardwareSerial *serial_ = nullptr;
  GpsData gps_;
  uint8_t sbfBlockBuffer_[4096] = {0};
  size_t sbfBlockSize_ = 0;
  size_t sbfExpectedLength_ = 0;
  uint32_t lastSbfByteMs_ = 0;
  uint32_t lastPvtUpdateMs_ = 0;
  uint32_t lastMeasEpochUpdateMs_ = 0;
  uint32_t lastReceiverTimeUpdateMs_ = 0;
  uint32_t sbfPvtBlockCount_ = 0;
  uint32_t sbfReceiverTimeBlockCount_ = 0;
  uint32_t sbfMeasEpochBlockCount_ = 0;
  uint32_t sbfUnknownBlockCount_ = 0;
  uint32_t sbfCrcRejectCount_ = 0;
  uint32_t sbfLengthRejectCount_ = 0;
  uint16_t lastSbfBlockNumber_ = 0;
  uint16_t lastSbfBlockLength_ = 0;
};
