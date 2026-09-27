#pragma once

// RP2040-Zero pin map for DF_DMC_2_MKS.
// USB CDC is binary DMC only — do not print debug text on Serial.

#include <Arduino.h>
#include <cstdint>

namespace dfmks {

constexpr int kMksAxes = 8;
constexpr int kServoAxes = 8;
constexpr int kAxisCount = kMksAxes + kServoAxes;
constexpr int kMaxUploadFrames = 1440;
constexpr int kDmxChannels = 512;

// 16384 encoder counts = 1 motor revolution. Arc steps-per-unit is per axis.
constexpr int32_t kEncoderCountsPerRev = 16384;
constexpr int kMksMaxRpm = 3000;
constexpr int kMksDefaultRpm = 120;
constexpr uint8_t kMksDefaultAcc = 200;
constexpr uint8_t kMksMicrostep = 16;
constexpr uint8_t kMksWorkMode = 0x05;  // SR_vFOC
constexpr uint32_t kMksHeartbeatMs = 500;
constexpr uint32_t kRs485Baud = 256000;
constexpr uint32_t kRs485ReplyMs = 25;
constexpr uint32_t kPositionReportMs = 100;

// 133 MHz / 6.625 / 65536 = 306.35 Hz. ±20000 around 30113 is about 0.5..2.5 ms.
constexpr float kServoClkDiv = 6.625f;
constexpr uint16_t kServoWrap = 65535;
constexpr int32_t kServoPwmZero = 30113;
constexpr int32_t kServoPwmMin = 10113;
constexpr int32_t kServoPwmMax = 50113;
constexpr int32_t kServoMinSteps = -20000;
constexpr int32_t kServoMaxSteps = 20000;
constexpr int32_t kServoDefaultStepsPerSec = 40000;

constexpr uint8_t kDmxTxPin = 0;
constexpr uint8_t kServoPins[kServoAxes] = {1, 2, 3, 4, 5, 6, 7, 8};
constexpr uint8_t kCameraPin = 9;
constexpr uint8_t kBuzzerPin = 10;
constexpr uint8_t kRs485DirPin = 11;
constexpr uint8_t kRs485TxPin = 12;
constexpr uint8_t kRs485RxPin = 13;
constexpr uint8_t kGioInPins[2] = {15, 14};       // GIO_IN0, GIO_IN1
constexpr uint8_t kNeoPixelPin = 16;              // internal WS2812
constexpr uint8_t kNeoPixelBrightness = 32;
constexpr uint8_t kGioOutPins[4] = {29, 28, 27, 26};  // GIO_OUT0..3
constexpr uint32_t kDmxPwmHz = 18000;

constexpr uint8_t kHelloVersionMajor = 1;
constexpr uint8_t kHelloVersionMinor = 0;
constexpr uint8_t kHelloVersionRev = 0;
constexpr uint16_t kHelloProtocolVersion = 2;

}  // namespace dfmks
