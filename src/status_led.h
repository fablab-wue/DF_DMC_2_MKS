#pragma once

// Internal status LED. The RP2040-Zero LED is a WS2812 on GP16.

#include "config.h"

#include <Adafruit_NeoPixel.h>
#include <cstdint>

namespace dfmks {

class StatusLed {
 public:
  enum class State {
    kBoot,
    kWaitingForDf,
    kBusWait,
    kReady,
    kActivity,
    kError,
  };

  void begin();
  void markActivity();
  void update(bool dfConnected, bool busReady, bool busFailed);

 private:
  void setColor(uint8_t r, uint8_t g, uint8_t b);

  Adafruit_NeoPixel pixels_{1, kNeoPixelPin, NEO_GRB + NEO_KHZ800};
  uint8_t lastR_ = 255;
  uint8_t lastG_ = 255;
  uint8_t lastB_ = 255;
  uint32_t lastPulseMs_ = 0;
  uint32_t bootMs_ = 0;
  uint32_t lastActivityMs_ = 0;
  bool pulseOn_ = false;
  State state_ = State::kBoot;
};

}  // namespace dfmks
