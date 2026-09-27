#include "status_led.h"

namespace dfmks {

void StatusLed::begin() {
  pixels_.begin();
  pixels_.setBrightness(kNeoPixelBrightness);
  pixels_.clear();
  pixels_.show();
  lastR_ = 0;
  lastG_ = 0;
  lastB_ = 0;
  bootMs_ = millis();
  state_ = State::kBoot;
  setColor(64, 0, 64);
}

void StatusLed::setColor(uint8_t r, uint8_t g, uint8_t b) {
  if (r == lastR_ && g == lastG_ && b == lastB_) {
    return;
  }
  lastR_ = r;
  lastG_ = g;
  lastB_ = b;
  pixels_.setPixelColor(0, pixels_.Color(r, g, b));
  pixels_.show();
}

void StatusLed::markActivity() {
  lastActivityMs_ = millis();
  state_ = State::kActivity;
}

void StatusLed::update(bool dfConnected, bool busReady, bool busFailed) {
  const uint32_t now = millis();
  if (busFailed) {
    state_ = State::kError;
  } else if (state_ == State::kActivity && now - lastActivityMs_ > 80) {
    if (!dfConnected) {
      state_ = State::kWaitingForDf;
    } else if (!busReady) {
      state_ = State::kBusWait;
    } else {
      state_ = State::kReady;
    }
  } else if (state_ != State::kActivity) {
    if (!dfConnected && now - bootMs_ < 600) {
      state_ = State::kBoot;
    } else if (!dfConnected) {
      state_ = State::kWaitingForDf;
    } else if (!busReady) {
      state_ = State::kBusWait;
    } else {
      state_ = State::kReady;
    }
  }

  const uint32_t window = (state_ == State::kError)          ? 120
                          : (state_ == State::kBoot)         ? 180
                          : (state_ == State::kWaitingForDf) ? 260
                          : (state_ == State::kActivity)     ? 40
                          : (state_ == State::kReady)        ? 500
                                                             : 300;
  if (now - lastPulseMs_ >= window) {
    lastPulseMs_ = now;
    pulseOn_ = !pulseOn_;
  }
  uint8_t r = 0;
  uint8_t g = 0;
  uint8_t b = 0;
  switch (state_) {
    case State::kBoot:
      r = pulseOn_ ? 64 : 0;
      b = 64;
      break;
    case State::kWaitingForDf:
      b = pulseOn_ ? 64 : 16;
      break;
    case State::kBusWait:
      g = pulseOn_ ? 48 : 16;
      b = pulseOn_ ? 64 : 24;
      break;
    case State::kReady:
      g = pulseOn_ ? 96 : 32;
      break;
    case State::kActivity:
      g = 96;
      b = 80;
      break;
    case State::kError:
      r = pulseOn_ ? 120 : 16;
      break;
  }
  setColor(r, g, b);
}

}  // namespace dfmks
