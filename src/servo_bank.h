#pragma once

// Eight hobby servos on GP1..GP8. No DMX levels are written here.

#include "config.h"

#include <dmc_protocol.h>

#include <cstdint>

namespace dfmks {

class ServoBank {
 public:
  void begin();
  int motorCount() const { return kServoAxes; }

  void moveToSteps(int axis0, int32_t steps);
  void slewTo(int axis0, int32_t steps, uint16_t speed);
  void startBlur(int axis0, int32_t endSteps, uint32_t delayMs, uint32_t accelMs, uint32_t cruiseMs);
  bool blurEnabled(int axis0) const;
  void setMaxSpeed(int axis0, int32_t stepsPerSec);
  void stopAxis(int axis0);
  void stopMotion();
  void configure(int axis0, uint8_t flags);
  void setLimits(int axis0, bool lowerEn, int32_t lower, bool upperEn, int32_t upper);
  int32_t positionSteps(int axis0) const;
  uint32_t movingMask() const { return movingMask_; }
  bool moving() const { return movingMask_ != 0; }
  void update();

 private:
  int32_t clampSteps(int axis0, int32_t steps) const;
  void writeAxis(int axis0);
  void slewUpdate();
  int32_t slewRate(int axis0) const;

  int32_t steps_[kServoAxes]{};
  bool enabled_[kServoAxes]{};
  uint8_t config_[kServoAxes]{};
  bool blurOn_[kServoAxes]{};
  int32_t blurStart_[kServoAxes]{};
  int32_t blurEnd_[kServoAxes]{};
  float blurV_[kServoAxes]{};
  uint32_t blurDelayMs_[kServoAxes]{};
  uint32_t blurAccelMs_[kServoAxes]{};
  uint32_t blurCruiseMs_[kServoAxes]{};
  uint32_t blurT0_ = 0;
  bool blurClock_ = false;
  bool lowerEn_[kServoAxes]{};
  bool upperEn_[kServoAxes]{};
  int32_t lower_[kServoAxes]{};
  int32_t upper_[kServoAxes]{};
  int32_t maxStepsPerSec_[kServoAxes]{};
  int32_t slewTarget_[kServoAxes]{};
  uint16_t slewSpeed_[kServoAxes]{};
  bool slewOn_[kServoAxes]{};
  uint32_t slewLastMs_ = 0;
  uint32_t movingMask_ = 0;
};

}  // namespace dfmks
