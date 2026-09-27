#pragma once

// Half-duplex RS485 master for eight MKS SERVO42/57D drives.
// Position unit on this side is DMC steps: 1 step = 1 encoder count.

#include "config.h"

#include <cstdint>

namespace dfmks {

class MksBus {
 public:
  void begin();
  void update();
  void setHostAlive(bool alive);
  void setPositionPoll(bool enabled);

  bool idle() const;
  bool initDone() const { return initLeft_ == 0; }
  bool busReady() const { return initDone() && onlineCount_ > 0; }
  bool busFailed() const { return initDone() && onlineCount_ == 0; }
  int onlineCount() const { return onlineCount_; }

  bool moveAxis(int axis0, int32_t steps);
  bool moveAxisRpm(int axis0, int32_t steps, int rpm, uint8_t acc);
  bool jogAxis(int axis0, int32_t steps, uint16_t speedWord);
  bool moveBlur(int axis0, int32_t endSteps, uint32_t accelMs, uint32_t cruiseMs);
  void finishGroup();
  void stopAxis(int axis0);
  void stopAll();
  void hardStop();
  void resetAxis(int axis0, int32_t steps);
  void setSpeed(int axis0, int32_t stepsPerSec, int32_t stepsPerSec2);
  void setLimits(int axis0, bool lowerEn, int32_t lower, bool upperEn, int32_t upper);
  void configure(int axis0, uint8_t flags);

  int32_t positionSteps(int axis0) const;
  bool blurEnabled(int axis0) const;
  uint32_t movingMask() const { return movingMask_; }
  bool moving() const { return movingMask_ != 0; }
  bool axisMoving(int axis0) const;

 private:
  enum class Kind : uint8_t { kInit, kRead, kMove, kSync, kStop, kZero, kEnable, kHard };

  struct Item {
    uint8_t raw[12]{};
    uint8_t len = 0;
    bool wantReply = false;
    Kind kind = Kind::kInit;
    uint8_t axis = 0;
  };

  bool queue(const Item& item);
  void pushBytes(Kind kind, uint8_t axis, const uint8_t* body, uint8_t bodyLen, bool wantReply);
  void startTx();
  void pollRx();
  void feedRx(uint8_t byte);
  void handleFrame();
  void finishItem(bool ok);
  void noteOnline(uint8_t addr, bool ok);
  void onEncoder(int axis0, int64_t encoder);
  int32_t clampSteps(int axis0, int32_t steps) const;
  bool queueMove(int axis0, int32_t steps, int rpm, uint8_t acc, bool blurStop);
  static int replyLen(uint8_t code);
  static uint8_t checksum(const uint8_t* data, int n);
  static int rpmFromStepsPerSec(int32_t stepsPerSec);
  static uint8_t accFromStepsPerSec2(int32_t stepsPerSec2);

  Item queue_[64]{};
  uint8_t qHead_ = 0;
  uint8_t qTail_ = 0;
  uint8_t qCount_ = 0;

  enum class Phase : uint8_t { kIdle, kWaitRx, kGap };
  Phase phase_ = Phase::kIdle;
  uint32_t deadlineMs_ = 0;
  Item inflight_{};
  bool haveInflight_ = false;

  uint8_t rx_[12]{};
  uint8_t rxLen_ = 0;
  uint32_t rxByteMs_ = 0;

  int64_t encoder_[kMksAxes]{};
  int64_t offset_[kMksAxes]{};
  int32_t targetDrive_[kMksAxes]{};
  bool haveEncoder_[kMksAxes]{};
  bool resetPending_[kMksAxes]{};
  int32_t resetTo_[kMksAxes]{};
  bool enabled_[kMksAxes]{};
  bool online_[kMksAxes]{};
  uint8_t config_[kMksAxes]{};
  int rpm_[kMksAxes]{};
  uint8_t acc_[kMksAxes]{};
  bool lowerEn_[kMksAxes]{};
  bool upperEn_[kMksAxes]{};
  int32_t lower_[kMksAxes]{};
  int32_t upper_[kMksAxes]{};
  uint8_t settle_[kMksAxes]{};
  uint32_t movingMask_ = 0;
  int onlineCount_ = 0;
  int initLeft_ = 0;
  uint8_t pollAxis_ = 0;
  uint32_t lastPollMs_ = 0;
  bool hostAlive_ = true;
  bool positionPoll_ = true;
  bool hostStopSent_ = false;
  bool groupDirty_ = false;
};

}  // namespace dfmks
