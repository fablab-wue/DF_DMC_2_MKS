#include "mks_bus.h"

#include <dmc_protocol.h>

#include <cmath>

namespace dfmks {

namespace {

constexpr uint8_t kQueueCap = 64;

int64_t signExtend48(uint64_t raw) {
  raw &= 0x0000FFFFFFFFFFFFULL;
  if ((raw & (1ULL << 47)) != 0) {
    raw |= 0xFFFF000000000000ULL;
  }
  return static_cast<int64_t>(raw);
}

int32_t clampI32(int64_t value) {
  if (value > 2147483647LL) {
    return 2147483647L;
  }
  if (value < -2147483647LL - 1) {
    return static_cast<int32_t>(-2147483647L - 1);
  }
  return static_cast<int32_t>(value);
}

}  // namespace

uint8_t MksBus::checksum(const uint8_t* data, int n) {
  uint8_t sum = 0;
  for (int i = 0; i < n; ++i) {
    sum = static_cast<uint8_t>(sum + data[i]);
  }
  return sum;
}

int MksBus::replyLen(uint8_t code) {
  if (code == 0x30 || code == 0x31) {
    return 10;
  }
  if (code == 0x32) {
    return 6;
  }
  return 5;
}

int MksBus::rpmFromStepsPerSec(int32_t stepsPerSec) {
  if (stepsPerSec < 0) {
    stepsPerSec = -stepsPerSec;
  }
  const int rpm = static_cast<int>((static_cast<int64_t>(stepsPerSec) * 60 + kEncoderCountsPerRev / 2) / kEncoderCountsPerRev);
  if (rpm < 1) {
    return 1;
  }
  if (rpm > kMksMaxRpm) {
    return kMksMaxRpm;
  }
  return rpm;
}

uint8_t MksBus::accFromStepsPerSec2(int32_t stepsPerSec2) {
  if (stepsPerSec2 < 0) {
    stepsPerSec2 = -stepsPerSec2;
  }
  if (stepsPerSec2 < 1) {
    return kMksDefaultAcc;
  }
  const double rpmPerSec = (static_cast<double>(stepsPerSec2) * 60.0) / static_cast<double>(kEncoderCountsPerRev);
  if (rpmPerSec < 1.0) {
    return 1;
  }
  int k = static_cast<int>(lround(20000.0 / rpmPerSec));
  if (k < 1) {
    k = 1;
  }
  if (k > 255) {
    k = 255;
  }
  return static_cast<uint8_t>(256 - k);
}

void MksBus::begin() {
  for (int i = 0; i < kMksAxes; ++i) {
    rpm_[i] = kMksDefaultRpm;
    acc_[i] = kMksDefaultAcc;
    enabled_[i] = false;
    online_[i] = false;
  }
  pinMode(kRs485DirPin, OUTPUT);
  digitalWrite(kRs485DirPin, LOW);
  Serial1.setTX(kRs485TxPin);
  Serial1.setRX(kRs485RxPin);
  Serial1.begin(kRs485Baud);
  initLeft_ = kMksAxes * 5;
  for (uint8_t axis = 0; axis < kMksAxes; ++axis) {
    const uint8_t addr = static_cast<uint8_t>(axis + 1);
    const uint8_t modeBody[] = {0xFA, addr, 0x82, kMksWorkMode};
    pushBytes(Kind::kInit, axis, modeBody, 4, true);
    const uint8_t stepBody[] = {0xFA, addr, 0x84, kMksMicrostep};
    pushBytes(Kind::kInit, axis, stepBody, 4, true);
    const uint8_t syncBody[] = {0xFA, addr, 0x4A, 0x01};
    pushBytes(Kind::kInit, axis, syncBody, 4, true);
    const uint32_t hb = kMksHeartbeatMs;
    const uint8_t hbBody[] = {0xFA, addr, 0x98, static_cast<uint8_t>(hb >> 24), static_cast<uint8_t>(hb >> 16),
                              static_cast<uint8_t>(hb >> 8), static_cast<uint8_t>(hb)};
    pushBytes(Kind::kInit, axis, hbBody, 7, true);
    const uint8_t readBody[] = {0xFA, addr, 0x31};
    pushBytes(Kind::kRead, axis, readBody, 3, true);
  }
}

void MksBus::setHostAlive(bool alive) {
  if (alive) {
    hostAlive_ = true;
    hostStopSent_ = false;
    return;
  }
  hostAlive_ = false;
  if (!hostStopSent_) {
    hardStop();
    hostStopSent_ = true;
  }
}

void MksBus::setPositionPoll(bool enabled) { positionPoll_ = enabled; }

bool MksBus::idle() const { return phase_ == Phase::kIdle && qCount_ == 0 && !haveInflight_; }

bool MksBus::queue(const Item& item) {
  if (qCount_ >= kQueueCap) {
    return false;
  }
  queue_[qTail_] = item;
  qTail_ = static_cast<uint8_t>((qTail_ + 1) % kQueueCap);
  ++qCount_;
  return true;
}

void MksBus::pushBytes(Kind kind, uint8_t axis, const uint8_t* body, uint8_t bodyLen, bool wantReply) {
  Item item;
  item.kind = kind;
  item.axis = axis;
  item.wantReply = wantReply;
  item.len = static_cast<uint8_t>(bodyLen + 1);
  for (uint8_t i = 0; i < bodyLen && i < 11; ++i) {
    item.raw[i] = body[i];
  }
  item.raw[bodyLen] = checksum(item.raw, bodyLen);
  if (kind == Kind::kInit) {
    // counted in begin()
  }
  queue(item);
}

void MksBus::startTx() {
  if (phase_ != Phase::kIdle || qCount_ == 0) {
    return;
  }
  inflight_ = queue_[qHead_];
  qHead_ = static_cast<uint8_t>((qHead_ + 1) % kQueueCap);
  --qCount_;
  haveInflight_ = true;
  while (Serial1.available()) {
    Serial1.read();
  }
  rxLen_ = 0;
  digitalWrite(kRs485DirPin, HIGH);
  delayMicroseconds(10);
  Serial1.write(inflight_.raw, inflight_.len);
  Serial1.flush();
  delayMicroseconds(50);
  digitalWrite(kRs485DirPin, LOW);
  if (inflight_.wantReply) {
    phase_ = Phase::kWaitRx;
    deadlineMs_ = millis() + kRs485ReplyMs;
  } else if (inflight_.kind == Kind::kSync) {
    phase_ = Phase::kGap;
    deadlineMs_ = millis() + 1;
    haveInflight_ = false;
  } else {
    haveInflight_ = false;
    phase_ = Phase::kIdle;
  }
}

void MksBus::finishItem(bool ok) {
  if (!haveInflight_) {
    phase_ = Phase::kIdle;
    return;
  }
  const Item done = inflight_;
  haveInflight_ = false;
  phase_ = Phase::kIdle;
  if (initLeft_ > 0 && (done.kind == Kind::kInit || done.kind == Kind::kRead)) {
    --initLeft_;
  }
  if (done.kind == Kind::kInit) {
    noteOnline(static_cast<uint8_t>(done.axis + 1), ok);
  } else if (!ok && done.wantReply && done.axis < kMksAxes) {
    if (done.kind == Kind::kMove || done.kind == Kind::kStop) {
      movingMask_ &= ~(1u << done.axis);
    }
  }
}

void MksBus::noteOnline(uint8_t addr, bool ok) {
  if (addr < 1 || addr > kMksAxes) {
    return;
  }
  const int axis = addr - 1;
  if (ok && !online_[axis]) {
    online_[axis] = true;
    ++onlineCount_;
  }
}

void MksBus::onEncoder(int axis0, int64_t encoder) {
  if (axis0 < 0 || axis0 >= kMksAxes) {
    return;
  }
  encoder_[axis0] = encoder;
  if (!haveEncoder_[axis0]) {
    haveEncoder_[axis0] = true;
    if (resetPending_[axis0]) {
      offset_[axis0] = encoder - static_cast<int64_t>(resetTo_[axis0]);
      resetPending_[axis0] = false;
    }
  }
  if ((movingMask_ & (1u << axis0)) != 0) {
    const int64_t err = encoder - static_cast<int64_t>(targetDrive_[axis0]);
    if (err > -64 && err < 64) {
      if (settle_[axis0] < 3) {
        ++settle_[axis0];
      }
      if (settle_[axis0] >= 2) {
        movingMask_ &= ~(1u << axis0);
      }
    } else {
      settle_[axis0] = 0;
    }
  }
}

void MksBus::handleFrame() {
  const int n = replyLen(rx_[2]);
  if (rxLen_ < n) {
    return;
  }
  if (checksum(rx_, n - 1) != rx_[n - 1]) {
    return;
  }
  const uint8_t addr = rx_[1];
  const uint8_t code = rx_[2];
  noteOnline(addr, true);
  const int axis = static_cast<int>(addr) - 1;
  if (code == 0x31 && n == 10 && axis >= 0 && axis < kMksAxes) {
    uint64_t raw = 0;
    for (int i = 0; i < 6; ++i) {
      raw = (raw << 8) | rx_[3 + i];
    }
    onEncoder(axis, signExtend48(raw));
  } else if (n == 5 && axis >= 0 && axis < kMksAxes) {
    const uint8_t status = rx_[3];
    if (code == 0xF5 || code == 0xF6 || code == 0xFD || code == 0xFE || code == 0xF4) {
      if (status == 0 || status == 2 || status == 3) {
        movingMask_ &= ~(1u << axis);
      }
    } else if (code == 0xF7) {
      movingMask_ &= ~(1u << axis);
    } else if (code == 0x92 && status == 1) {
      encoder_[axis] = 0;
      offset_[axis] = 0;
      haveEncoder_[axis] = true;
      resetPending_[axis] = false;
      movingMask_ &= ~(1u << axis);
    }
  }
  if (haveInflight_ && inflight_.wantReply && addr == inflight_.raw[1] && code == inflight_.raw[2]) {
    finishItem(true);
  }
}

void MksBus::feedRx(uint8_t byte) {
  const uint32_t now = millis();
  if (rxLen_ > 0 && static_cast<uint32_t>(now - rxByteMs_) > 5) {
    rxLen_ = 0;
  }
  rxByteMs_ = now;
  if (rxLen_ == 0) {
    if (byte != 0xFB) {
      return;
    }
  }
  if (rxLen_ < sizeof(rx_)) {
    rx_[rxLen_++] = byte;
  } else {
    rxLen_ = 0;
    return;
  }
  if (rxLen_ >= 3 && rxLen_ >= replyLen(rx_[2])) {
    handleFrame();
    rxLen_ = 0;
  }
}

void MksBus::pollRx() {
  while (Serial1.available()) {
    feedRx(static_cast<uint8_t>(Serial1.read()));
  }
}

void MksBus::update() {
  pollRx();
  const uint32_t now = millis();
  if (phase_ == Phase::kWaitRx && static_cast<int32_t>(now - deadlineMs_) >= 0) {
    finishItem(false);
  }
  if (phase_ == Phase::kGap && static_cast<int32_t>(now - deadlineMs_) >= 0) {
    phase_ = Phase::kIdle;
  }
  if (phase_ == Phase::kIdle && qCount_ > 0) {
    startTx();
    return;
  }
  if (!idle() || !hostAlive_ || !initDone() || !positionPoll_) {
    return;
  }
  if (static_cast<uint32_t>(now - lastPollMs_) < 12) {
    return;
  }
  lastPollMs_ = now;
  const uint8_t axis = pollAxis_;
  pollAxis_ = static_cast<uint8_t>((pollAxis_ + 1) % kMksAxes);
  if (!online_[axis]) {
    return;
  }
  const uint8_t addr = static_cast<uint8_t>(axis + 1);
  const uint8_t body[] = {0xFA, addr, 0x31};
  pushBytes(Kind::kRead, axis, body, 3, true);
}

int32_t MksBus::clampSteps(int axis0, int32_t steps) const {
  if (axis0 < 0 || axis0 >= kMksAxes) {
    return steps;
  }
  if (lowerEn_[axis0] && steps < lower_[axis0]) {
    steps = lower_[axis0];
  }
  if (upperEn_[axis0] && steps > upper_[axis0]) {
    steps = upper_[axis0];
  }
  return steps;
}

bool MksBus::queueMove(int axis0, int32_t steps, int rpm, uint8_t acc, bool blurStop) {
  if (axis0 < 0 || axis0 >= kMksAxes || !enabled_[axis0]) {
    return false;
  }
  steps = clampSteps(axis0, steps);
  const int64_t drive64 = static_cast<int64_t>(steps) + offset_[axis0];
  const int32_t drive = clampI32(drive64);
  if (!blurStop && haveEncoder_[axis0] && drive == clampI32(encoder_[axis0]) && (movingMask_ & (1u << axis0)) == 0) {
    return false;
  }
  if (rpm < 1) {
    rpm = 1;
  }
  if (rpm > kMksMaxRpm) {
    rpm = kMksMaxRpm;
  }
  if (acc == 0) {
    acc = 1;
  }
  const uint8_t addr = static_cast<uint8_t>(axis0 + 1);
  const uint32_t bits = static_cast<uint32_t>(drive);
  const uint8_t body[] = {0xFA,
                          addr,
                          0xF5,
                          static_cast<uint8_t>(rpm >> 8),
                          static_cast<uint8_t>(rpm),
                          acc,
                          static_cast<uint8_t>(bits >> 24),
                          static_cast<uint8_t>(bits >> 16),
                          static_cast<uint8_t>(bits >> 8),
                          static_cast<uint8_t>(bits)};
  if (!queue([&]() {
        Item item;
        item.kind = Kind::kMove;
        item.axis = static_cast<uint8_t>(axis0);
        item.wantReply = true;
        item.len = 11;
        for (int i = 0; i < 10; ++i) {
          item.raw[i] = body[i];
        }
        item.raw[10] = checksum(item.raw, 10);
        return item;
      }())) {
    return false;
  }
  targetDrive_[axis0] = drive;
  settle_[axis0] = 0;
  groupDirty_ = true;
  return true;
}

bool MksBus::moveAxis(int axis0, int32_t steps) {
  if (axis0 < 0 || axis0 >= kMksAxes) {
    return false;
  }
  return queueMove(axis0, steps, rpm_[axis0], acc_[axis0], false);
}

bool MksBus::moveAxisRpm(int axis0, int32_t steps, int rpm, uint8_t acc) {
  return queueMove(axis0, steps, rpm, acc, false);
}

bool MksBus::jogAxis(int axis0, int32_t steps, uint16_t speedWord) {
  if (axis0 < 0 || axis0 >= kMksAxes) {
    return false;
  }
  if (speedWord > 10000) {
    speedWord = 10000;
  }
  int rpm = static_cast<int>((static_cast<int32_t>(rpm_[axis0]) * speedWord) / 10000);
  if (rpm < 1) {
    rpm = 1;
  }
  return queueMove(axis0, steps, rpm, acc_[axis0], false);
}

bool MksBus::moveBlur(int axis0, int32_t endSteps, uint32_t accelMs, uint32_t cruiseMs) {
  if (axis0 < 0 || axis0 >= kMksAxes) {
    return false;
  }
  endSteps = clampSteps(axis0, endSteps);
  const int32_t start = positionSteps(axis0);
  const int32_t dist = endSteps - start;
  if (dist == 0) {
    return false;
  }
  uint32_t moveMs = accelMs + cruiseMs;
  if (moveMs < 1) {
    moveMs = 1;
  }
  const double stepsPerSec = fabs(static_cast<double>(dist)) * 1000.0 / static_cast<double>(moveMs);
  int rpm = rpmFromStepsPerSec(static_cast<int32_t>(lround(stepsPerSec)));
  const double accelSec = static_cast<double>(accelMs) / 1000.0;
  int k = 255;
  if (rpm > 0 && accelSec > 0.0) {
    k = static_cast<int>(lround(accelSec * 20000.0 / static_cast<double>(rpm)));
  }
  if (k < 1) {
    k = 1;
  }
  if (k > 255) {
    k = 255;
  }
  const uint8_t acc = static_cast<uint8_t>(256 - k);
  return queueMove(axis0, endSteps, rpm, acc == 0 ? 1 : acc, true);
}

void MksBus::finishGroup() {
  if (!groupDirty_) {
    return;
  }
  groupDirty_ = false;
  for (uint8_t i = 0; i < qCount_; ++i) {
    const uint8_t index = static_cast<uint8_t>((qHead_ + i) % kQueueCap);
    const Item& item = queue_[index];
    if (item.kind == Kind::kMove && item.axis < kMksAxes) {
      movingMask_ |= (1u << item.axis);
    }
  }
  const uint8_t body[] = {0xFA, 0x00, 0x4B};
  pushBytes(Kind::kSync, 0, body, 3, false);
  pushBytes(Kind::kSync, 0, body, 3, false);
}

void MksBus::stopAxis(int axis0) {
  if (axis0 < 0 || axis0 >= kMksAxes) {
    return;
  }
  const uint8_t addr = static_cast<uint8_t>(axis0 + 1);
  const uint8_t acc = acc_[axis0] == 0 ? 1 : acc_[axis0];
  const uint8_t body[] = {0xFA, addr, 0xF5, 0x00, 0x00, acc, 0x00, 0x00, 0x00, 0x00};
  Item item;
  item.kind = Kind::kStop;
  item.axis = static_cast<uint8_t>(axis0);
  item.wantReply = true;
  item.len = 11;
  for (int i = 0; i < 10; ++i) {
    item.raw[i] = body[i];
  }
  item.raw[10] = checksum(item.raw, 10);
  queue(item);
  movingMask_ |= (1u << axis0);
  groupDirty_ = true;
  finishGroup();
}

void MksBus::stopAll() {
  for (int axis = 0; axis < kMksAxes; ++axis) {
    if (!online_[axis] && !enabled_[axis]) {
      continue;
    }
    const uint8_t addr = static_cast<uint8_t>(axis + 1);
    const uint8_t acc = acc_[axis] == 0 ? 1 : acc_[axis];
    const uint8_t body[] = {0xFA, addr, 0xF5, 0x00, 0x00, acc, 0x00, 0x00, 0x00, 0x00};
    Item item;
    item.kind = Kind::kStop;
    item.axis = static_cast<uint8_t>(axis);
    item.wantReply = true;
    item.len = 11;
    for (int i = 0; i < 10; ++i) {
      item.raw[i] = body[i];
    }
    item.raw[10] = checksum(item.raw, 10);
    if (queue(item)) {
      movingMask_ |= (1u << axis);
      groupDirty_ = true;
    }
  }
  finishGroup();
}

void MksBus::hardStop() {
  qHead_ = 0;
  qTail_ = 0;
  qCount_ = 0;
  groupDirty_ = false;
  if (initLeft_ > 0) {
    initLeft_ = 0;
  }
  if (haveInflight_ && phase_ == Phase::kWaitRx) {
    // Let the byte on the wire finish; new stops follow.
  }
  for (int axis = 0; axis < kMksAxes; ++axis) {
    movingMask_ &= ~(1u << axis);
    const uint8_t addr = static_cast<uint8_t>(axis + 1);
    const uint8_t body[] = {0xFA, addr, 0xF7};
    pushBytes(Kind::kHard, static_cast<uint8_t>(axis), body, 3, true);
  }
}

void MksBus::resetAxis(int axis0, int32_t steps) {
  if (axis0 < 0 || axis0 >= kMksAxes) {
    return;
  }
  if (steps == 0) {
    const uint8_t addr = static_cast<uint8_t>(axis0 + 1);
    const uint8_t body[] = {0xFA, addr, 0x92};
    pushBytes(Kind::kZero, static_cast<uint8_t>(axis0), body, 3, true);
    offset_[axis0] = 0;
    encoder_[axis0] = 0;
    haveEncoder_[axis0] = true;
    resetPending_[axis0] = false;
    return;
  }
  if (haveEncoder_[axis0]) {
    offset_[axis0] = encoder_[axis0] - static_cast<int64_t>(steps);
    resetPending_[axis0] = false;
  } else {
    resetPending_[axis0] = true;
    resetTo_[axis0] = steps;
  }
}

void MksBus::setSpeed(int axis0, int32_t stepsPerSec, int32_t stepsPerSec2) {
  if (axis0 < 0 || axis0 >= kMksAxes) {
    return;
  }
  rpm_[axis0] = rpmFromStepsPerSec(stepsPerSec);
  acc_[axis0] = accFromStepsPerSec2(stepsPerSec2);
}

void MksBus::setLimits(int axis0, bool lowerEn, int32_t lower, bool upperEn, int32_t upper) {
  if (axis0 < 0 || axis0 >= kMksAxes) {
    return;
  }
  lowerEn_[axis0] = lowerEn;
  upperEn_[axis0] = upperEn;
  lower_[axis0] = lower;
  upper_[axis0] = upper;
}

void MksBus::configure(int axis0, uint8_t flags) {
  if (axis0 < 0 || axis0 >= kMksAxes) {
    return;
  }
  config_[axis0] = flags;
  const bool en = (flags & dfdmc::kDmcMotorConfigEnabled) != 0;
  enabled_[axis0] = en;
  const uint8_t addr = static_cast<uint8_t>(axis0 + 1);
  const uint8_t body[] = {0xFA, addr, 0xF3, static_cast<uint8_t>(en ? 0x01 : 0x00)};
  pushBytes(Kind::kEnable, static_cast<uint8_t>(axis0), body, 4, true);
}

int32_t MksBus::positionSteps(int axis0) const {
  if (axis0 < 0 || axis0 >= kMksAxes) {
    return 0;
  }
  if (resetPending_[axis0] && !haveEncoder_[axis0]) {
    return resetTo_[axis0];
  }
  return clampI32(encoder_[axis0] - offset_[axis0]);
}

bool MksBus::blurEnabled(int axis0) const {
  return axis0 >= 0 && axis0 < kMksAxes && (config_[axis0] & dfdmc::kDmcMotorConfigBlur) != 0;
}

bool MksBus::axisMoving(int axis0) const {
  return axis0 >= 0 && axis0 < kMksAxes && (movingMask_ & (1u << axis0)) != 0;
}

}  // namespace dfmks
