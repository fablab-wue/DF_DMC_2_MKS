#include "bridge.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

namespace dfmks {

using namespace dfdmc;

bool DmcBridge::motorIndexValid(uint8_t motor) const { return motor >= 1 && motor <= kAxisCount; }

int32_t DmcBridge::axisPosition(int axis0) const {
  if (axis0 < kMksAxes) {
    return mks_.positionSteps(axis0);
  }
  return servos_.positionSteps(axis0 - kMksAxes);
}

bool DmcBridge::axisBlur(int axis0) const {
  if (axis0 < kMksAxes) {
    return mks_.blurEnabled(axis0);
  }
  return servos_.blurEnabled(axis0 - kMksAxes);
}

bool DmcBridge::anyMoving() const { return pathActive_ || mks_.moving() || servos_.moving(); }

uint32_t DmcBridge::movingMask() const {
  if (pathActive_) {
    return (1u << kAxisCount) - 1u;
  }
  return mks_.movingMask() | (servos_.movingMask() << kMksAxes);
}

void DmcBridge::setup() {
  Serial.begin(115200);
  Serial.setTimeout(0);
  for (uint8_t pin = 17; pin <= 25; ++pin) {
    pinMode(pin, INPUT);
  }
  statusLed_.begin();
  const GioMap gioMap{kGioOutPins, 4, kGioInPins, 2, kCameraPin, kBuzzerPin, 255};
  gio_.begin(gioMap);
  dmx_.begin(kDmxTxPin, nullptr, 0, kDmxPwmHz, false);
  servos_.begin();
  mks_.begin();
  dfConnected_ = false;
  bootHelloSent_ = false;
}

void DmcBridge::loop() {
  noteCdc(static_cast<bool>(Serial));
  maybeSendBootHello();
  while (Serial.available()) {
    const uint8_t byte = static_cast<uint8_t>(Serial.read());
    DmcFrame frame;
    if (dmcParser_.feed(byte, &frame)) {
      statusLed_.markActivity();
      handleDmcFrame(frame);
    }
  }
  mks_.update();
  dmx_.update();
  gio_.tick();
  servos_.update();
  maybeSendPositionReport();
  maybeUnsolicitedGio();
  bloop_.tick(gio_, dmx_);
  shutter_.tick(gio_);
  pumpPath();
  maybeFinishPath();
  maybeUpdateShoot();
  pumpPendingPlay();
  statusLed_.update(dfConnected_, mks_.busReady(), mks_.busFailed());
}

void DmcBridge::sendDmcFrame(uint32_t id, uint16_t type, const std::vector<uint8_t>& payload) {
  std::vector<uint8_t> packet;
  packet.reserve(kDmcHeaderSize + payload.size() + kDmcCsumSize);
  packet.push_back('D');
  packet.push_back('F');
  appendDwordLE(packet, id);
  appendWordLE(packet, type);
  appendWordLE(packet, static_cast<uint16_t>(payload.size()));
  packet.insert(packet.end(), payload.begin(), payload.end());
  const uint16_t rawChecksum = computeChecksum(packet.data(), packet.size());
  appendWordLE(packet, encodeChecksum(rawChecksum));
  writeUsbFrame(packet.data(), packet.size());
}

void DmcBridge::sendDmcAck(uint32_t id, uint16_t type, uint32_t status) {
  std::vector<uint8_t> payload;
  appendDwordLE(payload, status);
  sendDmcFrame(id, static_cast<uint16_t>(type | kDmcMsgFlagAck), payload);
}

void DmcBridge::sendDmcHello(uint32_t id) {
  char name[48];
  const int wrote = snprintf(name, sizeof(name), "jDF-MKS V1 %dM+%dS+4O+2I+CT+DMX", kMksAxes, kServoAxes);
  std::vector<uint8_t> payload(32, 0);
  if (wrote > 0) {
    const size_t n = static_cast<size_t>(wrote) > 32 ? 32 : static_cast<size_t>(wrote);
    std::memcpy(payload.data(), name, n);
  }
  appendByte(payload, kHelloVersionMajor);
  appendByte(payload, kHelloVersionMinor);
  appendByte(payload, kHelloVersionRev);
  appendByte(payload, static_cast<uint8_t>(kAxisCount));
  appendWordLE(payload, static_cast<uint16_t>(kDmxChannels));
  appendByte(payload, 4);
  appendByte(payload, 2);
  appendByte(payload, 0);
  appendDwordLE(payload, static_cast<uint32_t>(kMaxUploadFrames));
  appendDwordLE(payload, kDmcCapRealTime | kDmcCapGoMotion | kDmcCapGoMotion2 | kDmcCapRealTimeCamera);
  appendWordLE(payload, kHelloProtocolVersion);
  sendDmcFrame(id, kDmcMsgHi, payload);
}

void DmcBridge::sendMotorStatus(uint32_t id) {
  std::vector<uint8_t> payload;
  appendDwordLE(payload, movingMask());
  appendByte(payload, dmx_.ramping() ? 1 : 0);
  sendDmcFrame(id, kDmcMsgMotorStatus, payload);
}

void DmcBridge::sendMotorPositions(uint32_t id, int32_t frameTime) {
  std::vector<uint8_t> payload;
  appendDwordLE(payload, static_cast<uint32_t>(frameTime));
  for (int a = 0; a < kAxisCount; ++a) {
    appendDwordLE(payload, static_cast<uint32_t>(axisPosition(a)));
  }
  sendDmcFrame(id, kDmcMsgMotorGetPosition, payload);
}

void DmcBridge::sendGioIn(uint32_t id) {
  std::vector<uint8_t> payload;
  appendDwordLE(payload, gio_.inputs());
  sendDmcFrame(id, kDmcMsgGioIn, payload);
}

void DmcBridge::maybeSendBootHello() {
  const bool cdcConnected = static_cast<bool>(Serial);
  if (cdcConnected && !bootHelloSent_) {
    sendDmcHello(0);
    bootHelloSent_ = true;
  } else if (!cdcConnected) {
    bootHelloSent_ = false;
    dfConnected_ = false;
  }
}

void DmcBridge::maybeSendPositionReport() {
  if (!dfConnected_ || !anyMoving() || pathActive_) {
    return;
  }
  const uint32_t now = millis();
  if (now - lastPositionTxMs_ < kPositionReportMs) {
    return;
  }
  lastPositionTxMs_ = now;
  sendMotorPositions(0);
}

void DmcBridge::maybeUnsolicitedGio() {
  if (dfConnected_ && gio_.pollInputChange()) {
    sendGioIn(0);
  }
}

void DmcBridge::noteCdc(bool up) {
  if (cdcWasUp_ && !up) {
    mks_.setHostAlive(false);
    dfConnected_ = false;
    bootHelloSent_ = false;
    dmcParser_.reset();
    wasPathActive_ = false;
    stopEverything();
    shutter_.endMove(gio_);
    bloop_.release(gio_, dmx_);
  } else if (up && !cdcWasUp_) {
    mks_.setHostAlive(true);
  }
  cdcWasUp_ = up;
}

void DmcBridge::maybeFinishPath() {
  const bool active = pathActive_;
  if (!active && wasPathActive_ && dfConnected_ && !shootRun_ && !shootArmed_) {
    shutter_.endMove(gio_);
    bloop_.release(gio_, dmx_);
    sendDmcFrame(0, kDmcMsgRtEnd, {});
  }
  wasPathActive_ = active;
}

void DmcBridge::endShoot(bool notify) {
  const bool pending = shootNeedsEnd_;
  if (shootShutter_) {
    gio_.setCameraShutter(false);
    shootShutter_ = false;
  }
  shootArmed_ = false;
  shootRun_ = false;
  mksBlurSent_ = false;
  shootNeedsEnd_ = false;
  if (notify && pending && dfConnected_) {
    sendDmcFrame(0, kDmcMsgRtEnd, {});
  }
}

void DmcBridge::stopEverything() {
  pathActive_ = false;
  pendingPlay_ = false;
  armedPlay_ = false;
  mks_.setPositionPoll(true);
  endShoot(false);
  servos_.stopMotion();
}

void DmcBridge::commandPlayback(int dfFrame) {
  currentFrame_ = dfFrame;
  bool any = false;
  for (int a = 0; a < kMksAxes; ++a) {
    if (mks_.moveAxis(a, path_.sampleSteps(a, static_cast<double>(dfFrame), true))) {
      any = true;
    }
  }
  if (any) {
    mks_.finishGroup();
  }
  for (int s = 0; s < kServoAxes; ++s) {
    servos_.moveToSteps(s, path_.sampleSteps(kMksAxes + s, static_cast<double>(dfFrame), true));
  }
  mksSentFrame_ = dfFrame;
  if (syncDmx_) {
    applyProgramDmx(dfFrame);
  }
  applyFrameTrigger(dfFrame);
}

void DmcBridge::moveToSample(double frameTime) {
  pathActive_ = false;
  pendingPlay_ = false;
  mks_.setPositionPoll(true);
  bool any = false;
  for (int a = 0; a < kMksAxes; ++a) {
    if (mks_.moveAxis(a, path_.sampleSteps(a, frameTime, true))) {
      any = true;
    }
  }
  if (any) {
    mks_.finishGroup();
  }
  for (int s = 0; s < kServoAxes; ++s) {
    servos_.moveToSteps(s, path_.sampleSteps(kMksAxes + s, frameTime, true));
  }
  currentFrame_ = static_cast<int>(lround(frameTime));
}

uint32_t DmcBridge::poseFault(double frameTime, bool extrapolate) const {
  for (int a = 0; a < kAxisCount; ++a) {
    const int32_t steps = path_.sampleSteps(a, frameTime, extrapolate);
    const uint32_t fault = a < kMksAxes ? mks_.limitFault(a, steps) : servos_.limitFault(a - kMksAxes, steps);
    if (fault != 0) {
      return fault;
    }
  }
  return 0;
}

bool DmcBridge::rejectRunLimits(const RtRunMove& move, const RtPlaySpan& span, uint32_t id) {
  if (poseFault(span.prerollFrame, true) != 0) {
    sendDmcAck(id, kDmcMsgRtRunMove, kDmcAckErrPreroll);
    return true;
  }
  if (poseFault(span.postrollFrame, true) != 0) {
    sendDmcAck(id, kDmcMsgRtRunMove, kDmcAckErrPostroll);
    return true;
  }
  const int lo = move.startFrame < move.endFrame ? move.startFrame : move.endFrame;
  const int hi = move.startFrame < move.endFrame ? move.endFrame : move.startFrame;
  const int from = lo > path_.startFrame() ? lo : path_.startFrame();
  const int to = hi < path_.endFrame() ? hi : path_.endFrame();
  for (int f = from; f <= to; ++f) {
    const uint32_t fault = poseFault(static_cast<double>(f), false);
    if (fault != 0) {
      sendDmcAck(id, kDmcMsgRtRunMove, fault);
      return true;
    }
  }
  return false;
}

bool DmcBridge::inRun(int frame) const {
  const int lo = runStart_ < runEnd_ ? runStart_ : runEnd_;
  const int hi = runStart_ < runEnd_ ? runEnd_ : runStart_;
  return frame >= lo && frame <= hi;
}

void DmcBridge::applyProgramDmx(int dfFrame) {
  int local = 0;
  if (!path_.localFrame(dfFrame, &local)) {
    return;
  }
  for (int slot = 0; slot < path_.dmxSlotCount(); ++slot) {
    const uint16_t channel = path_.dmxChannel(slot);
    const uint8_t level = path_.dmxLevel(slot, local);
    if (channel >= 1) {
      dmx_.apply(channel, &level, 1, false);
    }
  }
}

void DmcBridge::queueMksFrame(int dfFrame) {
  int local = 0;
  if (!path_.localFrame(dfFrame, &local)) {
    return;
  }
  bool any = false;
  for (int a = 0; a < kMksAxes; ++a) {
    if (mks_.moveAxis(a, path_.positionSteps(a, local))) {
      any = true;
    }
  }
  if (any) {
    mks_.finishGroup();
  }
  mksSentFrame_ = dfFrame;
}

void DmcBridge::commandFrame(int dfFrame, bool withDmx) {
  int local = 0;
  if (!path_.localFrame(dfFrame, &local)) {
    return;
  }
  currentFrame_ = dfFrame;
  queueMksFrame(dfFrame);
  for (int s = 0; s < kServoAxes; ++s) {
    servos_.moveToSteps(s, path_.positionSteps(kMksAxes + s, local));
  }
  if (withDmx) {
    applyProgramDmx(dfFrame);
  }
  applyFrameTrigger(dfFrame);
}

void DmcBridge::startArmedPlay() {
  playEndFrame_ = pendingEnd_;
  playDir_ = pendingStart_ <= pendingEnd_ ? 1 : -1;
  commandPlayback(pendingStart_);
  bloop_.fire(gio_, dmx_);
  shutter_.beginMove(gio_);
  if (inRun(pendingStart_)) {
    shutter_.beginFrame(gio_);
  }
  pathActive_ = true;
  mks_.setPositionPoll(false);
  nextFrameUs_ = micros() + pathSliceUs_;
  sendMotorPositions(0, static_cast<int32_t>(moveTimeThousandths(currentFrame_)));
}

void DmcBridge::maybeUpdateShoot() {
  if (!shootRun_) {
    return;
  }
  const uint32_t elapsed = millis() - shootT0_;
  if (!mksBlurSent_ && elapsed >= shootDelayMs_ && mks_.idle()) {
    bool any = false;
    for (int axis = 0; axis < kMksAxes; ++axis) {
      if (shootBlur_[axis] && mks_.moveBlur(axis, shootEndSteps_[axis], shootAccelMs_, shootCruiseMs_)) {
        any = true;
      }
    }
    if (any) {
      mks_.finishGroup();
    }
    mksBlurSent_ = true;
  }
  if (!shootShutter_ && elapsed >= shootShutterOpenMs_) {
    gio_.setCameraShutter(true);
    shootShutter_ = true;
  }
  if (shootShutter_ && elapsed >= shootShutterCloseMs_) {
    gio_.setCameraShutter(false);
    shootShutter_ = false;
  }
  if (elapsed >= shootDoneMs_ && !mks_.moving() && !servos_.moving()) {
    endShoot(true);
  }
}

namespace {

int32_t sampleSteps(const dfdmc::PathTable& path, int axis, double frameTime) {
  const int lo = path.startFrame() < path.endFrame() ? path.startFrame() : path.endFrame();
  const int hi = path.startFrame() > path.endFrame() ? path.startFrame() : path.endFrame();
  if (frameTime < lo) {
    frameTime = lo;
  }
  if (frameTime > hi) {
    frameTime = hi;
  }
  const int f0 = static_cast<int>(floor(frameTime));
  int f1 = f0 + 1;
  if (f1 > hi) {
    f1 = hi;
  }
  int local0 = 0;
  if (!path.localFrame(f0, &local0)) {
    return 0;
  }
  const int32_t p0 = path.positionSteps(axis, local0);
  if (f1 == f0) {
    return p0;
  }
  int local1 = 0;
  if (!path.localFrame(f1, &local1)) {
    return p0;
  }
  const double u = frameTime - static_cast<double>(f0);
  return p0 + static_cast<int32_t>(lround((path.positionSteps(axis, local1) - p0) * u));
}

}  // namespace

void DmcBridge::handleShootFrame(const DmcFrame& frame) {
  int32_t dfFrame = 0;
  uint8_t direction = 1;
  uint32_t exposureMs = 0;
  uint16_t blurX10 = 0;
  if (frame.payload.size() < 11 || !readSignedDwordLE(frame.payload, 0, &dfFrame) ||
      !readByte(frame.payload, 4, &direction) || !readDwordLE(frame.payload, 5, &exposureMs) ||
      !readWordLE(frame.payload, 9, &blurX10)) {
    sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
    return;
  }
  if (anyMoving() || shootRun_) {
    sendDmcAck(frame.id, frame.type, kDmcAckErrMoving);
    return;
  }
  if (exposureMs < 1 || exposureMs > 60000) {
    sendDmcAck(frame.id, frame.type, kDmcAckErrRange);
    return;
  }
  if (blurX10 == 0) {
    blurX10 = 1000;
  }
  bool overrideAxis[kAxisCount] = {};
  int32_t posA[kAxisCount] = {};
  int32_t posB[kAxisCount] = {};
  size_t off = 11;
  while (off + 9 <= frame.payload.size()) {
    uint8_t motor = 0;
    int32_t a = 0;
    int32_t b = 0;
    if (!readByte(frame.payload, off, &motor) || !readSignedDwordLE(frame.payload, off + 1, &a) ||
        !readSignedDwordLE(frame.payload, off + 5, &b)) {
      break;
    }
    if (motorIndexValid(motor)) {
      overrideAxis[motor - 1] = true;
      posA[motor - 1] = a;
      posB[motor - 1] = b;
    }
    off += 9;
  }
  const int dirSign = direction ? 1 : -1;
  const double dt = static_cast<double>(blurX10) * 0.0005;
  const double te = static_cast<double>(exposureMs) / 1000.0;
  bool anyMks = false;
  for (int axis = 0; axis < kAxisCount; ++axis) {
    int32_t pose = axisPosition(axis);
    int local = 0;
    if (path_.localFrame(dfFrame, &local)) {
      pose = path_.positionSteps(axis, local);
    }
    int32_t openPose = pose;
    int32_t closePose = pose;
    if (axisBlur(axis) && overrideAxis[axis]) {
      const double delta = (0.5 - fabs(dt)) * static_cast<double>(posB[axis] - posA[axis]);
      openPose = posA[axis] + static_cast<int32_t>(lround(delta));
      closePose = posB[axis] - static_cast<int32_t>(lround(delta));
    } else if (axisBlur(axis) && !path_.empty()) {
      openPose = sampleSteps(path_, axis, static_cast<double>(dfFrame) - dirSign * dt);
      closePose = sampleSteps(path_, axis, static_cast<double>(dfFrame) + dirSign * dt);
    }
    const int32_t span = closePose - openPose;
    const int sign = span >= 0 ? 1 : -1;
    const double v = te > 0.0 ? fabs(static_cast<double>(span)) / te : 0.0;
    const int32_t accel = static_cast<int32_t>(lround(0.5 * v));
    const int32_t pre = openPose - sign * accel;
    shootEndSteps_[axis] = closePose + sign * accel;
    shootBlur_[axis] = span != 0;
    const int32_t target = shootBlur_[axis] ? pre : pose;
    if (axis < kMksAxes) {
      if (mks_.moveAxis(axis, target)) {
        anyMks = true;
      }
    } else {
      servos_.slewTo(axis - kMksAxes, target, 10000);
    }
  }
  if (anyMks) {
    mks_.finishGroup();
  }
  shootDelayMs_ = 0;
  shootAccelMs_ = 1000;
  shootCruiseMs_ = exposureMs;
  shootShutterOpenMs_ = 1000;
  shootShutterCloseMs_ = 1000 + exposureMs;
  shootDoneMs_ = 2000 + exposureMs;
  shootShutter_ = false;
  shootRun_ = false;
  mksBlurSent_ = false;
  shootArmed_ = true;
  shootNeedsEnd_ = true;
  sendDmcAck(frame.id, frame.type, kDmcAckOk);
}

void DmcBridge::handleShootFrame2(const DmcFrame& frame) {
  int32_t dfFrame = 0;
  uint32_t exposureMs = 0;
  uint16_t openWord = 0;
  uint16_t closeWord = 0;
  if (frame.payload.size() < 12 || !readSignedDwordLE(frame.payload, 0, &dfFrame) ||
      !readDwordLE(frame.payload, 4, &exposureMs) || !readWordLE(frame.payload, 8, &openWord) ||
      !readWordLE(frame.payload, 10, &closeWord)) {
    sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
    return;
  }
  if (anyMoving() || shootRun_) {
    sendDmcAck(frame.id, frame.type, kDmcAckErrMoving);
    return;
  }
  if (exposureMs == 0) {
    exposureMs = 1000;
  }
  if (exposureMs > 60000) {
    sendDmcAck(frame.id, frame.type, kDmcAckErrRange);
    return;
  }
  const int16_t shutterOpen = static_cast<int16_t>(openWord);
  const int16_t shutterClose = static_cast<int16_t>(closeWord);
  if (shutterClose <= shutterOpen) {
    sendDmcAck(frame.id, frame.type, kDmcAckErrRange);
    return;
  }

  bool overrideAxis[kAxisCount] = {};
  int32_t posA[kAxisCount] = {};
  int32_t posB[kAxisCount] = {};
  size_t off = 12;
  while (off + 9 <= frame.payload.size()) {
    uint8_t motor = 0;
    int32_t a = 0;
    int32_t b = 0;
    if (!readByte(frame.payload, off, &motor) || !readSignedDwordLE(frame.payload, off + 1, &a) ||
        !readSignedDwordLE(frame.payload, off + 5, &b)) {
      break;
    }
    if (motorIndexValid(motor)) {
      overrideAxis[motor - 1] = true;
      posA[motor - 1] = a;
      posB[motor - 1] = b;
    }
    off += 9;
  }

  const double te = static_cast<double>(exposureMs) / 1000.0;
  const double degrees = static_cast<double>(shutterClose - shutterOpen);
  const double secondsPerDegree = te / degrees;
  const double moveT = 360.0 * secondsPerDegree;
  const uint32_t moveMs = static_cast<uint32_t>(lround(moveT * 1000.0));
  const uint32_t accelMs = static_cast<uint32_t>(lround(moveT * 0.125 * 1000.0));
  const uint32_t cruiseMs = moveMs > 2 * accelMs ? moveMs - 2 * accelMs : 0;
  uint32_t delayMs = 0;
  uint32_t shutterOpenMs = 0;
  if (shutterOpen < 0) {
    delayMs = static_cast<uint32_t>(lround(-shutterOpen * secondsPerDegree * 1000.0));
    shutterOpenMs = 0;
  } else {
    shutterOpenMs = static_cast<uint32_t>(lround(shutterOpen * secondsPerDegree * 1000.0));
  }
  const uint32_t shutterCloseMs = shutterOpenMs + exposureMs;
  uint32_t postMs = 0;
  if (shutterClose > 360) {
    postMs = static_cast<uint32_t>(lround((shutterClose - 360) * secondsPerDegree * 1000.0));
  }
  const uint32_t doneMs = delayMs + moveMs + postMs;
  if (doneMs > 120000 || accelMs < 1) {
    sendDmcAck(frame.id, frame.type, kDmcAckErrRange);
    return;
  }

  bool anyMks = false;
  for (int axis = 0; axis < kAxisCount; ++axis) {
    int32_t pose = axisPosition(axis);
    int local = 0;
    if (path_.localFrame(dfFrame, &local)) {
      pose = path_.positionSteps(axis, local);
    }
    int32_t startPose = pose;
    int32_t endPose = pose;
    if (axisBlur(axis) && overrideAxis[axis]) {
      startPose = posA[axis];
      endPose = posB[axis];
    } else if (axisBlur(axis) && !path_.empty()) {
      startPose = sampleSteps(path_, axis, static_cast<double>(dfFrame) - 0.5);
      endPose = sampleSteps(path_, axis, static_cast<double>(dfFrame) + 0.5);
    }
    shootBlur_[axis] = startPose != endPose;
    shootEndSteps_[axis] = endPose;
    const int32_t target = shootBlur_[axis] ? startPose : pose;
    if (axis < kMksAxes) {
      if (mks_.moveAxis(axis, target)) {
        anyMks = true;
      }
    } else {
      servos_.slewTo(axis - kMksAxes, target, 10000);
    }
  }
  if (anyMks) {
    mks_.finishGroup();
  }
  shootDelayMs_ = delayMs;
  shootAccelMs_ = accelMs;
  shootCruiseMs_ = cruiseMs;
  shootShutterOpenMs_ = shutterOpenMs;
  shootShutterCloseMs_ = shutterCloseMs;
  shootDoneMs_ = doneMs;
  shootShutter_ = false;
  shootRun_ = false;
  mksBlurSent_ = false;
  shootArmed_ = true;
  shootNeedsEnd_ = true;
  sendDmcAck(frame.id, frame.type, kDmcAckOk);
}

void DmcBridge::applyFrameTrigger(int dfFrame) {
  int local = 0;
  if (path_.triggerMask() == 0 || !path_.localFrame(dfFrame, &local)) {
    return;
  }
  gio_.setOutputs(path_.triggerAtLocal(local) | bloop_.heldOutputs());
}

void DmcBridge::pumpPendingPlay() {
  if (!pendingPlay_) {
    return;
  }
  if (millis() < pendingPlayAtMs_) {
    return;
  }
  if (!mks_.idle()) {
    return;
  }
  pendingPlay_ = false;
  startArmedPlay();
}

void DmcBridge::pumpPath() {
  mks_.setPositionPoll(!pathActive_);
  if (!pathActive_) {
    return;
  }
  const uint32_t now = micros();
  if (static_cast<int32_t>(now - nextFrameUs_) >= 0) {
    nextFrameUs_ += pathSliceUs_;
    if (static_cast<int32_t>(now - nextFrameUs_) > static_cast<int32_t>(pathSliceUs_)) {
      nextFrameUs_ = now;
    }
    const int next = currentFrame_ + playDir_;
    const bool done = playDir_ > 0 ? next > playEndFrame_ : next < playEndFrame_;
    if (done) {
      pathActive_ = false;
      currentFrame_ = playEndFrame_;
      mks_.setPositionPoll(true);
      return;
    }
    commandPlayback(next);
    if (inRun(next)) {
      shutter_.beginFrame(gio_);
    }
    sendMotorPositions(0, static_cast<int32_t>(moveTimeThousandths(next)));
  }
  if (mks_.idle() && mksSentFrame_ != currentFrame_) {
    bool any = false;
    for (int a = 0; a < kMksAxes; ++a) {
      if (mks_.moveAxis(a, path_.sampleSteps(a, static_cast<double>(currentFrame_), true))) {
        any = true;
      }
    }
    if (any) {
      mks_.finishGroup();
    }
    mksSentFrame_ = currentFrame_;
  }
}

uint32_t DmcBridge::psFromFpsX1000(uint32_t fpsX1000) const { return framePeriodUs(fpsX1000); }

void DmcBridge::handleDmcFrame(const DmcFrame& frame) {
  if (!frame.valid) {
    sendDmcAck(frame.id, frame.type, kDmcAckErrChecksum);
    return;
  }
  dfConnected_ = true;

  if (frame.type == kDmcMsgHi) {
    sendDmcHello(frame.id);
    return;
  }

  switch (frame.type) {
    case kDmcMsgGioOut: {
      uint32_t bits = 0;
      if (!readDwordLE(frame.payload, 0, &bits)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      gio_.setOutputs(bits);
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      break;
    }
    case kDmcMsgGioIn:
      sendGioIn(frame.id);
      break;
    case kDmcMsgGioCam: {
      uint32_t cam = 0;
      if (!readDwordLE(frame.payload, 0, &cam)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      gio_.setCameraShutter((cam & kDmcGioCamShutter) != 0);
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      break;
    }
    case kDmcMsgDmx: {
      LiveDmx live;
      const LiveDmxStatus st = parseLiveDmx(frame.payload, &live);
      if (st == LiveDmxStatus::kGeneral) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      if (st == LiveDmxStatus::kRange) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrRange);
        break;
      }
      dmx_.apply(live.channel, live.levels, live.count, live.ramp);
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      break;
    }
    case kDmcMsgMotorStatus:
      sendMotorStatus(frame.id);
      break;
    case kDmcMsgMotorGetPosition:
      if (pathActive_) {
        sendMotorPositions(frame.id, static_cast<int32_t>(moveTimeThousandths(currentFrame_)));
      } else {
        sendMotorPositions(frame.id);
      }
      break;
    default:
      handleMotorOrRt(frame);
      break;
  }
}

void DmcBridge::handleMotorOrRt(const DmcFrame& frame) {
  switch (frame.type) {
    case kDmcMsgMotorMove: {
      uint8_t motor = 0;
      int32_t position = 0;
      if (frame.payload.size() != 5 || !readByte(frame.payload, 0, &motor) ||
          !readSignedDwordLE(frame.payload, 1, &position)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      if (!motorIndexValid(motor)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrRange);
        break;
      }
      const uint32_t fault = isMks(motor) ? mks_.limitFault(motor - 1, position)
                                         : servos_.limitFault(motor - 1 - kMksAxes, position);
      if (fault != 0) {
        sendDmcAck(frame.id, frame.type, fault);
        break;
      }
      pathActive_ = false;
      armedPlay_ = false;
      mks_.setPositionPoll(true);
      if (shootNeedsEnd_) {
        endShoot(true);
        wasPathActive_ = false;
      }
      shutter_.endMove(gio_);
      bloop_.release(gio_, dmx_);
      if (isMks(motor)) {
        mks_.moveAxis(motor - 1, position);
        mks_.finishGroup();
      } else {
        servos_.slewTo(motor - 1 - kMksAxes, position, 10000);
      }
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      break;
    }
    case kDmcMsgMotorStop: {
      uint8_t motor = 0;
      if (!readByte(frame.payload, 0, &motor)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      if (!motorIndexValid(motor)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrRange);
        break;
      }
      pathActive_ = false;
      armedPlay_ = false;
      mks_.setPositionPoll(true);
      if (shootNeedsEnd_) {
        endShoot(true);
        wasPathActive_ = false;
      }
      shutter_.endMove(gio_);
      bloop_.release(gio_, dmx_);
      if (isMks(motor)) {
        mks_.stopAxis(motor - 1);
      } else {
        servos_.stopAxis(motor - 1 - kMksAxes);
      }
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      sendMotorPositions(frame.id);
      break;
    }
    case kDmcMsgMotorStopAll:
    case kDmcMsgMotorHardStop: {
      const uint32_t now = millis();
      const bool hard = frame.type == kDmcMsgMotorHardStop ||
                        (lastStopAllMs_ != 0 && static_cast<uint32_t>(now - lastStopAllMs_) < kDmcStopAllHardMs);
      if (frame.type == kDmcMsgMotorStopAll) {
        lastStopAllMs_ = now;
      }
      pathActive_ = false;
      armedPlay_ = false;
      mks_.setPositionPoll(true);
      if (shootNeedsEnd_) {
        endShoot(true);
        wasPathActive_ = false;
      }
      shutter_.endMove(gio_);
      bloop_.release(gio_, dmx_);
      if (hard) {
        mks_.hardStop();
      } else {
        mks_.stopAll();
      }
      servos_.stopMotion();
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      sendMotorPositions(frame.id);
      break;
    }
    case kDmcMsgMotorResetPosition: {
      uint8_t motor = 0;
      int32_t position = 0;
      if (frame.payload.size() != 5 || !readByte(frame.payload, 0, &motor) ||
          !readSignedDwordLE(frame.payload, 1, &position)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      if (!motorIndexValid(motor)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrRange);
        break;
      }
      if (pathActive_ || (isMks(motor) && mks_.axisMoving(motor - 1)) ||
          (!isMks(motor) && (servos_.movingMask() & (1u << (motor - 1 - kMksAxes))) != 0)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrMoving);
        break;
      }
      if (isMks(motor)) {
        mks_.resetAxis(motor - 1, position);
      }
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      sendMotorPositions(frame.id);
      break;
    }
    case kDmcMsgMotorJog: {
      uint8_t motor = 0;
      uint16_t speed = 0;
      int32_t destination = 0;
      if (frame.payload.size() != 7 || !readByte(frame.payload, 0, &motor) || !readWordLE(frame.payload, 1, &speed) ||
          !readSignedDwordLE(frame.payload, 3, &destination)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      if (!motorIndexValid(motor)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrRange);
        break;
      }
      const uint32_t fault = isMks(motor) ? mks_.limitFault(motor - 1, destination)
                                         : servos_.limitFault(motor - 1 - kMksAxes, destination);
      if (fault != 0) {
        sendDmcAck(frame.id, frame.type, fault);
        break;
      }
      pathActive_ = false;
      armedPlay_ = false;
      mks_.setPositionPoll(true);
      if (shootNeedsEnd_) {
        endShoot(true);
        wasPathActive_ = false;
      }
      shutter_.endMove(gio_);
      bloop_.release(gio_, dmx_);
      if (speed == 0) {
        speed = 1;
      }
      if (isMks(motor)) {
        mks_.jogAxis(motor - 1, destination, speed);
        mks_.finishGroup();
      } else {
        servos_.slewTo(motor - 1 - kMksAxes, destination, speed);
      }
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      break;
    }
    case kDmcMsgMotorConfigure: {
      uint8_t motor = 0;
      uint8_t flags = 0;
      if (frame.payload.size() < 2 || !readByte(frame.payload, 0, &motor) || !readByte(frame.payload, 1, &flags)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      if (!motorIndexValid(motor)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrRange);
        break;
      }
      if (isMks(motor)) {
        mks_.configure(motor - 1, flags);
      } else {
        servos_.configure(motor - 1 - kMksAxes, flags);
      }
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      break;
    }
    case kDmcMsgMotorSetSpeed: {
      uint8_t motor = 0;
      int32_t maxVelocity = 0;
      int32_t maxAccel = 0;
      if (frame.payload.size() != 9 || !readByte(frame.payload, 0, &motor) ||
          !readSignedDwordLE(frame.payload, 1, &maxVelocity) || !readSignedDwordLE(frame.payload, 5, &maxAccel)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      if (!motorIndexValid(motor)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrRange);
        break;
      }
      if (isMks(motor)) {
        mks_.setSpeed(motor - 1, maxVelocity, maxAccel);
      } else {
        servos_.setMaxSpeed(motor - 1 - kMksAxes, maxVelocity);
      }
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      break;
    }
    case kDmcMsgMotorSetLimits: {
      uint8_t motor = 0;
      uint8_t lowerEnable = 0;
      uint8_t upperEnable = 0;
      uint8_t hwSet = 0;
      int32_t lower = 0;
      int32_t upper = 0;
      if (frame.payload.size() < 12 || !readByte(frame.payload, 0, &motor) ||
          !readByte(frame.payload, 1, &lowerEnable) || !readSignedDwordLE(frame.payload, 2, &lower) ||
          !readByte(frame.payload, 6, &upperEnable) || !readSignedDwordLE(frame.payload, 7, &upper) ||
          !readByte(frame.payload, 11, &hwSet)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      (void)hwSet;
      if (!motorIndexValid(motor)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrRange);
        break;
      }
      if (isMks(motor)) {
        mks_.setLimits(motor - 1, lowerEnable != 0, lower, upperEnable != 0, upper);
      } else {
        servos_.setLimits(motor - 1 - kMksAxes, lowerEnable != 0, lower, upperEnable != 0, upper);
      }
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      break;
    }
    case kDmcMsgRtUploadBegin: {
      int32_t startFrame = 1;
      int32_t endFrame = 1;
      if (!readSignedDwordLE(frame.payload, 0, &startFrame) || !readSignedDwordLE(frame.payload, 4, &endFrame)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      path_.beginUpload(startFrame, endFrame, kAxisCount);
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      break;
    }
    case kDmcMsgRtUploadAxis: {
      uint8_t motor = 0;
      uint32_t index = 0;
      if (frame.payload.size() < 5 || !readByte(frame.payload, 0, &motor) || !readDwordLE(frame.payload, 1, &index)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      const bool finalFill = (index & kDmcDmxFlagFinalSet) != 0;
      index &= ~kDmcDmxFlagFinalSet;
      const int n = static_cast<int>((frame.payload.size() - 5) / 4);
      std::vector<int32_t> values(static_cast<size_t>(n));
      for (int i = 0; i < n; ++i) {
        readSignedDwordLE(frame.payload, 5 + static_cast<size_t>(i) * 4, &values[static_cast<size_t>(i)]);
      }
      if (!path_.storeAxis(motor, index, values.data(), n, finalFill)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrRange);
        break;
      }
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      break;
    }
    case kDmcMsgRtUploadDmx: {
      uint16_t channel = 0;
      uint32_t index = 0;
      if (frame.payload.size() < 7 || !readWordLE(frame.payload, 0, &channel) || !readDwordLE(frame.payload, 2, &index)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      const bool finalFill = (index & kDmcDmxFlagFinalSet) != 0;
      index &= ~kDmcDmxFlagFinalSet;
      if (channel < 1 || channel > kDmxChannels) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrRange);
        break;
      }
      const int n = static_cast<int>(frame.payload.size() - 6);
      if (!path_.storeDmx(channel, index, frame.payload.data() + 6, n, finalFill)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrRange);
        break;
      }
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      break;
    }
    case kDmcMsgRtUploadTriggers: {
      uint32_t mask = 0;
      if (!readDwordLE(frame.payload, 0, &mask)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      size_t off = 4;
      while (off + 8 <= frame.payload.size()) {
        uint32_t idx = 0;
        uint32_t val = 0;
        readDwordLE(frame.payload, off, &idx);
        readDwordLE(frame.payload, off + 4, &val);
        path_.storeTrigger(mask, idx, val);
        off += 8;
      }
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      break;
    }
    case kDmcMsgRtUploadEnd:
      if (!path_.finishUpload()) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      break;
    case kDmcMsgRtPositionFrame: {
      int32_t frameNo = 0;
      if (!readSignedDwordLE(frame.payload, 0, &frameNo)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      if (path_.empty()) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      pathActive_ = false;
      armedPlay_ = false;
      mks_.setPositionPoll(true);
      commandFrame(frameNo, true);
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      lastPositionTxMs_ = millis();
      sendMotorPositions(frame.id, static_cast<int32_t>(moveTimeThousandths(frameNo)));
      break;
    }
    case kDmcMsgRtRunMove: {
      RtRunMove move;
      if (!parseRtRunMove(frame.payload, &move)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      if (path_.empty()) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      RtPlaySpan span;
      rtPlaySpan(move, &span);
      if (rejectRunLimits(move, span, frame.id)) {
        break;
      }
      pathSliceUs_ = psFromFpsX1000(move.fpsX1000);
      wasPathActive_ = false;
      moveToSample(span.prerollFrame);
      syncDmx_ = move.syncDmx;
      runStart_ = move.startFrame;
      runEnd_ = move.endFrame;
      pendingStart_ = span.playFrom;
      pendingEnd_ = span.playTo;
      bloop_.release(gio_, dmx_);
      shutter_.endMove(gio_);
      bloop_.arm(move.bloopLocation, move.bloopDmx, move.bloopTimeMs);
      shutter_.arm(move.flags, move.cameraOpen, move.cameraClose, framePeriodUs(move.fpsX1000));
      pendingPlay_ = false;
      armedPlay_ = true;
      endShoot(true);
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      break;
    }
    case kDmcMsgRtShootFrame:
      handleShootFrame(frame);
      break;
    case kDmcMsgRtShootFrame2:
      handleShootFrame2(frame);
      break;
    case kDmcMsgRtGo:
      if (shootArmed_) {
        if (anyMoving() || shootRun_) {
          sendDmcAck(frame.id, frame.type, kDmcAckErrNotInPosition);
          break;
        }
        shootT0_ = millis();
        for (int axis = 0; axis < kServoAxes; ++axis) {
          if (shootBlur_[kMksAxes + axis]) {
            servos_.startBlur(axis, shootEndSteps_[kMksAxes + axis], shootDelayMs_, shootAccelMs_, shootCruiseMs_);
          }
        }
        shootArmed_ = false;
        shootRun_ = true;
        shootShutter_ = false;
        mksBlurSent_ = false;
        sendDmcAck(frame.id, frame.type, kDmcAckOk);
        break;
      }
      if (armedPlay_) {
        if (anyMoving()) {
          sendDmcAck(frame.id, frame.type, kDmcAckErrNotInPosition);
          break;
        }
        armedPlay_ = false;
        startArmedPlay();
        sendDmcAck(frame.id, frame.type, kDmcAckOk);
        break;
      }
      if (path_.empty()) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      pendingStart_ = path_.startFrame();
      pendingEnd_ = path_.endFrame();
      runStart_ = pendingStart_;
      runEnd_ = pendingEnd_;
      bloop_.release(gio_, dmx_);
      shutter_.endMove(gio_);
      pendingPlayAtMs_ = millis();
      pendingPlay_ = true;
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      break;
    case kDmcMsgRtJogAll: {
      uint32_t fpsX1000 = 24000;
      int32_t dest = 1;
      if (frame.payload.size() < 8 || !readDwordLE(frame.payload, 0, &fpsX1000) ||
          !readSignedDwordLE(frame.payload, 4, &dest)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      if (path_.empty()) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      int from = currentFrame_;
      if (from < path_.startFrame()) {
        from = path_.startFrame();
      }
      pathSliceUs_ = psFromFpsX1000(fpsX1000);
      pendingStart_ = from;
      pendingEnd_ = dest;
      runStart_ = from;
      runEnd_ = dest;
      bloop_.release(gio_, dmx_);
      shutter_.endMove(gio_);
      pendingPlayAtMs_ = millis();
      pendingPlay_ = true;
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      break;
    }
    default:
      sendDmcAck(frame.id, frame.type, kDmcAckErrUnsupported);
      break;
  }
}

}  // namespace dfmks
