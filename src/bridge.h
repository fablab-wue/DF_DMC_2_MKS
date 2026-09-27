#pragma once

#include "config.h"
#include "mks_bus.h"
#include "servo_bank.h"
#include "status_led.h"

#include <dmc_protocol.h>
#include <dmx_engine.h>
#include <gio_io.h>
#include <path_table.h>

namespace dfmks {

class DmcBridge {
 public:
  void setup();
  void loop();

 private:
  bool motorIndexValid(uint8_t motor) const;
  bool isMks(uint8_t motor) const { return motor >= 1 && motor <= kMksAxes; }
  int32_t axisPosition(int axis0) const;
  bool axisBlur(int axis0) const;
  bool anyMoving() const;
  uint32_t movingMask() const;
  void sendDmcFrame(uint32_t id, uint16_t type, const std::vector<uint8_t>& payload);
  void sendDmcAck(uint32_t id, uint16_t type, uint32_t status);
  void sendDmcHello(uint32_t id);
  void sendMotorStatus(uint32_t id);
  void sendMotorPositions(uint32_t id, int32_t frameTime = 0);
  void sendGioIn(uint32_t id);
  void maybeSendBootHello();
  void maybeSendPositionReport();
  void maybeUnsolicitedGio();
  void maybeFinishPath();
  void maybeRestoreBloop();
  void maybeUpdateShoot();
  void clearShoot();
  void stopEverything();
  void commandFrame(int dfFrame, bool withDmx);
  void applyProgramDmx(int dfFrame);
  void queueMksFrame(int dfFrame);
  void startArmedPlay();
  void handleShootFrame(const dfdmc::DmcFrame& frame);
  void handleShootFrame2(const dfdmc::DmcFrame& frame);
  void fireBloop(unsigned ms);
  void applyFrameTrigger(int dfFrame);
  void pumpPendingPlay();
  void pumpPath();
  uint32_t psFromFpsX1000(uint32_t fpsX1000) const;
  void handleDmcFrame(const dfdmc::DmcFrame& frame);
  void handleMotorOrRt(const dfdmc::DmcFrame& frame);

  dfdmc::DmcParser dmcParser_;
  dfdmc::DmcGio gio_;
  dfdmc::DmxEngine dmx_;
  dfdmc::PathTable path_;
  MksBus mks_;
  ServoBank servos_;
  StatusLed statusLed_;
  uint32_t lastPositionTxMs_ = 0;
  uint32_t pendingPlayAtMs_ = 0;
  bool dfConnected_ = false;
  bool bootHelloSent_ = false;
  bool pendingPlay_ = false;
  bool armedPlay_ = false;
  bool syncDmx_ = false;
  bool pathActive_ = false;
  bool wasPathActive_ = false;
  int pendingStart_ = 1;
  int pendingEnd_ = 1;
  int currentFrame_ = 1;
  int playEndFrame_ = 1;
  int playDir_ = 1;
  int mksSentFrame_ = -1;
  uint32_t pathSliceUs_ = 41667;
  uint32_t nextFrameMs_ = 0;
  unsigned pendingBloopMs_ = 0;
  uint16_t pendingBloopDmx_ = 0;
  uint16_t bloopDmxChannel_ = 0;
  uint8_t bloopSavedLevel_ = 0;
  uint32_t bloopDmxUntilMs_ = 0;
  bool bloopDmxOn_ = false;
  uint32_t pendingPostrollMs_ = 0;
  uint32_t postrollUntilMs_ = 0;
  bool postrollWaiting_ = false;
  bool shootArmed_ = false;
  bool shootRun_ = false;
  bool shootShutter_ = false;
  bool mksBlurSent_ = false;
  uint32_t shootT0_ = 0;
  uint32_t shootDelayMs_ = 0;
  uint32_t shootAccelMs_ = 0;
  uint32_t shootCruiseMs_ = 0;
  uint32_t shootShutterOpenMs_ = 0;
  uint32_t shootShutterCloseMs_ = 0;
  uint32_t shootDoneMs_ = 0;
  int32_t shootEndSteps_[kAxisCount]{};
  bool shootBlur_[kAxisCount]{};
  bool cdcWasUp_ = false;
};

}  // namespace dfmks
