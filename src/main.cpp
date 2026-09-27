#include "bridge.h"

namespace {
dfmks::DmcBridge bridge;
}

void setup() { bridge.setup(); }

void loop() { bridge.loop(); }
