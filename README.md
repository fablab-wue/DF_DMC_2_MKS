# DF_DMC_2_MKS

**Dragonframe DMC v2 on a Waveshare RP2040-Zero** — eight MKS SERVO42/57D closed-loop steppers on one RS485 bus, plus eight hobby servos.

## About

**DF_DMC_2_MKS** lets [Dragonframe](https://www.dragonframe.com/) Arc drive MKS drives and hobby servos from one USB port. The PC speaks binary **DMC v2**. Motors 1–8 are the steppers (encoder counts). Motors 9–16 are PWM servos. DMX512 leaves on GP0 only. The servo pins are not a DMX mirror.

**Documentation:** [docs/README.md](docs/README.md)

Shared connect, features, limits, GIO, and pin levels: [DF_DMC_Common user manual](https://github.com/fablab-wue/DF_DMC_Common/blob/main/docs/manual.md).

```text
Dragonframe (PC)
        USB CDC  — binary DMC v2  (device type dmc-lite)
DF_DMC_2_MKS  (RP2040-Zero)
        RS485 256000   GP12 TX / GP13 RX / GP11 DIR   — MKS addr 1..8
        PWM                GP1..GP8                 — hobby servos
        DMX_TX             GP0                      — universe only
```

---

## Features

- **Connect as dmc-lite** — hello name `jDF-MKS V1 8M+8S+4O+2I+CT+DMX`, 16 axes
- **Steppers** — 1 DMC step = 1 encoder count, 16384 counts per revolution. Speed and accel from Arc become RPM and the drive’s 0–255 accel code
- **Servos** — same pulse scale as DF_DMC_2_PWM, about −20000..+20000 around 1.5 ms
- **Live GIO** — 4 outputs (GP29, GP28, GP27, GP26, open-collector, active low) and 2 inputs (GP15, GP14, pull-up, active low)
- **DMX512** — 512 live channels on GP0. A move can store 32 channels and play them with the playhead
- **Camera / buzzer** — GP9 shutter (open-collector, active low), GP10 buzzer (active high)
- **Path upload** — 1440 frames. Play waits for Dragonframe’s go
- **Go motion** — PWM servos follow the requested trapezoid. Each MKS blur axis gets one linear RPM move. The shutter follows Dragonframe’s clock

---

## Quick start (VS Code)

1. Install [VS Code](https://code.visualstudio.com/) and the **PlatformIO IDE** extension.
2. Clone [DF_DMC_Common](https://github.com/fablab-wue/DF_DMC_Common) next to this repository (`../DF_DMC_Common`).
3. **File → Open Folder** → this repository (`DF_DMC_2_MKS`).
4. PlatformIO: **Build** / **Upload**.
5. Dragonframe: Scene → Connections → device type **dmc-lite** → this COM port → Connect.
6. In Arc, set steps per unit per axis. Servo axes 9–16: steps per unit 1, and a test keyframe of 0 and 15000.

USB CDC is binary DMC. Do not use the PlatformIO serial monitor as a console. Details: [docs/build.md](docs/build.md).

---

## Documentation

| Topic | Document |
|-------|----------|
| Axes, units, go-motion, LED | [docs/overview.md](docs/overview.md) |
| GPIO, pinout, MAX485 | [docs/pins.md](docs/pins.md) |
| Build / flash | [docs/build.md](docs/build.md) |
| Connect, features, limits, GIO, all three boards | [DF_DMC_Common user manual](https://github.com/fablab-wue/DF_DMC_Common/blob/main/docs/manual.md) |
| Official DMC v2 | [DMC-Protocol-2024-08-13.pdf](https://www.dragonframe.com/download/dmcproto/DMC-Protocol-2024-08-13.pdf) |
| Dragonframe user manual | [Using Dragonframe 2025](https://www.dragonframe.com/download/Using%20Dragonframe%202025.pdf) |

---

## Repository layout

```text
src/config.h       Pins, 8+8 axes, 256000 baud
src/mks_bus        RS485 master (F5, 31H, sync 4A/4B)
src/servo_bank     PWM on GP1–GP8
src/bridge         USB DMC dispatch
src/status_led     WS2812 on GP16
src/main.cpp       setup() / loop()
```

---

## License

Copyright (c) 2026 Jochen Krapf \<jk@nerd2nerd.org\>

Licensed under the [MIT License](LICENSE).

Company names and product names mentioned in this project are trademarks or registered trademarks of their respective owners. Use here is for identification only.
