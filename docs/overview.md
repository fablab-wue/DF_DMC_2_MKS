# DF_DMC_2_MKS overview

[← Index](README.md)

**DF_DMC_2_MKS** is a USB **DMC v2** device for Dragonframe Arc. It runs on a **Waveshare RP2040-Zero**.

```text
Dragonframe (PC)
        USB CDC  — binary DMC v2  (device type dmc-lite)
DF_DMC_2_MKS  (RP2040-Zero)
        RS485 256000   GP12 TX / GP13 RX / GP11 DIR   — MKS addr 1..8
        PWM                GP1..GP8                 — hobby servos
        DMX_TX             GP0                      — universe only
```

Motors **1–8** are MKS closed-loop steppers (`F5` absolute encoder counts). Motors **9–16** are the PWM servos. Hello reports 16 axes: `jDF-MKS V1 8M+8S+4O+2I+CT+DMX`.

## Units

MKS: **1 DMC step = 1 encoder count** (16384 counts per motor revolution). Set Arc steps-per-unit per axis from the mechanics. Speed and acceleration from Dragonframe are quantized to integer RPM (0–3000) and an acceleration code 0–255. The drive ramps linearly in RPM: each 1 RPM step takes `(256 - acc) * 50 µs`.

PWM servos use the same step scale as DF_DMC_2_PWM: raw counts around a 1.5 ms pulse, about −20000..+20000. A normal move slews. There is no acceleration ramp on that slew.

For a servo test in Arc, set steps per unit to 1 and keyframe 0 and 15000. Full travel is about ±20000 steps around the 1.5 ms pulse. A curve of 0–90 stays inside the deadband, so the horn does not move. MKS axes stay 1 count = 1 encoder count, 16384 per revolution; set steps per unit from the mechanics.

While a path is playing, servo pulses and stored DMX follow the frame clock even if the RS485 queue is still busy. Stepper frames catch up to the latest frame when the bus is free. Position polls pause for that time so the bus can carry the moves.

## Go-motion

Both `SHOOT_FRAME` and `SHOOT_FRAME2` are advertised. PWM servos with the blur flag follow the Dragonframe accel and cruise times. Each MKS blur axis gets one linear RPM ramp, not a one-second trapezoid. The shutter on GP9 still opens and closes on Dragonframe’s clock. Do not compare a stepper blur test with a one-second trapezoid.

## Status LED

The board LED is the internal **WS2812 on GP16**.

| Color | Meaning |
|-------|---------|
| Magenta pulse | Boot |
| Blue pulse | Waiting for Dragonframe |
| Cyan pulse | USB is up, RS485 init not finished |
| Green pulse | At least one MKS drive answered |
| Cyan flash | DMC frame |
| Red pulse | Init finished and no drive answered |

## Firmware `src/`

| File | Role |
|------|------|
| `config.h` | Pins, 8+8 axes, 256000 baud |
| `mks_bus` | RS485 master (`F5`, `31H`, sync `4A`/`4B`) |
| `servo_bank` | PWM on GP1–GP8 |
| `bridge` | USB DMC dispatch |
| `status_led` | WS2812 |
| `main.cpp` | `setup()` / `loop()` |

Framing, the path table, DMX transmit, and GIO come from DF_DMC_Common. USB is binary DMC. Do not print text on `Serial`.
