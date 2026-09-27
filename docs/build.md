# Build

[← Index](README.md)

PlatformIO, same core as DF_DMC_2_MC: earlephilhower Arduino on `board = pico` (the Zero uses that upload). `lib_deps` links `../DF_DMC_Common`, so that repo must sit next to this one.

```text
pio run -e rpipico
pio run -e rpipico -t upload
```

`DFDMC_MAX_AXES=16`. The path table is 16 × 1440 int32 steps, plus up to 32 DMX channels on those frames.

USB CDC is binary DMC. The serial monitor is not a text console.

The status LED is the onboard WS2812 on GP16 (Adafruit NeoPixel, one pixel, GRB 800 kHz).
