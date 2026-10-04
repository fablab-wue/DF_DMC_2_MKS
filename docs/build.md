# Build

[← Index](README.md)

PlatformIO, same core as DF_DMC_2_MC: earlephilhower Arduino on `board = pico` (the Zero uses that upload). `lib_deps` links `../DF_DMC_Common`, so that repo must sit next to this one before a source build.

## Flash a release

No compiler and no PlatformIO. Board: **Waveshare RP2040-Zero**.

1. Download `DF_DMC_2_MKS-<tag>-rp2040zero.uf2` from the [Releases](https://github.com/fablab-wue/DF_DMC_2_MKS/releases) page.
2. Hold **BOOTSEL**, plug in USB, then release BOOTSEL.
3. Copy the UF2 onto the `RPI-RP2` drive. The board reboots into the new firmware.

A new file is built when a `v*` tag is pushed. Rebuild an existing tag from the Actions page with **Run workflow**.

## Source build

```text
pio run -e rpipico
pio run -e rpipico -t upload
```

`DFDMC_MAX_AXES=16`. The path table is 16 × 1440 int32 steps, plus up to 32 DMX channels on those frames.

USB CDC is binary DMC. The serial monitor is not a text console.

The status LED is the onboard WS2812 on GP16 (Adafruit NeoPixel, one pixel, GRB 800 kHz).
