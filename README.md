# Project-Libelle

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.2525455.svg)](https://doi.org/10.5281/zenodo.2525455)

*Libelle* is a spectrally-resolving shortwave pyranometer that measures solar irradiance across six spectral bands from UV-B through short-wave infrared (~280–1700 nm). The name is German for both "dragonfly" — an organism with up to 30 types of photoreceptors spanning UV to near-IR — and "spirit level," a nod to the onboard accelerometer used to correct measurements for sensor tilt.

![Libelle bare board next to a US quarter for scale](Documentation/images/LibelleBareBoard.jpg)

## Measurements

| Channel | Sensor | Approx. range |
|---------|--------|---------------|
| UV-B | VEML6075 | ~280–315 nm |
| UV-A | VEML6075 | ~315–400 nm |
| Visible (ALS) | VEML6030 | ~400–700 nm |
| White (broadband) | VEML6030 | ~300–700 nm |
| Near-IR (short) | VEMD1060X01 | ~700–1100 nm |
| Near-IR (mid) | SD003-151-001 | ~1000–1700 nm |
| Temperature | NTC thermistor | — |
| Tilt (roll + pitch) | ADXL343 | — |

Spectral response curves for each channel are in [`Analysis/SpectrumResponsePlots/`](Analysis/SpectrumResponsePlots/).

## Hardware

The board integrates its sub-sensors through an ATtiny841, which aggregates their readings and presents a single I²C slave interface to the host. The host never addresses the sub-sensors directly.

**Key ICs:** VEML6075 (UV), VEML6030 (visible/lux), VEMD1060X01 + SD003-151-001 (NIR photodiodes), ADS1115 (16-bit ADC), ADXL343 (accelerometer), ATtiny841 (I²C bridge), TPS79733 (3.3 V LDO)

### Net radiation

Two Libelle modules can share a single I²C bus simultaneously — one facing skyward (UP) and one facing down (DOWN) — to compute net shortwave radiation (↓ − ↑). The I²C address is set by solder jumper JP1:

| Orientation | Address |
|-------------|---------|
| Up (default, JP1 open) | `0x40` |
| Down (JP1 bridged) | `0x41` |

## Electrical

- **Supply:** 3.3–5.5 V (onboard 3.3 V LDO, max input 5.5 V; I²C lines are level-shifted to match host supply)
- **Interface:** I²C (slave)
- **Connectors:** 4-pin screw terminal (J1) and 4-pin headers (J2–J4); ISP header for ATtiny firmware updates (ISP1)
- **JP1:** module I²C address select (open → `0x40`; bridged → `0x41`)
- **JP2:** ADXL343 accelerometer I²C address select (open → `0x1D`; bridged → `0x53`); for advanced use only — firmware assumes `0x1D`

### External connector pinout

| Pin | Function |
|-----|----------|
| VIN | Supply (3.3–5.5 V) |
| GND | Ground |
| SDA | I²C data |
| SCL | I²C clock |

### Interrupt / auxiliary pins

| Pin | Function |
|-----|----------|
| D0 | Lux interrupt (active low) |
| D1 | ADC interrupt (active low) |
| D7 | Address select (pull low → `0x41`) |

## Software

Install the [Libelle Arduino library](https://github.com/NorthernWidget-Skunkworks/Libelle_Library) from the Arduino Library Manager or by cloning the repository.

### Quick start

```cpp
#include <Libelle.h>

Libelle pyroUp(UP);

void setup() {
    Serial.begin(38400);
    pyroUp.begin();
    Serial.println(pyroUp.getHeader());
}

void loop() {
    Serial.println(pyroUp.getString());
    delay(1000);
}
```

For a net radiation setup with two modules:

```cpp
Libelle pyroUp(UP);
Libelle pyroDown(DOWN);
```

### Key API methods

| Method | Returns | Description |
|--------|---------|-------------|
| `begin()` | `bool` | Initialize; returns `false` if bridge or accelerometer unreachable |
| `getHeader()` | `String` | Comma-separated column names with units |
| `getString()` | `String` | Comma-separated measurement values |
| `getUVA()` | `long` | UV-A raw counts |
| `getUVB()` | `long` | UV-B raw counts |
| `getLux()` | `float` | Illuminance (lux) |
| `getIR_Short()` | `float` | NIR ~700–1100 nm, TIA output voltage (V) |
| `getIR_Mid()` | `float` | NIR ~1000–1700 nm, TIA output voltage (V) |
| `getTemp()` | `float` | Housing temperature (°C) |
| `getRoll()` | `float` | Roll angle (°) |
| `getPitch()` | `float` | Pitch angle (°) |

`getIR_Short()` and `getIR_Mid()` return transimpedance amplifier output voltage, not W/m². Converting to irradiance requires calibration against a reference pyranometer; see the [library documentation](https://github.com/NorthernWidget-Skunkworks/Libelle_Library) for details.

## Register map and firmware internals

The Libelle firmware runs on an ATtiny841 bridge, which exposes an I2C peripheral register map to the host logger. Default I2C addresses: `0x4C` (UP orientation, Schema 1 `'L'`) and `0x0C` (DOWN orientation: the UP address XOR `0x40`, selected by the solder jumper); the address stored in Page 0 byte `0x1F` overrides the UP default. Firmware before Schema 1 answered at `0x40` (UP) and `0x41` (DOWN). The onboard ADXL343 accelerometer is read directly by the master on the same I2C bus (not bridged through the ATtiny).

The firmware on `master` (`Firmware/Libelle_Driver_ShortWave`) implements [NW-Device-Specification](https://github.com/NorthernWidget/NW-Device-Specification) Schema 1 (firmware patch 1, 2026-09-23, unreleased and not yet validated on hardware). The layout that the last released firmware exposed is kept below for anyone reading a deployed unit.

### Legacy register map (firmware before Schema 1)

26-byte array. Status ready flag is bit 7.

```
0x00        CTRL/Status   bit 7=ready; bit 2=auto-range disable;
                          bit 3=manual auto-range trigger; bits 1:0=update rate
0x01        —             unused
0x02–0x05   UVA           int32, compensated VEML6075 counts, little-endian
0x06        —             gap (always 0x00) ← ⚠ BUG: see below
0x07–0x0A   UVB           int32, compensated VEML6075 counts, little-endian
0x0B–0x0C   ALS           uint16, raw VEML6030 counts (visible)
0x0D–0x0E   White         uint16, raw VEML6030 counts
0x0F        —             gap
0x10–0x11   Lux mult      uint16, auto-range gain×integration scaler
0x12        —             gap
0x13–0x14   IR_Mid        uint16, raw ADS1115 counts (×1.25e-4 → V)
0x15–0x16   IR_Short      uint16, raw ADS1115 counts (×1.25e-4 → V)
0x17–0x18   Therm         uint16, raw ADS1115 counts (Steinhart-Hart → °C)
```

### ⚠ Known bug: UVB reads 256× too large

The firmware writes UVB starting at register `0x07`. The library reads UVB starting at `UVB_ADR = 0x06`. This off-by-one causes the library to assemble the UVB int32 as:

```
Byte 0 (LSB): Reg[0x06] = 0x00  ← always zero (gap byte)
Byte 1:       Reg[0x07] = UVB true byte 0
Byte 2:       Reg[0x08] = UVB true byte 1
Byte 3 (MSB): Reg[0x09] = UVB true byte 2  ← true byte 3 (Reg[0x0A]) dropped
```

Result: `getUVB()` returns approximately `true_UVB × 256`. All historical UVB data collected with this firmware and library combination is affected by this systematic error. See [issue #TBD](https://github.com/NorthernWidget/Project-Libelle/issues) for tracking.

### Register map (NW-Device-Specification Schema 1)

Two 32-byte pages. The UVB register mismatch is corrected in this layout.

**Page 0 (0x00–0x1F) — Identity (EEPROM)**

```
Block 0 (0x00–0x07)   Core identity
  0x00        0x01                          Schema (NW-Device-Specification v1)
  0x01–0x07   'L','i','b','e','l','l','e'   Device name (7 bytes, exact fit)

Block 1 (0x08–0x0F)   Version
  0x08        HW major
  0x09        HW minor
  0x0A        FW patch          (NW combined-repo convention)
  0x0B–0x0D   0x00,0x00,0x00    Unused (combined repo)
  0x0E–0x0F   0x00,0x00         Reserved

Block 2 (0x10–0x17)   Serial number
  0x10–0x11   0x4C,0x01         Board type ('L'=0x4C, revision index 1)
  0x12–0x13   [manufacture]     Group ID
  0x14–0x15   [manufacture]     Unique ID
  0x16–0x17   0x00,0x00         FirmwareID (legacy, reserved)

Block 3 (0x18–0x1F)   Integrity + administration
  0x18–0x1C   0x00 ×5           Reserved
  0x1D        0x4E              Magic byte
  0x1E        [computed]        CRC-8 of bytes 0x00–0x1D
  0x1F        0x4C or 0x0C      I2C address (0x4C=UP, 0x0C=DOWN; writable)
```

**Page 1 (0x20–0x3F) — Sensor data (SRAM)**

Chip table:

| Index | Chip | Measurements |
|-------|------|--------------|
| 0 | VEML6075 | UVA, UVB |
| 1 | VEML6030 | ambient light, white |
| 2 | ADS1115 | IR short, IR mid, thermistor temperature |
| 3 | ADXL343 | X, Y, Z (hardware v2 only) |

Block 0 (0x20–0x27) is the universal block defined by [NW-Device-Specification](https://github.com/NorthernWidget/NW-Device-Specification#page-1-sensor-data). On Libelle: a reading starts on a trigger (Control `0x21` bit 0) or on the free-running timer that Config `0x26` bits 1:0 select (0 = 5 s, 1 = 10 s, 2 = 60 s, 3 = 300 s); Config bit 2 disables the VEML6030 auto-range and bit 3 runs it once (self-clearing); Control bit 1 selects the VEML6075, bit 2 the VEML6030 and bit 3 the ADS1115; ready (Status `0x20` bit 0) clears while the chips are read and returns with the reading counter (`0x22–0x23`) incremented. Boot latches unit kind 6 (reset), or kind 3 if Page 0 failed its CRC; a chip that does not acknowledge its address during a reading gets its status bit and the latched code kind 1 (no acknowledge); the firmware does not yet check the data a chip returns. The readings-requested word and the sleep bit are accepted without effect: status (ready, per-chip fault bits, pan-fault), control (trigger, chip select, sleep), reading counter, device config byte at 0x26, latched fault code at 0x27. Device data begins at 0x28. Config (0x26): bits 1:0 = update period (0 = 5 s, 1 = 10 s, 2 = 60 s, 3 = 300 s); bit 2 = auto-range disable; bit 3 = run auto-range once (self-clearing); bits 7:4 reserved. Libelle's 26 data bytes exceed Blocks 1–3, so the accelerometer continues on Page 3 (0x60–0x7F).

```
Block 1 (0x28–0x2F)   VEML6030 — visible light
  0x28–0x29   ALS          uint16, raw VEML6030 counts, little-endian
  0x2A–0x2B   White        uint16, raw VEML6030 counts, little-endian
  0x2C–0x2D   Lux mult     uint16, auto-range scaler (ALS × mult × 0.0036 → lux)
  0x2E–0x2F   Reserved

Block 2 (0x30–0x37)   VEML6075 — UV
  0x30–0x33   UVA          int32, compensated counts, little-endian
  0x34–0x37   UVB          int32, compensated counts, little-endian

Block 3 (0x38–0x3F)   ADS1115 — IR + temperature
  0x38–0x39   IR Short     uint16, raw ADC counts (×1.25e-4 → V)
  0x3A–0x3B   IR Mid       uint16, raw ADC counts (×1.25e-4 → V)
  0x3C–0x3D   Temperature  uint16, raw ADC counts (Steinhart-Hart → °C in library)
  0x3E–0x3F   Reserved

Page 3, Block 0 (0x60–0x67)   ADXL343 — accelerometer (hardware v2 only; see below)
  0x60–0x61   Accel X   int16, little-endian
  0x62–0x63   Accel Y   int16, little-endian
  0x64–0x65   Accel Z   int16, little-endian
  0x66–0x67   Reserved
```

No Page 2. Calibration constants (Steinhart-Hart coefficients, UV cross-talk compensation) are currently hardcoded in the library. If per-unit calibration is added, Page 2 is the natural home. Accelerometer calibration offsets, if needed, would also go in Page 2 following the pattern of the Apis sensor.

**Accelerometer (ADXL343):** In the current hardware (v1), the ADXL343 is wired to the master's I2C bus and read directly by the library at address `0x1D` (UP) or `0x53` (DOWN) — it is not bridged through the ATtiny. This requires the logger to manage two I2C addresses. In hardware v2, the ADXL343 will move to the ATtiny's software I2C bus so all data is accessible through a single address; Block 3 of Page 1 is reserved for this. See [issue #19](https://github.com/NorthernWidget-Skunkworks/Project-Libelle/issues/19).

### Migration notes for Schema 1 update

1. **Status bit:** Current firmware uses bit 7 of `Reg[0x00]` as the ready flag; Schema 1 places the status byte at 0x20 with bit 0 as the ready flag. Both the register address and the bit position must change together.
2. **UVB register offset:** Correct firmware to write UVB at `0x28` (Page 1, Block 1) — eliminates the ×256 error.
3. **Auto-range:** `bit 2` and `bit 3` of CTRL need equivalent representation in Schema 1 status/config byte.

## Mechanical

CNC-millable mounting hardware designs are available on [Easel (Inventables)](https://www.inventables.com/):

- [Pipe mount](https://easel.inventables.com/projects/cAirZefIUws53oghGYYopw) — accepts 1.25"–3.25" ID pipe with 1/4" U-bolts (common US sizes: 1.25", 1.5", 1.75", 2")
- [Drilling jig](https://easel.inventables.com/projects/s-fOn9DWTmkeizO0vO8MGw) — for the Libelle enclosure box

| Pipe mount | Enclosure circle cut |
|---|---|
| ![Pipe mount CNC design](Documentation/images/LibellePipeMount.png) | ![Enclosure circle cut CNC design](Documentation/images/LibelleBoxCircleCut.png) |

SolidWorks source files and STLs are in [`Mechanical/`](Mechanical/).

## Related projects

- [Project-Liasis](https://github.com/NorthernWidget-Skunkworks/Project-Liasis) — companion longwave (thermal IR) pyrgeometer
- [Liasis Library](https://github.com/NorthernWidget-Skunkworks/Liasis_Library) — Arduino library for the Liasis pyrgeometer

## NW-Device-Specification — Schema 1, Page 0

Implements [NW-Device-Specification](https://github.com/NorthernWidget/NW-Device-Specification) Schema 1. The 32-byte identity block (Page 0) is stored at the top of EEPROM:

```
Block 0:  Schema=0x01, Name='L','i','b','e','l','l','e'
Block 1:  HW major=[mfr], HW minor=[mfr], FW patch=[mfr], 0x00,0x00,0x00, Reserved
Block 2:  Board type=0x4C01 ('L'=0x4C, rev 1), Group ID=[mfr], Unique ID=[mfr], FirmwareID=0x0000
Block 3:  Reserved, Magic=0x00, CRC=[computed], I2C address=0x4C (UP) / 0x0C (DOWN)
```

Legacy deployed units carry board types `0x2300`/`0x2301` (formerly Dyson SW, Monarch SW).

## License

Hardware: <a rel="license" href="http://creativecommons.org/licenses/by-sa/4.0/"><img alt="Creative Commons License" style="border-width:0" src="https://i.creativecommons.org/l/by-sa/4.0/88x31.png" /></a> <a rel="license" href="http://creativecommons.org/licenses/by-sa/4.0/">Creative Commons Attribution-ShareAlike 4.0 International</a>

Firmware: [GNU General Public License v3](LICENSE_for_code)
