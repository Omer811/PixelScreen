# PixelScreen Wiring Guide

This document explains how the Arduino Mega/Uno, Raspberry Pi, stepper drivers, shift registers, and MCP23017 hall-sensor expanders are wired together for the mechanical pixel screen.

## Key modules
- **Arduino**: drives 32 × A4988 drivers via chained 74HC595 shift registers and polls DRV5032 hall-effect sensors through two MCP23017 I²C expanders using Adafruit's `Adafruit_MCP23X17` driver.
- **Raspberry Pi**: hosts the Python renderer/client, connects to Arduino over USB/serial, and provides any high-level scheduling.
- **Steppers**: 28BYJ-48 motors with geared shafts connect to the drums; only one direction of rotation is allowed.
- **Hall sensors**: DRV5032FBLPGM sensors detect the magnet for each column; inputs are read through MCP23017 expanders (via `Adafruit_MCP23X17`) with internal pull-ups.

## Stepper driver wiring
```
Arduino pin 5V ----+------------------------------------+--------------+ (VCC for A4988 and 74HC595)
                   |                                    |              |
                  GND ---+----------------------------+--+---+--+--+---+--.
                         |                            |      |  |  |  |   |
                     A4988 ENABLE (pin 5)     74HC595 OE 74HC595 MR ....(tie LOW)
```
- Use a separate 5 V supply for the steppers if needed; common ground back to Arduino.
- Tie `A4988 ENABLE`, `74HC595 OE`, and `74HC595 MR` low so the drivers stay enabled.

Each column has one A4988; configure inputs as follows:
- STEP line ← driven from a 74HC595 chain output (one per column).
- DIR line ← driven from a different 74HC595 output (all set to forward). Keep `DIR` pins grouped and preloaded.
- The firmware now assumes each 3-chip board provides 24 outputs (`DIR` bytes, then `STEP`, then unused). For column `i`, the driver expects the corresponding DIR bit to sit in the first byte of the board that handles `i // 8`, while the second byte supplies the STEP pulses. Tie the third byte’s `LDEN`/OE pins low to leave it unused.
- `SR_DATA_PIN` (pin 2), `SR_CLOCK_PIN` (pin 3), `SR_LATCH_PIN` (pin 4) must be wired to the 74HC595 serial input and control pins.

## Hall sensor wiring
- Wire the two MCP23017 expanders (accessed via the Adafruit_MCP23X17 driver) to the Arduino I²C bus (SDA/SCL). Address jumpers set to `0x20` and `0x21` by tying A0-A2 low/high accordingly.
- Each Hall sensor output connects to one MCP23017 input pin.
- Pull each input up inside the Arduino code (`pullUp(pin, HIGH)`), then read them to detect magnets (active `LOW`).
- DRV5032 sensors require 5 V and ground (shared with Arduino).

## Step-by-step wiring checklist
1. **Power & ground** – tie the Arduino’s 5 V/VIN to the 5 V bus that feeds every A4988, 74HC595, MCP23017, and DRV5032. Tie all grounds together (Arduino, motor supply, expanders, Raspberry Pi). After the bus is stable run `python pixel_screen_setup.py --config python/pixel_screen_config.json --tune` because you should tune at least the attached drivers before moving on.
2. **Shift-register chain** – connect `D2→SER`, `D3→SRCLK`, `D4→RCLK` and chain the 74HC595 outputs. Tie OE/MR/SRCLR low. Confirm the chain by clocking a known pattern and watching an LED on QA..QH toggle with the latch.
3. **Stepper drivers** – feed the first 32 outputs into the A4988 DIR pins and the second 32 into STEP. Tie every A4988 `RESET`/`SLEEP` high, `ENABLE` low (Arduino D5), and feed VMOT/VDD with 5 V, GND common. Each driver now listens to a column’s STEP signal while DIR stays forward.
4. **Hall expanders** – each MCP23017 sits on SDA/SCL with address jumpers set to `0x20`/`0x21`. During boot the firmware prints `[INFO] Hall expander 0x2X available` for every responsive chip; if one is missing you’ll see `[WARN] Hall expander 0x21 missing (supporting single-expander mode)` but the board keeps working with any remaining expander(s), so you can calibrate with a single sensor and add the second expander later.
5. **Hall sensors** – connect each DRV5032 VCC to 5 V, OUT to the MCP23017 inputs, and GND to the common ground. Run `python hall_sensor_test.py --config python/pixel_screen_config.json` (works even with a single sensor wired) and move a magnet near the expected column; the corresponding bit toggles from `.` to `1` while the others stay `.`.
6. **Serial console** – plug the Raspberry Pi into the Arduino’s USB port. Keep a serial monitor at 115200 baud open while running `python pixel_screen_setup.py`; the sketch prints `[STAGE]` messages that tell you when each column homed, when direction/step count is expected, and when to type the 4-bit pattern.

## Testing & diagnostics
- Use `python hall_sensor_test.py --config python/pixel_screen_config.json --interval 0.5` to poll all available hall inputs; the helper logs a 32-bit mask so you can confirm each sensor’s bit flips when a magnet is nearby, even if only a subset of sensors or expanders is physically connected.
- The script begins by sending `CMD_ABORT_SETUP` (visible as `[INFO] Setup abort requested ...` in the serial log) so you can run it even when the system was left mid-calibration.
- The interactive setup prints `[STAGE]` logs whenever it waits for acknowledgments, direction confirmations, steps, and bit patterns. Watch these logs when running `pixel_screen_setup.py` (or while manually entering data via serial) to avoid timeouts and know exactly what input the Arduino expects next.
- Run `python tune_a4988_current.py --config python/pixel_screen_config.json --column N` per driver while tuning its current pot; the helper issues `CMD_TUNE_COLUMN` bursts so you can feel/listen to each motor without moving the full frame.

## Arduino pin summary
- **Digital 2 (SER)** → 74HC595 first register `SER` input.
- **Digital 3 (SRCLK)** → all 74HC595 `SRCLK` inputs.
- **Digital 4 (RCLK)** → all 74HC595 `RCLK` latches.
- **Digital 5 (ENABLE)** → A4988 `ENABLE` line floored LOW.
- **SDA/SCL** → MCP23017 expanders (0x20/0x21) managed through `Adafruit_MCP23X17`.
- **5V/VIN** → 74HC595 VCC, MCP23017 VCC, DRV5032 VCC, A4988 VDD.
- **GND** → common ground for steppers, sensors, shift registers, and Raspberry Pi.

## Wiring diagram (simplified view)
```
             +----------+             +------------+
   USB ——>   | Arduino  |             | Raspberry  |
   Serial    |          |<—USB Serial—| Pi         |
             +----------+             +------------+
        |        |   |   \__I2C (SDA/SCL)\
        |        |   |                     \
        |        |   +--> MCP23017 #1 (pins 0-15 -> hall sensors 0-15)
        |        |         "-+--> DRV5032 column 0..15
        |        |   +--> MCP23017 #2 (pins 0-15 -> hall sensors 16-31)
        |        |         "-+--> DRV5032 column 16..31
        |        |
        |        +--> 74HC595 chain (64 outputs)
        |             |-> top 32 bits = DIR (preloaded forward)
        |             "-> bottom 32 bits = STEP signals to each A4988 driver
        |
        +--> 32 × A4988 drivers (powered by 5 V, tied enable low)
             |-> each drives one 28BYJ-48 (STEP from shift register, DIR steady)
```

### Complete pinout tables

| Device | Pin | Connected to |
| --- | --- | --- |
| Arduino | D2 (SER) | 74HC595 first register `SER` |
| Arduino | D3 (SRCLK) | All 74HC595 `SRCLK` pins |
| Arduino | D4 (RCLK) | All 74HC595 `RCLK` pins |
| Arduino | D5 (ENABLE) | A4988 `ENABLE` pin tied LOW |
| Arduino | SDA/SCL | MCP23017 expanders with addresses 0x20/0x21 |
| Arduino | 5V/VIN | 74HC595 VCC, MCP23017 VCC, DRV5032 VCC, A4988 VDD |
| Arduino | GND | Shared ground with drivers and Pi |
| 74HC595 | OE/MR/SRCLR | Tie to GND (enable outputs, release resets) |
| 74HC595 | QA-QH (1st/2nd chip) | DIR bits for columns 1-16 (held LOW for forward) |
| 74HC595 | QA-QH (3rd/4th chip) | STEP bits for columns 1-32 (pulsed) |
| A4988 | STEP | SHIFT register STEP output per column |
| A4988 | DIR | Corresponding DIR output (static) |
| A4988 | ENABLE | Arduino D5 low |
| A4988 | RESET/SLEEP | Tie together, keep HIGH or connect to 5V |
| A4988 | VMOT | 5V motor power to 28BYJ-48 |
| A4988 | VDD | 5V logic supply |
| A4988 | GND | Ground |
| DRV5032 | VCC | 5V |
| DRV5032 | OUT | MCP23017 input (with pull-up) |
| MCP23017 | GPA0-GPA7 / GPB0-GPB7 | DRV5032 hall sensors (columns 0-15 & 16-31) |
| MCP23017 | A0-A2 | Address pins (set for 0x20 and 0x21) |
| MCP23017 | SDA/SCL | Arduino I²C |
| Raspberry Pi | USB | USB-serial connection to Arduino |
| Raspberry Pi | GND | Shared ground |

### Notes
- Ensure the `74HC595` outputs are latched after updating both STEP and DIR bits.
- Keep `DIR` bits static at forward; only toggle STEP pulses for movement.
- Run the homing routine (`CMD_HOME`) after applying power so each column knows the magnet position.
- Use the JSON config to capture the 16-entry column mapping that the Arduino uses for future updates.
