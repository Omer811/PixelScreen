# PixelScreen controller

This repo contains the Arduino firmware, the UART client, the JSON configuration, and the font-powered renderer that work together to drive the mechanical PixelScreen.

## Configuration (python/pixel_screen_config.json)
- Define the UART settings (`serial.port`, `baudrate`, `timeout`) that the Raspberry Pi will use when opening the Arduino. The interactive setup and homing sequences take several seconds while the motors hunt for magnets, so give `timeout` 5–6 s (or more) to avoid spurious `timeout waiting for sync` errors. Add `serial.crc_enabled` (defaults to `true`) when you need to temporarily disable the firmware’s checksum check while debugging the framing.
- Provide the 16-entry `column_encoding` table and `config_version` that are uploaded to the Arduino via `CMD_CONFIGURE (0x04)` before drawing.
- Provide an optional `hardware` section that sets `hall_timeout_ms` (how long the hall sensors keep homing) and `step_interval_ms` (delay between pulses so the driver steps at the same cadence as the bring-up tester). Those values are sent via `CMD_SET_SETTINGS` as soon as the Python client opens the port.
- Set the font dimensions (4×3) and renderer hints such as `start_column`, `max_columns`, and `inter_char_spacing` so the Python helpers can pack the right number of columns.

## Arduino firmware (arduino/PixelScreen.ino)
- `CMD_WRITE_COLUMNS (0x01)` writes `4 × width` bits packed into `(width)` nibbles, stepping each drum forward as needed.
- `CMD_HOME (0x02)` homes every column using the hall sensors and clears the `homed[]` array.
- `CMD_STATUS (0x03)` replies with the homing state plus the currently active configuration version (detail byte = `config_version << 1 | homed_flag`).
- `CMD_CONFIGURE (0x04)` accepts `[version][mapping[0]..mapping[15]]` and updates the `columnToPosition[]` table used by future writes.
- The shift-register chain is now grouped in 3-chip modules: every board reports 24 outputs where the first byte is DIR, the second byte is STEP, and the third remains unused. Column `i`’s driver listens to the STEP bit in module `i//8` while the associated DIR bit sits in the same board’s DIR byte, so hook your 74HC595 modules accordingly (first chip = DIR, second = STEP).
- Responses always follow `FRAME_SYNC 0xAA`, `CMD_RESPONSE 0x10`, `length=2`, `[status][detail]`, CRC8 polynomial `0x07`.

Each drum column stores its current radial index (0-15) and only advances forward, wrapping `0x0F → 0`. The hall sensors are read over two MCP23017 expanders using Adafruit’s `Adafruit_MCP23X17` driver; once per session, the Python renderer uploads the configured column mapping and verifies the version before any drawing commands run. The sketch logs `[WARN] Hall expander 0x21 missing (supporting single-expander mode)` when only one expander is wired, so you can still run the interactive setup with a subset of the hardware and expand the harness later.
The sketch also prints `[INFO]` when each MCP23017 expander responds, `[ERROR]` when an expander is missing (useful when only one is wired), and `[STAGE]` whenever interactive setup reaches a new milestone. These messages appear on the same 115200 baud serial link that the Raspberry Pi uses so you can open the Arduino IDE’s monitor or `screen`/`miniterm` while the Python helper waits for your input.

## Raspberry Pi client (python/pixel_screen_client.py)
Install dependencies:

```bash
pip install pyserial
```

Usage example:

```python
from pixel_screen_client import PixelScreenClient
client = PixelScreenClient("/dev/ttyUSB0")
client.home()
pattern = [0, 1, 2, 3]
client.write_columns(pattern)
client.close()
```

The CLI helper also exposes a simple demonstration:

```bash
python pixel_screen_client.py /dev/ttyUSB0 --width 8
```

Pass `--no-crc` to that helper when the checksum bytes seem to break the handshake; the client keeps CRC disabled until you explicitly re-enable it with `PixelScreenClient.set_crc(True)` or flip `serial.crc_enabled` back to `true` in the JSON.

New APIs:
```

New APIs:
- `configure_column_mapping(mapping: Sequence[int], version: int = 1)` uploads the JSON-defined encoding before other commands run.
- `home()` and `status()` remain available to manage the steppers and check homing state.

## Word renderer (python/pixel_word_renderer.py)
- Loads the shared JSON configuration, feeds `column_encoding` into the Arduino handshake (`CMD_CONFIGURE`), and validates that the reported `config_version` matches before writing.
- Uses the 4×3 pixel font inspired by FontStruct's `4x3_pixel` (https://fontstruct.com/fontstructions/show/325977/4x3_pixel) to convert uppercase letters, digits, space, and `?` into columns of four radial pixels.
- CLI example:

```bash
python pixel_word_renderer.py --config python/pixel_screen_config.json --home HELLO
```

The renderer pads the requested `start_column` with blanks, enforces `max_columns`, and sends the packed `4*w` bits through the shared client.

## Interactive setup (python/pixel_screen_setup.py)
- Runs the Arduino-side homing/mapping flow that homes every drum, verifies direction, measures steps per revolution, and asks for the 16 column bit patterns (the four digits you type are the column pixels from top to bottom; e.g., `1101` where the MSB is the top pixel).
- The script stores the resulting column encoding, step count, and direction in `python/pixel_screen_config.json`, increments `setup.config_version`, and uploads the table (`CMD_CONFIGURE`) so future runs only need to home every stepper (`CMD_HOME`) before writing.
- Use `--force` to rerun the interactive routine even when a previous calibration exists; otherwise the script offers to replay the stored configuration automatically. The saved configuration also lets the host immediately reconfigure the Arduino before drawing, so homing is the only servo work once the dump is captured.
- Pass `--tune` to run the recommended current-pot tuning stage first; it pulses the columns listed in `setup.tune_columns` (default 4 but bounded by `setup.motor_count`) so you can adjust each A4988 before heavy motion. This lets you tune only the drivers that are currently wired and defer the rest until later. Use `--tune-count <n>` if you need more/fewer columns than stored in config.
- Run any helper with `--verbose` (including `pixel_screen_client.py`, `pixel_screen_setup.py`, `tune_a4988_current.py`, `hall_sensor_test.py`, and `pixel_word_renderer.py`) to dump every UART frame the Pi sees; it prints the raw bytes prefixed by `>>>` or `<<<` while the Arduino prints `[DBG]` lines when it encounters CRCs or bad commands. During the interactive setup the sketch now prints `[STAGE]` markers for every milestone (e.g., “Column 0 homed; waiting for acknowledgment”, “Direction confirmed forward”, “Prompting pattern for position 3”, “Pattern for position 3 = 0x0F”, “Interactive encoding stored; re-homed column 0”, “Setup complete; configuration saved”) plus `[INFO]`/`[ERROR]` for expander detection. Watch these messages so you always know what the Arduino is waiting for, even if motors or sensors are disconnected.

## Hall sensor validation (python/hall_sensor_test.py)
- Uses `CMD_ABORT_SETUP` before `CMD_READ_HALL` so the tester can run even if the interactive setup never finished; the Arduino logs `[INFO] Setup abort requested ...` when the abort command arrives. Afterwards it queries each MCP23017 input and prints a 32-bit mask every half second (configurable with `--interval`). Bring a magnet close to each DRV5032; the corresponding bit flips between `.` (no magnet) and `1` (magnet present), letting you confirm wiring before moving on to homing.
  Use `PixelScreenClient.dump_logs()` to pull the `[STAGE]`/`[INFO]` buffers into the Python terminal if you need to inspect what the Arduino just logged (no serial monitor required).
  If the board still ignores a command you can set `serial.crc_enabled` to `false` in the JSON (or pass `--no-crc` to the CLI) so the Python helper issues `CMD_SET_CRC 0x00` as soon as it connects; re-enable the checksum later using `PixelScreenClient.set_crc(True)` when the framing is stable.

### Input to the Arduino
- Each display update consists of `width` nibbles packed into `(4*width)` bits. The host sends them inside `CMD_WRITE_COLUMNS` (0x01) as `[width][payload...]` where each nibble is one column’s on/off pixels and the MSB of the nibble represents the top-most pixel.

See `PROTOCOL.md` for the full command set, framing rules, and the coordinated usage of the Arduino protocol together with the Python helpers.

## Driver tuning helper (python/tune_a4988_current.py)
- Homes the system, then repeatedly issues `CMD_TUNE_COLUMN` bursts to the driver you select so you can adjust the current-limit potentiometer while feeling/listening to the motor. Run it before drawing heavy frames to be sure the drivers are set correctly.
# PixelScreen
