# PixelScreen Serial & Python Protocol

## Arduino command set
| Command | Code | Payload | Description |
| --- | --- | --- | --- |
| `CMD_WRITE_COLUMNS` | `0x01` | `[width][4*width bits]` | Write that many columns (packed nibble per column). MSB of each nibble = top pixel. Arduino assumes homed before accepting. |
| `CMD_HOME` | `0x02` | `[]` | Run the hall-sensor homing sweep for every column, leaving each just past the magnet. |
| `CMD_STATUS` | `0x03` | `[]` | Arduino replies `CMD_RESPONSE` (`0x10`) with `[status=0][detail=config_version<<1 | homed_flag][extra=steps_low][steps_high][dir_flag]`. |
| `CMD_CONFIGURE` | `0x04` | `[version][mapping0..15]` | Uploads the 16-entry column-to-detent table that `CMD_WRITE_COLUMNS` uses; send this from the Python client whenever a saved setup exists. |
| `CMD_SETUP_START` | `0x05` | `[]` | Begin the interactive calibration flow (homing, direction check, rev count, 16 positions). Arduino emits prompts described below. |
| `CMD_SETUP_ACK_COLUMN` | `0x06` | `[column_index]` | Sent by Python immediately after the Arduino acknowledges column `n` homing during setup. |
| `CMD_SETUP_DIRECTION` | `0x07` | `[0=reverse|1=forward]` | Tells the Arduino whether the first micro-steps were in the expected direction. |
| `CMD_SETUP_POSITION` | `0x08` | `[ascii bits...]` | Provides up to four ASCII `0`/`1` characters that encode the on/off pixels for the current calibration position (top pixel = MSB). |
| `CMD_TUNE_COLUMN` | `0x09` | `[column][steps_low][steps_high]` | Pulse the requested column forward a number of micro-steps. Helps you tweak the driver's current pot while feeling the motor. |
| `CMD_READ_HALL` | `0x0A` | `[]` | Returns 32 bits describing each hall switch (1=magnet present). Enables the Python hall sensor tester. |
| `CMD_ABORT_SETUP` | `0x0B` | `[]` | Tell the firmware to leave the interactive calibration flow and return to idle (responds even if setup already completed). |
| `CMD_DUMP_LOGS` | `0x0C` | `[]` | Fetches the buffered `[STAGE]/[INFO]/[DBG]` lines; the response payload (after the status/detail bytes) is newline-delimited ASCII. |
| `CMD_SET_CRC` | `0x0D` | `[0=disable|1=enable]` | Enable/disable CRC checking on the Arduino; helpful when the framed packet isn’t making it past `processSerial()`. |
| `CMD_SET_SETTINGS` | `0x0E` | `[hall_timeout_low][hall_timeout_high][step_interval_low][step_interval_high]` | Configure the hall-sensor timeout (ms) and step interval (ms) so the firmware matches the bringup tester’s cadence. |

### Interactive setup prompts
When the Arduino is in setup mode it replies with `CMD_RESPONSE` frames (`status=0`) whose `detail` byte indicates what it waits for:
- `0x80 + col`: A homed column `col` is ready → send `CMD_SETUP_ACK_COLUMN` with `col`.
- `0x81`: Confirm direction; respond with `CMD_SETUP_DIRECTION`.
- `0x82`: The Arduino reports the measured revolution length; the `extra` bytes are `[steps_low][steps_high]`.
- `0x90 + pos`: Move to the `pos`th of 16 equidistant detents and send its 4-bit pattern (`CMD_SETUP_POSITION`).
- `0xF0`: Calibration complete; final extra bytes echo the revolution count and direction flag.

The Arduino now holds the newly discovered `columnToPosition[]` table and will resume normal command processing. Use the Python setup script to capture the 16 patterns so future runs simply call `CMD_CONFIGURE` + `CMD_HOME` instead of rerunning the interactive dance.

## Python helper scripts
1. `pixel_screen_setup.py` – runs the interactive calibration or replays a saved one:
   - Reads `python/pixel_screen_config.json` for the serial port and optional `setup` block; if `setup.completed` is `true` and `--force` is not used, it just uploads the saved `setup.column_encoding` via `CMD_CONFIGURE` and issues `CMD_HOME`.
   - Otherwise it triggers `CMD_SETUP_START`, follows the Arduino prompts, saves the 16 bit patterns, step count, direction, and increments `setup.config_version`. After saving, the script reprovisions the Arduino (`CMD_CONFIGURE` + `CMD_HOME`) so the new mapping is ready for future runs.
   - Pass `--tune` to run the optional current-limit tuning that pulses `setup.tune_columns` drivers (bounded by `setup.motor_count`) before the rest of the calibration, ensuring each A4988 pot is set even if the full array isn’t attached yet.
2. `pixel_screen_client.py` – general UART client used by both setup and rendering scripts; it frames commands with CRC8 (poly `0x07`), exposes `configure_column_mapping()`, `write_columns()`, `home()`, `status()`, and the new `start_interactive_setup()` for the calibration flow.
3. `pixel_word_renderer.py` – loads the same `python/pixel_screen_config.json`, ensures the stored `setup.column_encoding` is fed to the Arduino (`CMD_CONFIGURE`), homing is performed, and then writes words (4×3 font) via `CMD_WRITE_COLUMNS`.
4. `tune_a4988_current.py` – homes, then issues `CMD_TUNE_COLUMN` bursts on whichever column you select so you can adjust the A4988 current-limit potentiometer by feel/aural cues (set the number of micro-steps per burst and repeat until the motor spins smoothly without skipping). Use this before final tuning so you don’t overheat the drivers when the screen is under load.
5. `hall_sensor_test.py` – polls `CMD_READ_HALL` every half-second (configurable) and prints the 32-bit hall mask so you can slide magnets over each sensor while the script confirms which inputs change.

## Configuration file
The shared `python/pixel_screen_config.json` contains:
```json
{
  "serial": {"port": "/dev/ttyUSB0", "baudrate": 115200, "timeout": 1.0, "crc_enabled": true},
  "renderer": {"start_column": 0, "max_columns": 32, "inter_char_spacing": 1},
  "setup": {
    "motor_count": 32,
    "tune_columns": 4,
    "config_version": 1,
    "column_encoding": [0, 1, 2, ..., 15],
    "position_patterns": ["0000", ..., "1111"],
    "steps_per_revolution": 0,
    "direction_forward": true,
    "completed": false
  }
}
```
Run `python pixel_screen_setup.py --config python/pixel_screen_config.json` to calibrate; once done, the script saves the discovered `column_encoding` (and increments `config_version`) so you can later call `python pixel_word_renderer.py --config ...` and the Arduino will just need a `CMD_HOME` before writes.
Set `setup.motor_count` to the total number of steppers wired in your build (even if only a subset is physically attached during tuning), and adjust `setup.tune_columns` if you need to pulse more or fewer drivers during the `--tune` stage.
Add a `hardware` block when you want to tune the timing: `hall_timeout_ms` sets how long each column waits for a hall sensor event, and `step_interval_ms` gives the pause between pulses so the firmware steps at the same cadence as the bring-up tester. The Python helpers send these values via `CMD_SET_SETTINGS` before running the first real command.
Set `serial.crc_enabled` to `false` when you suspect the framed packets are being mangled; the client calls `CMD_SET_CRC` as soon as it connects so you can test without CRC and flip the flag back to `true` once the bytes are healthy.

### Response status codes
The Arduino uses the `STATUS_*` codes defined in the firmware to indicate problems:
| Code | Meaning |
| --- | --- |
| `0x00` | `STATUS_OK` – command succeeded. |
| `0x01` | `STATUS_ERR_WIDTH` – malformed payload or invalid column/stepper index. |
| `0x02` | `STATUS_ERR_NOT_HOMED` – a write was requested before the drivers had homed. |
| `0x03` | `STATUS_ERR_CRC` – a protocol frame arrived with a bad checksum. |
| `0x04` | `STATUS_ERR_TIMEOUT` – a homing or setup stage timed out (e.g., hall sensor never triggered). |
| `0x05` | `STATUS_ERR_BUSY` – the interactive flow expected a different response, so it aborted to avoid state corruption. |

### Diagnostic logging
- Watch the serial monitor (115200 baud) while calibrating. The sketch prints:
- `[INFO]` when each MCP23017 expander successfully initializes.
- `[WARN]` when an expected expander is missing (e.g., `Hall expander 0x21 missing (supporting single-expander mode)`); the sketch keeps reading whatever expanders are available so single-expander builds are still supported.
- `[ERROR]` whenever a column fails to home or an interactive prompt receives an out-of-order response, so you can see exactly which step needs rerunning.
- `[STAGE]` before every setup milestone: after columns home, before each direction/position prompt, after steps are measured, and once the final encoding is stored. These logs guarantee you can tell if the sketch is waiting for input (no timeouts are used) or if something silently failed.
- `[DBG]` when `DEBUG_SERIAL` is enabled, showing raw request/response frames for CRC troubleshooting.
- Call `PixelScreenClient.dump_logs()` to poll `CMD_DUMP_LOGS` and show any captured debug lines while the Python helper still owns the port.

During hall-sensor troubleshooting use `python hall_sensor_test.py --config python/pixel_screen_config.json --verbose` to read the same status bits the Arduino uses; absent expanders will simply keep their bits `0`.
