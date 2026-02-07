# PixelScreen bring-up checklist

Follow this step-by-step guide for wiring, testing, and booting the system. **Do not move to the wiring steps until the current limiting of every A4988 is tuned**—rapid motion with untuned drivers risks overheating the steppers. Each stage ends with a simple Arduino/serial check so you can confirm the connections before moving on to the next module.

## 0. Current-limit tuning (before wiring everything)
- **Why now**: an untuned A4988 pot can push too much current, so the first action should be configuring the current limit on whichever columns are wired at the moment rather than waiting until the whole frame is connected.
- **Tool**: run `python pixel_screen_setup.py --config python/pixel_screen_config.json --tune` (add `--tune-count <n>` to limit how many drivers are pulsed). The helper references `setup.motor_count` to know the total motors your build has while allowing you to tune only a subset for now, so you can calibrate a single driver and skip the rest until later.
- **Confirm**: each column moves smoothly and feels consistent without heat or skipping; note the pot positions so you can carry them over to the remaining drivers and proceed to the wiring steps.
- **Manual alternative**: if you prefer to tune one driver at a time, run `python tune_a4988_current.py --config ... --column N` after connecting that column.

## 1. Power & common ground
- **Wire**: connect your Arduino 5 V (or VIN if you regulate separately) to the 5 V bus used by each A4988, 74HC595, MCP23017, and DRV5032 sensor. Tie all grounds (Arduino, stepper motor supply, MCP23017, DRV5032, Raspberry Pi) together to avoid floating references.
- **Test**: power up briefly and use a multimeter to ensure the 5 V rail is stable; confirm the Arduino USB LED is lit. There is no sketch needed yet, but you can open the serial monitor; the board should enumerate normally.

## 2. Shift-register control wiring
- **Wire**: attach `SR_DATA_PIN` (Arduino D2) to the SER input of the first 74HC595, `SR_CLOCK_PIN` (D3) to SRCLK on every register, and `SR_LATCH_PIN` (D4) to RCLK. Tie all OE/MR/SRCLR pins low so outputs are enabled. Chain as: SER → QA..QH → SER' of next chip for both DIR and STEP blocks (total 64 outputs, 32 DIR + 32 STEP).
- **Test**: flash this minimal Arduino sketch to cycle a single SHIFT register output:
  ```cpp
  #define DATA_PIN 2
  #define CLOCK_PIN 3
  #define LATCH_PIN 4

  void setup() {
    pinMode(DATA_PIN, OUTPUT);
    pinMode(CLOCK_PIN, OUTPUT);
    pinMode(LATCH_PIN, OUTPUT);
  }

  void loop() {
    digitalWrite(LATCH_PIN, LOW);
    for (int bit = 0; bit < 8; ++bit) {
      digitalWrite(CLOCK_PIN, LOW);
      digitalWrite(DATA_PIN, bit % 2 ? HIGH : LOW);
      digitalWrite(CLOCK_PIN, HIGH);
    }
    digitalWrite(LATCH_PIN, HIGH);
    delay(500);
  }
  ```
  Hook an LED (with resistor) to any QA-QH pin and see it toggle every 500 ms. This proves the shift chain and latch pulses work. After the test, revert to the main sketch.

## 3. Stepper driver & step signals
- **Wire**: each A4988 gets a STEP signal from the 74HC595 STEP outputs and a DIR signal from the DIR outputs. Tie `ENABLE` to Arduino D5 (low = enabled). VDD/VMOT go to the 5 V rail, GND common.
- **Test**: use the built-in `pulseStep()` helper from `PixelScreen.ino` by temporarily adding:
  ```cpp
  void testSingleStepper() {
    for (int i = 0; i < 100; ++i) {
      pulseStep(0); // column 0
    }
  }
  void loop() { testSingleStepper(); delay(1000); }
  ```
  After uploading, the first column’s drum should rotate forward slightly; feeling for motion verifies the outputs. Remove test code afterward.

## 4. MCP23017 + hall sensors
- **Wire**: daisy-chain both MCP23017 expanders on SDA/SCL (A4/A5 on UNO). Set address pins to 0x20 (A0/A1/A2=0) and 0x21 (A0=1). Connect DRV5032 OUT wires to GPA0-GPA7/GPB0-GPB7 for total 32 inputs, powering DRV5032 with the 5 V rail and tying their grounds to Arduino. Keep pull-ups enabled in code.
- **Test**: use this Arduino snippet inside `setup()` to read hall inputs:
  ```cpp
  void checkHallInputs() {
    for (int chip = 0; chip < 2; ++chip) {
      for (int pin = 0; pin < 16; ++pin) {
        int state = hallExpanders[chip].digitalRead(pin);
        Serial.printf("EXP%u P%u=%d\n", chip, pin, state);
      }
    }
  }
  ```
  Run the sketch with the drums near magnets to see the sensor readings change; the serial log should flip between LOW/HIGH as magnets approach.
- **Alternative (Python)**: run `python hall_sensor_test.py --config python/pixel_screen_config.json` to repeatedly query `CMD_READ_HALL`. Each printed row shows eight columns (`. = no magnet`, `1 = magnet detected`); move magnets near each sensor to ensure its bit changes and the wiring is correct before homing. The helper can also keep reading while you manually rotate drums to ensure consistent detection.
  The firmware prints `[WARN] Hall expander 0x21 missing (supporting single-expander mode)` when only the first expander is present; that warning is expected and the system continues operating, so you can test each hall sensor alone before hooking up the rest.
  Use this script even when only one expander or a single sensor is wired; the bit for the connected sensor will flip while the others stay `.` so you can test isolation before the full 32-column harness exists.

## 5. Stepper + hall integrated homing
- **Wire**: ensure each A4988 is connected to its drum encoder/hall sensor. Trigger a single hall sensor manually and confirm the Arduino sees it by deploying the `runInteractiveSetup()` routine (via `pixel_screen_setup.py`); you'll be prompted for each column once it homes.
- **Test**: call `python pixel_screen_setup.py --config python/pixel_screen_config.json --force` while connected; when the Arduino prompts with `Column n homed`, the script will ask you to confirm by typing the column index. If it reports `SETUP_DETAIL_STEPS`, the hall sensor triggered and the Arduino measured a full revolution.
- **Monitor**: keep a serial console (115200 baud) open on the Arduino while the setup helper runs; `[STAGE]` lines show exactly when a column homed, when direction or step count is expected, and when a pattern should be sent, so you can proceed even without physical motors attached. `[INFO]`/`[ERROR]` messages also reveal whether each MCP23017 expander was detected.

## 6. Final integration
- **Wire**: confirm Raspberry Pi USB is connected to the Arduino. Ensure the `pixel_screen_config.json` contains the latest calibration (`setup.completed = true`).
- **Test**: run `python pixel_word_renderer.py --config python/pixel_screen_config.json --home HELLO` to verify the end-to-end flow, ensuring the Arduino now just needs homing and can draw letters using the saved encoding.

Always revert temporary test code and keep the main firmware (`PixelScreen.ino`) as your production sketch. Use `PROTOCOL.md` for framing rules, and the Python helpers for configuration and rendering once wiring is validated. 
