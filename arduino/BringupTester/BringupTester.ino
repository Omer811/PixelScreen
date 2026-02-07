#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include <string.h>

#define FRAME_SYNC 0xAA
#define STATUS_OK 0x00

const uint8_t SR_DATA_PIN = 2;
const uint8_t SR_CLOCK_PIN = 3;
const uint8_t SR_LATCH_PIN = 4;
const uint8_t A4988_ENABLE_PIN = 5;
const unsigned long STEP_PULSE_US = 600;
const unsigned long STEP_GAP_US = 200;
const uint8_t HALL_EXPANDER_ADDRESSES[] = {0x20, 0x21};
const uint8_t HALL_EXPANDER_COUNT = sizeof(HALL_EXPANDER_ADDRESSES) / sizeof(HALL_EXPANDER_ADDRESSES[0]);
Adafruit_MCP23X17 hallExpanders[HALL_EXPANDER_COUNT];
bool hallExpanderActive[HALL_EXPANDER_COUNT];

uint64_t stepState = 0;
uint64_t dirState = ~uint64_t(0);

const uint8_t NUM_COLUMNS = 32;
const uint8_t SHIFT_CHIPS_PER_BOARD = 3;
const uint8_t SHIFT_BYTES_PER_BOARD = SHIFT_CHIPS_PER_BOARD;
const uint8_t SHIFT_BOARD_COUNT = (NUM_COLUMNS + 7) / 8;
const uint8_t SHIFT_TOTAL_BYTES = SHIFT_BOARD_COUNT * SHIFT_BYTES_PER_BOARD;

enum Stage { STAGE_SHIFT, STAGE_DRIVER, STAGE_HALL, STAGE_SERIAL, STAGE_DONE };
Stage currentStage = STAGE_SHIFT;
bool stageInitialized = false;

void latchOutputs() {
  uint8_t shiftBytes[SHIFT_TOTAL_BYTES];
  memset(shiftBytes, 0, sizeof(shiftBytes));
  for (uint8_t board = 0; board < SHIFT_BOARD_COUNT; ++board) {
    uint8_t dirByte = (dirState >> (board * 8)) & 0xFF;
    uint8_t stepByte = (stepState >> (board * 8)) & 0xFF;
    uint8_t offset = board * SHIFT_BYTES_PER_BOARD;
    shiftBytes[offset + 0] = dirByte;
    shiftBytes[offset + 1] = stepByte;
    shiftBytes[offset + 2] = 0;
  }

  digitalWrite(SR_LATCH_PIN, LOW);
  for (int byteIndex = SHIFT_TOTAL_BYTES - 1; byteIndex >= 0; --byteIndex) {
    for (int8_t bit = 7; bit >= 0; --bit) {
      digitalWrite(SR_CLOCK_PIN, LOW);
      digitalWrite(SR_DATA_PIN, (shiftBytes[byteIndex] >> bit) & 0x01);
      digitalWrite(SR_CLOCK_PIN, HIGH);
    }
  }
  digitalWrite(SR_LATCH_PIN, HIGH);
}

void pulseStep(uint8_t column) {
  stepState |= (uint64_t(1) << column);
  latchOutputs();
  delayMicroseconds(STEP_PULSE_US);
  stepState &= ~(uint64_t(1) << column);
  latchOutputs();
  delayMicroseconds(STEP_GAP_US);
}

bool readNextCommand() {
  if (!Serial.available()) {
    return false;
  }
  String input = Serial.readStringUntil('\n');
  input.trim();
  if (input.equalsIgnoreCase("next")) {
    return true;
  }
  if (input.length()) {
    Serial.println("[INFO] Send 'next' when you want to continue to the next stage.");
  }
  return false;
}

uint8_t computeCRC8(const uint8_t* data, size_t len) {
  uint8_t crc = 0;
  while (len--) {
    crc ^= *data++;
    for (uint8_t i = 0; i < 8; ++i) {
      crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
    }
  }
  return crc;
}

void ackSerialFrame(uint8_t detail = 0) {
  uint8_t payload[2] = {STATUS_OK, detail};
  const uint8_t header[2] = {0x10, 2};
  uint8_t crcData[sizeof(header) + sizeof(payload)];
  memcpy(crcData, header, sizeof(header));
  memcpy(crcData + sizeof(header), payload, sizeof(payload));
  uint8_t crc = computeCRC8(crcData, sizeof(crcData));
  Serial.write(FRAME_SYNC);
  Serial.write(header, sizeof(header));
  Serial.write(payload, sizeof(payload));
  Serial.write(crc);
}

void printHallMask() {
  uint8_t hallStates[4] = {0};
  for (uint8_t col = 0; col < 32; ++col) {
    uint8_t exp = (col < 16) ? 0 : 1;
    uint8_t pin = col % 16;
    if (exp < HALL_EXPANDER_COUNT && hallExpanderActive[exp]) {
      bool active = hallExpanders[exp].digitalRead(pin) == LOW;
      if (active) {
        hallStates[col / 8] |= 1 << (col % 8);
      }
    }
  }
  for (uint8_t row = 0; row < 4; ++row) {
    Serial.print("Halls ");
    Serial.print(row * 8);
    Serial.print('-');
    Serial.print(row * 8 + 7);
    Serial.print(" : ");
    for (uint8_t bit = 0; bit < 8; ++bit) {
      bool active = (hallStates[row] >> bit) & 1;
      Serial.print(active ? '1' : '.');
      Serial.print(' ');
    }
    Serial.println();
  }
}

void advanceStage(Stage next) {
  currentStage = next;
  stageInitialized = false;
  Serial.print("[STAGE] ");
  switch (next) {
    case STAGE_DRIVER:
      Serial.println("Driver test ready (step pulses will run on column 0). Send 'next' when you are happy and want to continue.");
      break;
    case STAGE_HALL:
      Serial.println("Hall sensors ready (magnets will flip the bits). Send 'next' once you confirm the readings change.");
      break;
    case STAGE_SERIAL:
      Serial.println("Serial handshake test ready. The sketch will ACK the next framed command backed by FRAME_SYNC. Send the frame now.");
      break;
    case STAGE_DONE:
      Serial.println("Bring-up completed. Power-cycle to restart the tester.");
      break;
    default:
      break;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(SR_DATA_PIN, OUTPUT);
  pinMode(SR_CLOCK_PIN, OUTPUT);
  pinMode(SR_LATCH_PIN, OUTPUT);
  pinMode(A4988_ENABLE_PIN, OUTPUT);
  digitalWrite(A4988_ENABLE_PIN, LOW);
  Wire.begin();
  for (uint8_t i = 0; i < HALL_EXPANDER_COUNT; ++i) {
    hallExpanderActive[i] = false;
    if (hallExpanders[i].begin_I2C(HALL_EXPANDER_ADDRESSES[i])) {
      hallExpanderActive[i] = true;
      for (uint8_t pin = 0; pin < 16; ++pin) {
        hallExpanders[i].pinMode(pin, INPUT_PULLUP);
      }
      Serial.print("[INFO] Hall expander 0x");
      Serial.print(HALL_EXPANDER_ADDRESSES[i], HEX);
      Serial.println(" ready");
    } else {
      Serial.print("[WARN] Hall expander 0x");
      Serial.print(HALL_EXPANDER_ADDRESSES[i], HEX);
      Serial.println(" missing");
    }
  }
  Serial.println("[STAGE] Shift register test ready. Verify 74HC595 outputs and send 'next'.");
}

enum SerialReadState { WAIT_SYNC, WAIT_CMD, WAIT_LEN, WAIT_PAYLOAD, WAIT_CRC };
SerialReadState readState = WAIT_SYNC;
uint8_t cmdByte = 0;
uint8_t lenByte = 0;
uint8_t payloadBuffer[64];
uint8_t payloadIndex = 0;

void loop() {
  switch (currentStage) {
    case STAGE_SHIFT:
      if (!stageInitialized) {
        Serial.println("[INFO] Driving a visible pattern across the shift register chain.");
        for (uint8_t i = 0; i < 32; ++i) {
          stepState = uint64_t(1) << i;
          dirState = ~uint64_t(0);
          latchOutputs();
          delay(150);
        }
        stageInitialized = true;
        Serial.println("[INFO] Stage complete — send 'next' to proceed.");
      } else if (readNextCommand()) {
        advanceStage(STAGE_DRIVER);
      }
      break;
    case STAGE_DRIVER:
      if (!stageInitialized) {
        stageInitialized = true;
        Serial.println("[INFO] Motor pulses now running; listen for the motor. Send 'next' anytime.");
      }
      {
        uint8_t dirByte = 0xFF;
        uint8_t stepByte = 0x01;
        Serial.print("[DBG] driver DIR byte=0x");
        Serial.print(dirByte, HEX);
        Serial.print(" STEP byte=0x");
        Serial.println(stepByte, HEX);
      }
      pulseStep(0);
      delay(0);
      if (readNextCommand()) {
        advanceStage(STAGE_HALL);
      }
      break;
    case STAGE_HALL:
      if (!stageInitialized) {
        stageInitialized = true;
      }
      static unsigned long lastReport = 0;
      if (millis() - lastReport > 1200) {
        printHallMask();
        lastReport = millis();
      }
      if (readNextCommand()) {
        advanceStage(STAGE_SERIAL);
      }
      break;
    case STAGE_SERIAL:
      if (!stageInitialized) {
        stageInitialized = true;
      }
      while (Serial.available()) {
        uint8_t incoming = Serial.read();
        if (readState == WAIT_SYNC) {
          if (incoming == FRAME_SYNC) {
            Serial.println("[DBG] FRAME_SYNC seen");
            readState = WAIT_CMD;
          }
        } else if (readState == WAIT_CMD) {
          cmdByte = incoming;
          readState = WAIT_LEN;
        } else if (readState == WAIT_LEN) {
          lenByte = incoming;
          payloadIndex = 0;
          if (lenByte == 0) {
            readState = WAIT_CRC;
          } else {
            readState = WAIT_PAYLOAD;
          }
        } else if (readState == WAIT_PAYLOAD) {
          payloadBuffer[payloadIndex++] = incoming;
          if (payloadIndex >= lenByte) {
            readState = WAIT_CRC;
          }
        } else if (readState == WAIT_CRC) {
          uint8_t crc = incoming;
          Serial.print("[INFO] Frame received cmd=");
          Serial.print(cmdByte, HEX);
          Serial.print(" len=");
          Serial.print(lenByte);
          Serial.println(" — replying with ACK");
          ackSerialFrame();
          advanceStage(STAGE_DONE);
          readState = WAIT_SYNC;
          break;
        }
      }
      break;
    case STAGE_DONE:
      // Nothing else to do once done
      break;
  }
}
