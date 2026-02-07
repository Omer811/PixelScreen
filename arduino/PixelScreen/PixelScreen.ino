#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include <stdarg.h>
#include <string.h>

// Enable debug logging via Serial
#define DEBUG_SERIAL 1

#define LOG_BUFFER_SIZE 96
const uint8_t LOG_BUFFER_LINES = 8;
const uint8_t LOG_LINE_LENGTH = 48;
char logBuffer[LOG_BUFFER_LINES][LOG_LINE_LENGTH];
uint8_t logBufferStart = 0;
uint8_t logLineCount = 0;

#if DEBUG_SERIAL
void debugPrintBytes(const char* label, const uint8_t* data, uint8_t len) {
  Serial.print("[DBG] ");
  Serial.print(label);
  Serial.print(":");
  for (uint8_t i = 0; i < len; ++i) {
    Serial.print(" ");
    if (data[i] < 0x10) {
      Serial.print("0");
    }
    Serial.print(data[i], HEX);
  }
  Serial.println();
}

void appendLogLine(const char* line) {
  char tmp[LOG_LINE_LENGTH];
  strncpy(tmp, line, LOG_LINE_LENGTH - 1);
  tmp[LOG_LINE_LENGTH - 1] = '\0';
  uint8_t idx;
  if (logLineCount < LOG_BUFFER_LINES) {
    idx = (logBufferStart + logLineCount) % LOG_BUFFER_LINES;
    logLineCount++;
  } else {
    idx = logBufferStart;
    logBufferStart = (logBufferStart + 1) % LOG_BUFFER_LINES;
  }
  strncpy(logBuffer[idx], tmp, LOG_LINE_LENGTH);
  logBuffer[idx][LOG_LINE_LENGTH - 1] = '\0';
}

void logMessage(const char* label, const char* fmt, ...) {
  char buffer[LOG_BUFFER_SIZE];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);
  Serial.print("[");
  Serial.print(label);
  Serial.print("] ");
  Serial.println(buffer);
  appendLogLine(buffer);
}

void debugLog(const char* msg) {
  Serial.print("[DBG] ");
  Serial.println(msg);
  appendLogLine(msg);
}
#else
#define debugPrintBytes(label, data, len)
#define debugLog(msg)
#endif

// === Hardware configuration ===
#define NUM_COLUMNS 32

// Shift register pins (74HC595 chain): first 8 outputs per board drive DIR, next 8 STEP, last 8 unused.
const uint8_t SR_DATA_PIN = 2;
const uint8_t SR_CLOCK_PIN = 3;
const uint8_t SR_LATCH_PIN = 4;
const uint8_t A4988_ENABLE_PIN = 5; // active LOW
const uint8_t SHIFT_CHIPS_PER_BOARD = 3;
const uint8_t SHIFT_BYTES_PER_BOARD = SHIFT_CHIPS_PER_BOARD;
const uint8_t SHIFT_BOARD_COUNT = (NUM_COLUMNS + 7) / 8;
const uint8_t SHIFT_TOTAL_BYTES = SHIFT_BOARD_COUNT * SHIFT_BYTES_PER_BOARD;

const unsigned long STEP_PULSE_US = 600;
const unsigned long STEP_GAP_US = 200;

// Homing timing
const uint16_t DEFAULT_HALL_TIMEOUT_MS = 5000;
const uint8_t  HALL_ACTIVE_LEVEL = LOW;

// Serial protocol
const uint8_t FRAME_SYNC = 0xAA;
const uint8_t CMD_WRITE_COLUMNS = 0x01;
const uint8_t CMD_HOME = 0x02;
const uint8_t CMD_STATUS = 0x03;
const uint8_t CMD_RESPONSE = 0x10;
const uint8_t CMD_CONFIGURE = 0x04;
const uint8_t CMD_SETUP_START = 0x05;
const uint8_t CMD_SETUP_ACK_COLUMN = 0x06;
const uint8_t CMD_SETUP_DIRECTION = 0x07;
const uint8_t CMD_SETUP_POSITION = 0x08;
const uint8_t CMD_TUNE_COLUMN = 0x09;
const uint8_t CMD_READ_HALL = 0x0A;
const uint8_t CMD_ABORT_SETUP = 0x0B;
const uint8_t CMD_DUMP_LOGS = 0x0C;
const uint8_t CMD_SET_CRC = 0x0D;
const uint8_t CMD_SET_SETTINGS = 0x0E;
const uint8_t CMD_SET_SKIP_HALL = 0x0F;
const uint8_t CMD_DEBUG_REPORT = 0x11;
const size_t MAX_PAYLOAD = 64; // ample for columns

const uint8_t SETUP_DETAIL_COLUMN = 0x80;
const uint8_t SETUP_DETAIL_DIRECTION = 0x81;
const uint8_t SETUP_DETAIL_STEPS = 0x82;
const uint8_t SETUP_DETAIL_POSITION_BASE = 0x90;
const uint8_t SETUP_DETAIL_DONE = 0xF0;
const uint8_t DEBUG_DETAIL_REPORT = 0xE0;

enum ResponseStatus : uint8_t {
  STATUS_OK = 0x00,
  STATUS_ERR_WIDTH = 0x01,
  STATUS_ERR_NOT_HOMED = 0x02,
  STATUS_ERR_CRC = 0x03,
  STATUS_ERR_TIMEOUT = 0x04,
  STATUS_ERR_BUSY = 0x05,
};

// Hall sensors are read through two MCP23017 expanders (16 pins each)
const uint8_t HALL_EXPANDER_ADDRESSES[] = {0x20, 0x21};
const uint8_t HALL_EXPANDER_COUNT = sizeof(HALL_EXPANDER_ADDRESSES) / sizeof(HALL_EXPANDER_ADDRESSES[0]);
Adafruit_MCP23X17 hallExpanders[HALL_EXPANDER_COUNT];
bool hallExpanderActive[HALL_EXPANDER_COUNT];

const uint8_t hallExpanderIndex[NUM_COLUMNS] = {
  // 0..15 map to first expander, 16..31 to second
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1
};
const uint8_t hallPinIndex[NUM_COLUMNS] = {
  0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,
  0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15
};

uint8_t currentPositions[NUM_COLUMNS];
bool homed[NUM_COLUMNS];
bool interactiveSetupActive = false;
bool abortSetupRequested = false;

uint32_t stepState = 0;
uint32_t dirState = 0xFFFFFFFF; // always forward
bool skipHallHome = false;
uint16_t hallTimeoutMs = DEFAULT_HALL_TIMEOUT_MS;
uint16_t stepIntervalMs = 0;

// Receive state machine
enum RecvState {
  WAIT_SYNC,
  READ_CMD,
  READ_LEN,
  READ_DATA,
  READ_CRC
};

bool crcEnabled = true;
RecvState recvState = WAIT_SYNC;
uint8_t recvCmd = 0;
uint8_t recvLen = 0;
uint8_t recvPayload[MAX_PAYLOAD];
size_t recvIndex = 0;
uint8_t recvCRC = 0;

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

typedef void (*CommandHandlerFn)(const uint8_t*, uint8_t);

struct CommandHandler {
  uint8_t cmd;
  CommandHandlerFn handler;
};

void buildShiftBytes(uint8_t* shiftBytes) {
  memset(shiftBytes, 0, SHIFT_TOTAL_BYTES);
  for (uint8_t board = 0; board < SHIFT_BOARD_COUNT; ++board) {
    uint8_t dirByte = (dirState >> (board * 8)) & 0xFF;
    uint8_t stepByte = (stepState >> (board * 8)) & 0xFF;
    uint8_t base = board * SHIFT_BYTES_PER_BOARD;
    shiftBytes[base + 0] = dirByte;
    shiftBytes[base + 1] = stepByte;
    shiftBytes[base + 2] = 0;
  }
}

void latchOutputs() {
  uint8_t shiftBytes[SHIFT_TOTAL_BYTES];
  buildShiftBytes(shiftBytes);
  digitalWrite(SR_LATCH_PIN, LOW);
  for (int byteIndex = SHIFT_TOTAL_BYTES - 1; byteIndex >= 0; --byteIndex) {
    for (int8_t bit = 7; bit >= 0; --bit) {
      digitalWrite(SR_CLOCK_PIN, LOW);
      digitalWrite(SR_DATA_PIN, (shiftBytes[byteIndex] >> bit) & 0x01);
      digitalWrite(SR_CLOCK_PIN, HIGH);
    }
  }
  digitalWrite(SR_CLOCK_PIN, LOW);
  digitalWrite(SR_LATCH_PIN, HIGH);
}

void pulseStep(uint8_t column) {
  stepState |= (1UL << column);
  latchOutputs();
  delayMicroseconds(STEP_PULSE_US);
  stepState &= ~(1UL << column);
  latchOutputs();
  delayMicroseconds(STEP_GAP_US);
  if (stepIntervalMs) {
    delay(stepIntervalMs);
  }
}

bool isHallTriggered(uint8_t column) {
  uint8_t expanderId = hallExpanderIndex[column];
  uint8_t pin = hallPinIndex[column];
  if (expanderId >= HALL_EXPANDER_COUNT || !hallExpanderActive[expanderId]) {
    return false;
  }
  return hallExpanders[expanderId].digitalRead(pin) == HALL_ACTIVE_LEVEL;
}

bool homeColumn(uint8_t column) {
  unsigned long start = millis();
  if (skipHallHome) {
    currentPositions[column] = 0;
    homed[column] = true;
    logMessage("INFO", "Skipping hall sensor for column %u", column);
    return true;
  }
  while (millis() - start < hallTimeoutMs) {
    if (tryHandleAbortDuringHoming()) {
      return false;
    }
    if (isHallTriggered(column)) {
      currentPositions[column] = 0;
      homed[column] = true;
      return true;
    }
    pulseStep(column);
  }
  logMessage("ERROR", "Column %u homing timeout (check hall wiring)", column);
  return false;
}

bool areAllHomed() {
  for (uint8_t i = 0; i < NUM_COLUMNS; ++i) {
    if (!homed[i]) {
      return false;
    }
  }
  return true;
}

uint8_t configVersion = 0;
uint8_t columnToPosition[16];
uint8_t positionPatterns[16];
uint16_t stepsPerRevolution = 0;
bool directionForward = true;
const uint16_t MAX_STEPS_THRESHOLD = 20000;
uint8_t configuredMotorCount = NUM_COLUMNS;

void installDefaultMapping() {
  for (uint8_t i = 0; i < 16; ++i) {
    columnToPosition[i] = i;
  }
}

void sendResponse(uint8_t status, uint8_t detail = 0, const uint8_t* extra = nullptr, uint8_t extraLen = 0) {
  uint8_t payloadLen = 2 + extraLen;
  uint8_t frame[2 + 2 + 16]; // cmd, len, payload up to 16 bytes
  frame[0] = CMD_RESPONSE;
  frame[1] = payloadLen;
  frame[2] = status;
  frame[3] = detail;
  if (extraLen) {
    memcpy(frame + 4, extra, extraLen);
  }
  uint8_t total = 2 + payloadLen;
  uint8_t crc = computeCRC8(frame, total);
#if DEBUG_SERIAL
  debugPrintBytes("RESP", frame, total);
  if (status != STATUS_OK) {
    debugLog("Response status != STATUS_OK");
  }
#endif
  Serial.write(FRAME_SYNC);
  Serial.write(frame, total);
  Serial.write(crc);
}

bool decodeColumns(const uint8_t* payload, uint8_t width, uint8_t* decoded) {
  uint16_t bitCount = width * 4;
  uint16_t packedBytes = (bitCount + 7) / 8;
  const uint8_t* stream = payload + 1;

  for (uint8_t col = 0; col < width; ++col) {
    uint16_t bitIndex = col * 4;
    uint16_t byteIndex = bitIndex / 8;
    uint8_t shift = bitIndex % 8;
    uint8_t nextByte = (byteIndex + 1 < packedBytes) ? stream[byteIndex + 1] : 0;
    uint16_t composite = stream[byteIndex] | (uint16_t(nextByte) << 8);
    decoded[col] = (composite >> shift) & 0x0F;
  }
  return true;
}

void readBytesBlocking(uint8_t* dest, uint8_t count) {
  uint8_t index = 0;
  while (index < count) {
    if (Serial.available()) {
      dest[index++] = Serial.read();
    }
  }
}

bool readSetupFrame(uint8_t& cmd, uint8_t& length, uint8_t* payload, size_t maxPayload) {
  while (true) {
    if (!Serial.available()) {
      continue;
    }
    if (Serial.read() != FRAME_SYNC) {
      continue;
    }
    uint8_t header[2];
    readBytesBlocking(header, 2);
    cmd = header[0];
    length = header[1];
    if (length > maxPayload) {
      return false;
    }
    readBytesBlocking(payload, length);
    uint8_t crcByte;
    readBytesBlocking(&crcByte, 1);
    uint8_t crcData[MAX_PAYLOAD + 2];
    crcData[0] = cmd;
    crcData[1] = length;
    memcpy(crcData + 2, payload, length);
    if (computeCRC8(crcData, 2 + length) != crcByte) {
      continue;
    }
    return true;
  }
  return false;
}

bool tryHandleAbortDuringHoming() {
  while (Serial.available()) {
    if (Serial.peek() != FRAME_SYNC) {
      Serial.read();
      continue;
    }
    if (Serial.available() < 4) {
      return false;
    }
    uint8_t cmd = 0;
    uint8_t len = 0;
    uint8_t payload[MAX_PAYLOAD];
    if (!readSetupFrame(cmd, len, payload, MAX_PAYLOAD)) {
      return false;
    }
    if (cmd == CMD_ABORT_SETUP) {
      abortSetupRequested = true;
      handleAbortSetup();
      return true;
    }
  }
  return false;
}

bool waitForSetupCommand(uint8_t expectedCmd, uint8_t* payload, uint8_t& length) {
  uint8_t cmd = 0;
  uint8_t len = 0;
  if (!readSetupFrame(cmd, len, payload, MAX_PAYLOAD)) {
    return false;
  }
  if (cmd == CMD_ABORT_SETUP) {
    abortSetupRequested = true;
    logMessage("INFO", "Abort command received while waiting for host");
    sendResponse(STATUS_OK, CMD_ABORT_SETUP);
    return false;
  }
  if (cmd != expectedCmd) {
    return false;
  }
  length = len;
  return true;
}

void sendSetupPrompt(uint8_t detail, const uint8_t* extra = nullptr, uint8_t extraLen = 0) {
  sendResponse(STATUS_OK, detail, extra, extraLen);
}

void handleWriteColumns(const uint8_t* payload, uint8_t length) {
  logMessage("STAGE", "handleWriteColumns: start width byte %u", payload[0]);
  if (length < 1) {
    sendResponse(STATUS_ERR_WIDTH);
    return;
  }
  uint8_t width = payload[0];
  if (width == 0 || width > NUM_COLUMNS) {
    sendResponse(STATUS_ERR_WIDTH);
    return;
  }
  uint16_t bitCount = width * 4;
  uint16_t packedBytes = (bitCount + 7) / 8;
  if (length != 1 + packedBytes) {
    sendResponse(STATUS_ERR_WIDTH);
    return;
  }

  if (!areAllHomed()) {
    logMessage("ERROR", "handleWriteColumns: rejected before homing");
    sendResponse(STATUS_ERR_NOT_HOMED);
    return;
  }

  uint8_t columnBits[NUM_COLUMNS];
  if (!decodeColumns(payload, width, columnBits)) {
    sendResponse(STATUS_ERR_WIDTH);
    return;
  }

  for (uint8_t col = 0; col < width; ++col) {
    uint8_t requested = columnToPosition[columnBits[col]];
    while (currentPositions[col] != requested) {
      pulseStep(col);
      currentPositions[col] = (currentPositions[col] + 1) & 0x0F;
    }
    logMessage("STAGE", "handleWriteColumns: column %u now at %u", col, currentPositions[col]);
  }

  sendResponse(STATUS_OK);
  logMessage("STAGE", "handleWriteColumns: done");
}

bool runInteractiveSetup() {
  uint8_t buffer[MAX_PAYLOAD];
  uint8_t length = 0;
  uint8_t limit = min(configuredMotorCount, (uint8_t)NUM_COLUMNS);
  if (limit == 0) {
    debugLog("No columns configured");
    return false;
  }
  abortSetupRequested = false;
  interactiveSetupActive = true;
  logMessage("STAGE", "Interactive setup for %u columns started", limit);
  for (uint8_t col = 0; col < limit; ++col) {
    if (abortSetupRequested) {
      logMessage("INFO", "Interactive setup aborted during column homing");
      interactiveSetupActive = false;
      return false;
    }
    if (!homeColumn(col)) {
      sendResponse(STATUS_ERR_TIMEOUT, col);
      return false;
    }
    char msg[32];
    snprintf(msg, sizeof(msg), "Column %u homed", col);
    debugLog(msg);
    logMessage("STAGE", "Column %u homed; waiting for acknowledgment", col);
    sendSetupPrompt(SETUP_DETAIL_COLUMN | (col & 0x1F));
    if (!waitForSetupCommand(CMD_SETUP_ACK_COLUMN, buffer, length) || length < 1 || buffer[0] != col) {
      sendResponse(STATUS_ERR_BUSY);
      logMessage("ERROR", "Column %u ack failed", col);
      return false;
    }
    logMessage("STAGE", "Column %u acknowledged", col);
  }

  const uint8_t interactiveColumn = 0;
  for (uint8_t i = 0; i < 4; ++i) {
    pulseStep(interactiveColumn);
    currentPositions[interactiveColumn] = (currentPositions[interactiveColumn] + 1) & 0x0F;
  }

  sendSetupPrompt(SETUP_DETAIL_DIRECTION);
  if (!waitForSetupCommand(CMD_SETUP_DIRECTION, buffer, length) || length < 1) {
    sendResponse(STATUS_ERR_BUSY);
    return false;
  }
  directionForward = buffer[0] != 0;
  logMessage("STAGE", "Direction confirmed %s", directionForward ? "forward" : "reverse");
  if (abortSetupRequested) {
    logMessage("INFO", "Interactive setup aborted after direction confirmation");
    interactiveSetupActive = false;
    return false;
  }

  uint16_t stepCount = 0;
  while (!isHallTriggered(interactiveColumn)) {
    pulseStep(interactiveColumn);
    currentPositions[interactiveColumn] = (currentPositions[interactiveColumn] + 1) & 0x0F;
    stepCount++;
    if (stepCount > MAX_STEPS_THRESHOLD) {
      sendResponse(STATUS_ERR_TIMEOUT);
      return false;
    }
  }

  stepsPerRevolution = stepCount;
  uint8_t stepBytes[2] = {uint8_t(stepCount & 0xFF), uint8_t((stepCount >> 8) & 0xFF)};
  sendSetupPrompt(SETUP_DETAIL_STEPS, stepBytes, 2);
  logMessage("STAGE", "Measured %u steps per revolution", stepCount);
  debugPrintBytes("Steps measured", stepBytes, 2);

  uint16_t baseSteps = stepsPerRevolution / 16;
  uint8_t remainder = stepsPerRevolution % 16;
  uint8_t extras = 0;
  for (uint8_t pos = 0; pos < 16; ++pos) {
    if (pos > 0) {
      uint16_t toAdvance = baseSteps + (extras < remainder ? 1 : 0);
      if (extras < remainder) {
        extras++;
      }
      for (uint16_t step = 0; step < toAdvance; ++step) {
        pulseStep(interactiveColumn);
        currentPositions[interactiveColumn] = (currentPositions[interactiveColumn] + 1) & 0x0F;
      }
    }
    logMessage("STAGE", "Prompting pattern for position %u", pos);
    sendSetupPrompt(SETUP_DETAIL_POSITION_BASE + pos);
    if (!waitForSetupCommand(CMD_SETUP_POSITION, buffer, length)) {
      sendResponse(STATUS_ERR_BUSY);
      return false;
    }
    if (abortSetupRequested) {
      logMessage("INFO", "Interactive setup aborted during pattern entry");
      interactiveSetupActive = false;
      return false;
    }
    uint8_t nibble = 0;
    uint8_t digits = 0;
    for (uint8_t idx = 0; idx < length && digits < 4; ++idx) {
      char c = buffer[idx];
      if (c != '0' && c != '1') {
        continue;
      }
      if (c == '1') {
        nibble |= 1 << (3 - digits);
      }
      digits++;
    }
    if (digits != 4) {
      sendResponse(STATUS_ERR_WIDTH);
      return false;
    }
    positionPatterns[pos] = nibble;
    logMessage("STAGE", "Pattern for position %u = 0x%02X", pos, nibble);
    debugPrintBytes("Pattern", &nibble, 1);
  }

  for (uint8_t i = 0; i < 16; ++i) {
    columnToPosition[i] = 0xFF;
  }
  for (uint8_t pos = 0; pos < 16; ++pos) {
    uint8_t code = positionPatterns[pos] & 0x0F;
    columnToPosition[code] = pos;
  }
  for (uint8_t i = 0; i < 16; ++i) {
    if (columnToPosition[i] == 0xFF) {
      columnToPosition[i] = i;
    }
  }

  homeColumn(interactiveColumn);
  logMessage("STAGE", "Interactive encoding stored; re-homed column %u", interactiveColumn);
  sendSetupPrompt(SETUP_DETAIL_DONE, stepBytes, 2);
  debugLog("Interactive setup complete");
  logMessage("STAGE", "Setup complete; configuration saved");
  interactiveSetupActive = false;
  return true;
}

void handleConfigure(const uint8_t* payload, uint8_t length) {
  logMessage("STAGE", "handleConfigure: length %u", length);
  if (length != 17) {
    sendResponse(STATUS_ERR_WIDTH);
    return;
  }
  configVersion = payload[0];
  for (uint8_t i = 0; i < 16; ++i) {
    columnToPosition[i] = payload[1 + i] & 0x0F;
  }
  sendResponse(STATUS_OK, configVersion);
  logMessage("STAGE", "handleConfigure: version %u stored", configVersion);
}

void handleTuneColumn(const uint8_t* payload, uint8_t length) {
  logMessage("STAGE", "handleTuneColumn: length %u", length);
  if (length != 3) {
    sendResponse(STATUS_ERR_WIDTH);
    return;
  }
  uint8_t column = payload[0];
  uint16_t steps = payload[1] | (uint16_t(payload[2]) << 8);
  if (column >= NUM_COLUMNS) {
    sendResponse(STATUS_ERR_WIDTH);
    return;
  }
  logMessage("STAGE", "handleTuneColumn: column %u steps %u", column, steps);
  for (uint16_t i = 0; i < steps; ++i) {
    pulseStep(column);
  }
  logMessage("STAGE", "handleTuneColumn: column %u done", column);
  sendResponse(STATUS_OK, column);
}

void composeHallStates(uint8_t* hallStates) {
  memset(hallStates, 0, 4);
  for (uint8_t col = 0; col < NUM_COLUMNS; ++col) {
    uint8_t exp = hallExpanderIndex[col];
    uint8_t pin = hallPinIndex[col];
    if (exp < HALL_EXPANDER_COUNT && hallExpanderActive[exp]) {
      if (hallExpanders[exp].digitalRead(pin) == HALL_ACTIVE_LEVEL) {
        hallStates[col / 8] |= 1 << (col % 8);
      }
    }
  }
}

void handleReadHall() {
  logMessage("STAGE", "handleReadHall: dumping hall mask");
  uint8_t hallStates[4];
  composeHallStates(hallStates);
  sendResponse(STATUS_OK, 0, hallStates, sizeof(hallStates));
}

void handleDebugReport(const uint8_t* payload, uint8_t length) {
  logMessage("INFO", "handleDebugReport: streaming shift+hall state");
  uint8_t report[SHIFT_TOTAL_BYTES + 4];
  buildShiftBytes(report);
  composeHallStates(report + SHIFT_TOTAL_BYTES);
  sendResponse(STATUS_OK, DEBUG_DETAIL_REPORT, report, sizeof(report));
}

void handleAbortSetup() {
  abortSetupRequested = true;
  if (!interactiveSetupActive) {
    logMessage("INFO", "Setup abort requested while idle");
  } else {
    logMessage("INFO", "Setup abort requested while interactive");
  }
  sendResponse(STATUS_OK, CMD_ABORT_SETUP);
}

void handleDumpLogs() {
  logMessage("STAGE", "handleDumpLogs: sending %u entries", logLineCount);
  uint8_t payload[MAX_PAYLOAD];
  uint8_t offset = 0;
  for (uint8_t i = 0; i < logLineCount; ++i) {
    uint8_t idx = (logBufferStart + i) % LOG_BUFFER_LINES;
    size_t len = strnlen(logBuffer[idx], LOG_LINE_LENGTH);
    if (len == 0 || offset + len + 1 > MAX_PAYLOAD) {
      continue;
    }
    memcpy(payload + offset, logBuffer[idx], len);
    offset += len;
    payload[offset++] = '\n';
  }
  logLineCount = 0;
  logBufferStart = 0;
  sendResponse(STATUS_OK, 0, payload, offset);
}

void handleSetCRC(const uint8_t* payload, uint8_t length) {
  if (length < 1) {
    sendResponse(STATUS_ERR_WIDTH);
    return;
  }
  crcEnabled = payload[0] != 0;
  logMessage("STAGE", "handleSetCRC: CRC %sabled", crcEnabled ? "en" : "dis");
  resetReceive();
  sendResponse(STATUS_OK, crcEnabled ? 1 : 0);
}

void handleSetSettings(const uint8_t* payload, uint8_t length) {
  if (length != 4) {
    sendResponse(STATUS_ERR_WIDTH);
    return;
  }
  uint16_t hall = payload[0] | (uint16_t(payload[1]) << 8);
  uint16_t interval = payload[2] | (uint16_t(payload[3]) << 8);
  hallTimeoutMs = hall ? hall : DEFAULT_HALL_TIMEOUT_MS;
  stepIntervalMs = interval;
  logMessage("STAGE", "handleSetSettings: hall %u ms, step interval %u ms", hallTimeoutMs, stepIntervalMs);
  sendResponse(STATUS_OK);
}

void handleSetSkipHall(const uint8_t* payload, uint8_t length) {
  if (length != 1) {
    sendResponse(STATUS_ERR_WIDTH);
    return;
  }
  skipHallHome = payload[0] != 0;
  logMessage("STAGE", "handleSetSkipHall: skip %s", skipHallHome ? "enabled" : "disabled");
  sendResponse(STATUS_OK, skipHallHome ? 1 : 0);
}

void handleSetupStart() {
  for (uint8_t i = 0; i < NUM_COLUMNS; ++i) {
    homed[i] = false;
    currentPositions[i] = 0;
  }
  logMessage("INFO", "Interactive setup requested");
  runInteractiveSetup();
}

void handleHome() {
  logMessage("STAGE", "handleHome: requested");
  for (uint8_t col = 0; col < NUM_COLUMNS; ++col) {
    homed[col] = false;
    currentPositions[col] = 0;
  }
  for (uint8_t col = 0; col < NUM_COLUMNS; ++col) {
    logMessage("STAGE", "handleHome: column %u homing", col);
    if (!homeColumn(col)) {
      logMessage("ERROR", "handleHome: column %u failed to home", col);
      sendResponse(STATUS_ERR_TIMEOUT, col);
      return;
    }
    // rotate a little past the magnet so future writes can advance
    for (uint8_t i = 0; i < 2; ++i) {
      pulseStep(col);
      currentPositions[col] = (currentPositions[col] + 1) & 0x0F;
    }
  }
  logMessage("STAGE", "handleHome: complete");
  sendResponse(STATUS_OK);
}

void handleStatusRequest() {
  uint8_t detail = (configVersion << 1) | (areAllHomed() ? 1 : 0);
  uint8_t extra[3];
  extra[0] = uint8_t(stepsPerRevolution & 0xFF);
  extra[1] = uint8_t((stepsPerRevolution >> 8) & 0xFF);
  extra[2] = directionForward ? 1 : 0;
  sendResponse(STATUS_OK, detail, extra, 3);
}

const CommandHandler kCommandHandlers[] = {
  {CMD_WRITE_COLUMNS, handleWriteColumns},
  {CMD_HOME, handleHome},
  {CMD_STATUS, handleStatusRequest},
  {CMD_CONFIGURE, handleConfigure},
  {CMD_SETUP_START, handleSetupStart},
  {CMD_TUNE_COLUMN, handleTuneColumn},
  {CMD_READ_HALL, handleReadHall},
  {CMD_ABORT_SETUP, handleAbortSetup},
  {CMD_DUMP_LOGS, handleDumpLogs},
  {CMD_SET_CRC, handleSetCRC},
  {CMD_SET_SETTINGS, handleSetSettings},
  {CMD_SET_SKIP_HALL, handleSetSkipHall},
  {CMD_DEBUG_REPORT, handleDebugReport},
};
const size_t kCommandHandlerCount = sizeof(kCommandHandlers) / sizeof(kCommandHandlers[0]);

bool dispatchCommand(uint8_t cmd, const uint8_t* payload, uint8_t length) {
  for (size_t i = 0; i < kCommandHandlerCount; ++i) {
    if (kCommandHandlers[i].cmd == cmd) {
      kCommandHandlers[i].handler(payload, length);
      return true;
    }
  }
  return false;
}

void resetReceive() {
  recvState = WAIT_SYNC;
  recvCmd = 0;
  recvLen = 0;
  recvIndex = 0;
  recvCRC = 0;
}

void processSerial() {
  while (Serial.available()) {
    uint8_t incoming = Serial.read();
    debugPrintBytes("RAW", &incoming, 1);
    switch (recvState) {
      case WAIT_SYNC:
        if (incoming == FRAME_SYNC) {
          debugLog("processSerial: sync found");
          recvState = READ_CMD;
        } else {
          debugLog("processSerial: ignored byte while waiting for sync");
        }
        break;
      case READ_CMD:
        recvCmd = incoming;
        debugPrintBytes("processSerial: cmd", &recvCmd, 1);
        recvState = READ_LEN;
        break;
      case READ_LEN:
        recvLen = incoming;
        debugPrintBytes("processSerial: len", &recvLen, 1);
        if (recvLen > MAX_PAYLOAD) {
          resetReceive();
        } else {
          recvIndex = 0;
          recvState = recvLen ? READ_DATA : READ_CRC;
        }
        break;
      case READ_DATA:
        recvPayload[recvIndex++] = incoming;
        if ((recvIndex % 8) == 0) {
          debugPrintBytes("processSerial: data chunk", recvPayload + recvIndex - 8, 8);
        }
        if (recvIndex >= recvLen) {
          recvState = READ_CRC;
        }
        break;
      case READ_CRC:
        recvCRC = incoming;
        debugPrintBytes("processSerial: crc", &recvCRC, 1);
        uint8_t crcData[MAX_PAYLOAD + 2];
        crcData[0] = recvCmd;
        crcData[1] = recvLen;
        memcpy(crcData + 2, recvPayload, recvLen);
        uint8_t computed = computeCRC8(crcData, recvLen + 2);
        bool crcOk = !crcEnabled || computed == recvCRC;
        if (crcOk) {
#if DEBUG_SERIAL
          uint8_t header[2] = {recvCmd, recvLen};
          debugPrintBytes("REQ", header, 2);
#endif
          if (!dispatchCommand(recvCmd, recvPayload, recvLen)) {
            logMessage("WARN", "Unknown command 0x%02X", recvCmd);
            sendResponse(STATUS_ERR_WIDTH);
          }
        } else {
          debugLog("CRC mismatch");
          sendResponse(STATUS_ERR_CRC);
        }
        resetReceive();
        break;
    }
  }
}

void setup() {
  Serial.begin(115200);
  logMessage("STAGE", "Setup start: Serial 115200 & control pins init");
  pinMode(SR_DATA_PIN, OUTPUT);
  pinMode(SR_CLOCK_PIN, OUTPUT);
  pinMode(SR_LATCH_PIN, OUTPUT);
  pinMode(A4988_ENABLE_PIN, OUTPUT);
  digitalWrite(A4988_ENABLE_PIN, LOW); // enable drivers
  latchOutputs();
  logMessage("STAGE", "Step/dir shift registers enabled, drivers released");

  Wire.begin();
  uint8_t activeExpanders = 0;
  for (uint8_t i = 0; i < HALL_EXPANDER_COUNT; ++i) {
    hallExpanderActive[i] = false;
    if (hallExpanders[i].begin_I2C(HALL_EXPANDER_ADDRESSES[i])) {
      hallExpanderActive[i] = true;
      logMessage("INFO", "Hall expander 0x%02X available", HALL_EXPANDER_ADDRESSES[i]);
      activeExpanders++;
      for (uint8_t pin = 0; pin < 16; ++pin) {
        hallExpanders[i].pinMode(pin, INPUT_PULLUP);
      }
    } else {
      logMessage("WARN", "Hall expander 0x%02X missing (supporting single-expander mode)", HALL_EXPANDER_ADDRESSES[i]);
    }
  }
  logMessage("INFO", "Detected %u/%u MCP23017 expanders", activeExpanders, HALL_EXPANDER_COUNT);
  logMessage("STAGE", "Hall expanders initialized");

  for (uint8_t i = 0; i < NUM_COLUMNS; ++i) {
    currentPositions[i] = 0;
    homed[i] = false;
  }
  logMessage("STAGE", "Column state reset");
  resetReceive();
  installDefaultMapping();
  stepsPerRevolution = 0;
  directionForward = true;
  logMessage("STAGE", "Setup complete; waiting for commands");
}

void loop() {
  processSerial();
}
