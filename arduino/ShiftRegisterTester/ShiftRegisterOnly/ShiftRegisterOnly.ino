#include <Arduino.h>
#include <string.h>

const uint8_t SHIFT_SR_DATA_PIN = 2;
const uint8_t SHIFT_SR_CLOCK_PIN = 3;
const uint8_t SHIFT_SR_LATCH_PIN = 4;
const uint8_t SHIFT_LDEN_PIN = 6; // tie low to enable outputs
const uint8_t NUM_COLUMNS = 32;
const uint8_t SHIFT_CHIPS_PER_BOARD = 3;
const uint8_t SHIFT_BYTES_PER_BOARD = SHIFT_CHIPS_PER_BOARD;
const uint8_t SHIFT_BOARD_COUNT = (NUM_COLUMNS + 7) / 8;
const uint8_t SHIFT_TOTAL_BYTES = SHIFT_BOARD_COUNT * SHIFT_BYTES_PER_BOARD;

void setup() {
  Serial.begin(115200);
  pinMode(SHIFT_SR_DATA_PIN, OUTPUT);
  pinMode(SHIFT_SR_CLOCK_PIN, OUTPUT);
  pinMode(SHIFT_SR_LATCH_PIN, OUTPUT);
  pinMode(SHIFT_LDEN_PIN, OUTPUT);
  digitalWrite(SHIFT_LDEN_PIN, LOW);
  digitalWrite(SHIFT_SR_DATA_PIN, LOW);
  digitalWrite(SHIFT_SR_CLOCK_PIN, LOW);
  digitalWrite(SHIFT_SR_LATCH_PIN, LOW);
  Serial.println("[STAGE] Shift register isolation ready");
}

void latchBoard(uint32_t stepPattern, uint32_t dirPattern) {
  uint8_t shiftBytes[SHIFT_TOTAL_BYTES];
  memset(shiftBytes, 0, sizeof(shiftBytes));
  for (uint8_t board = 0; board < SHIFT_BOARD_COUNT; ++board) {
    const uint8_t offset = board * SHIFT_BYTES_PER_BOARD;
    shiftBytes[offset + 0] = (dirPattern >> (board * 8)) & 0xFF;
    shiftBytes[offset + 1] = (stepPattern >> (board * 8)) & 0xFF;
    shiftBytes[offset + 2] = 0;
  }

  digitalWrite(SHIFT_SR_LATCH_PIN, LOW);
  for (int byteIndex = SHIFT_TOTAL_BYTES - 1; byteIndex >= 0; --byteIndex) {
    for (int8_t bit = 7; bit >= 0; --bit) {
      digitalWrite(SHIFT_SR_CLOCK_PIN, LOW);
      digitalWrite(SHIFT_SR_DATA_PIN, (shiftBytes[byteIndex] >> bit) & 0x01);
      digitalWrite(SHIFT_SR_CLOCK_PIN, HIGH);
    }
  }
  digitalWrite(SHIFT_SR_LATCH_PIN, HIGH);
}

void loop() {
  static uint32_t step = 0x00000001;
  const uint32_t dir = ~0UL;
  Serial.print("[DBG] STEP=0x");
  Serial.print(step, HEX);
  Serial.print(" DIR=0x");
  Serial.println(dir, HEX);
  latchBoard(step, dir);
  step <<= 1;
  if (step == 0) {
    step = 0x00000001;
  }
  delay(200);
}
