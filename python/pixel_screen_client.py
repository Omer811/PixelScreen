"""Simple Raspberry Pi client for the PixelScreen Arduino protocol."""
from __future__ import annotations

import serial
import time
from typing import Sequence

FRAME_SYNC = 0xAA
CMD_WRITE_COLUMNS = 0x01
CMD_HOME = 0x02
CMD_STATUS = 0x03
CMD_RESPONSE = 0x10
CMD_CONFIGURE = 0x04
CMD_SETUP_START = 0x05
CMD_SETUP_ACK_COLUMN = 0x06
CMD_SETUP_DIRECTION = 0x07
CMD_SETUP_POSITION = 0x08
CMD_TUNE_COLUMN = 0x09
CMD_READ_HALL = 0x0A
CMD_ABORT_SETUP = 0x0B
CMD_DUMP_LOGS = 0x0C
CMD_SET_CRC = 0x0D
CMD_SET_SETTINGS = 0x0E
CMD_SET_SKIP_HALL = 0x0F
MAX_COLUMNS = 32
CMD_DEBUG_REPORT = 0x11
DEBUG_DETAIL_REPORT = 0xE0
DEBUG_SHIFT_TOTAL_BYTES = 12

class ProtocolError(Exception):
    pass


def _crc8(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) if (crc & 0x80) else (crc << 1)
            crc &= 0xFF
    return crc


def _make_frame(cmd: int, payload: bytes) -> bytes:
    length = len(payload)
    if length > 0xFF:
        raise ValueError("payload too large")
    header = bytes((cmd, length))
    crc = _crc8(header + payload)
    return bytes((FRAME_SYNC,)) + header + payload + bytes((crc,))


def _parse_response(stream: serial.Serial, verbose: bool = False) -> tuple[int, bytes, bytes]:
    ignore_warned = False
    while True:
        sync = stream.read(1)
        if not sync:
            if verbose:
                print("<<< timeout waiting for sync")
            raise ProtocolError("timeout waiting for sync")
        if sync[0] != FRAME_SYNC:
            if verbose and not ignore_warned:
                print("<<< ignoring non-sync bytes until FRAME_SYNC arrives")
                ignore_warned = True
            continue
        header = stream.read(2)
        if len(header) != 2:
            if verbose:
                print("<<< short header")
            raise ProtocolError("short header")
        cmd, length = header
        payload = stream.read(length)
        if len(payload) != length:
            if verbose:
                print(f"<<< short payload ({len(payload)}/{length})")
            raise ProtocolError("short payload")
        crc_bytes = stream.read(1)
        if not crc_bytes:
            if verbose:
                print("<<< missing crc")
            raise ProtocolError("missing crc")
        crc = crc_bytes[0]
        computed = _crc8(header + payload)
        if computed != crc:
            if verbose:
                print("<<< crc mismatch")
            raise ProtocolError("crc mismatch")
        frame = header + payload + bytes((crc,))
        return cmd, payload, frame


class PixelScreenClient:
    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        timeout: float = 5.0,
        verbose: bool = False,
        crc_enabled: bool = True,
        hardware_settings: dict | None = None,
    ):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(1.0)
        if hasattr(self.ser, "reset_input_buffer"):
            self.ser.reset_input_buffer()
        self.verbose = verbose
        self.auto_dump_logs = True
        self._crc_enabled = True
        if not crc_enabled:
            self._apply_crc_setting(False, suppress_log_dump=True)
        self._wait_for_ready()
        if hardware_settings:
            self.configure_hardware_settings(hardware_settings)

    def close(self):
        self.ser.close()

    def read_response(self) -> tuple[int, bytes]:
        """Read a protocol frame without sending a command first."""
        cmd, payload, _ = _parse_response(self.ser, self.verbose)
        return cmd, payload

    def send_setup_command(self, cmd: int, payload: bytes = b"") -> None:
        """Low-level helper for interactive setup commands."""
        self.ser.write(_make_frame(cmd, payload))

    def _send_frame(self, cmd: int, payload: bytes) -> tuple[int, bytes, bytes]:
        frame = _make_frame(cmd, payload)
        if self.verbose:
            self._dump_frame(">>>", frame)
        self.ser.write(frame)
        resp_cmd, resp_payload, resp_frame = _parse_response(self.ser, self.verbose)
        if self.verbose:
            self._dump_frame("<<<", resp_frame)
        return resp_cmd, resp_payload, resp_frame

    def _send_command(self, cmd: int, payload: bytes = b"") -> bytes:
        resp_cmd, resp_payload, _ = self._send_frame(cmd, payload)
        if resp_cmd != CMD_RESPONSE:
            raise ProtocolError(f"unexpected response {resp_cmd:02X}")
        if len(resp_payload) < 2:
            raise ProtocolError("response payload too small")
        status, detail = resp_payload[:2]
        if status != 0:
            raise ProtocolError(f"remote status {status:02X} detail {detail}")
        if self.verbose and self.auto_dump_logs:
            for line in self.dump_logs():
                print(f"[LOG] {line}")
        return resp_payload

    def _dump_frame(self, label: str, data: bytes) -> None:
        if not self.verbose:
            return
        print(f"{label} {' '.join(f'{b:02X}' for b in data)}")

    def home(self) -> None:
        """Run the homing routine on every column."""
        self._send_command(CMD_HOME)

    def start_interactive_setup(self) -> None:
        """Trigger the Arduino-side calibration routine."""
        self.send_setup_command(CMD_SETUP_START)

    def configure_column_mapping(self, mapping: Sequence[int], version: int = 1, motor_count: int = MAX_COLUMNS) -> None:
        """Send the 16-entry column->detent encoding before operating."""
        if len(mapping) != 16:
            raise ValueError("mapping must be 16 entries")
        for value in mapping:
            if not 0 <= value <= 0x0F:
                raise ValueError("mapping entries must be 0-15")
        payload = bytes((version, max(1, min(motor_count, MAX_COLUMNS)))) + bytes(mapping)
        self._send_command(CMD_CONFIGURE, payload)

    def write_columns(self, columns: Sequence[int]) -> None:
        """Write one nibble (0-15) per column sequentially."""
        if not 0 < len(columns) <= MAX_COLUMNS:
            raise ValueError(f"columns must be 1-{MAX_COLUMNS}, got {len(columns)}")
        for value in columns:
            if not 0 <= value <= 0x0F:
                raise ValueError("column data must be 0-15")
        payload = bytearray()
        payload.append(len(columns))
        bit_acc = 0
        bits_pending = 0
        for nibble in columns:
            bit_acc |= (nibble & 0x0F) << bits_pending
            bits_pending += 4
            if bits_pending >= 8:
                payload.append(bit_acc & 0xFF)
                bit_acc >>= 8
                bits_pending -= 8
        if bits_pending:
            payload.append(bit_acc & 0xFF)
        self._send_command(CMD_WRITE_COLUMNS, bytes(payload))

    def status(self) -> bytes:
        """Return raw status payload for diagnostics."""
        return self._send_command(CMD_STATUS)

    def tune_column(self, column: int, steps: int) -> None:
        """Pulse a column repeatedly (used for current tuning)."""
        if not 0 <= column < MAX_COLUMNS:
            raise ValueError("column must be 0-31")
        if steps <= 0 or steps > 0xFFFF:
            raise ValueError("steps must be 1-65535")
        payload = bytes((column, steps & 0xFF, (steps >> 8) & 0xFF))
        self._send_command(CMD_TUNE_COLUMN, payload)

    def read_hall_sensors(self) -> list[int]:
        """Return the raw hall sensor bitmask (32 sensors)."""
        payload = self._send_command(CMD_READ_HALL)
        if len(payload) < 6:
            raise ProtocolError("hall response too short")
        # first two bytes were status/detail, following bytes contain hall bits
        return list(payload[2 : 2 + 4])

    def read_debug_report(self) -> tuple[list[int], list[int]]:
        """Request the CRC/shift register debug snapshot from the Arduino."""
        payload = self._send_command(CMD_DEBUG_REPORT)
        if len(payload) < 2:
            raise ProtocolError("debug response too short")
        detail = payload[1]
        if detail != DEBUG_DETAIL_REPORT:
            raise ProtocolError(f"unexpected debug detail {detail:02X}")
        data = payload[2:]
        if len(data) < DEBUG_SHIFT_TOTAL_BYTES + 4:
            raise ProtocolError("debug payload too short")
        shift_inputs = list(data[:DEBUG_SHIFT_TOTAL_BYTES])
        hall_states = list(data[DEBUG_SHIFT_TOTAL_BYTES : DEBUG_SHIFT_TOTAL_BYTES + 4])
        return shift_inputs, hall_states

    def _wait_for_ready(self, timeout: float = 2.0) -> None:
        """Ensure the Arduino has finished printing its setup logs."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            lines = self.dump_logs()
            if any("Setup complete; waiting for commands" in line for line in lines):
                return
            if not lines:
                time.sleep(0.05)
                continue
            # if we drained logs but haven’t seen the marker yet, poll again.
        # fall back to no-op if the board never printed the marker

    def set_crc(self, enabled: bool) -> None:
        """Enable/disable CRC checking on the Arduino for debugging."""
        self._apply_crc_setting(enabled)

    def configure_hardware_settings(self, settings: dict) -> None:
        """Apply board-specific timing settings stored in the JSON config."""
        hall_timeout = int(settings.get("hall_timeout_ms", 0))
        step_interval = int(settings.get("step_interval_ms", 0))
        payload = bytes(
            (
                hall_timeout & 0xFF,
                (hall_timeout >> 8) & 0xFF,
                step_interval & 0xFF,
                (step_interval >> 8) & 0xFF,
            )
        )
        self._send_command(CMD_SET_SETTINGS, payload)
        if settings.get("skip_hall_home"):
            self._send_command(CMD_SET_SKIP_HALL, bytes((1,)))

    def dump_logs(self) -> list[str]:
        """Request the Arduino’s debug backlog (ASCII lines)."""
        resp_cmd, payload, _ = self._send_frame(CMD_DUMP_LOGS, b"")
        if resp_cmd != CMD_RESPONSE or len(payload) <= 2:
            return []
        text = payload[2:].decode("ascii", errors="ignore")
        return [line for line in text.splitlines() if line]

    def _apply_crc_setting(self, enabled: bool, *, suppress_log_dump: bool = False) -> None:
        """Internal helper that tells the Arduino whether to enforce CRC checking."""
        previous_auto = self.auto_dump_logs
        if suppress_log_dump:
            self.auto_dump_logs = False
        try:
            payload = bytes((1 if enabled else 0,))
            self._send_command(CMD_SET_CRC, payload)
        finally:
            if suppress_log_dump:
                self.auto_dump_logs = previous_auto
        self._crc_enabled = enabled

    def abort_setup(self) -> None:
        """Ask the Arduino to exit the interactive setup flow."""
        try:
            self._send_command(CMD_ABORT_SETUP)
        except ProtocolError:
            pass


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Send data to the PixelScreen Arduino.")
    parser.add_argument("port")
    parser.add_argument("--no-crc", action="store_true", help="Disable CRC checking on the Arduino")
    parser.add_argument("--width", type=int, default=8, help="columns to write")
    parser.add_argument("--verbose", action="store_true", help="Dump raw UART frames for debugging")
    args = parser.parse_args()

    client = PixelScreenClient(args.port, verbose=args.verbose, crc_enabled=not args.no_crc)
    try:
        client.home()
        pattern = [i & 0x0F for i in range(args.width)]
        client.write_columns(pattern)
    finally:
        client.close()
