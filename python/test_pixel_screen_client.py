"""Unit tests that ensure the UART client actually transmits and receives frames."""
from __future__ import annotations

import io
from typing import Any, Sequence

import pytest

from pixel_screen_client import (
    CMD_RESPONSE,
    CMD_SET_SETTINGS,
    CMD_SET_SKIP_HALL,
    CMD_DEBUG_REPORT,
    DEBUG_DETAIL_REPORT,
    DEBUG_SHIFT_TOTAL_BYTES,
    FRAME_SYNC,
    PixelScreenClient,
    _crc8,
    _make_frame,
)


class DummySerial:
    def __init__(self, response: bytes):
        self._response = bytearray(response)
        self.written = bytearray()

    def write(self, data: bytes) -> int:
        self.written += data
        return len(data)

    def read(self, count: int = 1) -> bytes:
        if count == 0:
            return b""
        out = self._response[:count]
        self._response = self._response[count:]
        return bytes(out)

    def close(self) -> None:
        pass


class DummySerialFactory:
    def __init__(self, response: bytes) -> None:
        self.response = response
        self.created = 0
        self.last: DummySerial | None = None

    def __call__(self, *args, **kwargs) -> DummySerial:
        self.created += 1
        self.last = DummySerial(self.response)
        return self.last


@pytest.fixture(autouse=True)
def stub_ready_wait(monkeypatch):
    monkeypatch.setattr(PixelScreenClient, "_wait_for_ready", lambda self, timeout=2.0: None)
    yield


def build_response(detail: int = 0, extra: bytes = b"") -> bytes:
    payload = bytes((0, detail)) + extra
    header = bytes((CMD_RESPONSE, len(payload)))
    frame = bytes((FRAME_SYNC,)) + header + payload
    frame += bytes((_crc8(header + payload),))
    return frame


@pytest.mark.parametrize("payload", [b"", b"\x01\x02"])
def test_send_command_writes_frame(monkeypatch, payload: bytes) -> None:
    """Verify that `_send_command` actually writes a framed request."""
    response = build_response(detail=0)
    factory = DummySerialFactory(response)
    monkeypatch.setattr("pixel_screen_client.serial.Serial", factory)
    client = PixelScreenClient("/dev/null")
    try:
        client.verbose = True
        client.auto_dump_logs = False
        client._send_command(0x09, payload)
    finally:
        client.close()
    expected = _make_frame(0x09, payload)
    assert factory.created == 1
    assert factory.last is not None
    assert expected == factory.last.written


def test_parse_response_reports_payload(monkeypatch) -> None:
    """Ensure `_parse_response` returns the command and payload from the raw stream."""
    payload = b"\x01\x02"
    response = build_response(detail=0, extra=payload)
    dummy = DummySerial(response)
    # read_response uses _parse_response directly
    monkeypatch.setattr("pixel_screen_client.serial.Serial", lambda *args, **kwargs: dummy)
    client = PixelScreenClient("/dev/null")
    try:
        cmd, resp_payload = client.read_response()
    finally:
        client.close()
    assert cmd == CMD_RESPONSE
    assert resp_payload[2:] == payload


def build_debug_response(shift: bytes, hall: bytes) -> bytes:
    payload = bytes((0, DEBUG_DETAIL_REPORT)) + shift + hall
    header = bytes((CMD_RESPONSE, len(payload)))
    frame = bytes((FRAME_SYNC,)) + header + payload
    frame += bytes((_crc8(header + payload),))
    return frame


def test_read_debug_report(monkeypatch) -> None:
    shift_bytes = bytes(range(DEBUG_SHIFT_TOTAL_BYTES))
    hall_bytes = bytes((0x01, 0x02, 0x03, 0x04))
    response = build_debug_response(shift_bytes, hall_bytes)
    dummy = DummySerialFactory(response)
    monkeypatch.setattr("pixel_screen_client.serial.Serial", dummy)
    client = PixelScreenClient("/dev/null")
    try:
        shift_values, hall_values = client.read_debug_report()
    finally:
        client.close()
    assert shift_values == list(shift_bytes)
    assert hall_values == list(hall_bytes)


def test_dump_logs_returns_lines(monkeypatch) -> None:
    """Verify dump_logs splits ASCII output into lines."""
    response_payload = bytes([0, 0]) + b"line1\nline2\n"
    dummy = DummySerialFactory(build_response(detail=0))
    monkeypatch.setattr("pixel_screen_client.serial.Serial", dummy)
    client = PixelScreenClient("/dev/null")
    def fake_send_frame(cmd, payload: bytes):
        return CMD_RESPONSE, response_payload, b""
    monkeypatch.setattr(client, "_send_frame", fake_send_frame)
    try:
        lines = client.dump_logs()
    finally:
        client.close()
    assert lines == ["line1", "line2"]


def test_configure_hardware_settings_emits_skip(monkeypatch) -> None:
    """Ensure the hardware settings commands (including skip) fire."""
    captured: list[tuple[int, bytes]] = []

    def fake_send(cmd: int, payload: bytes) -> bytes:
        captured.append((cmd, payload))
        return bytes()

    dummy = DummySerialFactory(build_response(detail=0))
    monkeypatch.setattr("pixel_screen_client.serial.Serial", dummy)
    client = PixelScreenClient("/dev/null")
    monkeypatch.setattr(client, "_send_command", fake_send)

    client.configure_hardware_settings(
        {"hall_timeout_ms": 0x04D2, "step_interval_ms": 0x0A, "skip_hall_home": True}
    )

    assert (CMD_SET_SETTINGS, bytes((0xD2, 0x04, 0x0A, 0x00))) in captured
    assert (CMD_SET_SKIP_HALL, bytes((1,))) in captured


def test_init_waits_for_ready_before_config(monkeypatch) -> None:
    order: list[str] = []

    class DummySerial:
        def __init__(self, *args: Any, **kwargs: Any) -> None:
            pass

        def reset_input_buffer(self) -> None:
            pass

        def close(self) -> None:
            pass

    def fake_wait(self, timeout: float = 2.0) -> None:
        order.append("wait")

    def fake_config(self, settings: dict) -> None:
        order.append("config")

    monkeypatch.setattr("pixel_screen_client.serial.Serial", DummySerial)
    monkeypatch.setattr(PixelScreenClient, "_wait_for_ready", fake_wait)
    monkeypatch.setattr(PixelScreenClient, "configure_hardware_settings", fake_config)

    PixelScreenClient("/dev/null", hardware_settings={"hall_timeout_ms": 10})

    assert order == ["wait", "config"]
