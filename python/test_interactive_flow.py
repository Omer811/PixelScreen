from __future__ import annotations

import shutil
from collections import deque
from pathlib import Path
from typing import Any

import pytest

from pixel_screen_client import (
    CMD_RESPONSE,
    CMD_SETUP_ACK_COLUMN,
    CMD_SETUP_DIRECTION,
    CMD_SETUP_POSITION,
    ProtocolError,
)
from pixel_screen_setup import run_interactive_setup

SETUP_DETAIL_COLUMN = 0x80
SETUP_DETAIL_DIRECTION = 0x81
SETUP_DETAIL_STEPS = 0x82
SETUP_DETAIL_POSITION_BASE = 0x90
SETUP_DETAIL_DONE = 0xF0


def _build_response(detail: int, extra: bytes = b"") -> tuple[int, bytes]:
    return (CMD_RESPONSE, bytes((0, detail)) + extra)


def _interactive_responses() -> list[tuple[int, bytes]]:
    responses: list[tuple[int, bytes]] = [
        _build_response(SETUP_DETAIL_COLUMN),
        _build_response(SETUP_DETAIL_DIRECTION),
        _build_response(SETUP_DETAIL_STEPS, bytes((0x20, 0x00))),
    ]
    for pos in range(16):
        responses.append(_build_response(SETUP_DETAIL_POSITION_BASE + pos))
    responses.append(_build_response(SETUP_DETAIL_DONE, bytes((0x20, 0x00))))
    return responses


class FakeInteractiveClient:
    def __init__(self, responses: list[tuple[int, bytes]]):
        self._responses = deque(responses)
        self.commands: list[tuple[str, Any]] = []
        self.mapping: list[int] | None = None

    def home(self) -> None:
        self.commands.append(("home", None))

    def configure_hardware_settings(self, settings: dict) -> None:
        self.commands.append(("settings", dict(settings)))

    def start_interactive_setup(self) -> None:
        self.commands.append(("start", None))

    def read_response(self) -> tuple[int, bytes]:
        if not self._responses:
            raise ProtocolError("no more responses")
        return self._responses.popleft()

    def send_setup_command(self, cmd: int, payload: bytes = b"") -> None:
        self.commands.append(("command", cmd, payload))

    def configure_column_mapping(self, mapping: list[int], version: int = 1, motor_count: int = 32) -> None:
        self.mapping = list(mapping)
        self.commands.append(("configure", tuple(mapping), version, motor_count))

    def abort_setup(self) -> None:
        self.commands.append(("abort", None))

    def close(self) -> None:
        self.commands.append(("close", None))


def _prepare_config(tmp_path: Path) -> Path:
    config_path = tmp_path / "config.json"
    shutil.copy(Path("python/pixel_screen_config.json"), config_path)
    return config_path


def test_interactive_flow_direction_observed(monkeypatch, tmp_path):
    config_path = _prepare_config(tmp_path)
    patterns = iter(f"{i:04b}" for i in range(16))
    monkeypatch.setattr("pixel_screen_setup.prompt_bits", lambda pos: next(patterns))
    monkeypatch.setattr("pixel_screen_setup.prompt_direction_confirmation", lambda: False)
    client = FakeInteractiveClient(_interactive_responses())
    factory = lambda *args, **kwargs: client

    run_interactive_setup(config_path, force=True, client_factory=factory)

    direction_cmds = [
        entry for entry in client.commands if entry[0] == "command" and entry[1] == CMD_SETUP_DIRECTION
    ]
    assert len(direction_cmds) == 1
    assert direction_cmds[0][2] == b"\x00"

    ack_cmds = [
        entry for entry in client.commands if entry[0] == "command" and entry[1] == CMD_SETUP_ACK_COLUMN
    ]
    assert ack_cmds == [("command", CMD_SETUP_ACK_COLUMN, b"\x00")]

    assert client.mapping is not None
    assert client.mapping[0] == 0


def test_interactive_flow_positions_sent(monkeypatch, tmp_path):
    config_path = _prepare_config(tmp_path)
    patterns = iter(f"{(i + 1) % 16:04b}" for i in range(16))
    monkeypatch.setattr("pixel_screen_setup.prompt_bits", lambda pos: next(patterns))
    monkeypatch.setattr("pixel_screen_setup.prompt_direction_confirmation", lambda: True)
    client = FakeInteractiveClient(_interactive_responses())
    factory = lambda *args, **kwargs: client

    run_interactive_setup(config_path, force=True, client_factory=factory)

    position_cmds = [
        entry for entry in client.commands if entry[0] == "command" and entry[1] == CMD_SETUP_POSITION
    ]
    assert len(position_cmds) == 16

    mapped = set()
    for _, _, payload in position_cmds:
        bits = payload.decode("ascii")
        mapped.add(bits)
    assert len(mapped) == 16

    assert client.mapping is not None
    assert set(client.mapping) == set(range(16))
