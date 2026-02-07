"""Automated check that the Arduino asks for direction once the column homing stage completes."""
from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Sequence

from pixel_screen_client import (
    PixelScreenClient,
    CMD_SETUP_ACK_COLUMN,
    CMD_SETUP_DIRECTION,
    CMD_SETUP_START,
    CMD_RESPONSE,
    ProtocolError,
)

SETUP_DETAIL_COLUMN = 0x80
SETUP_DETAIL_DIRECTION = 0x81


def load_config(config_path: Path) -> dict:
    return json.loads(config_path.read_text())


def run_direction_check(config_path: Path, verbose: bool = False) -> bool:
    config_data = load_config(config_path)
    serial_cfg = config_data["serial"]
    hardware = config_data.get("hardware") or {}
    client = PixelScreenClient(
        serial_cfg["port"],
        serial_cfg.get("baudrate", 115200),
        serial_cfg.get("timeout", 5.0),
        verbose=verbose,
        crc_enabled=serial_cfg.get("crc_enabled", True),
        hardware_settings=hardware,
    )
    try:
        client.start_interactive_setup()
        while True:
            cmd, payload = client.read_response()
            if cmd != CMD_RESPONSE or len(payload) < 2:
                continue
            detail = payload[1]
            if SETUP_DETAIL_COLUMN <= detail < SETUP_DETAIL_DIRECTION:
                column = detail & 0x1F
                print(f"[CHECK] Column {column} homed; acking.")
                client.send_setup_command(CMD_SETUP_ACK_COLUMN, bytes((column,)))
            elif detail == SETUP_DETAIL_DIRECTION:
                print("[CHECK] Direction prompt received.")
                client.send_setup_command(CMD_SETUP_DIRECTION, b"\x01")
                return True
            else:
                print(f"[CHECK] Ignoring detail {detail}")
    finally:
        client.abort_setup()
        client.close()
    return False


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser(description="Verify the direction prompt appears.")
    parser.add_argument(
        "--config",
        default="python/pixel_screen_config.json",
        help="Path to the JSON configuration file",
    )
    parser.add_argument("--verbose", action="store_true", help="Verbose UART logging")
    args = parser.parse_args()

    try:
        success = run_direction_check(Path(args.config), verbose=args.verbose)
    except ProtocolError as exc:
        print(f"Protocol error: {exc}")
        sys.exit(1)
    if success:
        print("Direction prompt verified.")
    else:
        print("Direction prompt not seen.")


if __name__ == "__main__":
    main()
